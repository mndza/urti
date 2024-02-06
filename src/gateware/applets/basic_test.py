#
# This file is part of URTI.
#
# Copyright (c) 2024 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

import usb
import time

from amaranth                                   import Elaboratable, Module, Signal, Cat, DomainRenamer
from amaranth.lib.fifo                          import AsyncFIFO
from amaranth.lib.wiring                        import Out, Signature, connect
from amaranth_soc                               import wishbone
from amaranth_soc                               import csr
from amaranth_soc.csr.wishbone                  import WishboneCSRBridge

from usb_protocol.emitters                      import DeviceDescriptorCollection
from usb_protocol.types                         import USBRequestType

from luna                                       import top_level_cli
from luna.usb2                                  import USBDevice, USBStreamInEndpoint
from luna.gateware.usb.request.control          import ControlRequestHandler
from luna.gateware.usb.usb2.transfer            import USBInStreamInterface
from luna.gateware.stream.generator             import StreamSerializer

from urti.gateware.architecture.car             import URTIDomainGenerator
from urti.gateware.interface.max5865            import MAX5865DataInterface, MAX5865OpModeSetter
from urti.gateware.interface.max2120            import MAX2120
from urti.gateware.interface.rffc5072           import RFFC5072RegisterInterface


class URTITestRequestHandler(ControlRequestHandler):
    REQUEST_READ_REG   = 0
    REQUEST_WRITE_REG  = 1

    def __init__(self):
        super().__init__()
        self.signature = Signature({
            "bus": Out(wishbone.Signature(addr_width=8, data_width=16))
        }).create()
        self.bus = self.signature.bus

    def elaborate(self, platform):
        m = Module()

        interface = self.interface
        setup     = self.interface.setup
        read_data = Signal.like(self.bus.dat_r)

        # Handler for read register requests.
        m.submodules.transmitter = transmitter = \
            StreamSerializer(data_length=2, domain="usb", stream_type=USBInStreamInterface, max_length_width=2)

        with m.If(setup.type == USBRequestType.VENDOR):
            
            if hasattr(interface, "claim"):
                m.d.comb += interface.claim.eq(1)

            with m.FSM(domain="usb"):

                with m.State("IDLE"):

                    with m.If(setup.received):

                        with m.Switch(setup.request):

                            with m.Case(self.REQUEST_READ_REG):
                                m.d.usb += [
                                    self.bus.cyc                .eq(1),
                                    self.bus.stb                .eq(1),
                                    self.bus.we                 .eq(0),
                                    self.bus.sel                .eq(1),
                                    self.bus.adr                .eq(setup.index),
                                ]
                                m.next = "READ_WAIT_FOR_ACK"

                            with m.Case(self.REQUEST_WRITE_REG):
                                m.d.usb += [
                                    self.bus.cyc                .eq(1),
                                    self.bus.stb                .eq(1),
                                    self.bus.we                 .eq(1),
                                    self.bus.sel                .eq(1),
                                    self.bus.adr                .eq(setup.index),
                                    self.bus.dat_w              .eq(setup.value),
                                ]
                                m.next = "WRITE_WAIT_FOR_ACK"

                            with m.Default():
                                m.next = "UNHANDLED"

                with m.State("READ_WAIT_FOR_ACK"):
                    with m.If(self.bus.ack):
                        m.d.usb += [
                            self.bus.cyc    .eq(0),
                            self.bus.stb    .eq(0),
                            read_data       .eq(self.bus.dat_r),
                        ]
                        m.next = "READ_REG_FINISH"
                
                with m.State("WRITE_WAIT_FOR_ACK"):
                    with m.If(self.bus.ack):
                        m.d.usb += [
                            self.bus.cyc    .eq(0),
                            self.bus.stb    .eq(0),
                        ]
                        m.next = "WRITE_REG_FINISH"

                with m.State("READ_REG_FINISH"):
                    self.handle_simple_data_request(m, transmitter, read_data, length=len(read_data)//8)

                with m.State("WRITE_REG_FINISH"):
                    # Provide an response to the STATUS stage.
                    with m.If(interface.status_requested):
                        m.d.comb += self.send_zlp()
                        m.next = "IDLE"

                with m.State("UNHANDLED"):

                    #
                    # Stall unhandled requests.
                    #
                    with m.If(interface.status_requested | interface.data_requested):
                        m.d.comb += interface.handshakes_out.stall.eq(1)
                        m.next = "IDLE"

        return m



class URTIBasicTestGateware(Elaboratable):

    BULK_ENDPOINT_NUMBER = 1
    MAX_BULK_PACKET_SIZE = 512

    def create_descriptors(self):
        """ Create the descriptors we want to use for our device. """

        descriptors = DeviceDescriptorCollection()

        #
        # We'll add the major components of the descriptors we we want.
        # The collection we build here will be necessary to create a standard endpoint.
        #

        # We'll need a device descriptor...
        with descriptors.DeviceDescriptor() as d:
            d.idVendor           = 0x16d0
            d.idProduct          = 0xf3b

            d.iManufacturer      = "URTI"
            d.iProduct           = "Test gateware"
            d.iSerialNumber      = "1234"

            d.bNumConfigurations = 1

        # ... and a description of the USB configuration we'll provide.
        with descriptors.ConfigurationDescriptor() as c:

            with c.InterfaceDescriptor() as i:
                i.bInterfaceNumber = 0

                with i.EndpointDescriptor() as e:
                    e.bEndpointAddress = 0x80 | self.BULK_ENDPOINT_NUMBER
                    e.wMaxPacketSize   = self.MAX_BULK_PACKET_SIZE

        return descriptors


    def elaborate(self, platform):
        m = Module()

        # Generate our domain clocks/resets.
        m.submodules.car = URTIDomainGenerator()

        # Create our USB device interface...
        ulpi = platform.request(platform.default_usb_connection)
        m.submodules.usb = usb = USBDevice(bus=ulpi)

        # Add our standard control endpoint to the device.
        descriptors = self.create_descriptors()
        control_ep = usb.add_standard_control_endpoint(descriptors)

        # Add our custom request handler.
        req_handler = URTITestRequestHandler()
        control_ep.add_request_handler(req_handler)

        # Configure power supplies
        m.d.comb += [
            platform.request("pwr_3v3_a").o   .eq(1),
            platform.request("pwr_3v3_rx").o  .eq(1),
            platform.request("pwr_3v3_tx").o  .eq(0),
        ]

        # MAX5865 analog frontend ADC/DAC.
        max5865_intf = platform.request("afe")
        m.submodules.afe_data = afe_data = DomainRenamer("radio")(MAX5865DataInterface(pads=max5865_intf))
        m.submodules.afe_ctrl = afe_ctrl = DomainRenamer("usb")(MAX5865OpModeSetter(pads=max5865_intf, divisor=4))
        # MAX2120 Direct-Conversion Tuner.
        max2120_intf = platform.request("max2120")
        m.submodules.max2120  = max2120  = DomainRenamer("usb")(MAX2120(pads=max2120_intf))
        # RFFC5072 3-wire serial interface and gain DAC.
        rffc5072_intf = platform.request("rx_mix_ctrl")
        m.submodules.rffc5072 = rffc5072 = DomainRenamer("usb")(RFFC5072RegisterInterface(pads=rffc5072_intf, divisor=256))
        

        # Create a Wishbone decoder that routes requests to the different targets.
        # TODO: generate memory map automatically
        m.submodules.decoder = decoder = wishbone.Decoder(addr_width=8, data_width=16)
        decoder.add(max2120.bus,  addr=0x00, sparse=True)
        decoder.add(rffc5072.bus, addr=0x20, sparse=True)
        decoder.add(afe_ctrl.bus, addr=0x40, sparse=True)

        # Define set of CSR registers.abs
        # TODO: move this inside MAX2120?
        max2120_gain = csr.Element(width=len(max2120_intf.gain.o), access="rw")
        m.d.comb += max2120_gain.r_data.eq(max2120_intf.gain.o)
        with m.If(max2120_gain.w_stb):
            m.d.sync += max2120_intf.gain.o.eq(max2120_gain.w_data)

        # Add the CSR registers to the decoder using a multiplexer and a Wishbone-CSR bridge.
        csr_mux = csr.Multiplexer(addr_width=4, data_width=8)
        csr_mux.add(max2120_gain, name="max2120_gain")
        csr_bridge = WishboneCSRBridge(csr_mux.bus)
        decoder.add(csr_bridge.wb_bus, addr=0x50, sparse=True)
        m.submodules += [csr_mux, csr_bridge]


        # Connect our request handler to the Wshbone decoder.
        connect(m, req_handler.bus, decoder.bus)

        # Add a stream endpoint to our device with samples truncated to 4 bits.
        stream_ep = USBStreamInEndpoint(
            endpoint_number=self.BULK_ENDPOINT_NUMBER,
            max_packet_size=self.MAX_BULK_PACKET_SIZE
        )
        usb.add_endpoint(stream_ep)

        # Async FIFOs for interfacing with the AFE, cross-domain clocking.
        # TODO: Write only when reception is enabled.
        m.submodules.rx_fifo = rx_fifo = AsyncFIFO(width=8, depth=64, r_domain="usb", w_domain="radio")
        m.d.comb += [
            rx_fifo.w_data              .eq(Cat(afe_data.adc_data_i[:4], afe_data.adc_data_q[:4])),
            rx_fifo.w_en                .eq(1),
            stream_ep.stream.payload    .eq(rx_fifo.r_data),
            stream_ep.stream.valid      .eq(rx_fifo.r_rdy),
            rx_fifo.r_en                .eq(stream_ep.stream.ready),
        ]

        # Connect our device by default.
        m.d.comb += usb.connect.eq(1)

        return m



class URTIBasicTestGatewareConnection:
    """ Class representing a link to the URTI test gateware. """

    USB_ID  = (0x16d0, 0xf3b)

    MAX2120_BASE_ADDR     = 0x00
    RFFC5072_RX_BASE_ADDR = 0x20
    MAX5865_BASE_ADDR     = 0x40
    MAX2120_GAIN_ADDR     = 0x50

    def __init__(self):
        """ Sets up a connection to the URTI device. """

        # Try to create a connection.
        device = usb.core.find(idVendor=self.USB_ID[0], idProduct=self.USB_ID[1])

        # If we couldn't find a valid device, bail out.
        if device is None:
            raise IOError("Unable to find URTI device.")

        self.device = device

    def _print_methods(self):
        import inspect

        for name, member in inspect.getmembers(self):
            if inspect.ismethod(member) and not name.startswith('_'):
                func_args = list(inspect.signature(member).parameters.keys())
                print(f"- {name}({', '.join(func_args)})")
        
    def _out_request(self, number, value=0, index=0, data=None, timeout=1000):
        """ Helper that issues an OUT control request to the debugger. """

        request_type = usb.ENDPOINT_OUT | usb.RECIP_DEVICE | usb.TYPE_VENDOR
        return self.device.ctrl_transfer(request_type, number, value, index, data, timeout=timeout)

    def _in_request(self, number, value=0, index=0, length=0, timeout=1000):
        """ Helper that issues an IN control request to the debugger. """

        request_type = usb.ENDPOINT_IN | usb.RECIP_DEVICE | usb.TYPE_VENDOR
        result = self.device.ctrl_transfer(request_type, number, value, index, length, timeout=timeout)

        return bytes(result)

    def max5865_set_opmode(self, opmode):
        self._out_request(URTITestRequestHandler.REQUEST_WRITE_REG, value=opmode, index=self.MAX5865_BASE_ADDR)

    def max2120_set_gain(self, gain):
        self._out_request(URTITestRequestHandler.REQUEST_WRITE_REG, value=gain, index=self.MAX2120_GAIN_ADDR)

    def max2120_read(self, address):
        return self._in_request(URTITestRequestHandler.REQUEST_READ_REG, index=self.MAX2120_BASE_ADDR | address)

    def max2120_write(self, address, value):
        self._out_request(URTITestRequestHandler.REQUEST_WRITE_REG, value=value, index=self.MAX2120_BASE_ADDR | address)

    def rffc5072_read(self, address):
        return self._in_request(URTITestRequestHandler.REQUEST_READ_REG, index=self.RFFC5072_RX_BASE_ADDR | address)

    def rffc5072_write(self, address, value):
        self._out_request(URTITestRequestHandler.REQUEST_WRITE_REG, value=value, index=self.RFFC5072_RX_BASE_ADDR | address)


if __name__ == "__main__":
    # Build and program the test gateware.
    top_level_cli(URTIBasicTestGateware)

    # Wait for the USB device to be enumerated.
    time.sleep(2)

    # Create a connection to URTI and print a list of methods.
    urti = URTIBasicTestGatewareConnection()
    print("'urti' methods:")
    urti._print_methods()
