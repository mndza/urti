#
# This file is part of URTI.
#
# Copyright (c) 2024 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

from amaranth                                   import Elaboratable, Module, Signal, Cat, DomainRenamer
from amaranth.lib.fifo                          import AsyncFIFO

from usb_protocol.emitters                      import DeviceDescriptorCollection
from usb_protocol.types                         import USBRequestType

from luna                                       import top_level_cli
from luna.usb2                                  import USBDevice, USBStreamInEndpoint
from luna.gateware.usb.request.control          import ControlRequestHandler
from luna.gateware.usb.usb2.transfer            import USBInStreamInterface
from luna.gateware.stream.generator             import StreamSerializer

from urti.gateware.architecture.car             import URTIDomainGenerator
from urti.gateware.interface.max5865            import MAX5865DataInterface, MAX5865OpMode, MAX5865OpModeSetter
from urti.gateware.interface.max2120            import MAX2120
from urti.gateware.interface.rffc5072           import RFFC5072RegisterInterface


class URTITestRequestHandler(ControlRequestHandler):
    REQUEST_MAX5865_SET_OPMODE = 0
    REQUEST_MAX2120_READ_REG   = 1
    REQUEST_MAX2120_WRITE_REG  = 2
    REQUEST_RFFC5072_READ_REG  = 3
    REQUEST_RFFC5072_WRITE_REG = 4

    def __init__(self):
        super().__init__()
        
        # Interface for setting MAX5865 opmode
        self.max5865_opmode        = Signal(MAX5865OpMode)
        self.max5865_set           = Signal()

        # Interface for MAX2120
        self.max2120_busy          = Signal()
        self.max2120_address       = Signal(8)
        self.max2120_done          = Signal()
        self.max2120_read_request  = Signal()
        self.max2120_read_data     = Signal(8)
        self.max2120_write_request = Signal()
        self.max2120_write_data    = Signal(8)

        # Interface for RFFC5072
        self.rffc5072_start        = Signal()
        self.rffc5072_busy         = Signal()
        self.rffc5072_address      = Signal(7)
        self.rffc5072_is_write     = Signal()
        self.rffc5072_data_in      = Signal(16)
        self.rffc5072_data_out     = Signal(16)

    def elaborate(self, platform):
        m = Module()

        interface         = self.interface
        setup             = self.interface.setup

        # Handler for read register requests.
        m.submodules.transmitter = transmitter = \
            StreamSerializer(data_length=2, domain="usb", stream_type=USBInStreamInterface, max_length_width=2)

        m.d.usb += [
            self.max5865_set            .eq(0),
            self.rffc5072_start         .eq(0),
            self.max2120_read_request   .eq(0),
            self.max2120_write_request  .eq(0),
        ]

        with m.If(setup.type == USBRequestType.VENDOR):
            
            if hasattr(interface, "claim"):
                m.d.comb += interface.claim.eq(1)

            with m.FSM(domain="usb"):

                with m.State("IDLE"):

                    with m.If(setup.received):

                        with m.Switch(setup.request):

                            with m.Case(self.REQUEST_MAX5865_SET_OPMODE):
                                m.d.usb += [
                                    self.max5865_opmode .eq(setup.value[:3]),
                                    self.max5865_set    .eq(1),
                                ]
                                # TODO: wait for the module to finish before ACK
                                m.next = "WRITE_REG_FINISH"

                            with m.Case(self.REQUEST_MAX2120_READ_REG):
                                m.d.usb += [
                                    self.max2120_read_request   .eq(1),
                                    self.max2120_address        .eq(setup.index[:8]),
                                ]
                                m.next = "MAX2120_READ_REG"

                            with m.Case(self.REQUEST_MAX2120_WRITE_REG):
                                m.d.usb += [
                                    self.max2120_write_request  .eq(1),
                                    self.max2120_write_data     .eq(setup.value[:8]),
                                    self.max2120_address        .eq(setup.index[:8]),
                                ]
                                m.next = "MAX2120_WRITE_REG"
                                
                            with m.Case(self.REQUEST_RFFC5072_READ_REG):
                                m.d.usb += [
                                    self.rffc5072_start         .eq(1),
                                    self.rffc5072_is_write      .eq(0),
                                    self.rffc5072_address       .eq(setup.index[:7]),
                                ]
                                m.next = "RFFC5072_READ_REG"

                            with m.Case(self.REQUEST_RFFC5072_WRITE_REG):
                                m.d.usb += [
                                    self.rffc5072_start         .eq(1),
                                    self.rffc5072_is_write      .eq(1),
                                    self.rffc5072_data_in       .eq(setup.value[:16]),
                                    self.rffc5072_address       .eq(setup.index[:7]),
                                ]
                                m.next = "RFFC5072_WRITE_REG"

                            with m.Default():
                                m.next = "UNHANDLED"

                with m.State("MAX2120_READ_REG"):
                    with m.If(self.max2120_done):
                        m.next = "MAX2120_READ_REG_FINISH"

                with m.State("MAX2120_READ_REG_FINISH"):
                    self.handle_simple_data_request(m, transmitter, self.max2120_read_data, length=1)

                with m.State("RFFC5072_READ_REG"):
                    with m.If(~self.rffc5072_busy):
                        m.next = "RFFC5072_READ_REG_FINISH"

                with m.State("RFFC5072_READ_REG_FINISH"):
                    self.handle_simple_data_request(m, transmitter, self.rffc5072_data_out, length=2)

                with m.State("MAX2120_WRITE_REG"):
                    with m.If(self.max2120_done):
                        m.next = "WRITE_REG_FINISH"

                with m.State("RFFC5072_WRITE_REG"):
                    with m.If(~self.rffc5072_busy):
                        m.next = "WRITE_REG_FINISH"

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
        m.submodules.max2120  = max2120  = DomainRenamer("usb")(MAX2120())
        # RFFC5072 3-wire serial interface and gain DAC.
        rffc5072_intf = platform.request("rx_mix_ctrl")
        m.submodules.rffc5072 = rffc5072 = DomainRenamer("usb")(RFFC5072RegisterInterface(pads=rffc5072_intf, divisor=256))
        
        # Connect interface lines to request handler.
        # TODO: Use amaranth.lib.wiring to greatly simplify these connections.
        m.d.comb += [
            afe_ctrl.opmode                     .eq(req_handler.max5865_opmode),
            afe_ctrl.set                        .eq(req_handler.max5865_set),
            
            req_handler.max2120_busy            .eq(max2120.busy),
            max2120.address                     .eq(req_handler.max2120_address),
            req_handler.max2120_done            .eq(max2120.done),
            max2120.read_request                .eq(req_handler.max2120_read_request),
            req_handler.max2120_read_data       .eq(max2120.read_data),
            max2120.write_request               .eq(req_handler.max2120_write_request),
            max2120.write_data                  .eq(req_handler.max2120_write_data),

            rffc5072.start                      .eq(req_handler.rffc5072_start),
            req_handler.rffc5072_busy           .eq(rffc5072.busy),
            rffc5072.address                    .eq(req_handler.rffc5072_address),
            rffc5072.is_write                   .eq(req_handler.rffc5072_is_write),
            rffc5072.data_in                    .eq(req_handler.rffc5072_data_in),
            req_handler.rffc5072_data_out       .eq(rffc5072.data_out),
        ]


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


if __name__ == "__main__":
    top_level_cli(URTIBasicTestGateware)
