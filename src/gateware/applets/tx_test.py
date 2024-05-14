#
# This file is part of URTI.
#
# Copyright (c) 2024 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

import usb
import time
import math

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
from urti.gateware.interface                    import \
    MAX5865DataInterface, MAX5865OpModeSetter, MAX2120, MAX2831, RFFC5072RegisterInterface


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
        read_len  = Signal(2)

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
                                    read_len                    .eq(setup.length),
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

                    # Tell the host we are not ready for the DATA/STATUS stages yet
                    with m.If(interface.data_requested | interface.status_requested):
                        m.d.comb += interface.handshakes_out.nak.eq(1)

                with m.State("WRITE_WAIT_FOR_ACK"):
                    with m.If(self.bus.ack):
                        m.d.usb += [
                            self.bus.cyc    .eq(0),
                            self.bus.stb    .eq(0),
                        ]
                        m.next = "WRITE_REG_FINISH"

                    # Tell the host we are not ready for the STATUS stage yet
                    with m.If(interface.status_requested):
                        m.d.comb += interface.handshakes_out.nak.eq(1)

                with m.State("READ_REG_FINISH"):
                    self.handle_simple_data_request(m, transmitter, read_data, length=len(read_data)//8)
                    m.d.comb += transmitter.max_length.eq(read_len)

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
            platform.request("pwr_3v3_rx").o  .eq(0),
            platform.request("pwr_3v3_tx").o  .eq(1),
        ]

        # MAX5865 analog frontend ADC/DAC.
        max5865_intf = platform.request("afe")
        m.submodules.afe_data = afe_data = DomainRenamer("radio")(MAX5865DataInterface(pads=max5865_intf))
        m.submodules.afe_ctrl = afe_ctrl = DomainRenamer("usb")(MAX5865OpModeSetter(pads=max5865_intf, divisor=4))
        # MAX2120 Direct-Conversion Tuner.
        max2120_intf = platform.request("max2120")
        m.submodules.max2120  = max2120  = DomainRenamer("usb")(MAX2120(pads=max2120_intf, divisor=152))
        # RFFC5072 wideband synthesizer / VCO with integrated mixer (RX and TX).
        rffc5072_rx_intf = platform.request("rx_mix_serial")
        m.submodules.rffc5072_rx = rffc5072_rx = DomainRenamer("usb")(
            RFFC5072RegisterInterface(pads=rffc5072_rx_intf, divisor=4, name="rffc5072_rx"))
        rffc5072_tx_intf = platform.request("tx_mix_serial")
        m.submodules.rffc5072_tx = rffc5072_tx = DomainRenamer("usb")(
            RFFC5072RegisterInterface(pads=rffc5072_tx_intf, divisor=4, name="rffc5072_tx"))
        # MAX2831 RF transceiver (used for TX only).
        max2831_intf = platform.request("max2831")
        m.submodules.max2831 = max2831 = DomainRenamer("usb")(MAX2831(pads=max2831_intf, divisor=4))


        # Create a Wishbone decoder that routes requests to the different targets.
        # TODO: generate memory map automatically
        m.submodules.decoder = decoder = wishbone.Decoder(addr_width=8, data_width=16)
        decoder.add(max2120.bus,     addr=0x00, sparse=True)
        decoder.add(rffc5072_rx.bus, addr=0x20, sparse=True)
        decoder.add(afe_ctrl.bus,    addr=0x40, sparse=True)
        decoder.add(max2831.bus,     addr=0x50, sparse=True)
        decoder.add(rffc5072_tx.bus, addr=0x60, sparse=True)

        #
        # Define set of CSR registers.
        #

        # TODO: move this inside MAX2120?
        max2120_gain_reg = csr.reg.Register({
            "gain": csr.Field(csr.action.RW, len(max2120_intf.gain.o))
        }, access="rw")
        m.d.comb += max2120_intf.gain.o.eq(max2120_gain_reg.f.gain.data)

        # LEDs
        leds = Cat([platform.request("led", i, dir="o").o for i in range(3)])
        leds_reg = csr.reg.Register({
            "led": csr.Field(csr.action.RW, len(leds))
        }, access="rw")
        m.d.comb += leds.eq(leds_reg.f.led.data)

        # Select correct RFFC5072 mixer
        mix_ctrl = platform.request("tx_mix_ctrl")
        m.d.comb += mix_ctrl.mode.o.eq(1)

        # Use LED signals for RF control
        bypass = platform.request("tx_mix_bypass").o
        m.d.comb += bypass.eq(leds[0])
        m.d.comb += mix_ctrl.enbl.o.eq(leds[1])
        #m.d.comb += mix_ctrl.reset.o.eq(leds[2])

        # Gather all CSR definitions.
        regs = csr.Builder(addr_width=4, data_width=8)
        regs.add("max2120_gain", max2120_gain_reg)
        regs.add("leds", leds_reg)
        
        # Add the CSR registers to the Wishbone decoder.
        csr_bridge = csr.Bridge(regs.as_memory_map())
        wb_bridge = WishboneCSRBridge(csr_bridge.bus)
        m.submodules += [csr_bridge, wb_bridge]
        decoder.add(wb_bridge.wb_bus, addr=0x80, sparse=True)

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

        m.d.comb += [
            afe_data.dac_data_i.eq(0x280),
            afe_data.dac_data_q.eq(0x17f),
        ]

        m.d.comb += max2831_intf.rxtx.o.eq(1)

        return m



class URTIBasicTestGatewareConnection:
    """ Class representing a link to the URTI test gateware. """

    USB_ID  = (0x16d0, 0xf3b)

    # Request numbers
    REQUEST_READ_REG      = 0
    REQUEST_WRITE_REG     = 1

    # Base addresses
    MAX2120_BASE_ADDR     = 0x00
    RFFC5072_RX_BASE_ADDR = 0x20
    MAX5865_BASE_ADDR     = 0x40
    MAX2831_BASE_ADDR     = 0x50
    RFFC5072_TX_BASE_ADDR = 0x60
    CSR_BASE_ADDR         = 0x80

    # CSR addresses
    CSR_MAX2120_GAIN_ADDR = 0x00
    CSR_LEDS_ADDR         = 0x01


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

    def write_reg(self, address, value, **kwargs):
        return self._out_request(self.REQUEST_WRITE_REG, index=address, value=value, **kwargs)

    def read_reg(self, address, length, **kwargs):
        return self._in_request(self.REQUEST_READ_REG, index=address, length=length, **kwargs)

    def max5865_set_opmode(self, opmode):
        self.write_reg(self.MAX5865_BASE_ADDR, opmode)

    def max2120_set_gain(self, gain):
        self.write_reg(self.CSR_BASE_ADDR | self.CSR_MAX2120_GAIN_ADDR, gain)

    def max2120_read(self, address):
        return self.read_reg(self.MAX2120_BASE_ADDR | address, 1)

    def max2120_write(self, address, value):
        self.write_reg(self.MAX2120_BASE_ADDR | address, value)

    def rffc5072_rx_read(self, address):
        return self.read_reg(self.RFFC5072_RX_BASE_ADDR | address, 2)

    def rffc5072_rx_write(self, address, value):
        self.write_reg(self.RFFC5072_RX_BASE_ADDR | address, value)

    def rffc5072_tx_read(self, address):
        return self.read_reg(self.RFFC5072_TX_BASE_ADDR | address, 2)

    def rffc5072_tx_write(self, address, value):
        self.write_reg(self.RFFC5072_TX_BASE_ADDR | address, value)

    def max2831_write(self, address, value):
        self.write_reg(self.MAX2831_BASE_ADDR | address, value)

    def set_led_pattern(self, pattern):
        self.write_reg(self.CSR_BASE_ADDR | self.CSR_LEDS_ADDR, pattern)

    def get_led_pattern(self):
        value = self.read_reg(self.CSR_BASE_ADDR | self.CSR_LEDS_ADDR, 1)
        return int.from_bytes(value)


class MAX2831_control:
    def __init__(self, urti):
        self.urti = urti
        self.set_registers_recommended()

    def write(self, address, value):
        self.urti.max2831_write(address, value)

    # from Table 15
    def set_registers_recommended(self):
        self.write(0, 0x1740) # enable fractional mode (Table 16 recommends 0x0740, clearing unknown bit)
        self.write(1, 0x119a)
        self.write(2, 0x1003)
        self.write(3, 0x0079) # PLL divider settings for 2437 MHz
        self.write(4, 0x3666) # PLL divider settings for 2437 MHz
        self.write(5, 0x00a4) # divide reference frequency by 2
        self.write(6, 0x0060) # enable TX power detector
        self.write(7, 0x1022) # 110% TX LPF bandwidth
        self.write(8, 0x2021) # pin control of RX gain, 11 MHz LPF bandwidth
        self.write(9, 0x03b5) # pin control of TX gain
        self.write(10, 0x1da4) # 3.5 us PA enable delay, moderate PA bias
        self.write(11, 0x0074) # LNA high gain, RX VGA moderate gain (Table 27 recommends 0x007f, maximum gain)
        self.write(12, 0x0140) # TX VGA minimum
        self.write(13, 0x0e92)
        self.write(14, 0x033b) # reference clock output divide by 1, enable
        self.write(15, 0x0145) # RX IQ common mode 1.1 V

    def set_freq(self, freq_hz):
        # reference divider: 1 or 2
        # 8-bit integer main divider: 64 to 255
        # 20-bit fractional main divider: 0 to 1048575
        reference_clock_divider = 2
        reference_clock_hz = 40e6

        fcomp = reference_clock_hz / reference_clock_divider
        divider = freq_hz / fcomp
        integer_portion = int(divider)
        if (integer_portion < 64 or integer_portion > 255):
            print(integer_portion)
            return
        fractional_portion = divider - integer_portion
        # Data sheet text says fractional portion is multiplied by (2**20 - 1), but
        # Table 14 and app note 6269 multiply by 2**20.
        fractional_setting = round(2**20 * fractional_portion)
        self.write(3, ((fractional_setting & 0x3f) << 8) | integer_portion)
        self.write(4, (fractional_setting >> 6))


class RFFC5072_control:
    def __init__(self, urti):
        self.urti = urti
        self.set_registers_recommended()

    def write(self, address, value):
        self.urti.rffc5072_tx_write(address, value)

    def set_registers_recommended(self):
        self.write(0x00, 0xbefa)
        self.write(0x01, 0x4064)
        self.write(0x02, 0x9055)
        self.write(0x03, 0x2d02)
        self.write(0x04, 0xacbf)
        self.write(0x05, 0xacbf)
        self.write(0x06, 0x0028)
        self.write(0x07, 0x0028)
        self.write(0x08, 0xff00)
        self.write(0x09, 0x8220)
        self.write(0x0a, 0x0202)
        self.write(0x0b, 0x4800)
        self.write(0x0c, 0x1a94)
        self.write(0x0d, 0xd89d)
        self.write(0x0e, 0x8900)
        self.write(0x0f, 0x1e84)
        self.write(0x10, 0x89d8)
        self.write(0x11, 0x9d00)
        self.write(0x12, 0x2a20)
        self.write(0x13, 0x0000)
        self.write(0x14, 0x0000)
        self.write(0x15, 0x0000)
        self.write(0x16, 0x0206)
        self.write(0x17, 0x4900)
        self.write(0x18, 0x0281)
        self.write(0x19, 0xf00f)
        self.write(0x1a, 0x0000)
        self.write(0x1b, 0x0000)
        self.write(0x1c, 0xc840)
        self.write(0x1d, 0x1000)
        self.write(0x1e, 0x0005)

    # disable RFFC5072 before calling set_freq()
    def set_freq(self, freq_hz):
        # calculate n_lo
        lo_max_hz = 5400e6
        n_lo = int(math.log((lo_max_hz / freq_hz), 2))

        lodiv = 1 << n_lo
        fvco_hz = lodiv * freq_hz
        assert(fvco_hz >= 2700e6)
        assert(fvco_hz <= 5400e6)

        # Higher divider and charge pump current are required above
        # 3.2 GHz. Programming guide says these values (fbkdiv, n,
        # maybe pump?) can be changed back after enable in order to
        # improve phase noise since the VCO will already be stable
        # and will be unaffected.
        if fvco_hz > 3200e6:
                fbkdiv = 4;
                pllcpl = 3;
        else:
                fbkdiv = 2;
                pllcpl = 2;
        self.write(0x00, 0xbef8 | pllcpl)

        reference_clock_hz = 40e6
        n_div = fvco_hz / (fbkdiv * reference_clock_hz)
        n = int(n_div)

        nummsb = int(2**16 * (n_div - n)) & 0xffff
        numlsb = round(2**8 * (2**16 * (n_div - n) - nummsb)) & 0xff

        n_div = ((n << 24) + (nummsb << 8) + numlsb) / 2**24

        tune_freq_hz = (reference_clock_hz * n_div * fbkdiv) / lodiv
        print(f"n: {n}")
        print(f"nummsb: {nummsb}")
        print(f"numlsb: {numlsb}")

        # configure mixer path 2
        presc = fbkdiv >> 1
        self.write(0x0f, (n << 7) | (n_lo << 4) | (presc << 2))
        self.write(0x10, nummsb)
        self.write(0x11, numlsb)

        return tune_freq_hz


if __name__ == "__main__":
    # Build and program the test gateware.
    top_level_cli(URTIBasicTestGateware)

    # Wait for the USB device to be enumerated.
    time.sleep(5)

    # Create a connection to URTI and print a list of methods.
    urti = URTIBasicTestGatewareConnection()
    print("'urti' methods:")
    urti._print_methods()

    max2831 = MAX2831_control(urti)
    max2831.set_freq(2500e6)
    rffc5072 = RFFC5072_control(urti)
    rffc5072.set_freq(1500e6)
