#
# This file is part of URTI.
#
# Copyright (c) 2024 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

import os

from amaranth.build import *
from amaranth_boards.resources import *

from .core import URTIPlatform
from .adv import control_phy_hook

__all__ = ["URTIPlatformRev0D1"]

class URTIPlatformRev0D1(URTIPlatform):
    """ Board description for URTI r0.1 """

    name        = "URTI r0.1"

    device      = "LFE5U-25F"
    package     = "BG256"
    speed       = os.getenv("ECP5_SPEED_GRADE", "8")

    default_clk = "clk_40MHz"
    default_usb_connection = "usb_phy"

    #
    # Default clock frequencies for each of our clock domains.
    #
    # Different revisions have different FPGA speed grades, and thus the
    # default frequencies will vary.
    #
    DEFAULT_CLOCK_FREQUENCIES_MHZ = {
        "fast": 240,
        "sync": 120,
        "usb":   60,
        "radio": 40,
    }

    # Provides any platform-specific ULPI registers necessary.
    # This is the spot to put any platform-specific vendor registers that need
    # to be written.
    ulpi_extra_registers = {
        0x39: 0b000110 # USB3343: swap D+ and D- to match the hardware design
    }

    #
    # I/O resources.
    #
    resources   = [

        # Pseudo-supply pins
        #
        # These I/O pins are connected to VCCIO or GND and are intended to be
        # driven as outputs in order to source or sink additional supply
        # current.
        
        # TODO: fix 3V3 and 1V8 conflicts
        # Resource("pseudo_vccio", 0,
        #     Pins("E6 E7 F12 G12 J12 K12 L12 N13 P13 M11 P11 P12 M5 N5 M6 N6 T6 R6 "     # 3V3
        #          "D10 E10 E11 D12 C13 D13 F5 G5 H5 H4 J4 J5 J3 K3 J1 J2 K2", dir="o")), # 1V8
        # Resource("pseudo_gnd", 0,
        #     Pins("E5 E8 E9 E12 E13 F13 M13 M12 N12 N11 L4 L5 L3 M3 P5 P6 F4 G2 G3 H3 H2 K1", dir="o")),

        # Primary, discrete 40MHz oscillator.
        Resource("clk_40MHz", 0, Pins("K16", dir="i"),
            Clock(40e6), Attrs(IO_TYPE="LVCMOS33")),

        # USB PHY
        ULPIResource("usb_phy", 0,
            data="N16 N14 P16 P15 R16 R15 T15 P14", clk="L14", clk_dir='o',
            dir="M16", nxt="M15", stp="L15", rst="M14", rst_invert=True,
            attrs=Attrs(IO_TYPE="LVCMOS33", SLEWRATE="FAST")),

        # Control signals for the different power supplies (Analog, RX, TX).
        Resource("pwr_3v3_a",  0, Pins("E4", dir="o"),  Attrs(IO_TYPE="LVCMOS33")),
        Resource("pwr_3v3_rx", 0, Pins("A2", dir="o"),  Attrs(IO_TYPE="LVCMOS33")),
        Resource("pwr_3v3_tx", 0, Pins("J16", dir="o"), Attrs(IO_TYPE="LVCMOS33")),

        # MAX5865 analog frontend (DAC/ADC).
        Resource("afe", 0,
            Subsignal("dd",     Pins("B8 A9 B9 A10 B10 A11 B11 A12 B12 A13", dir="o")),
            Subsignal("da",     Pins("C12 C11 C10 D11 C9 D9 C8 D8", dir="i")),
            # 3-wire interface
            Subsignal("pico",   Pins("B13", dir="o")),
            Subsignal("sck",    Pins("A14", dir="o")),
            Subsignal("cs",     PinsN("B14", dir="o")),
            Attrs(IO_TYPE="LVCMOS18")
        ),

        #
        # Receiver chain
        #

        # RX switch control
        Resource("rx_mix_bypass", 0, PinsN("C6", dir="o"), Attrs(IO_TYPE="LVCMOS33")),

        # RFFC5072 (RX)
        Resource("rx_mix_ctrl", 0,
            Subsignal("fm",     Pins("A6",  dir="io")),
            Subsignal("ld",     Pins("B6",  dir="i")),
            Subsignal("enbl",   Pins("A5",  dir="o")),
            Subsignal("mode",   Pins("B5",  dir="o")),
            Subsignal("reset",  PinsN("A7", dir="o")),
            Attrs(IO_TYPE="LVCMOS33")
        ),
        Resource("rx_mix_serial", 0,
            # 3-wire serial interface
            Subsignal("enx",    PinsN("B7", dir="o")),
            Subsignal("sclk",   Pins("A4",  dir="o")),
            Subsignal("sdata",  Pins("B4",  dir="io")),
            Attrs(IO_TYPE="LVCMOS33")
        ),

        # MAX2120
        Resource("max2120", 0,
            Subsignal("gain",   Pins("D6 C5 D5 C4 D4 A8",  dir="o")),
            Subsignal("scl",    Pins("D7",  dir="o")),
            Subsignal("sda",    Pins("C7",  dir="io")),
            Attrs(IO_TYPE="LVCMOS33")
        ),


        #
        # Transmitter chain
        #

        # TX switch control
        Resource("tx_mix_bypass", 0, PinsN("J14", dir="o"), Attrs(IO_TYPE="LVCMOS33")),

        # RFFC5072 (TX)
        Resource("tx_mix_ctrl", 0,
            Subsignal("fm",     Pins("K14",  dir="io")),
            Subsignal("ld",     Pins("K15",  dir="i")),
            Subsignal("enbl",   Pins("H15",  dir="o")),
            Subsignal("mode",   Pins("J15",  dir="o")),
            Subsignal("reset",  PinsN("C15", dir="o")),
            Attrs(IO_TYPE="LVCMOS33")
        ),
        Resource("tx_mix_serial", 0,
            # 3-wire serial interface
            Subsignal("enx",    PinsN("F16", dir="o")),
            Subsignal("sclk",   Pins("G15",  dir="o")),
            Subsignal("sdata",  Pins("G16",  dir="io")),
            Attrs(IO_TYPE="LVCMOS33")
        ),

        # MAX2831
        Resource("max2831", 0,
            Subsignal("gain",   Pins("D14 H13 C16 E14 H14 B16 C14", dir="o")),
            Subsignal("shdn",   PinsN("J13", dir="o")),
            Subsignal("rxhp",   Pins("G14", dir="o")),
            Subsignal("rxtx",   Pins("B15", dir="o")),
            Subsignal("ld",     Pins("F14", dir="i")),
            # RSSI ?
            # 3-wire serial interface
            Subsignal("din",    Pins("G13", dir="o")),
            Subsignal("sclk",   Pins("E16", dir="o")),
            Subsignal("cs",     PinsN("H12", dir="o")),
            Attrs(IO_TYPE="LVCMOS33")
        ),


        # Connection to our SPI flash; can be used to work with the flash
        # from e.g. a bootloader.
        Resource("spi_flash", 0,

            # SCK is on pin N9; but doesn't have a traditional I/O buffer.
            # Instead, we'll need to drive a clock into a USRMCLK instance.
            # See interfaces/flash.py for more information.
            Subsignal("sdi",  Pins("T8",  dir="o")),
            Subsignal("sdo",  Pins("T7",  dir="i")),
            Subsignal("cs",   PinsN("N8", dir="o")),
            Attrs(IO_TYPE="LVCMOS33")
        ),

        # Note: UART pins R13 and R12 are connected to JTAG pins R11 (TDI)
        # and T11 (TMS) respectively, so the microcontroller can use either
        # function but not both simultaneously.

        # UART connected to the debug controller; can be routed to a host via CDC-ACM.
        Resource("uart", 0,
            Subsignal("rx",  Pins("R13",  dir="i")),
            Subsignal("tx",  Pins("R12",  dir="oe"), Attrs(PULLMODE="UP")),
            Attrs(IO_TYPE="LVCMOS33")
        ),

        # Advertisement output to microcontroller.
        Resource("int", 0, Pins("R14", dir="o"), Attrs(IO_TYPE="LVCMOS33")),

        # USER button
        Resource("button_user", 0, PinsN("T14", dir="i"), Attrs(IO_TYPE="LVCMOS33", PULLMODE="NONE")),

        # Output signal connected to PROGRAMN to trigger FPGA reconfiguration.
        Resource("self_program", 0, PinsN("T13", dir="o"), Attrs(IO_TYPE="LVCMOS33", PULLMODE="UP")),

        # FPGA LEDs
        *LEDResources(pins="L16 L13 K13", attrs=Attrs(IO_TYPE="LVCMOS33"), invert=True),

        # HyperRAM
        Resource("ram", 0,
            Subsignal("clk",   DiffPairs("C3", "D3", dir="o"), Attrs(IO_TYPE="LVCMOS33D")),
            Subsignal("dq",    Pins("F2 B1 C2 E1 E3 E2 F3 G4", dir="io")),
            Subsignal("rwds",  Pins( "D1", dir="io")),
            Subsignal("cs",    PinsN("B2", dir="o")),
            Subsignal("reset", PinsN("C1", dir="o")),
            Attrs(IO_TYPE="LVCMOS18", SLEWRATE="FAST")
        ),

        # User I/O connections.
        Resource("user_pmod", 0, Pins("1 2 3 4 7 8 9 10", conn=("pmod", 0), dir="io"), Attrs(IO_TYPE="LVCMOS33")),
        Resource("user_mezzanine", 0,
                Pins("3 4 5 6 7 8 9 10 11 12 13 18 19 20 21 22 23 24 25 26 27 28", conn=("mezzanine", 0), dir="io"),
                Attrs(IO_TYPE="LVCMOS33", SLEWRATE="FAST")),
    ]

    connectors = [
        # PMOD is connected to a subset of the mezzanine lines.
        Connector("pmod", 0, "M4 N3 N4 P4 - - L2 L1 K5 L3 - -"),
        Connector("mezzanine", 0,
            "- - R4 T4 R5 M4 N3 N4 P4 L3 K5 L1 L2 - - - - M1 M2 N1 P2 P1 P3 R1 R2 T2 R3 T3 - -"),
    ]

    usb_device_hooks = {
        "usb_phy_0": control_phy_hook
    }
