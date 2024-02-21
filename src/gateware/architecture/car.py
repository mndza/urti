#
# This file is part of URTI.
#
# Copyright (c) 2024 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

""" Clock and reset (CAR) controllers for URTI. """

from amaranth import Signal, ClockDomain, ClockSignal, Instance, ResetSignal

from luna.gateware.architecture.car import LunaDomainGenerator

class URTIDomainGenerator(LunaDomainGenerator):
    """ """

    def __init__(self):
        super().__init__(clock_signal_frequency=40e6)

    def create_submodules(self, m, platform):

        # Grab our default input clock.
        input_clock = platform.request(platform.default_clk, dir="i").i

        # Internal signals for each of our clocks.
        self._clk_240MHz = Signal()
        self._clk_120MHz = Signal()
        self._clk_60MHz  = Signal()
        self._clk_40MHz  = Signal()
        self._clock_options = {
            40:  self._clk_40MHz,
            60:  self._clk_60MHz,
            120: self._clk_120MHz,
            240: self._clk_240MHz
        }

        # Create the radio clock domain. Others are created in the parent class.
        m.domains.radio  = ClockDomain()
        m.d.comb += ClockSignal(domain="radio").eq(self._clk_40MHz)

        locked   = Signal()

        # Instantiate the ECP5 PLL.
        # Generated with Clarity Designer.
        m.submodules.pll = Instance("EHXPLLL",

            # Clock in.
            i_CLKI=input_clock,

            # Generated clock outputs.
            o_CLKOP=self._clk_240MHz,
            o_CLKOS=self._clk_120MHz,
            o_CLKOS2=self._clk_60MHz,
            o_CLKOS3=self._clk_40MHz,

            # Status.
            o_LOCK=locked,

            # PLL parameters...
            p_PLLRST_ENA="DISABLED",
            p_INTFB_WAKE="DISABLED",
            p_STDBY_ENABLE="DISABLED",
            p_DPHASE_SOURCE="DISABLED",
            p_OUTDIVIDER_MUXA="DIVA",
            p_OUTDIVIDER_MUXB="DIVB",
            p_OUTDIVIDER_MUXC="DIVC",
            p_OUTDIVIDER_MUXD="DIVD",
            p_CLKOP_ENABLE="ENABLED",
            p_CLKOS_ENABLE="ENABLED",
            p_CLKOS2_ENABLE="ENABLED",
            p_CLKOS3_ENABLE="ENABLED",
            p_CLKOS3_FPHASE=0,
            p_CLKOS3_CPHASE=11,
            p_CLKOS2_FPHASE=0,
            p_CLKOS2_CPHASE=7,
            p_CLKOS_FPHASE=0,
            p_CLKOS_CPHASE=3,
            p_CLKOP_FPHASE=0,
            p_CLKOP_CPHASE=1,
            p_PLL_LOCK_MODE=0,
            p_CLKOS_TRIM_DELAY="0",
            p_CLKOS_TRIM_POL="FALLING",
            p_CLKOP_TRIM_DELAY="0",
            p_CLKOP_TRIM_POL="FALLING",
            p_CLKOS3_DIV=12,
            p_CLKOS2_DIV=8,
            p_CLKOS_DIV=4,
            p_CLKOP_DIV=2,
            p_CLKFB_DIV=6,
            p_CLKI_DIV=1,
            p_FEEDBK_PATH="CLKOP",

            # Internal feedback.
            i_CLKFB=self._clk_240MHz,

            # Control signals.
            i_RST=0,
            i_PHASESEL0=0,
            i_PHASESEL1=0,
            i_PHASEDIR=0,
            i_PHASESTEP=0,
            i_PHASELOADREG=0,
            i_STDBY=0,
            i_PLLWAKESYNC=0,

            # Output Enables.
            i_ENCLKOP=0,
            i_ENCLKOS=0,
            i_ENCLKOS2=0,
            i_ENCLKOS3=0,

            # Synthesis attributes.
            a_FREQUENCY_PIN_CLKI="40.000000",
            a_FREQUENCY_PIN_CLKOS3="40.000000",
            a_FREQUENCY_PIN_CLKOS2="60.000000",
            a_FREQUENCY_PIN_CLKOS="120.000000",
            a_FREQUENCY_PIN_CLKOP="240.000000",
            a_ICP_CURRENT="9",
            a_LPF_RESISTOR="8"
        )

        # Set up our global resets so the system is kept fully in reset until
        # our core PLL is fully stable. This prevents us from internally clock
        # glitching ourselves before our PLL is locked. :)
        m.d.comb += [
            ResetSignal("radio")   .eq(~locked),
            ResetSignal("sync")    .eq(~locked),
            ResetSignal("fast")    .eq(~locked),
        ]

    def generate_usb_clock(self, m, platform):
        return self._clock_options[60]

    def generate_sync_clock(self, m, platform):
        return self._clock_options[120]

    def generate_fast_clock(self, m, platform):
        return self._clock_options[240]
