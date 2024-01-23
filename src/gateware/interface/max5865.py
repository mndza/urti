#
# This file is part of URTI.
#
# Copyright (c) 2024 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

from enum import IntEnum

from amaranth import Elaboratable, Module, Signal, Instance, ClockSignal, ResetSignal

class MAX5865DataInterface(Elaboratable):
    
    def __init__(self, *, pads):
        self.pads       = pads

        self.adc_data_i = Signal(8)
        self.adc_data_q = Signal(8)
        self.dac_data_i = Signal(10)
        self.dac_data_q = Signal(10)

    def elaborate(self, platform):
        m = Module()

        # ADC input data lines
        # FPGA-TN-02035, GDDRX1_RX.SCLK.Centered Interface (Static Delay)
        for i in range(8):
            da_delay = Signal()
            m.submodules += [
                Instance("DELAYG",
                    p_DEL_MODE="SCLK_CENTERED",
                    i_A=self.pads.da[i],
                    o_Z=da_delay,
                ),
                Instance("IDDRX1F",
                    i_D=da_delay,
                    i_SCLK=ClockSignal(),
                    i_RST=ResetSignal(),
                    o_Q0=self.adc_data_i[i],
                    o_Q1=self.adc_data_q[i],
                ),
            ]

        # DAC output data lines
        # FPGA-TN-02035, GDDRX1_TX.SCLK.Centered Interface
        for i in range(10):
            m.submodules += [
                Instance("ODDRX1F",
                    i_D0=self.dac_data_i[i],
                    i_D1=self.dac_data_q[i],
                    i_SCLK=ClockSignal(),
                    i_RST=ResetSignal(),
                    o_Q=self.pads.dd[i],
                ),
            ]

        return m


class MAX5865OpMode(IntEnum):
    SHUTDOWN = 0
    IDLE     = 1
    RX       = 2
    TX       = 3
    XCVR     = 4
    STANDBY  = 5


class MAX5865OpModeSetter(Elaboratable):
    def __init__(self, *, pads, divisor):
        assert divisor & (divisor - 1) == 0, "divisor must be a power of 2"
        self.pads   = pads
        self.cycles = divisor
        self.opmode = Signal(MAX5865OpMode)
        self.set    = Signal()

    def elaborate(self, platform):
        m = Module()

        bus    = self.pads
        cycles = self.cycles

        # these 3 signals could be reset-less
        clock_period = Signal(range(cycles))
        opmode_sreg  = Signal(8)  # top 5 bits: don´t care
        rem_bits     = Signal(range(8))

        m.d.comb += [
            bus.pico .eq(opmode_sreg[-1]),
            bus.sck  .eq(clock_period[-1]),
        ]
        falling_edge = clock_period == cycles - 1

        with m.FSM() as fsm:
            m.d.comb += bus.cs.eq(fsm.ongoing("XMIT"))

            with m.State("IDLE"):
                with m.If(self.set):
                    m.d.sync += [
                        opmode_sreg     .eq(self.opmode),
                        clock_period    .eq(cycles//2 - 1), # early rising edge
                        rem_bits        .eq(7),
                    ]
                    m.next = "XMIT"

            with m.State("XMIT"):
                m.d.sync += clock_period.eq(clock_period + 1)
                with m.If(falling_edge):
                    m.d.sync += opmode_sreg.eq(opmode_sreg << 1)
                    with m.If(rem_bits == 0):
                        m.next = "IDLE"
                    with m.Else():
                        m.d.sync += rem_bits.eq(rem_bits - 1)

        return m

