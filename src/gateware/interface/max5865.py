#
# This file is part of URTI.
#
# Copyright (c) 2024 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

import unittest
from enum                import IntEnum

from amaranth            import Elaboratable, Module, Signal, Instance, ClockSignal, ResetSignal
from amaranth.lib.wiring import Component, In, Out, Signature
from amaranth_soc        import wishbone
from amaranth_soc.memory import MemoryMap

from luna.gateware.test  import LunaGatewareTestCase, sync_test_case

__all__ = ["MAX5865DataInterface", "MAX5865OpMode", "MAX5865OpModeSetter"]


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


class MAX5865OpModeSetter(Component):
    bus: In(wishbone.Signature(addr_width=1, data_width=8))

    def __init__(self, *, pads, divisor):
        assert divisor & (divisor - 1) == 0, "divisor must be a power of 2"
        self.pads   = pads
        self.cycles = divisor

        super().__init__()
        self.bus.memory_map = MemoryMap(addr_width=1, data_width=8, name="max5865")


    def elaborate(self, platform):
        m = Module()

        bus    = self.pads
        cycles = self.cycles

        # TODO: These 3 signals could be reset-less?
        clock_period = Signal(range(cycles))
        opmode_sreg  = Signal(8)  # top 5 bits: don´t care
        rem_bits     = Signal(range(8))

        m.d.comb += [
            bus.pico .eq(opmode_sreg[-1]),
            bus.sck  .eq(clock_period[-1]),
        ]
        falling_edge = clock_period == cycles - 1

        m.d.comb += self.bus.dat_r.eq(0)
        m.d.sync += self.bus.ack  .eq(0)

        with m.FSM() as fsm:
            m.d.comb += bus.cs.eq(fsm.ongoing("XMIT"))

            with m.State("IDLE"):
                with m.If(self.bus.cyc & self.bus.stb & ~self.bus.ack):
                    with m.If(self.bus.we):
                        m.d.sync += [
                            opmode_sreg     .eq(self.bus.dat_w),
                            clock_period    .eq(cycles//2 - 1), # early rising edge
                            rem_bits        .eq(7),
                        ]
                        m.next = "XMIT"
                    with m.Else():
                        # Dummy read request
                        m.d.sync += self.bus.ack.eq(1)
                        m.next = "IDLE"

            with m.State("XMIT"):
                m.d.sync += clock_period.eq(clock_period + 1)
                with m.If(falling_edge):
                    m.d.sync += opmode_sreg.eq(opmode_sreg << 1)
                    with m.If(rem_bits == 0):
                        m.d.sync += self.bus.ack.eq(1)
                        m.next = "IDLE"
                    with m.Else():
                        m.d.sync += rem_bits.eq(rem_bits - 1)

        return m

#
# Tests
#

class TestMAX5865OpModeSetter(LunaGatewareTestCase):
    FRAGMENT_UNDER_TEST = MAX5865OpModeSetter
    FRAGMENT_ARGUMENTS  = dict(divisor=4, pads=Signature({
        "cs":   Out(1),
        "sck":  Out(1),
        "pico": Out(1),
    }).create())

    @sync_test_case
    def test_register_read(self):
        # Check initial assumptions before the transaction
        yield
        self.assertEqual((yield self.dut.pads.cs), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)

        yield from self.advance_cycles(10)
        self.assertEqual((yield self.dut.pads.cs), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)

        # Request a register read to address 0
        yield self.dut.bus.cyc.eq(1)
        yield self.dut.bus.stb.eq(1)
        yield self.dut.bus.we .eq(0)
        yield self.dut.bus.adr.eq(0)

        # Wait for the transaction to complete, clear the request lines
        # and ensure the ACK signal only lasts one cycle
        yield from self.wait_until(self.dut.bus.ack, timeout=32)
        yield self.dut.bus.cyc.eq(0)
        yield self.dut.bus.stb.eq(0)
        yield
        self.assertEqual((yield self.dut.bus.ack), 0)

        # Check conditions after transaction finish
        yield from self.advance_cycles(10)
        self.assertEqual((yield self.dut.pads.cs), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)


    @sync_test_case
    def test_register_write(self):
        # Check initial assumptions before the transaction
        yield
        self.assertEqual((yield self.dut.pads.cs), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)

        yield from self.advance_cycles(10)
        self.assertEqual((yield self.dut.pads.cs), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)

        # Request a register write to address 0
        yield self.dut.bus.cyc  .eq(1)
        yield self.dut.bus.stb  .eq(1)
        yield self.dut.bus.we   .eq(1)
        yield self.dut.bus.adr  .eq(0)
        yield self.dut.bus.dat_w.eq(3)

        # Bus should be enabled soon
        yield from self.wait_until(self.dut.pads.cs, timeout=2)

        # Wait for the transaction to complete, clear the request lines
        # and ensure the ACK signal only lasts one cycle
        yield from self.wait_until(self.dut.bus.ack, timeout=32)
        yield self.dut.bus.cyc.eq(0)
        yield self.dut.bus.stb.eq(0)
        yield
        self.assertEqual((yield self.dut.bus.ack), 0)

        # Check conditions after transaction finish
        yield from self.advance_cycles(10)
        self.assertEqual((yield self.dut.pads.cs), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)


if __name__ == "__main__":
    unittest.main()
