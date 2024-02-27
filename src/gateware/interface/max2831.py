#
# This file is part of URTI.
#
# Copyright (c) 2024 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

import unittest

from amaranth            import Module, Signal, Cat
from amaranth.lib.wiring import Component, In, Out, Signature
from amaranth_soc        import wishbone
from amaranth_soc.memory import MemoryMap

from luna.gateware.test  import LunaGatewareTestCase, sync_test_case

__all__ = ["MAX2831"]


class MAX2831(Component):
    bus: In(wishbone.Signature(addr_width=4, data_width=16))

    def __init__(self, *, pads, divisor):
        assert divisor & (divisor - 1) == 0, "divisor must be a power of 2"
        self.pads = pads
        self.cycles = divisor
        super().__init__()
        self.bus.memory_map = MemoryMap(addr_width=4, data_width=16, name="max2831")

    def elaborate(self, platform):
        m = Module()

        pads   = self.pads
        cycles = self.cycles

        # TODO: These 3 signals could be reset-less?
        clock_period = Signal(range(cycles))
        data_sreg    = Signal(18)
        rem_bits     = Signal(range(18))

        m.d.comb += [
            pads.din  .eq(data_sreg[-1]),
            pads.sclk .eq(clock_period[-1]),
        ]
        falling_edge = clock_period == cycles - 1

        m.d.comb += self.bus.dat_r.eq(0)
        m.d.sync += self.bus.ack  .eq(0)

        with m.FSM() as fsm:
            m.d.comb += pads.cs.eq(fsm.ongoing("XMIT"))

            with m.State("IDLE"):
                with m.If(self.bus.cyc & self.bus.stb & ~self.bus.ack):
                    with m.If(self.bus.we):
                        m.d.sync += [
                            data_sreg       .eq(Cat(self.bus.adr, self.bus.dat_w[:14])),
                            clock_period    .eq(cycles//2 - 1), # early rising edge
                            rem_bits        .eq(17),
                        ]
                        m.next = "XMIT"
                    with m.Else():
                        # Dummy read request
                        m.d.sync += self.bus.ack.eq(1)

            with m.State("XMIT"):
                m.d.sync += clock_period.eq(clock_period + 1)
                with m.If(falling_edge):
                    m.d.sync += data_sreg.eq(data_sreg << 1)
                    with m.If(rem_bits == 0):
                        m.d.sync += self.bus.ack.eq(1)
                        m.next = "IDLE"
                    with m.Else():
                        m.d.sync += rem_bits.eq(rem_bits - 1)
            
        return m


#
# Tests
#

class TestMAX2831(LunaGatewareTestCase):

    def instantiate_dut(self):
        return MAX2831(divisor=4, pads=Signature({
            "cs":   Out(1),
            "sclk": Out(1),
            "din":  Out(1),
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

        # Request a register read to address 0x7
        yield self.dut.bus.cyc.eq(1)
        yield self.dut.bus.stb.eq(1)
        yield self.dut.bus.we .eq(0)
        yield self.dut.bus.adr.eq(0x7)

        # Wait for the transaction to complete, clear the request lines
        # and ensure the ACK signal only lasts one cycle
        yield from self.wait_until(self.dut.bus.ack, timeout=5)
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

        # Request a register write to address 0x7
        yield self.dut.bus.cyc  .eq(1)
        yield self.dut.bus.stb  .eq(1)
        yield self.dut.bus.we   .eq(1)
        yield self.dut.bus.adr  .eq(0x7)
        yield self.dut.bus.dat_w.eq(3)

        # Bus should be enabled soon
        yield from self.wait_until(self.dut.pads.cs, timeout=2)

        # Wait for the transaction to complete, clear the request lines
        # and ensure the ACK signal only lasts one cycle
        yield from self.wait_until(self.dut.bus.ack, timeout=20*self.dut.cycles)
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
