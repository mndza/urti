#
# This file is part of URTI.
#
# Copyright (c) 2024 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

import unittest

from amaranth            import Signal, Module, Cat, C
from amaranth.lib.wiring import Component, In, Out, Signature
from amaranth_soc        import wishbone
from amaranth_soc.memory import MemoryMap

from luna.gateware.test  import LunaGatewareTestCase, sync_test_case

__all__ = ["RFFC5072RegisterInterface"]


class RFFC5072RegisterInterface(Component):
    """
    Simple register interface for RFFC5072 three-wire bus.
    """
    bus: In(wishbone.Signature(addr_width=5, data_width=16))
    
    def __init__(self, *, pads, divisor, name=None):
        """
        Parameters:
            pads         -- contains the three-wire bus signals
            divisor      -- the divisor of the clock divider providing the 3-wire clock
        """
        assert divisor & (divisor - 1) == 0, "divisor must be a power of 2"
        self.divisor = divisor
        self.pads    = pads
        super().__init__()
        self.bus.memory_map = MemoryMap(addr_width=5, data_width=16, name=name or "rffc5072")


    def elaborate(self, platform):
        m = Module()

        cycles     = self.divisor
        pads       = self.pads
        shift_reg  = Signal(8+16)
        bits_write = Signal(range(16+8+1))
        bits_read  = Signal(range(16+1+1))  # +1 cycle to account for read delay
        idle_cycles = Signal(2)

        clock_period = Signal(range(cycles))
        m.d.comb += pads.sclk.o.eq(clock_period[-1])

        falling_edge = clock_period == cycles - 1

        m.d.sync += self.bus.ack.eq(0)

        with m.FSM() as fsm:

            with m.State('IDLE'):
                m.d.sync += [
                    clock_period.eq(0),
                    pads.sdata.o.eq(0),
                ]
                with m.If(self.bus.cyc & self.bus.stb & ~self.bus.ack):
                    with m.If(self.bus.we):
                        m.d.sync += [
                            shift_reg   .eq(Cat(self.bus.dat_w, self.bus.adr, 0, 0, 0)),
                            bits_write  .eq(24),
                            bits_read   .eq(0),
                        ]
                    with m.Else():
                        m.d.sync += [
                            shift_reg   .eq(Cat(C(0,16), self.bus.adr, 0, 0, 1)),
                            bits_write  .eq(8),
                            bits_read   .eq(16+1),
                        ]
                    m.d.sync += idle_cycles.eq(1)
                    m.next = "PREAMBLE"

            with m.State("PREAMBLE"):
                m.d.sync += clock_period.eq(clock_period + 1)
                with m.If(falling_edge):
                    with m.If(idle_cycles == 0):
                        m.next = "WRITE_PHASE"
                    with m.Else():
                        m.d.sync += idle_cycles.eq(idle_cycles - 1)

            with m.State("WRITE_PHASE"):
                m.d.comb += pads.enx.o.eq(1)
                m.d.comb += pads.sdata.oe.eq(1)
                m.d.sync += clock_period.eq(clock_period + 1)
                with m.If(falling_edge):
                    with m.If(bits_write == 0):
                        with m.If(bits_read == 0):
                            m.d.sync += self.bus.ack.eq(1)
                            m.next = "IDLE"
                        with m.Else():
                            m.next = "READ_PHASE"
                    with m.Else():
                        m.d.sync += [
                            pads.sdata.o.eq(shift_reg[-1]),
                            shift_reg   .eq(shift_reg << 1),
                            bits_write  .eq(bits_write - 1),
                        ]
                        
            with m.State("READ_PHASE"):
                m.d.comb += pads.enx.o.eq(1)
                m.d.sync += clock_period.eq(clock_period + 1)
                with m.If(falling_edge):
                    with m.If(bits_read == 0):
                        m.d.sync += self.bus.dat_r.eq(shift_reg[:16])
                        m.d.sync += self.bus.ack.eq(1)
                        m.d.sync += idle_cycles.eq(0)
                        m.next = "POSTAMBLE"
                    with m.Else():
                        m.d.sync += [
                            shift_reg   .eq(Cat(pads.sdata.i, shift_reg)),
                            bits_read   .eq(bits_read - 1),
                        ]

            with m.State("POSTAMBLE"):
                m.d.sync += clock_period.eq(clock_period + 1)
                with m.If(falling_edge):
                    with m.If(idle_cycles == 0):
                        m.next = "IDLE"
                    with m.Else():
                        m.d.sync += idle_cycles.eq(idle_cycles - 1)


        return m

#
# Tests
#

pin_o  = Signature({"o": Out(1)})
pin_io = Signature({"i": In(1), "o": Out(1), "oe": Out(1)})

class TestRFFC5072RegisterInterface(LunaGatewareTestCase):
    FRAGMENT_UNDER_TEST = RFFC5072RegisterInterface
    FRAGMENT_ARGUMENTS  = dict(divisor=4, pads=Signature({
        "enx":   Out(pin_o),
        "sclk":  Out(pin_o),
        "sdata": Out(pin_io),
    }).create())

    @sync_test_case
    def test_register_read(self):
        # Check initial assumptions before the transaction
        yield
        self.assertEqual((yield self.dut.pads.enx.o), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)

        yield from self.advance_cycles(10)
        self.assertEqual((yield self.dut.pads.enx.o), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)

        # Request a register read to address 0x19
        yield self.dut.bus.cyc.eq(1)
        yield self.dut.bus.stb.eq(1)
        yield self.dut.bus.we .eq(0)
        yield self.dut.bus.adr.eq(0x19)

        # Bus should be enabled soon
        yield from self.wait_until(self.dut.pads.enx.o, timeout=10*self.dut.divisor)

        # Wait for the transaction to complete, clear the request lines
        # and ensure the ACK signal only lasts one cycle
        yield from self.wait_until(self.dut.bus.ack, timeout=32*self.dut.divisor)
        yield self.dut.bus.cyc.eq(0)
        yield self.dut.bus.stb.eq(0)
        yield
        self.assertEqual((yield self.dut.bus.ack), 0)

        # Check conditions after transaction finish
        yield from self.advance_cycles(10)
        self.assertEqual((yield self.dut.pads.enx.o), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)


    @sync_test_case
    def test_register_write(self):
        # Check initial assumptions before the transaction
        yield
        self.assertEqual((yield self.dut.pads.enx.o), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)

        yield from self.advance_cycles(10)
        self.assertEqual((yield self.dut.pads.enx.o), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)

        # Request a register write to address 0x10
        yield self.dut.bus.cyc  .eq(1)
        yield self.dut.bus.stb  .eq(1)
        yield self.dut.bus.we   .eq(1)
        yield self.dut.bus.adr  .eq(0x10)
        yield self.dut.bus.dat_w.eq(0xF50F)

        # Bus should be enabled soon
        yield from self.wait_until(self.dut.pads.enx.o, timeout=10*self.dut.divisor)

        # Wait for the transaction to complete, clear the request lines
        # and ensure the ACK signal only lasts one cycle
        yield from self.wait_until(self.dut.bus.ack, timeout=32*self.dut.divisor)
        yield self.dut.bus.cyc.eq(0)
        yield self.dut.bus.stb.eq(0)
        yield
        self.assertEqual((yield self.dut.bus.ack), 0)

        # Check conditions after transaction finish
        yield from self.advance_cycles(10)
        self.assertEqual((yield self.dut.pads.enx.o), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)


if __name__ == "__main__":
    unittest.main()