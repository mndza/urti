#
# This file is part of URTI.
#
# Copyright (c) 2024 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

import unittest

from amaranth            import Signal, Module, Cat, C, Elaboratable
from amaranth.lib.cdc    import FFSynchronizer
from amaranth.lib.wiring import Component, In
from amaranth_soc        import wishbone
from amaranth_soc.memory import MemoryMap
from amaranth.hdl.rec    import Record, DIR_FANIN, DIR_FANOUT

from luna.gateware.test  import LunaGatewareTestCase, sync_test_case

class ThreeWireControllerBus(Elaboratable):
    """
    3-wire controller bus
    
    Provides synchronization.
    """
    def __init__(self, pads):
        self.pads     = pads

        self.enx      = Signal()
        self.sclk     = Signal()
        self.sdata_oe = Signal()
        self.sdata_o  = Signal()
        self.sdata_i  = Signal()

    def elaborate(self, platform):
        m = Module()

        m.d.comb += [
            self.pads.enx.o      .eq(self.enx),
            self.pads.sclk.o     .eq(self.sclk),
            self.pads.sdata.oe   .eq(self.sdata_oe),
            self.pads.sdata.o    .eq(self.sdata_o),
        ]

        m.submodules += [
            FFSynchronizer(self.pads.sdata.i, self.sdata_i, reset=1),
        ]

        return m


class RFFC5072RegisterInterface(Component):
    """
    Simple register interface for RFFC5072 three-wire bus.
    """
    def __init__(self, *, pads, divisor, name=None):
        """
        Parameters:
            pads         -- contains the three-wire bus signals
            divisor      -- the divisor of the clock divider providing the 3-wire clock
        """
        assert divisor & (divisor - 1) == 0, "divisor must be a power of 2"
        self.divisor = divisor
        self.pads    = ThreeWireControllerBus(pads)

        super().__init__({
            "bus": In(wishbone.Signature(addr_width=5, data_width=16))  # address is actually 7 bits
        })
        self.bus.memory_map = MemoryMap(addr_width=5, data_width=16, name=name or "rffc5072")


    def elaborate(self, platform):
        m = Module()

        m.submodules.pads = pads = self.pads

        cycles     = self.divisor
        shift_reg  = Signal(8+16)
        bits_write = Signal(range(16+8+1))
        bits_read  = Signal(range(16+1+1))  # +1 cycle to account for read delay

        clock_period = Signal(range(cycles))
        m.d.comb += pads.sclk.eq(clock_period[-1])

        falling_edge = clock_period == cycles - 1

        m.d.sync += self.bus.ack.eq(0)

        with m.FSM() as fsm:
            m.d.comb += pads.enx.eq(fsm.ongoing("WRITE_PHASE") | fsm.ongoing("READ_PHASE"))

            with m.State('IDLE'):
                with m.If(self.bus.cyc & self.bus.stb):
                    m.d.sync += clock_period.eq(0)
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
                    m.next = "WRITE_PHASE"

            with m.State("WRITE_PHASE"):
                m.d.comb += pads.sdata_oe.eq(1)
                m.d.sync += clock_period.eq(clock_period + 1)
                with m.If(falling_edge):
                    with m.If(bits_write == 0):
                        with m.If(bits_read == 0):
                            m.d.sync += self.bus.ack.eq(1)
                            m.next = "FINISH"
                        with m.Else():
                            m.next = "READ_PHASE"
                    with m.Else():
                        m.d.sync += [
                            pads.sdata_o .eq(shift_reg[-1]),
                            shift_reg   .eq(shift_reg << 1),
                            bits_write  .eq(bits_write - 1),
                        ]
                        
            with m.State("READ_PHASE"):
                m.d.sync += clock_period.eq(clock_period + 1)
                with m.If(falling_edge):
                    with m.If(bits_read == 0):
                        m.d.sync += self.bus.dat_r.eq(shift_reg[:16])
                        m.d.sync += self.bus.ack.eq(1)
                        m.next = "FINISH"
                    with m.Else():
                        m.d.sync += [
                            shift_reg   .eq(Cat(pads.sdata_i, shift_reg)),
                            bits_read   .eq(bits_read - 1),
                        ]

            with m.State("FINISH"):
                # Wait a cycle to let the master deassert the CYC/STB lines 
                m.next = "IDLE"

        return m

#
# Tests
#

class ThreeWirePads(Record):
    """ Record representing a 3-wire bus. Used for tests. """
    def __init__(self):
        super().__init__([
            ('enx', [('o', 1, DIR_FANOUT)]),
            ('sclk', [('o', 1, DIR_FANOUT)]),
            ('sdata', [('i', 1, DIR_FANIN), ('o', 1, DIR_FANOUT), ('oe', 1, DIR_FANOUT)]),
        ])

class TestRFFC5072RegisterInterface(LunaGatewareTestCase):
    FRAGMENT_UNDER_TEST = RFFC5072RegisterInterface
    FRAGMENT_ARGUMENTS  = dict(divisor=4, pads=ThreeWirePads())

    @sync_test_case
    def test_register_read(self):
        # Check initial assumptions before the transaction
        yield
        self.assertEqual((yield self.dut.pads.enx), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)

        yield from self.advance_cycles(10)
        self.assertEqual((yield self.dut.pads.enx), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)

        # Request a register read to address 0x19
        yield self.dut.bus.cyc.eq(1)
        yield self.dut.bus.stb.eq(1)
        yield self.dut.bus.we .eq(0)
        yield self.dut.bus.adr.eq(0x19)

        # Bus should be enabled soon
        yield from self.wait_until(self.dut.pads.enx, timeout=2*self.dut.divisor)

        # Wait for the transaction to complete, clear the request lines
        # and ensure the ACK signal only lasts one cycle
        yield from self.wait_until(self.dut.bus.ack, timeout=32*self.dut.divisor)
        yield self.dut.bus.cyc.eq(0)
        yield self.dut.bus.stb.eq(0)
        yield
        self.assertEqual((yield self.dut.bus.ack), 0)

        # Check conditions after transaction finish
        yield from self.advance_cycles(10)
        self.assertEqual((yield self.dut.pads.enx), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)


    @sync_test_case
    def test_register_write(self):
        # Check initial assumptions before the transaction
        yield
        self.assertEqual((yield self.dut.pads.enx), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)

        yield from self.advance_cycles(10)
        self.assertEqual((yield self.dut.pads.enx), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)

        # Request a register write to address 0x10
        yield self.dut.bus.cyc  .eq(1)
        yield self.dut.bus.stb  .eq(1)
        yield self.dut.bus.we   .eq(1)
        yield self.dut.bus.adr  .eq(0x10)
        yield self.dut.bus.dat_w.eq(0xF50F)

        # Bus should be enabled soon
        yield from self.wait_until(self.dut.pads.enx, timeout=2*self.dut.divisor)

        # Wait for the transaction to complete, clear the request lines
        # and ensure the ACK signal only lasts one cycle
        yield from self.wait_until(self.dut.bus.ack, timeout=32*self.dut.divisor)
        yield self.dut.bus.cyc.eq(0)
        yield self.dut.bus.stb.eq(0)
        yield
        self.assertEqual((yield self.dut.bus.ack), 0)

        # Check conditions after transaction finish
        yield from self.advance_cycles(10)
        self.assertEqual((yield self.dut.pads.enx), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)


if __name__ == "__main__":
    unittest.main()