#
# This file is part of URTI.
#
# Copyright (c) 2024 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

from amaranth            import Signal, Module, Cat, C, Elaboratable
from amaranth.lib.cdc    import FFSynchronizer
from amaranth.lib.wiring import Component, In
from amaranth_soc        import wishbone
from amaranth_soc.memory import MemoryMap

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
    def __init__(self, *, pads, divisor):
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
        self.bus.memory_map = MemoryMap(addr_width=5, data_width=16, name="rffc5072")


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
            m.d.comb += pads.enx  .eq(~fsm.ongoing("IDLE"))

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
                            m.next = "IDLE"
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
                        m.next = "IDLE"
                    with m.Else():
                        m.d.sync += [
                            shift_reg   .eq(Cat(pads.sdata_i, shift_reg)),
                            bits_read   .eq(bits_read - 1),
                        ]

        return m
