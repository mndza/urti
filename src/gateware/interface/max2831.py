#
# This file is part of URTI.
#
# Copyright (c) 2024 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

from amaranth            import Module, Signal, Cat
from amaranth.lib.wiring import Component, In
from amaranth_soc        import wishbone
from amaranth_soc.memory import MemoryMap


class MAX2831(Component):
    def __init__(self, *, pads, divisor):
        assert divisor & (divisor - 1) == 0, "divisor must be a power of 2"
        self.pads   = pads
        self.cycles = divisor

        super().__init__({
            "bus": In(wishbone.Signature(addr_width=4, data_width=16))
        })
        self.bus.memory_map = MemoryMap(addr_width=4, data_width=16, name="max2831")


    def elaborate(self, platform):
        m = Module()

        pads   = self.pads
        cycles = self.cycles

        # TODO: These 3 signals could be reset-less?
        clock_period = Signal(range(cycles))
        data_sreg    = Signal(18)
        rem_bits     = Signal(range(8))

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
                with m.If(self.bus.cyc & self.bus.stb):
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
                        m.next = "FINISH"
                    with m.Else():
                        m.d.sync += rem_bits.eq(rem_bits - 1)
            
            with m.State("FINISH"):
                # Wait a cycle to let the master deassert the CYC/STB lines 
                m.next = "IDLE"

        return m

