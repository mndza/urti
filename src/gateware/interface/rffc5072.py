#
# This file is part of URTI.
#
# Copyright (c) 2024 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

from amaranth import Signal, Module, Cat, C, Elaboratable

from amaranth.lib.cdc import FFSynchronizer


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


class RFFC5072RegisterInterface(Elaboratable):
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
        self.divisor           = divisor
        self.bus               = ThreeWireControllerBus(pads)
        #
        # I/O port.
        #
        self.start             = Signal()
        self.busy              = Signal()
        self.address           = Signal(7)
        self.is_write          = Signal()
        self.data_in           = Signal(16)
        self.data_out          = Signal(16)

    def elaborate(self, platform):
        m = Module()

        m.submodules.bus = bus = self.bus

        cycles     = self.divisor
        shift_reg  = Signal(8+16)
        bits_write = Signal(range(16+8+1))
        bits_read  = Signal(range(16+1+1))  # +1 cycle to account for read delay

        clock_period = Signal(range(cycles))
        m.d.comb += bus.sclk.eq(clock_period[-1])

        falling_edge = clock_period == cycles - 1

        
        with m.FSM() as fsm:
            m.d.comb += bus.enx  .eq(~fsm.ongoing("IDLE"))
            m.d.comb += self.busy.eq(~fsm.ongoing("IDLE"))

            with m.State('IDLE'):
                with m.If(self.start):
                    m.d.sync += clock_period.eq(0)
                    with m.If(self.is_write):
                        m.d.sync += [
                            shift_reg   .eq(Cat(self.data_in, self.address, 0)),
                            bits_write  .eq(24),
                            bits_read   .eq(0),
                        ]
                    with m.Else():
                        m.d.sync += [
                            shift_reg   .eq(Cat(C(0,16), self.address, 1)),
                            bits_write  .eq(8),
                            bits_read   .eq(16+1),
                        ]
                    m.next = "WRITE_PHASE"

            with m.State("WRITE_PHASE"):
                m.d.comb += bus.sdata_oe.eq(1)
                m.d.sync += clock_period.eq(clock_period + 1)
                with m.If(falling_edge):
                    with m.If(bits_write == 0):
                        with m.If(bits_read == 0):
                            m.next = "IDLE"
                        with m.Else():
                            m.next = "READ_PHASE"
                    with m.Else():
                        m.d.sync += [
                            bus.sdata_o .eq(shift_reg[-1]),
                            shift_reg   .eq(shift_reg << 1),
                            bits_write  .eq(bits_write - 1),
                        ]
                        
            with m.State("READ_PHASE"):
                m.d.sync += clock_period.eq(clock_period + 1)
                with m.If(falling_edge):
                    with m.If(bits_read == 0):
                        m.d.sync += self.data_out.eq(shift_reg[:16])
                        m.next = "IDLE"
                    with m.Else():
                        m.d.sync += [
                            shift_reg   .eq(Cat(bus.sdata_i, shift_reg)),
                            bits_read   .eq(bits_read - 1),
                        ]

        return m
