#
# This file is part of URTI.
#
# Copyright (c) 2024 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

import unittest

from amaranth                    import Module, Signal
from amaranth.lib.wiring         import Component, In
from amaranth_soc                import wishbone
from amaranth_soc.memory         import MemoryMap

from luna.gateware.interface.i2c import I2CRegisterInterface, I2CBus
from luna.gateware.test          import LunaGatewareTestCase, sync_test_case

__all__ = ["MAX2120"]

class MAX2120(Component):
    bus: In(wishbone.Signature(addr_width=4, data_width=8))

    def __init__(self, *, pads, divisor, address=0b1100000):
        self.pads = pads
        self.divisor = divisor
        self.address = address
        super().__init__()
        self.bus.memory_map = MemoryMap(addr_width=4, data_width=8)

    def elaborate(self, platform):
        m = Module()

        # TODO: we may be interested in reading/writing multiple bytes at a time
        # Max I2C clock frequency is 400 kHz
        m.submodules.i2c_regs = i2c_regs = I2CRegisterInterface(self.pads, period_cyc=self.divisor, address=self.address)

        start = Signal()
        done  = Signal()

        m.d.comb += [
            i2c_regs.size           .eq(1),
            i2c_regs.address        .eq(self.bus.adr),

            i2c_regs.write_data     .eq(self.bus.dat_w),
            self.bus.dat_r          .eq(i2c_regs.read_data),

            start                   .eq(
                self.bus.cyc    &    # Transaction is active.
                self.bus.stb    &    # Valid data is being provided.
                ~self.bus.ack   &    # Avoid restarting with previous transaction.
                ~i2c_regs.busy       # The I2C register interface is idle.
            ),

            i2c_regs.read_request   .eq(start & ~self.bus.we),
            i2c_regs.write_request  .eq(start &  self.bus.we),

            self.bus.ack            .eq(done),
        ]

        # Workaround to stop a request when it failed, as the I2C submodule does
        # not expose a request abort flag (yet?).
        with m.FSM():
            with m.State("IDLE"):
                with m.If(~i2c_regs.busy & start):
                    m.next = "BUSY"
            with m.State("BUSY"):
                with m.If(~i2c_regs.busy | i2c_regs.done):
                    m.d.comb += done.eq(1)
                    m.next = "IDLE"

        return m

#
# Tests
#

class TestMAX2120(LunaGatewareTestCase):

    def instantiate_dut(self):
        return MAX2120(pads=I2CBus(), divisor=4)
    
    @sync_test_case
    def test_register_read(self):
        # Keep the input data tied to 0 to simulate ACKs
        yield self.dut.pads.sda.i.eq(0)

        # Check initial assumptions before the transaction
        yield
        self.assertEqual((yield self.dut.pads.scl.o), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)

        yield from self.advance_cycles(10)
        self.assertEqual((yield self.dut.pads.scl.o), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)

        # Request a register read to address 0x7
        yield self.dut.bus.cyc.eq(1)
        yield self.dut.bus.stb.eq(1)
        yield self.dut.bus.we .eq(0)
        yield self.dut.bus.adr.eq(0x7)

        # Wait for the transaction to complete, clear the request lines
        # and ensure the ACK signal only lasts one cycle

        yield from self.wait_until(self.dut.bus.ack, timeout=100*self.dut.divisor)
        yield self.dut.bus.cyc.eq(0)
        yield self.dut.bus.stb.eq(0)
        yield
        self.assertEqual((yield self.dut.bus.ack), 0)

        # Check conditions after transaction finish
        yield from self.advance_cycles(10)
        self.assertEqual((yield self.dut.pads.scl.o), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)

    @sync_test_case
    def test_register_write(self):
        # Keep the input data tied to 0 to simulate ACKs
        yield self.dut.pads.sda.i.eq(0)

        # Check initial assumptions before the transaction
        yield
        self.assertEqual((yield self.dut.pads.scl.o), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)

        yield from self.advance_cycles(10)
        self.assertEqual((yield self.dut.pads.scl.o), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)

        # Request a register write to address 0x7
        yield self.dut.bus.cyc  .eq(1)
        yield self.dut.bus.stb  .eq(1)
        yield self.dut.bus.we   .eq(1)
        yield self.dut.bus.adr  .eq(0x7)
        yield self.dut.bus.dat_w.eq(3)

        # Wait for the transaction to complete, clear the request lines
        # and ensure the ACK signal only lasts one cycle
        yield from self.wait_until(self.dut.bus.ack, timeout=100*self.dut.divisor)
        yield self.dut.bus.cyc.eq(0)
        yield self.dut.bus.stb.eq(0)
        yield
        self.assertEqual((yield self.dut.bus.ack), 0)

        # Check conditions after transaction finish
        yield from self.advance_cycles(10)
        self.assertEqual((yield self.dut.pads.scl.o), 0)
        self.assertEqual((yield self.dut.bus.ack), 0)


if __name__ == "__main__":
    unittest.main()
