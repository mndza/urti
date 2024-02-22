#
# This file is part of URTI.
#
# Copyright (c) 2024 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

from amaranth                    import Module, Signal
from amaranth.lib.wiring         import Component, In
from amaranth_soc                import wishbone
from amaranth_soc.memory         import MemoryMap

from luna.gateware.interface.i2c import I2CRegisterInterface

class MAX2120(Component):
    def __init__(self, *, pads, divisor):
        self.pads = pads
        self.divisor = divisor
        super().__init__({
            "bus": In(wishbone.Signature(addr_width=4, data_width=8))
        })
        self.bus.memory_map = MemoryMap(addr_width=4, data_width=8, name="max2120")

    def elaborate(self, platform):
        m = Module()

        # TODO: we may be interested in reading/writing multiple bytes at a time
        # Max I2C clock frequency is 400 kHz
        m.submodules.i2c_regs = i2c_regs = I2CRegisterInterface(self.pads, period_cyc=self.divisor, address=0b1100001)

        start = Signal()
        m.d.comb += [
            i2c_regs.address        .eq(self.bus.adr),

            i2c_regs.write_data     .eq(self.bus.dat_w),
            self.bus.dat_r          .eq(i2c_regs.read_data),

            start                   .eq(
                self.bus.cyc    &    # Transaction is active.
                self.bus.stb    &    # Valid data is being provided.
                self.bus.we     &    # This is a write.
                self.bus.sel[0] &    # The relevant data lane is being targeted.
                ~i2c_regs.busy       # The I2C register interface is idle.
            ),

            i2c_regs.read_request   .eq(start & ~self.bus.we),
            i2c_regs.write_request  .eq(start &  self.bus.we),

            self.bus.ack            .eq(i2c_regs.done),
        ]

        return m
