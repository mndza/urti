from amaranth import Elaboratable, Module, Signal

from luna.gateware.interface.i2c import I2CRegisterInterface

class MAX2120(Elaboratable):
    def __init__(self, *, pads):
        self.pads          = pads

        self.busy          = Signal()
        self.address       = Signal(8)
        self.done          = Signal()

        self.read_request  = Signal()
        self.read_data     = Signal(8)

        self.write_request = Signal()
        self.write_data    = Signal(8)

    def elaborate(self, platform):
        m = Module()

        # TODO: we may be interested in reading/writing multiple bytes at a time
        # Max I2C clock frequency is 400 kHz
        m.submodules.i2c_regs = i2c_regs = I2CRegisterInterface(self.pads, period_cyc=100, address=0b1100001)

        m.d.comb += [
            self.busy               .eq(i2c_regs.busy),
            i2c_regs.address        .eq(self.address),
            self.done               .eq(i2c_regs.done),

            i2c_regs.read_request   .eq(self.read_request),
            self.read_data          .eq(i2c_regs.read_data),

            i2c_regs.write_request  .eq(self.write_request),
            i2c_regs.write_data     .eq(self.write_data),
        ]

        return m

