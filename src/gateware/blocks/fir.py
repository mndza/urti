#
# This file is part of URTI.
#
# Copyright (c) 2024 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

from math                   import ceil, log2
from itertools              import zip_longest

from amaranth               import Module, Signal, Cat, signed, ValueCastable
from amaranth.utils         import bits_for
from amaranth.lib           import wiring
from amaranth.lib.wiring    import In, Out


class Stream(wiring.Signature):
    def __init__(self, data_shape):
        members = {
            "data":  Out(data_shape),
            "valid": Out(1),
            "first": Out(1),
            "last":  Out(1),
            "ready": In(1),
        }
        super().__init__(members)


class HalfBandDecimationFilter(wiring.Component):
    def __init__(self, taps, shape_in, delay=0, shape_out=None):
        midtap = taps[len(taps)//2]
        assert taps[1::2] == [0]*(len(taps)//4) + [midtap] + [0]*(len(taps)//4)
        self.taps = taps
        self.shape_in = shape_in
        self.delay = delay

        if shape_out is None:
            # If output shape is not defined, add normalized DC gain
            shape_out = signed(bits_for(-2**(shape_in.width-1) * sum(taps)))

        super().__init__({
            "input":  In(Stream(shape_in)),
            "output": Out(Stream(shape_out)),
        })

    def elaborate(self, platform):
        m = Module()

        # Arms
        m.submodules.fir0 = fir0 = FIRFilter(self.taps[0::2], self.shape_in)
        m.submodules.fir1 = fir1 = FIRFilter(self.taps[1::2], self.shape_in)
        arms = [fir0, fir1]

        # Arm index selection: switch after every delivered sample
        arm_index = Signal(range(2), init=self.delay)
        with m.If(self.input.ready & self.input.valid):
            m.d.sync += arm_index.eq(arm_index - 1)

        # Input switching
        with m.Switch(arm_index):
            for i, arm in enumerate(arms):
                m.d.comb += arm.input.data.eq(self.input.data)
                with m.Case(i):
                    m.d.comb += [
                        arm.input.valid     .eq(self.input.valid),
                        self.input.ready    .eq(arm.input.ready),
                    ]

        # Output accumulator
        accumulator = Signal(signed(max(len(arm.output.data) for arm in arms) + bits_for(len(arms) - 1)))
        for i, arm in enumerate(arms):
            m.d.comb += arm.output.ready.eq(self.output.ready)
            with m.If(arm.output.valid & arm.output.ready):
                if i == len(arms) - 1:
                    m.d.sync += accumulator.eq(arm.output.data)
                else:
                    m.d.sync += accumulator.eq(accumulator + arm.output.data)
        
        m.d.comb += self.output.data.eq(convergent_round(accumulator, self.output.data))
        m.d.sync += self.output.valid.eq(arms[0].output.valid)

        return m



class HalfBandInterpolationFilter(wiring.Component):
    def __init__(self, taps, shape_in, delay=0, shape_out=None):
        midtap = taps[len(taps)//2]
        assert taps[1::2] == [0]*(len(taps)//4) + [midtap] + [0]*(len(taps)//4)
        self.taps = taps
        self.shape_in = shape_in
        self.delay = delay

        if shape_out is None:
            # If output shape is not defined, add normalized DC gain
            shape_out = signed(bits_for(-2**(shape_in.width-1) * sum(taps)))

        super().__init__({
            "input":  In(Stream(shape_in)),
            "output": Out(Stream(shape_out)),
        })

    def elaborate(self, platform):
        m = Module()

        # Arms
        m.submodules.fir0 = fir0 = FIRFilter(self.taps[0::2], self.shape_in)
        m.submodules.fir1 = fir1 = FIRFilter(self.taps[1::2], self.shape_in)
        arms = [fir0, fir1]

        # Arm index selection: switch after every delivered sample
        arm_index = Signal(range(2), init=self.delay)
        with m.If(self.output.ready & self.output.valid):
            m.d.sync += arm_index.eq(arm_index - 1)

        # Deliver input samples
        for i, arm in enumerate(arms):
            m.d.comb += [
                arm.input.data      .eq(self.input.data),
                arm.input.valid     .eq(self.input.valid & arms[i^1].input.ready),
            ]
        m.d.comb += self.input.ready.eq(arms[0].input.ready & arms[1].input.ready)

        # Output switching
        with m.Switch(arm_index):
            for i, arm in enumerate(arms):
                with m.Case(i):
                    m.d.comb += [
                        self.output.data    .eq(convergent_round(arm.output.data, self.output.data)),
                        self.output.valid   .eq(arm.output.valid),
                        arm.output.ready    .eq(self.output.ready),
                    ]

        return m

class FIRFilter(wiring.Component):
    
    def __init__(self, taps, shape_in):
        shape_out = signed(bits_for(-2**(shape_in.width-1) * sum(taps)))  # normalized DC gain
        self.taps = list(taps)
        
        super().__init__({
            "input":  In(Stream(shape_in)),
            "output": Out(Stream(shape_out)),
        })

    def elaborate(self, platform):
        m = Module()
        
        # History of previous samples
        delay_line = [ Signal.like(self.input.data, name=f"delay_{i}") for i in range(len(self.taps)-1) ]
        with m.If(self.input.ready):
            with m.If(self.input.valid):
                # Update sample history
                m.d.sync += Cat(delay_line).eq(Cat(self.input.data, *delay_line))

        # Check for coefficient symmetry
        taps      = self.taps
        symmetric = taps == taps[::-1]
        if symmetric:
            taps = taps[:(len(taps)+1)//2]

        # Sample window for multiplication with taps
        window = [self.input.data] + delay_line
        if symmetric:
            window_valid = Signal()
            new_window = [ window[i] + window[-i-1] for i in range(len(window)//2) ]
            if len(window) % 2 == 1:
                new_window.append(window[len(window)//2])
            adder_reg = [ Signal.like(a, name=f"sym_{i}") for i, a in enumerate(new_window) ]
            with m.If(self.input.ready):
                m.d.sync += window_valid.eq(self.input.valid)
                with m.If(self.input.valid):
                    m.d.sync += [ reg.eq(value) for reg, value in zip(adder_reg, new_window) ]
            window = adder_reg
        else:
            window_valid = self.input.valid

        # Multiplication stage: definitions and stream processing
        muls_val = [ Sample(b) * Sample(a) for a, b in zip(taps, window) ]
        muls_reg = [ Signal.like(m, name="mul") for m in muls_val ]

        muls_valid = Signal()
        muls_ready = Signal()

        m.d.comb += self.input.ready.eq(~muls_valid | muls_ready)

        with m.If(self.input.ready):
            m.d.sync += muls_valid.eq(window_valid)
            with m.If(window_valid):
                # Multiply current window and store results
                m.d.sync += [ reg.eq(value) for reg, value in zip(muls_reg, muls_val) ]
                
        # Adder tree stages, with ceil(log2(N)) levels
        level, level_valid, level_ready = muls_reg, muls_valid, muls_ready
        lev = 0
        while len(level) > 1:
            even = level[0::2]
            odd  = level[1::2]
            results = [ a+b if b is not None else a for a,b in zip_longest(even, odd) ]
            new_level = [ Signal.like(r, name=f"add_{lev}_{i}") for i, r in enumerate(results) ]
            new_valid = Signal()
            new_ready = Signal()
            m.d.comb += level_ready.eq(~new_valid | new_ready)
            with m.If(level_ready):
                m.d.sync += new_valid.eq(level_valid)
                with m.If(level_valid):
                    m.d.sync += [ reg.eq(value) for reg, value in zip(new_level, results) ]
            level, level_valid, level_ready = new_level, new_valid, new_ready
            lev += 1

        # Output wiring
        m.d.comb += self.output.data   .eq(level[0])
        m.d.comb += self.output.valid  .eq(level_valid)
        m.d.comb += level_ready        .eq(self.output.ready)

        return m


class Sample(ValueCastable):
    def __init__(self, value, max_abs=None):
        self._value = value
        self._max_abs = max_abs
        if max_abs is None:
            if isinstance(value, int):
                self._max_abs = abs(value)
            else:
                self._max_abs = 2**(len(value)-1)

    def as_value(self):
        return self._value

    def shape(self):
        return signed(bits_for(self._max_abs)+1)

    def __add__(self, other):
        assert isinstance(other, Sample)
        if self._max_abs == 0:
            return other
        if other._max_abs == 0:
            return self
        max_abs = self._max_abs + other._max_abs
        return Sample(self._value + other._value, max_abs)

    def __mul__(self, other):
        assert isinstance(other, Sample)
        if self._max_abs == 0 or other._max_abs == 0:
            return Sample(0, 0)
        max_abs = self._max_abs * other._max_abs
        if 0:
            return Sample(self._value * other._value, max_abs)
        else:
            return Sample(shift_and_add_mpy(self._value, other._value), max_abs)


def shift_and_add_mpy(sample, tap):
    if tap == 0:
        return 0

    # Remove lower 0 bits from csd representation
    # TODO: we can also do it by checking tap % 2 == 0
    shift_left = 0
    while tap % 2 == 0:
        tap //= 2
        shift_left += 1

    csd_tap = to_csd_i(tap)

    terms = []
    for i, c in enumerate(csd_tap[::-1]):
        if c == '0':
            continue
        elif c == '+':
            terms.append(sample << i if i>0 else sample)
        elif c == '-':
            terms.append(-sample << i if i>0 else -sample)
    summed = sum(terms) if len(terms) > 1 else terms[0]

    return summed << shift_left


def truncation(n, w):
    return n[-w:]


def round_to_zero(n, w):
    if len(n)-w-2 > 0:
        return (n + Cat((~n[-1]).replicate((len(n)-w)-2), n[-1]))[:-1][-w:]
    else:
        return (n + ~n[-1])[:-1][-w:]

def convergent_round(input, output):
    iw = len(input)
    ow = len(output)
    if ow >= iw:
        return input
    w_convergent = input + Cat((~input[iw-ow]).replicate(iw-ow-1), input[iw-ow])
    return w_convergent[:-1][-ow:]



def to_csd_i(decimal_value: int) -> str:
    """
    The `to_csd_i` function converts a given integer into a Canonical Signed Digit (CSD) representation.

    Original author: Harnesser
    <https://sourceforge.net/projects/pycsd/>
    License: GPL2

    :param decimal_value: The `decimal_value` parameter is an integer that represents the decimal value to be converted to
    CSD format
    :type decimal_value: int
    :return: The function `to_csd_i` returns a string containing the CSD (Canonical Signed Digit) value.

    Examples:
        >>> to_csd_i(28)
        '+00-00'
        >>> to_csd_i(-0)
        '0'
        >>> to_csd_i(0)
        '0'
    """
    # figure out binary range, special case for 0
    if decimal_value == 0:
        return "0"

    rem = ceil(log2(abs(decimal_value) * 1.5))
    p2n = pow(2, rem)
    csd = ""
    while p2n > 1:
        # convert the number
        p2n_half = p2n // 2
        det = 3 * decimal_value
        if det > p2n:
            csd += "+"
            decimal_value -= p2n_half
        elif det < -p2n:
            csd += "-"
            decimal_value += p2n_half
        else:
            csd += "0"
        p2n = p2n_half
    return csd


#
# Tests
#
import unittest
import numpy as np
from amaranth.sim import Simulator, Tick

class _TestFilter(unittest.TestCase):

    def _generate_samples(self, count, width):
        # Generate `count` random samples
        rng = np.random.default_rng()
        samples = rng.normal(0, 1, count)

        # Convert to integer
        samples = np.round(samples / max(abs(samples)) * (2**(width-1) - 1)).astype(int)
        assert max(samples) < 2**(width-1) and min(samples) >= -2**(width-1)  # sanity check

        return samples

    def _filter(self, dut, samples, count):

        def input_process():
            for sample in samples:
                yield dut.input.data .eq(int(sample))
                yield dut.input.valid.eq(1)
                yield Tick()
                while not (yield dut.input.ready):
                    yield Tick()
            yield dut.input.valid.eq(0)
        
        filtered = []
        def output_process():
            yield dut.output.ready.eq(1)
            while len(filtered) < count:
                yield Tick()
                if (yield dut.output.valid):
                    filtered.append((yield dut.output.data))

        sim = Simulator(dut)
        sim.add_clock(1/100e6)
        sim.add_process(input_process)
        sim.add_process(output_process)
        sim.run()
        
        return filtered


class TestFIRFilter(_TestFilter):

    def test_filter(self):
        taps = [-1, 0, 9, 16, 9, 0, -1]
        num_samples = 16*1024
        input_width = 8
        input_samples = self._generate_samples(num_samples, input_width)

        # Compute the expected result
        filtered_np = list(np.convolve(input_samples, taps).astype(int))

        # Simulate DUT
        dut = FIRFilter(taps, signed(input_width))
        filtered = self._filter(dut, input_samples, len(input_samples))

        self.assertListEqual(filtered_np[:len(filtered)], filtered)


class TestHalfBandDecimator(_TestFilter):

    def test_filter(self):
        taps = [-1, 0, 9, 16, 9, 0, -1]
        num_samples = 16*1024
        input_width = 8
        input_samples = self._generate_samples(num_samples, input_width)

        # Compute the expected result
        filtered_np = np.convolve(input_samples, taps).astype(int)
        filtered_np = list(filtered_np[0::2])  # decimate

        # Simulate DUT
        dut = HalfBandDecimationFilter(taps, signed(input_width))
        filtered = self._filter(dut, input_samples, len(input_samples) // 2)

        self.assertListEqual(filtered_np[:len(filtered)], filtered)

    def test_filter_rounding(self):
        taps = [-1, 0, 9, 16, 9, 0, -1]
        num_samples = 16*1024
        input_width = 8
        input_samples = self._generate_samples(num_samples, input_width)

        # Compute the expected result
        filtered_np = np.convolve(input_samples, taps).astype(int)
        filtered_np = filtered_np[0::2]                 # decimate
        filtered_np = list(np.round(filtered_np / 32))  # round

        # Simulate DUT
        dut = HalfBandDecimationFilter(taps, signed(input_width), shape_out=signed(input_width))
        filtered = self._filter(dut, input_samples, len(input_samples) // 2)

        self.assertListEqual(filtered_np[:len(filtered)], filtered)


class TestHalfBandInterpolator(_TestFilter):

    def test_filter(self):
        taps = [-1, 0, 9, 16, 9, 0, -1]
        num_samples = 16*1024
        input_width = 8
        input_samples = self._generate_samples(num_samples, input_width)

        # Compute the expected result
        input_samples_2x = np.zeros(2*len(input_samples))
        input_samples_2x[0::2] = input_samples
        filtered_np = np.convolve(input_samples_2x, taps).astype(int)
        filtered_np = list(filtered_np)

        # Simulate DUT
        dut = HalfBandInterpolationFilter(taps, signed(input_width))
        filtered = self._filter(dut, input_samples, 2*len(input_samples))

        self.assertListEqual(filtered_np[:len(filtered)], filtered)

    def test_filter_rounding(self):
        taps = [-1, 0, 9, 16, 9, 0, -1]
        num_samples = 16*1024
        input_width = 8
        input_samples = self._generate_samples(num_samples, input_width)

        # Compute the expected result
        input_samples_2x = np.zeros(2*len(input_samples))
        input_samples_2x[0::2] = input_samples
        filtered_np = np.convolve(input_samples_2x, taps).astype(int)
        filtered_np = list(np.round(2 * filtered_np / 32))  # round

        # Simulate DUT
        dut = HalfBandInterpolationFilter(taps, signed(input_width), shape_out=signed(input_width))
        filtered = self._filter(dut, input_samples, 2*len(input_samples))

        self.assertListEqual(filtered_np[:len(filtered)], filtered)


if __name__ == "__main__":
    unittest.main()
