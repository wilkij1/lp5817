# MIT License
# 
# Copyright (c) 2025 Jeff Wilkinson
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# 
"""
Driver for Texas Instruments LP5817 3-channel LED driver.

Driver is connected to processor via I2C. Supports setting dot current, PWM
current and timed fades.

The variable names used in the driver assume it is hooked up to an RGB LED
with OUT0 for red, OUT1 for green and OUT2 for blue. The code will work fine
for other connection configurations but the user will have to map the variable
names manually.
"""

DEBUG = False

# from micropython import const
import time

if DEBUG:
    from machine import SPI
    from teensyview_spi import TeensyView_SPI

LP5817_ADDR		= 0x2D

ST_STANDBY, ST_NORMAL, ST_SHUTDOWN, ST_THERMAL = list(range(4))

RG_CHIP_EN		= 0x0
BT_CHIP_EN		= 0b0000_0001
RG_DEV_CONFIG0	= 0x1
BT_MAX_CURRENT	= 0b0000_0001
RG_DEV_CONFIG1	= 0x2
BT_OUT2_EN		= 0b0000_0100
BT_OUT1_EN		= 0b0000_0010
BT_OUT0_EN		= 0b0000_0001
MSK_OUTx_EN		= BT_OUT2_EN | BT_OUT1_EN | BT_OUT0_EN
RG_DEV_CONFIG2	= 0x3
MSK_LED_FADE_TIME = 0xF0
SH_FADE_TIME	= 4
BT_OUT2_FADE_EN	= 0b0000_0100
BT_OUT1_FADE_EN = 0b0000_0010
BT_OUT0_FADE_EN = 0b0000_0001
RG_DEV_CONFIG3	= 0x4
BT_OUT2_EXP_EN	= 0b0100_0000
BT_OUT1_EXP_EN	= 0b0010_0000
BT_OUT0_EXP_EN	= 0b0001_0000
RG_SHUTDOWN_CMD	= 0x0D
CMD_SHUTDOWN	= 0x33
RG_RESET_CMD	= 0x0E
CMD_RESET		= 0xCC
RG_UPDATE_CMD	= 0x0F
CMD_UPDATE		= 0x55
RG_FLAG_CLR		= 0x13
BT_TSD_CLR		= 0b0000_0010
BT_POR_CLR		= 0b0000_0001
RG_OUT0_DC		= 0x14
RG_OUT1_DC		= 0x15
RG_OUT2_DC		= 0x16
RG_OUT0_MANUAL_PWM	= 0x18
RG_OUT1_MANUAL_PWM	= 0x19
RG_OUT2_MANUAL_PWM	= 0x1A
RG_FLAG			= 0x40
BT_TSD			= 0b0000_0010
BT_POR			= 0b0000_0001

en_bits = [BT_OUT0_EN, BT_OUT1_EN, BT_OUT2_EN]
dc_regs = [RG_OUT0_DC, RG_OUT1_DC, RG_OUT2_DC]
pwm_regs = [RG_OUT0_MANUAL_PWM, RG_OUT1_MANUAL_PWM, RG_OUT2_MANUAL_PWM]

lp5817_rd_regs = [
    (RG_CHIP_EN, 0x01),
    (RG_DEV_CONFIG0, 0x01),
    (RG_DEV_CONFIG1, 0x07),
    (RG_DEV_CONFIG2, 0x07),
    (RG_DEV_CONFIG3, 0x70),
    (RG_OUT0_DC, 0xFF), (RG_OUT1_DC, 0xFF), (RG_OUT2_DC, 0xFF),
    (RG_OUT0_MANUAL_PWM, 0xFF), (RG_OUT1_MANUAL_PWM, 0xFF), (RG_OUT2_MANUAL_PWM, 0xFF),
    (RG_FLAG, 0x03)
    ]

# Fade time boundaries
# The register value corresponds to the index into the list
#  for the closest time.
fade_times = [
    0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35,
    0.40, 0.45, 0.50, 1., 2., 4., 6., 8.
    ]

debug_regs = bytearray([0]*0x41)
debug_shadow_regs = bytearray([0]*0x41)

class LP5817:
    def __init__(self, hi2c, max_current=False,
                 initialize=True, logger=None, debug=False):
        """initialize LP5817 driver and enable the chip

        @param hi2c: An I2C object connected to the LP5817.
        @param max_current: a False value sets the MAX_CURRENT bit to
            0, corresponding to 25.5 mA. Otherwise, MAX_CURRENT is set
            to 1 for a 51 mA limit.
        @param initialize: call .initialize() if True
        @param logger: A logging callback used for debugging messages.
            The callback displays a line of text. A callable argument is
            used directly. If the argument is not callable it is assumed
            to be an SDD1306 OLED and manipulated directly. A value of
            None disables logging.
        @param debug: bypass the I2C I/O to a real chip and simulate the
            registers internally.
        
        The LP5817 uses 3 parameters to set the output brightness:
            - Imax, controlled by the MAX_CURRENT bit in DEV_CONFIG0,
                having a value of 25.5 mA (MAX_CURRENT==0) or 51 mA. The
                value of Imax is shared by all 3 outputs.
            - Ddc (Dot Current), controlled by the values in the 3
                Dot Current registers (RG_OUTx_DC). The dot current value
                ranges from 0 to 255.
            - Dpwm (PWM fraction), controlled by the values in the
                RG_OUTx_MANUAL_PWM registers. These values range from 0
                to 255, also, corresponding to values from 0% (0) to 100% (255).
                
            Output current Iout = Imax * Ddc/255 * Dpwm/255
            
        This code assumes that more than 1 ms has elapsed since the
        chip was powered on so that it has entered the ST_STANDBY state. The
        user is responsible for ensuring that this minimum delay has
        occurred prior to instantiating this driver.
        """

        self._i2c = hi2c
        
        self._debug = debug

        if debug and logger is None:
            logger = lambda s: print(s)
        self._logger = logger
        self._log_enable = bool(self._logger)
            
        time.sleep_ms(2)
        
        self.log(f"LP5817: calling initialize({max_current=})")
        
        if initialize:
            self.initialize(max_current=max_current)
            
#         # confirm that BT_POR is set in the RG_FLAG register, confirming
#         #  that this state has been cleanly entered from power on. Raise
#         #  an exception if BT_POR is not set or BT_TSD (thermal) is set.
#         flag = self._readreg(RG_FLAG)
#         if not (flag & BT_POR) or (flag & BT_TSD):
#             regs = self._dump_regs()
#             raise ValueError(f"chip not found in the POR state: {regs=}")
#         self._writereg(RG_FLAG_CLR, BT_POR)
        
#         # enable channels that are requested by enabled and stash its
#         #  value for later reference
#         if not hasattr(channels, "index"):
#             channels = [channels]
#             
#         enabled = 0
#         for ch in channels:
#             bit = self._index_into(en_bits, ch)
#             enabled |= bit
#             
#         self._enabled = enabled & MSK_OUTx_EN
#         self._writereg(RG_DEV_CONFIG1, self._enabled)
#         
#         # set DEV_CONFIG 2 & 3 to their default values with fade disabled
#         self._writereg(RG_DEV_CONFIG2, 0)
#         self._writereg(RG_DEV_CONFIG3, 0)
#         
#         # send UPDATE_CMD to make changes to DEV_CONFIGx take effect
#         self._writereg(RG_UPDATE_CMD, CMD_UPDATE)
#         
#         # assume that the chip has completed initialization
#         self._state = ST_NORMAL

    def initialize(self, max_current=False):
        
        self.init()
    
#         self.log("dump registers after .init()")
#         self.log(f"{self._dump_regs()=}")

        # set global max current
        self.log("set max current")
        self._max_current = bool(max_current)
        self._writereg(RG_DEV_CONFIG0,
                       BT_MAX_CURRENT if self._max_current else 0)
        
        # enable all 3 channels
        self.log("enable all 3 channels")
        self._writereg(RG_DEV_CONFIG1, BT_OUT2_EN | BT_OUT1_EN | BT_OUT0_EN)
        
        # disable fade on all channels
        self.log("disable fade, all channels")
        self._writereg(RG_DEV_CONFIG2, 0)
        
        # disable exponential dimming on all channels
        self.log("disable exponential dimming, all channels")
        self._writereg(RG_DEV_CONFIG3, 0)
        
        # set dot current to 3/4 for all channels
        self.log("set dot current to 192, all channels")
        for ch in range(3):
            self._writereg(RG_OUT0_DC+ch, 192+ch)
            
        # set PWM at mid-level
        self.log("set PWM to 128, all channels")
        for ch in range(3):
            self._writereg(RG_OUT0_MANUAL_PWM+ch, 128+ch)
            
#         self.log("dump registers, before ._update()")
#         self.log(f"{self._dump_regs()=}")

        # send an UPDATE command
        self.log("send UPDATE command")
        self._update()

#         self.log("dump registers, after ._update()")
#         self.log(f"{self._dump_regs()=}")
# 
#         self.log("enable again")
#         self.enable(True)
#         
        self.log("dump registers, at end")
        self.log(f"{self._dump_regs()=}")

    def init(self):
        """enable and reset LP5817"""
        
        self.reset()
        time.sleep_ms(2)
        self.enable(True)
        time.sleep_ms(10)
        
    def enable(self, on):
        
        on = bool(on)
        self.log(f"LP5817: setting enable to {on}")
        self._writereg(RG_CHIP_EN, BT_CHIP_EN if on else 0)
        
    def reset(self):
        """reset the LP5817 and delay to give it time to be ready"""

        self.log(f"LP5817: resetting")
        self._writereg(RG_RESET_CMD, CMD_RESET)
        time.sleep_ms(2)
        
    def _update(self):
        self._writereg(RG_UPDATE_CMD, CMD_UPDATE)
        
    def _dump_regs(self):
        regs = [f"{rg:02X}:{self._readreg(rg) & mask:02X}"
                for rg, mask in lp5817_rd_regs]
        return regs
    
    def set_pwm(self, ch, value):
        
        assert 0 <= ch <= 2, f"channel ({ch}) must be in [0,2]"
        assert 0 <= value <= 255, f"value ({value}) must be in [0, 255]"
        
        self._writereg(RG_OUT0_MANUAL_PWM+ch, value)
        
    def get_state(self):
        return self._state
    
    def enter_shutdown(self):
        """Place the LP5817 in shutdown mode using the shutdown command"""
        
        self.log("> shutdown")
        self._writereg(RG_SHUTDOWN_CMD, CMD_SHUTDOWN)
        
        self._state = ST_SHUTDOWN
        
    def exit_shutdown(self):
        """Exit shutdown by toggling SDA 8 times while holding SCL high

        The only registers that are preserved through a shutdown are
        RG_CHIP_EN (set to BT_CHIP_EN), DEV_CONFIG0, and DEV_CONFIG1.
        The values in DEV_CONFIG0 and 1 are set to the values determined
        when the driver was instantiated.
        
        All other registers assume their POR values.
        """
        
        self.log("< shutdown")
        
        if self._i2c:
            # make sure SDA is high
            self._i2c.sda.value(1)
            time.sleep_ms(1)
            
            # generate 8 falling and 8 rising edges
            for _ in range(8):            
                self._i2c.sda.value(0)
                time.sleep_ms(1)
                self._i2c.sda.value(1)
                time.sleep_ms(1)
            
        # wait for chip to initialize, and then enable it
        time.sleep_ms(2)
        self._writereg(RG_CHIP_EN, BT_CHIP_EN)
        
        # set max current and enable channels
        self._writereg(RG_DEV_CONFIG0,
                       BT_MAX_CURRENT if self._max_current else 0)
        self._writereg(RG_DEV_CONFIG1, self._enabled)
        
        # send UPDATE_CMD to make changes to DEV_CONFIGx take effect
        self._writereg(RG_UPDATE_CMD, CMD_UPDATE)

        # mark in NORMAL mode (all register will have been set
        #  back to POR values, except CHIP_EN and MAX_CURRENT)
        self._state = ST_NORMAL
        
    def is_shutdown(self):
        """Return True if in SHUTDOWN state, False otherwise"""
        return self._state == ST_SHUTDOWN
            
    def is_thermal_shutdown(self):
        """Return True if in thermal shutdown and update the state"""
        
        if self._state == ST_SHUTDOWN:
            # not in thermal shutdown, because it was manually shutdown
            return False
        else:
            # check for thermal shutdown and update the state
            flag = self._readreg(RG_FLAG)
            if flag & BT_TSD:
                self._state = ST_THERMAL
                return True
            else:
                self._state = ST_NORMAL
    
    def set_dot_current(self, dot_currents=None, channels=None):
        """Set dot current for an enabled channel

        @param channels: channels specifies the channel using red==0, green==1,
            or blue==2. An integer value applies to a single channel while
            a list/tuple of values specifies multiple channels. None, which
            is the default, specifies all channels
            
        @param dot_currents: The dot_currents parameter is either a scalar
            value to apply to all channels or an iterable matching the
            length of channels.
            The dot_current parameter is either an integer from 0 to 255
            (inclusive) or a float from 0. to 1. Assuming that
            the channel is enabled, then the dot current register
            is set to a value from 0 to 255. The dot_current defaults to
            255 (fully on).
        """

        # determine the channels to be updated
        channels, _ = self._check_channels(channels)
        dot_currents = self._expand_values4channels(dot_currents, channels)
        
        for ch, dc in zip(channels, dot_currents):
            bit = self._index_into(en_bits, ch)
            if not (bit & self._enabled):
                # silently ignore disabled channel
                continue
            
            reg = self._index_into(dc_regs, ch)
            
            self._writereg(reg, self._scale_value(dc))
      
    def get_dot_current(self, channels=None):
        """Return dot current(s) as an integer from 0 to 255

        @param channels: channels specifies the channel using red==0, green==1,
            or blue==2. An integer value applies to a single channel while
            a list/tuple of values specifies multiple channels. None, which
            is the default, specifies all channels.
            
        @return: Returns dot currents as a single integer in range of 0 to 255
            if only a single channel was requested. Otherwise, returns a list
            of dot current values in the order that they were requested.
        """
        
        channels, return_iterable = self._check_channels(channels)
            
        dcs = []
        for ch in channels:
            reg = self._index_into(dc_regs, ch)
            dcs.append(self._readreg(reg))
        
        if return_iterable:
            return dcs
        else:
            return dcs[0]
            
    def set_pwm2(self, pwms=None, channels=None):
        """Set the PWM value for 1 or more channels
        
        @param channels: channels specifies the channel using red==0, green==1,
            or blue==2. An integer value applies to a single channel while
            a list/tuple of values specifies multiple channels.
        
        @param pwms: PWM values to set, corresponding to the requested
            channels. If pwms is a scalar then it is applied to all enabled
            channels. Otherwise, it must match the length of channels and
            specifies individual pwm values in the same order.
            
            A pwm that is float in the range [0, 1.] is mapped to integers
            0 to 255. An integer value must be in the range of [0, 255].
        """
        
        # make sure channels is iterable
        channels, _ = self._check_channels(channels)
        pwms = self._expand_values4channels(pwms, channels)
        
        for ch, pwm in zip(channels, pwms):
            bit = self._index_into(en_bits, ch)
            if not (bit & self._enabled):
                # silently ignore disabled channel
                continue
            
            reg = self._index_into(pwm_regs, ch)
            
            # write the scaled PWM value
            self._writereg(reg, self._scale_value(pwm))
        
        
    def get_pwm(self, channels=None):
        """Return dot current(s) as integers from 0 to 255

        @param channels: channels specifies the channel using red==0, green==1,
            or blue==2. An integer value applies to a single channel while
            a list/tuple of values specifies multiple channels. None, which
            is the default, specifies all channels.
            
        @return: Returns PWM values as a single integer in range of 0 to 255
            if only a single channel was requested. Otherwise, returns a list
            of PWM values in the order that they were requested.
        """
        
        channels, return_iterable = self._check_channels(channels)
            
        pwms = []
        for ch in channels:
            reg = self._index_into(pwm_regs, ch)
            pwms.append(self._readreg(reg))
        
        if return_iterable:
            return pwms
        else:
            return pwms[0]
            
    def set_fade(self, fade_time, exp_fade=False):
        """Update DEV_CONFIG2 and DEV_CONFIG3 with fade time for enabled channels

        @param fade_time: if fade_time is a float it is assumed to
            specify a desired fade time and is mapped to the values
            available in the chip. The value is compared to available
            fade times and set to the one that is equal to or next fastest.
            An integer value of fade_time in the range of [0, 15] is
            set directly as the value in DEV_CONFIG2 and interpreted as
            specified in table 7-11 of the TI datasheet.
            Calling set_fade() will establish fade parameters for all
            enabled channels together, even though the LP5817 allows
            finer grained control.
            If fade_time == 0, then the corresponding OUTx_FADE_EN and
            OUTx_EXP_EN bits are cleared for all enabled channels.
            
        @param exp_fade: Linear fade is chosen if exp_fade is False. Otherwise,
            exponential fade is selected.
        """
        
        # find the largest, legal fade value <= the fade_time parameter
        jj = [ii for ii, ft in enumerate(fade_times) if ft <= fade_time][-1]
        fade_val = jj << SH_FADE_TIME
        
        # disable fade bits if fade_val == 0. Otherwise, set the bits appropriately
        if fade_val == 0:
            self._writereg(RG_DEV_CONFIG2, 0)
            self._writereg(RG_DEV_CONFIG3, 0)
        else:
            # mirror the enable bits in the OUTx_FADE_EN bits
            self._writereg(RG_DEV_CONFIG2, fade_val | self._enabled)
            # shift the enabled bits to upper nybble to get OUTx_EXP_EN bits
            en_bits = self._enabled if exp_fade else 0
            self._writereg(RG_DEV_CONFIG3, en_bits << 4)
        
        # poke the UPDATE_CMD register to write to DEV_CONFIGx regs
        self._writereg(RG_UPDATE_CMD, CMD_UPDATE)
        
    def get_fade(self):
        """Return the fade value and status information

        @return: A namedtuple containing the actual fade time (float), DEV_CONFIG2 and
            DEV_CONFIG3 and individual enable bits.
        """
        from collections import namedtuple
        Fade = namedtuple("Fade",
            ("time OUT0_FADE_EN OUT1_FADE_EN OUT2_FADE_EN"
             " OUT0_EXP_EN OUT1_EXP_EN OUT2_EXP_EN C2 C3").split())
        
        # read DEV_CONFIG 2 and 3
        c2 = self._readreg(RG_DEV_CONFIG2)
        c3 = self._readreg(RG_DEV_CONFIG3)
        
        dd = {}
        fade_val = (c2 >> SH_FADE_TIME) & 0xF
        dd['time'] = fade_times[fade_val]
        for nm, msk in zip("OUT0_FADE_EN OUT1_FADE_EN OUT2_FADE_EN".split(),
                           (BT_OUT0_FADE_EN, BT_OUT1_FADE_EN, BT_OUT2_FADE_EN)):
            dd[nm] = int(bool(c2 & msk))
        for nm, msk in zip("OUT0_EXP_EN OUT1_EXP_EN OUT2_EXP_EN".split(),
                           (BT_OUT0_EXP_EN, BT_OUT1_EXP_EN, BT_OUT2_EXP_EN)):
            dd[nm] = int(bool(c3 & msk))
        
        dd["C2"] = c2
        dd["C3"] = c3
        
        fade = Fade(**dd)
        
        return fade
        
    def log(self, msg, **kw):
        """Log debug messages if a logger was setup during __init__"""
        
        if self._logger is None or not self._log_enable:
            return
        else:
            self._logger(msg, **kw)
            
    def log_enable(self):
        self._log_enable = True
        
    def log_disable(self):
        self._log_enable = False
        
    def _readreg(self, reg):
        
        if not self._debug:
            buf = self._i2c.readfrom_mem(LP5817_ADDR, reg, 1)
            val = buf[0]
        else:
            val = int(debug_regs[reg])
            
        self.log(f"rd R{reg:02X}={val:02X}")
        return val
    
    def _writereg(self, reg, val):
        
        self.log(f"wr {val:02X}->R{reg:02X}")

        if not self._debug:
            buf = bytearray([val & 0xFF])
            self._i2c.writeto_mem(LP5817_ADDR, reg, buf)
        else:
            if reg == RG_FLAG_CLR:
                # clear TSD or POR, as requested
                debug_regs[RG_FLAG] &= (~val & 0x03)
            elif reg == RG_RESET_CMD:
                if val == CMD_RESET:
                    for ii in range(RG_CHIP_EN, RG_FLAG+1):
                        debug_regs[ii] = 0
                        debug_shadow_regs[ii] = 0
                debug_regs[RG_FLAG] = BT_POR
            elif reg == RG_UPDATE_CMD:
                if val == CMD_UPDATE:
                    for ii in range(RG_DEV_CONFIG0, RG_DEV_CONFIG3+1):
                        debug_regs[ii] = debug_shadow_regs[ii]
            elif reg in (RG_DEV_CONFIG0, RG_DEV_CONFIG1,
                         RG_DEV_CONFIG2, RG_DEV_CONFIG3):
                debug_shadow_regs[reg] = val
            else:
                debug_regs[reg] = val
                    
    def _index_into(self, array, ch):
        """Index into provided array by the channel number argument
        @param array: 3 element array of values to be return for
            channel 0, 1 or 2
        @param ch: integer value of 0, 1 or 2.
        
        @raise: ValueError if ch is not in range [0, 2]
        @return: array value corresponding to the ch argument
        """
        
        try:
            return array[ch]
        except IndexError as e:
            msg = f"channel numbers must be in [0,1,2], found {ch=}"
            raise ValueError(msg) from e
        
    def _check_channels(self, channels):

        if channels is None:
            # default is all channels and that return value should be iterable
            return [0, 1, 2], True
        elif not hasattr(channels, "index"):
            # make it iterable for processing, but the return value should be scalar
            return [channels], False
        else:
            return channels, True

    def _expand_values4channels(self, values, channels):
        
        if not hasattr(values, "index"):
            return [values]*len(channels)
        else:
            if len(values) != len(channels):
                raise ValueError(f"mismatch between channels and values lengths")
            return values
        
    def _scale_value(self, value):
        """Convert 0-1 float to 0-255 int and range check

        The float scaling first maps 0-1 to 0-256 and clamps the result to
        0-255. This makes the scaled values correspond to more intuitive
        breakpoints, such as 0.75 -> 0xC0 rather than 0xBF"""
        
        if isinstance(value, float):
            if (value < 0.) or (value > 1.):
                raise ValueError(f"value as a float must be in [0, 1.]")
            value = int(value * 256)
            return 255 if value > 255 else value
        else:
            if (value < 0) or (value > 255):
                raise ValueError(f"value as an int must be in [0, 255]")
            return value
