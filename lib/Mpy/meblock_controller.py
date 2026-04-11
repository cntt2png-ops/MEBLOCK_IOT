"""
meblock_controller.py
MicroPython joystick/controller helper adapted from the uploaded C++ MeblockController.

Main features:
- Read 2 analog joysticks + 2 push buttons
- Per-axis mapping from raw ADC to custom output range
- Deadzone support
- Optional average filter (on/off)
- Center calibration + min/max tracking
- Can manually set center and raw min/max for each axis
- Can map current readings or arbitrary raw values
- Optional save/load config to JSON

Default logic follows the C++ library:
- XL/XR use inverted mapping
- YL/YR use normal mapping
- Default output maps:
    XL: -10 .. 10
    YL: -20 .. 20
    XR: -15 .. 15
    YR:  15 .. -15
"""

from machine import Pin, ADC
from time import sleep_ms, ticks_ms, ticks_diff

try:
    import ujson as json
except ImportError:
    import json


class MeblockController:
    AXES = ("XL", "YL", "XR", "YR")

    def __init__(
        self,
        xl_pin=3,
        yl_pin=4,
        swl_pin=6,
        xr_pin=1,
        yr_pin=2,
        swr_pin=5,
        deadzone=60,
        loop_delay_ms=50,
        use_average=True,
        avg_samples=5,
        auto_track_minmax=True,
        config_file="meblock_controller.json",
    ):
        self.loop_delay_ms = int(loop_delay_ms)
        self.deadzone = int(deadzone)
        self.use_average = bool(use_average)
        self.avg_samples = max(1, int(avg_samples))
        self.auto_track_minmax = bool(auto_track_minmax)
        self.config_file = config_file

        # Pins
        self.xl_pin_num = xl_pin
        self.yl_pin_num = yl_pin
        self.xr_pin_num = xr_pin
        self.yr_pin_num = yr_pin
        self.swl_pin_num = swl_pin
        self.swr_pin_num = swr_pin

        self.swl_pin = Pin(swl_pin, Pin.IN, Pin.PULL_UP)
        self.swr_pin = Pin(swr_pin, Pin.IN, Pin.PULL_UP)

        self.adc_xl = ADC(Pin(xl_pin))
        self.adc_yl = ADC(Pin(yl_pin))
        self.adc_xr = ADC(Pin(xr_pin))
        self.adc_yr = ADC(Pin(yr_pin))
        self._setup_adc(self.adc_xl)
        self._setup_adc(self.adc_yl)
        self._setup_adc(self.adc_xr)
        self._setup_adc(self.adc_yr)

        # Current raw states
        self.XL = 0
        self.YL = 0
        self.XR = 0
        self.YR = 0
        self.SWL = 0
        self.SWR = 0

        # Calibration raw MIN / MAX
        self.XL_min = 4095
        self.YL_min = 4095
        self.XR_min = 4095
        self.YR_min = 4095

        self.XL_max = 0
        self.YL_max = 0
        self.XR_max = 0
        self.YR_max = 0

        # Calibration CENTER
        self.XL_center = 0
        self.YL_center = 0
        self.XR_center = 0
        self.YR_center = 0

        # Output mapping defaults copied from C++
        self.XLMinMap = -10
        self.XLMaxMap = 10
        self.YLMinMap = -20
        self.YLMaxMap = 20
        self.XRMinMap = -15
        self.XRMaxMap = 15
        self.YRMinMap = 15
        self.YRMaxMap = -15

    def _setup_adc(self, adc):
        try:
            adc.atten(ADC.ATTN_11DB)
        except Exception:
            pass
        try:
            adc.width(ADC.WIDTH_12BIT)
        except Exception:
            pass

    def begin(self, load_config=False):
        if load_config:
            self.load()
        return self

    # ---------- Filter ----------
    def set_average_filter(self, enabled=True, samples=None):
        self.use_average = bool(enabled)
        if samples is not None:
            self.avg_samples = max(1, int(samples))
        return self

    # ---------- Internal helpers ----------
    def _normalize_axis(self, axis):
        axis = str(axis).upper()
        if axis not in self.AXES:
            raise ValueError("axis must be one of: XL, YL, XR, YR")
        return axis

    def _set_attr(self, axis, suffix, value):
        setattr(self, "{}_{}".format(axis, suffix), int(value))

    def _get_attr(self, axis, suffix):
        return getattr(self, "{}_{}".format(axis, suffix))

    def _set_map_attr(self, axis, side, value):
        setattr(self, "{}{}Map".format(axis, side), int(value))

    def _get_map_attr(self, axis, side):
        return getattr(self, "{}{}Map".format(axis, side))

    def _read_adc(self, adc):
        if not self.use_average:
            return int(adc.read())

        total = 0
        for _ in range(self.avg_samples):
            total += int(adc.read())
        return total // self.avg_samples

    def _read_buttons(self):
        self.SWL = 0 if self.swl_pin.value() else 1
        self.SWR = 0 if self.swr_pin.value() else 1

    def _track_minmax(self):
        if self.XL < self.XL_min:
            self.XL_min = self.XL
        if self.XL > self.XL_max:
            self.XL_max = self.XL

        if self.YL < self.YL_min:
            self.YL_min = self.YL
        if self.YL > self.YL_max:
            self.YL_max = self.YL

        if self.XR < self.XR_min:
            self.XR_min = self.XR
        if self.XR > self.XR_max:
            self.XR_max = self.XR

        if self.YR < self.YR_min:
            self.YR_min = self.YR
        if self.YR > self.YR_max:
            self.YR_max = self.YR

    def _sample(self, update_minmax=True):
        self.XL = self._read_adc(self.adc_xl)
        self.YL = self._read_adc(self.adc_yl)
        self.XR = self._read_adc(self.adc_xr)
        self.YR = self._read_adc(self.adc_yr)
        self._read_buttons()

        if update_minmax and self.auto_track_minmax:
            self._track_minmax()

    # ---------- Calibration ----------
    def reset_minmax(self):
        self.XL_min = self.YL_min = self.XR_min = self.YR_min = 4095
        self.XL_max = self.YL_max = self.XR_max = self.YR_max = 0
        return self

    def reset_center(self):
        self.XL_center = self.YL_center = self.XR_center = self.YR_center = 0
        return self

    def reset_all(self):
        self.reset_minmax()
        self.reset_center()
        return self

    def set_center(self, xl=None, yl=None, xr=None, yr=None):
        if xl is not None:
            self.XL_center = int(xl)
        if yl is not None:
            self.YL_center = int(yl)
        if xr is not None:
            self.XR_center = int(xr)
        if yr is not None:
            self.YR_center = int(yr)
        return self

    def set_axis_center(self, axis, center):
        axis = self._normalize_axis(axis)
        self._set_attr(axis, "center", center)
        return self

    def set_minmax(
        self,
        xl_min=None, xl_max=None,
        yl_min=None, yl_max=None,
        xr_min=None, xr_max=None,
        yr_min=None, yr_max=None,
    ):
        if xl_min is not None:
            self.XL_min = int(xl_min)
        if xl_max is not None:
            self.XL_max = int(xl_max)
        if yl_min is not None:
            self.YL_min = int(yl_min)
        if yl_max is not None:
            self.YL_max = int(yl_max)
        if xr_min is not None:
            self.XR_min = int(xr_min)
        if xr_max is not None:
            self.XR_max = int(xr_max)
        if yr_min is not None:
            self.YR_min = int(yr_min)
        if yr_max is not None:
            self.YR_max = int(yr_max)
        return self

    def set_axis_minmax(self, axis, raw_min=None, raw_max=None):
        axis = self._normalize_axis(axis)
        if raw_min is not None:
            self._set_attr(axis, "min", raw_min)
        if raw_max is not None:
            self._set_attr(axis, "max", raw_max)
        return self

    def set_axis_calibration(self, axis, center=None, raw_min=None, raw_max=None):
        axis = self._normalize_axis(axis)
        if center is not None:
            self._set_attr(axis, "center", center)
        if raw_min is not None:
            self._set_attr(axis, "min", raw_min)
        if raw_max is not None:
            self._set_attr(axis, "max", raw_max)
        return self

    def set_calibration(
        self,
        xl_center=None, yl_center=None, xr_center=None, yr_center=None,
        xl_min=None, xl_max=None,
        yl_min=None, yl_max=None,
        xr_min=None, xr_max=None,
        yr_min=None, yr_max=None,
    ):
        self.set_center(
            xl=xl_center, yl=yl_center, xr=xr_center, yr=yr_center
        )
        self.set_minmax(
            xl_min=xl_min, xl_max=xl_max,
            yl_min=yl_min, yl_max=yl_max,
            xr_min=xr_min, xr_max=xr_max,
            yr_min=yr_min, yr_max=yr_max,
        )
        return self

    def calibrate_center(self, samples=20, delay_ms=5):
        samples = max(1, int(samples))
        sxl = syl = sxr = syr = 0
        for _ in range(samples):
            sxl += self._read_adc(self.adc_xl)
            syl += self._read_adc(self.adc_yl)
            sxr += self._read_adc(self.adc_xr)
            syr += self._read_adc(self.adc_yr)
            if delay_ms > 0:
                sleep_ms(int(delay_ms))

        self.XL_center = sxl // samples
        self.YL_center = syl // samples
        self.XR_center = sxr // samples
        self.YR_center = syr // samples
        return self.get_center()

    def capture_minmax(self, duration_ms=3000, interval_ms=20):
        self.reset_minmax()
        start = ticks_ms()
        while ticks_diff(ticks_ms(), start) < int(duration_ms):
            self._sample(update_minmax=True)
            if interval_ms > 0:
                sleep_ms(int(interval_ms))
        return self.get_minmax()

    # ---------- Mapping configuration ----------
    def set_deadzone(self, deadzone):
        self.deadzone = int(deadzone)
        return self

    def set_map(
        self,
        xl_min=None, xl_max=None,
        yl_min=None, yl_max=None,
        xr_min=None, xr_max=None,
        yr_min=None, yr_max=None,
    ):
        if xl_min is not None:
            self.XLMinMap = int(xl_min)
        if xl_max is not None:
            self.XLMaxMap = int(xl_max)
        if yl_min is not None:
            self.YLMinMap = int(yl_min)
        if yl_max is not None:
            self.YLMaxMap = int(yl_max)
        if xr_min is not None:
            self.XRMinMap = int(xr_min)
        if xr_max is not None:
            self.XRMaxMap = int(xr_max)
        if yr_min is not None:
            self.YRMinMap = int(yr_min)
        if yr_max is not None:
            self.YRMaxMap = int(yr_max)
        return self

    def set_axis_map(self, axis, out_min=None, out_max=None):
        axis = self._normalize_axis(axis)
        if out_min is not None:
            self._set_map_attr(axis, "Min", out_min)
        if out_max is not None:
            self._set_map_attr(axis, "Max", out_max)
        return self

    def configure_axis(self, axis, center=None, raw_min=None, raw_max=None, out_min=None, out_max=None):
        axis = self._normalize_axis(axis)
        self.set_axis_calibration(axis, center=center, raw_min=raw_min, raw_max=raw_max)
        self.set_axis_map(axis, out_min=out_min, out_max=out_max)
        return self

    def _map_value(self, x, in_min, in_max, out_min, out_max):
        if in_max == in_min:
            return int(out_min)

        if in_min < in_max:
            if x < in_min:
                x = in_min
            elif x > in_max:
                x = in_max
        else:
            if x > in_min:
                x = in_min
            elif x < in_max:
                x = in_max

        return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

    def _axis_map_inverted(self, raw, center, raw_min, raw_max, out_min_map, out_max_map):
        dz = self.deadzone
        hi_start = center + dz
        lo_start = center - dz

        if raw > hi_start:
            if raw_max <= hi_start:
                return 0
            return self._map_value(raw, hi_start, raw_max, 0, -out_max_map)

        if raw < lo_start:
            if raw_min >= lo_start:
                return 0
            return self._map_value(raw, lo_start, raw_min, 0, -out_min_map)

        return 0

    def _axis_map_normal(self, raw, center, raw_min, raw_max, out_min_map, out_max_map):
        dz = self.deadzone
        hi_start = center + dz
        lo_start = center - dz

        if raw > hi_start:
            if raw_max <= hi_start:
                return 0
            return self._map_value(raw, hi_start, raw_max, 0, out_max_map)

        if raw < lo_start:
            if raw_min >= lo_start:
                return 0
            return self._map_value(raw, lo_start, raw_min, 0, out_min_map)

        return 0

    def map_axis(
        self,
        axis,
        raw_value,
        center=None,
        raw_min=None,
        raw_max=None,
        out_min=None,
        out_max=None,
    ):
        axis = self._normalize_axis(axis)

        if center is None:
            center = self._get_attr(axis, "center")
        if raw_min is None:
            raw_min = self._get_attr(axis, "min")
        if raw_max is None:
            raw_max = self._get_attr(axis, "max")
        if out_min is None:
            out_min = self._get_map_attr(axis, "Min")
        if out_max is None:
            out_max = self._get_map_attr(axis, "Max")

        raw_value = int(raw_value)
        if axis in ("XL", "XR"):
            return self._axis_map_inverted(raw_value, center, raw_min, raw_max, out_min, out_max)
        return self._axis_map_normal(raw_value, center, raw_min, raw_max, out_min, out_max)

    def map_values(
        self,
        xl=None, yl=None, xr=None, yr=None,
        xl_center=None, yl_center=None, xr_center=None, yr_center=None,
        xl_raw_min=None, xl_raw_max=None,
        yl_raw_min=None, yl_raw_max=None,
        xr_raw_min=None, xr_raw_max=None,
        yr_raw_min=None, yr_raw_max=None,
        xl_map_min=None, xl_map_max=None,
        yl_map_min=None, yl_map_max=None,
        xr_map_min=None, xr_map_max=None,
        yr_map_min=None, yr_map_max=None,
        swl=None, swr=None,
    ):
        if xl is None:
            xl = self.XL
        if yl is None:
            yl = self.YL
        if xr is None:
            xr = self.XR
        if yr is None:
            yr = self.YR

        if swl is None:
            swl = self.SWL
        if swr is None:
            swr = self.SWR

        return {
            "XL": self.map_axis("XL", xl, xl_center, xl_raw_min, xl_raw_max, xl_map_min, xl_map_max),
            "YL": self.map_axis("YL", yl, yl_center, yl_raw_min, yl_raw_max, yl_map_min, yl_map_max),
            "XR": self.map_axis("XR", xr, xr_center, xr_raw_min, xr_raw_max, xr_map_min, xr_map_max),
            "YR": self.map_axis("YR", yr, yr_center, yr_raw_min, yr_raw_max, yr_map_min, yr_map_max),
            "SWL": int(bool(swl)),
            "SWR": int(bool(swr)),
        }

    def map_current(self):
        return self.map_values()

    # ---------- Public read APIs ----------
    def read_raw(self, update_minmax=True):
        self._sample(update_minmax=update_minmax)
        return self.get_raw()

    def read_mapped(self, update_minmax=True):
        self._sample(update_minmax=update_minmax)
        return self.map_current()

    def update(self, update_minmax=True):
        self._sample(update_minmax=update_minmax)
        return {
            "raw": self.get_raw(),
            "mapped": self.map_current(),
            "center": self.get_center(),
            "minmax": self.get_minmax(),
        }

    def delay(self):
        if self.loop_delay_ms > 0:
            sleep_ms(self.loop_delay_ms)

    # ---------- Getters ----------
    def get_raw(self):
        return {
            "XL": self.XL,
            "YL": self.YL,
            "XR": self.XR,
            "YR": self.YR,
            "SWL": self.SWL,
            "SWR": self.SWR,
        }

    def get_center(self):
        return {
            "XL_center": self.XL_center,
            "YL_center": self.YL_center,
            "XR_center": self.XR_center,
            "YR_center": self.YR_center,
        }

    def get_minmax(self):
        return {
            "XL_min": self.XL_min, "XL_max": self.XL_max,
            "YL_min": self.YL_min, "YL_max": self.YL_max,
            "XR_min": self.XR_min, "XR_max": self.XR_max,
            "YR_min": self.YR_min, "YR_max": self.YR_max,
        }

    def get_map(self):
        return {
            "XLMinMap": self.XLMinMap, "XLMaxMap": self.XLMaxMap,
            "YLMinMap": self.YLMinMap, "YLMaxMap": self.YLMaxMap,
            "XRMinMap": self.XRMinMap, "XRMaxMap": self.XRMaxMap,
            "YRMinMap": self.YRMinMap, "YRMaxMap": self.YRMaxMap,
        }

    def get_axis_config(self, axis):
        axis = self._normalize_axis(axis)
        return {
            "axis": axis,
            "center": self._get_attr(axis, "center"),
            "raw_min": self._get_attr(axis, "min"),
            "raw_max": self._get_attr(axis, "max"),
            "map_min": self._get_map_attr(axis, "Min"),
            "map_max": self._get_map_attr(axis, "Max"),
        }

    # ---------- Save / Load ----------
    def save(self, filename=None):
        filename = filename or self.config_file
        data = {
            "deadzone": self.deadzone,
            "loop_delay_ms": self.loop_delay_ms,
            "use_average": self.use_average,
            "avg_samples": self.avg_samples,
            "auto_track_minmax": self.auto_track_minmax,
            "map": self.get_map(),
            "center": self.get_center(),
            "minmax": self.get_minmax(),
            "pins": {
                "XL": self.xl_pin_num,
                "YL": self.yl_pin_num,
                "XR": self.xr_pin_num,
                "YR": self.yr_pin_num,
                "SWL": self.swl_pin_num,
                "SWR": self.swr_pin_num,
            },
        }
        with open(filename, "w") as f:
            json.dump(data, f)
        return filename

    def load(self, filename=None):
        filename = filename or self.config_file
        with open(filename, "r") as f:
            data = json.load(f)

        self.deadzone = int(data.get("deadzone", self.deadzone))
        self.loop_delay_ms = int(data.get("loop_delay_ms", self.loop_delay_ms))
        self.use_average = bool(data.get("use_average", self.use_average))
        self.avg_samples = max(1, int(data.get("avg_samples", self.avg_samples)))
        self.auto_track_minmax = bool(data.get("auto_track_minmax", self.auto_track_minmax))

        mp = data.get("map", {})
        self.set_map(
            xl_min=mp.get("XLMinMap"),
            xl_max=mp.get("XLMaxMap"),
            yl_min=mp.get("YLMinMap"),
            yl_max=mp.get("YLMaxMap"),
            xr_min=mp.get("XRMinMap"),
            xr_max=mp.get("XRMaxMap"),
            yr_min=mp.get("YRMinMap"),
            yr_max=mp.get("YRMaxMap"),
        )

        ct = data.get("center", {})
        self.set_center(
            xl=ct.get("XL_center"),
            yl=ct.get("YL_center"),
            xr=ct.get("XR_center"),
            yr=ct.get("YR_center"),
        )

        mm = data.get("minmax", {})
        self.set_minmax(
            xl_min=mm.get("XL_min"), xl_max=mm.get("XL_max"),
            yl_min=mm.get("YL_min"), yl_max=mm.get("YL_max"),
            xr_min=mm.get("XR_min"), xr_max=mm.get("XR_max"),
            yr_min=mm.get("YR_min"), yr_max=mm.get("YR_max"),
        )
        return True


if __name__ == "__main__":
    ctl = MeblockController(use_average=True, avg_samples=5)
    ctl.begin()

    # Dat tay cam o giua, roi dat center
    ctl.set_center(xl=2048, yl=2048, xr=2048, yr=2048)

    # Tu dat raw MIN/MAX de map
    ctl.set_minmax(
        xl_min=200, xl_max=3900,
        yl_min=150, yl_max=3920,
        xr_min=180, xr_max=3880,
        yr_min=170, yr_max=3910,
    )

    # Dat output map
    ctl.set_map(
        xl_min=-10, xl_max=10,
        yl_min=-20, yl_max=20,
        xr_min=-15, xr_max=15,
        yr_min=15, yr_max=-15,
    )

    # Hoac cau hinh tung truc mot lan
    ctl.configure_axis("XL", center=2048, raw_min=200, raw_max=3900, out_min=-10, out_max=10)

    while True:
        print("RAW   :", ctl.read_raw())
        print("MAPPED:", ctl.map_current())
        print("-" * 40)
        ctl.delay()
