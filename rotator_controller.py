import time
from machine import PWM, Pin


class RotatorController:
    """Closed-loop azimuth and elevation control using four PWM outputs that
    drive an H-bridge."""

    DIR_UP = 2
    DIR_DOWN = 4
    DIR_LEFT = 8
    DIR_RIGHT = 16

    # Largest gap, as a multiple of control_update_ms, that one cycle may
    # use for its acceleration step
    MAX_RAMP_INTERVAL_FACTOR = 4

    # Motion parameters editable via the web UI or /api/tuning.
    # Bounds apply on every write.
    TUNING_LIMITS = {
        "position_tolerance_deg": (0.1, 30.0),
        "approach_speed_percent": (1.0, 100.0),
        "speed_ramp_deg": (1.0, 180.0),
        "speed_accel_percent_per_sec": (1.0, 10000.0),
    }

    def __init__(
        self,
        sensor,
        az_plus_pin,
        az_minus_pin,
        el_plus_pin,
        el_minus_pin,
        pwm_freq_hz,
        position_tolerance_deg,
        control_update_ms,
        min_elevation_deg,
        max_elevation_deg,
        park_azimuth_deg,
        park_elevation_deg,
        approach_speed_percent=15.0,
        speed_ramp_deg=15.0,
        speed_accel_percent_per_sec=200.0,
    ):
        self.sensor = sensor

        self.az_plus_pwm = PWM(Pin(az_plus_pin))
        self.az_minus_pwm = PWM(Pin(az_minus_pin))
        self.el_plus_pwm = PWM(Pin(el_plus_pin))
        self.el_minus_pwm = PWM(Pin(el_minus_pin))
        for pwm in (self.az_plus_pwm, self.az_minus_pwm, self.el_plus_pwm, self.el_minus_pwm):
            pwm.freq(pwm_freq_hz)

        self.position_tolerance_deg = position_tolerance_deg
        self.control_update_ms = control_update_ms
        self.min_elevation_deg = min_elevation_deg
        self.max_elevation_deg = max_elevation_deg

        self.park_azimuth_deg = park_azimuth_deg
        self.park_elevation_deg = self._clamp_elevation(park_elevation_deg)

        self.approach_speed_percent = approach_speed_percent
        self.speed_ramp_deg = speed_ramp_deg
        self.speed_accel_percent_per_sec = speed_accel_percent_per_sec

        # Speed change allowed this cycle
        self._max_speed_change = speed_accel_percent_per_sec * (
            control_update_ms / 1000.0
        )

        self.current_az = 0.0
        self.current_el = 0.0

        self.target_az = None
        self.target_el = None

        self.manual_direction = None
        self.manual_speed = 0

        self._have_position = False

        self.last_error = None

        self._az_speed = 0.0
        self._el_speed = 0.0

        self._last_control_ms = time.ticks_ms()

        self.stop()

    @staticmethod
    def _wrap_360(value):
        while value < 0.0:
            value += 360.0
        while value >= 360.0:
            value -= 360.0
        return value

    @staticmethod
    def _shortest_az_error(target, current):
        error = (target - current + 540.0) % 360.0 - 180.0
        return error

    def _clamp_elevation(self, elevation):
        if elevation < self.min_elevation_deg:
            return self.min_elevation_deg
        if elevation > self.max_elevation_deg:
            return self.max_elevation_deg
        return elevation

    @staticmethod
    def _speed_to_duty_u16(speed_percent):
        speed_percent = max(0, min(100, int(speed_percent)))
        return int((speed_percent * 65535) / 100)

    def _set_axis_pwm(self, plus_pwm, minus_pwm, signed_speed_percent):
        duty = self._speed_to_duty_u16(abs(signed_speed_percent))
        if signed_speed_percent > 0:
            plus_pwm.duty_u16(duty)
            minus_pwm.duty_u16(0)
        elif signed_speed_percent < 0:
            plus_pwm.duty_u16(0)
            minus_pwm.duty_u16(duty)
        else:
            plus_pwm.duty_u16(0)
            minus_pwm.duty_u16(0)

    @staticmethod
    def _stop_axis_pwm(plus_pwm, minus_pwm):
        plus_pwm.duty_u16(0)
        minus_pwm.duty_u16(0)

    def _refresh_position(self):
        self.current_az, self.current_el = self.sensor.read_orientation()
        self._have_position = True

    def get_position(self):
        """Returns the latest position from the control loop's last
        sample."""
        if not self._have_position:
            self._refresh_position()
        return self.current_az, self.current_el

    def set_target(self, azimuth_deg, elevation_deg):
        self.target_az = self._wrap_360(float(azimuth_deg))
        self.target_el = self._clamp_elevation(float(elevation_deg))
        self.manual_direction = None
        self.manual_speed = 0

    def move(self, direction, speed_percent):
        self.manual_direction = int(direction)
        self.manual_speed = max(0, min(100, int(speed_percent)))
        self.target_az = None
        self.target_el = None

    def stop(self):
        self.manual_direction = None
        self.manual_speed = 0
        self.target_az = None
        self.target_el = None
        self._az_speed = 0.0
        self._el_speed = 0.0
        self._stop_axis_pwm(self.az_plus_pwm, self.az_minus_pwm)
        self._stop_axis_pwm(self.el_plus_pwm, self.el_minus_pwm)

    def park(self):
        self.set_target(self.park_azimuth_deg, self.park_elevation_deg)

    def reset(self):
        self.stop()

    def get_tuning(self):
        return {name: getattr(self, name) for name in self.TUNING_LIMITS}

    @classmethod
    def _validate_tuning(cls, values):
        if values["speed_ramp_deg"] <= values["position_tolerance_deg"]:
            raise ValueError(
                "Slow-down distance ({:.2f}deg) must be greater than the arrival"
                " window ({:.2f}deg), or there is no room to slow down in".format(
                    values["speed_ramp_deg"], values["position_tolerance_deg"]
                )
            )

    def set_tuning(self, **values):
        """Validates and applies motion tuning"""
        candidate = self.get_tuning()

        for name, value in values.items():
            if value is None:
                continue
            if name not in self.TUNING_LIMITS:
                raise ValueError("Unknown tuning parameter: {}".format(name))
            try:
                value = float(value)
            except (TypeError, ValueError):
                raise ValueError("{} must be a number".format(name))
            low, high = self.TUNING_LIMITS[name]
            if value < low or value > high:
                raise ValueError(
                    "{} must be between {} and {} (got {})".format(name, low, high, value)
                )
            candidate[name] = value

        self._validate_tuning(candidate)

        for name, value in candidate.items():
            setattr(self, name, value)
        return self.get_tuning()

    def _ramp_toward(self, current_speed, desired_speed):
        max_change = self._max_speed_change
        if desired_speed > current_speed:
            return min(desired_speed, current_speed + max_change)
        if desired_speed < current_speed:
            return max(desired_speed, current_speed - max_change)
        return current_speed

    def _speed_for_error(self, error_deg):
        """Returns the signed desired speed for a signed position error.
        Speed stays full beyond speed_ramp_deg, tapers linearly to
        approach_speed_percent at the tolerance boundary, and drops to zero
        inside it."""
        abs_error = abs(error_deg)

        if abs_error <= self.position_tolerance_deg:
            magnitude = 0.0
        else:
            span = self.speed_ramp_deg - self.position_tolerance_deg
            if abs_error >= self.speed_ramp_deg or span <= 0:
                magnitude = 100.0
            else:
                frac = (abs_error - self.position_tolerance_deg) / span
                magnitude = (
                    self.approach_speed_percent
                    + frac * (100.0 - self.approach_speed_percent)
                )
        return magnitude if error_deg >= 0 else -magnitude

    def _apply_manual_motion(self):
        az_desired = 0.0
        el_desired = 0.0

        if self.manual_direction == self.DIR_RIGHT:
            az_desired = float(self.manual_speed)
        elif self.manual_direction == self.DIR_LEFT:
            az_desired = -float(self.manual_speed)
        elif self.manual_direction == self.DIR_UP:
            el_desired = float(self.manual_speed)
        elif self.manual_direction == self.DIR_DOWN:
            el_desired = -float(self.manual_speed)

        self._az_speed = self._ramp_toward(self._az_speed, az_desired)
        self._el_speed = self._ramp_toward(self._el_speed, el_desired)

        self._set_axis_pwm(self.az_plus_pwm, self.az_minus_pwm, self._az_speed)
        self._set_axis_pwm(self.el_plus_pwm, self.el_minus_pwm, self._el_speed)

    def _apply_target_motion(self):
        az_error = self._shortest_az_error(self.target_az, self.current_az)
        el_error = self.target_el - self.current_el

        az_reached = abs(az_error) <= self.position_tolerance_deg
        el_reached = abs(el_error) <= self.position_tolerance_deg

        az_desired = 0.0 if az_reached else self._speed_for_error(az_error)
        el_desired = 0.0 if el_reached else self._speed_for_error(el_error)

        self._az_speed = self._ramp_toward(self._az_speed, az_desired)
        self._el_speed = self._ramp_toward(self._el_speed, el_desired)

        self._set_axis_pwm(self.az_plus_pwm, self.az_minus_pwm, self._az_speed)
        self._set_axis_pwm(self.el_plus_pwm, self.el_minus_pwm, self._el_speed)

        if az_reached and el_reached:
            self.target_az = None
            self.target_el = None
            self._az_speed = 0.0
            self._el_speed = 0.0
            self._stop_axis_pwm(self.az_plus_pwm, self.az_minus_pwm)
            self._stop_axis_pwm(self.el_plus_pwm, self.el_minus_pwm)

    def update(self):
        """Returns True if this call did work, False if control_update_ms
        has not elapsed yet. Callers tracking success state should treat a
        no-op as neither success nor failure."""
        now = time.ticks_ms()
        elapsed_ms = time.ticks_diff(now, self._last_control_ms)
        if elapsed_ms < self.control_update_ms:
            return False
        self._last_control_ms = now

        max_interval_ms = self.control_update_ms * self.MAX_RAMP_INTERVAL_FACTOR
        if elapsed_ms > max_interval_ms:
            elapsed_ms = max_interval_ms
        self._max_speed_change = self.speed_accel_percent_per_sec * (
            elapsed_ms / 1000.0
        )

        try:
            self._refresh_position()
        except Exception:
            self._have_position = False
            raise

        if self.manual_direction is not None:
            self._apply_manual_motion()
            return True

        if self.target_az is not None and self.target_el is not None:
            self._apply_target_motion()
            return True

        self._az_speed = 0.0
        self._el_speed = 0.0
        self._stop_axis_pwm(self.az_plus_pwm, self.az_minus_pwm)
        self._stop_axis_pwm(self.el_plus_pwm, self.el_minus_pwm)
        return True
