import math


class LSM303:
    """LSM303 accelerometer and magnetometer driver"""

    ACCEL_ADDR = 0x19
    MAG_ADDR = 0x1E

    # CTRL_REG1_A ODR field (bits 7:4).
    ACCEL_ODR_BITS = {1: 0x1, 10: 0x2, 25: 0x3, 50: 0x4, 100: 0x5, 200: 0x6, 400: 0x7}

    # CRA_REG_M output-rate field (bits 4:2).
    MAG_ODR_BITS = {15: 0x4, 30: 0x5, 75: 0x6, 220: 0x7}

    VALID_AXIS_PAIRS = ("XY", "XZ", "YZ")
    VALID_FORWARD_AXES = ("X", "Y", "Z")
    _AXIS_INDEX = {"X": 0, "Y": 1, "Z": 2}
    _AXIS_UNIT_VECTOR = {
        "X": (1.0, 0.0, 0.0),
        "Y": (0.0, 1.0, 0.0),
        "Z": (0.0, 0.0, 1.0),
    }

    def __init__(
        self,
        i2c,
        mag_declination_deg=0.0,
        azimuth_offset_deg=0.0,
        elevation_offset_deg=0.0,
        invert_azimuth=False,
        invert_elevation=False,
        mag_offset_x=0.0,
        mag_offset_y=0.0,
        mag_offset_z=0.0,
        mag_scale_x=1.0,
        mag_scale_y=1.0,
        mag_scale_z=1.0,
        azimuth_forward_axis="X",
        elevation_axis_pair="XZ",
        filter_strength=0.0,
        motion_threshold_deg=3.0,
        accel_odr_hz=100,
        mag_odr_hz=75,
    ):
        self.i2c = i2c
        self.accel_odr_hz = accel_odr_hz
        self.mag_odr_hz = mag_odr_hz
        self.mag_declination_deg = mag_declination_deg
        self.azimuth_offset_deg = azimuth_offset_deg
        self.elevation_offset_deg = elevation_offset_deg
        self.invert_azimuth = invert_azimuth
        self.invert_elevation = invert_elevation

        self.mag_offset_x = mag_offset_x
        self.mag_offset_y = mag_offset_y
        self.mag_offset_z = mag_offset_z
        self.mag_scale_x = mag_scale_x
        self.mag_scale_y = mag_scale_y
        self.mag_scale_z = mag_scale_z

        self.azimuth_forward_axis = (
            azimuth_forward_axis if azimuth_forward_axis in self.VALID_FORWARD_AXES else "X"
        )
        self.elevation_axis_pair = (
            elevation_axis_pair if elevation_axis_pair in self.VALID_AXIS_PAIRS else "XZ"
        )
        self.filter_strength = max(0.0, min(0.98, float(filter_strength)))
        self.motion_threshold_deg = max(0.0, float(motion_threshold_deg))

        self._smoothed_az_vec = None
        self._smoothed_el = None
        self._last_raw_heading = 0.0

        # Rolling 3-sample window for the median filter. Holds three (x, y,
        # z) tuples per sensor.
        self._accel_window = None
        self._mag_window = None

        self._mag_calibrating = False
        self._mag_cal_min = None
        self._mag_cal_max = None

        self.ready = False

    def begin(self):
        """Runs the I2C setup and raises OSError if the sensor does not
        respond. Kept out of __init__ so construction succeeds with no
        sensor attached, and the caller can retry."""
        self._init_sensor()
        self.ready = True

    def _init_sensor(self):
        # CTRL_REG1_A: ODR in bits 7:4, normal power mode, XYZ enabled.
        accel_bits = self.ACCEL_ODR_BITS.get(self.accel_odr_hz, 0x5)
        self.i2c.writeto_mem(self.ACCEL_ADDR, 0x20, bytes([(accel_bits << 4) | 0x07]))

        # CTRL_REG4_A: BDU=1, +-2g, high resolution. BDU stops a read from
        # mixing bytes from two samples.
        self.i2c.writeto_mem(self.ACCEL_ADDR, 0x23, bytes([0x88]))

        # CRA_REG_M: output rate in bits 4:2, temperature sensor off.
        mag_bits = self.MAG_ODR_BITS.get(self.mag_odr_hz, 0x6)
        self.i2c.writeto_mem(self.MAG_ADDR, 0x00, bytes([mag_bits << 2]))
        # CRB_REG_M: +-1.3 gauss, the most sensitive gain.
        self.i2c.writeto_mem(self.MAG_ADDR, 0x01, bytes([0x20]))
        # MR_REG_M: continuous-conversion mode.
        self.i2c.writeto_mem(self.MAG_ADDR, 0x02, bytes([0x00]))

    @staticmethod
    def _to_signed_16(msb, lsb):
        value = (msb << 8) | lsb
        if value & 0x8000:
            value -= 0x10000
        return value

    @staticmethod
    def _median3(a, b, c):
        """Returns the middle of three values, using no branch."""
        return max(min(a, b), min(max(a, b), c))

    def _despike(self, window, sample):
        """Takes the median of the last three readings for each axis, to
        remove single-sample outliers from PWM noise. Returns the updated
        window and the filtered sample."""
        if window is None:
            window = [sample, sample, sample]
        else:
            window[0] = window[1]
            window[1] = window[2]
            window[2] = sample
        a, b, c = window
        return window, (
            self._median3(a[0], b[0], c[0]),
            self._median3(a[1], b[1], c[1]),
            self._median3(a[2], b[2], c[2]),
        )

    def read_accel_raw(self):
        if not self.ready:
            raise OSError(110, "LSM303 not initialized")
        # Reads all axes in one burst, starting at OUT_X_L_A (0x28) with
        # auto-increment set.
        data = self.i2c.readfrom_mem(self.ACCEL_ADDR, 0x28 | 0x80, 6)
        x = self._to_signed_16(data[1], data[0]) >> 4
        y = self._to_signed_16(data[3], data[2]) >> 4
        z = self._to_signed_16(data[5], data[4]) >> 4
        self._accel_window, sample = self._despike(self._accel_window, (x, y, z))
        return sample

    def read_mag_raw(self):
        if not self.ready:
            raise OSError(110, "LSM303 not initialized")
        # LSM303DLHC register order is X, Z, Y (MSB first).
        data = self.i2c.readfrom_mem(self.MAG_ADDR, 0x03, 6)
        x = self._to_signed_16(data[0], data[1])
        z = self._to_signed_16(data[2], data[3])
        y = self._to_signed_16(data[4], data[5])

        self._mag_window, (x, y, z) = self._despike(self._mag_window, (x, y, z))

        if self._mag_calibrating:
            self._track_mag_calibration(x, y, z)

        return x, y, z

    @staticmethod
    def _normalize_deg(angle):
        while angle < 0.0:
            angle += 360.0
        while angle >= 360.0:
            angle -= 360.0
        return angle

    @classmethod
    def _pair_angle_deg(cls, values, pair):
        a = values[cls._AXIS_INDEX[pair[0]]]
        b = values[cls._AXIS_INDEX[pair[1]]]
        return math.degrees(math.atan2(b, a))

    @staticmethod
    def _dot3(a, b):
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    @staticmethod
    def _cross3(a, b):
        return (
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        )

    @staticmethod
    def _normalize3(v):
        mag = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        if mag < 1e-6:
            return None
        return (v[0] / mag, v[1] / mag, v[2] / mag)

    def _tilt_compensated_heading_deg(self, accel, mag):
        up = self._normalize3(accel)
        if up is None:
            return None  # no usable gravity signal

        forward_body = self._AXIS_UNIT_VECTOR[self.azimuth_forward_axis]
        d = self._dot3(forward_body, up)
        # Projects the forward axis onto the horizontal plane. This keeps
        # azimuth correct at any tilt angle.
        forward_h = self._normalize3(
            (
                forward_body[0] - d * up[0],
                forward_body[1] - d * up[1],
                forward_body[2] - d * up[2],
            )
        )
        if forward_h is None:
            # The forward axis points straight along gravity here, so
            # azimuth is undefined.
            return None

        right_h = self._cross3(up, forward_h)

        m_forward = self._dot3(mag, forward_h)
        m_right = self._dot3(mag, right_h)
        return math.degrees(math.atan2(m_right, m_forward))

    def _effective_alpha(self, deviation_deg):
        """Returns the smoothing factor for this sample. It is full strength
        for small deviations, and drops to zero by motion_threshold_deg
        following a quadratic curve."""
        alpha = self.filter_strength
        if alpha <= 0.0:
            return 0.0
        threshold = self.motion_threshold_deg
        if threshold <= 0.0:
            return alpha
        t = abs(deviation_deg) / threshold
        if t >= 1.0:
            return 0.0
        return alpha * (1.0 - t * t)

    def _apply_smoothing(self, heading, elevation):
        if self.filter_strength <= 0.0:
            self._smoothed_az_vec = None
            self._smoothed_el = None
            return heading, elevation

        # Azimuth wraps at 0/360, so this averages unit vectors instead of
        # the angle itself.
        rad = math.radians(heading)
        x, y = math.cos(rad), math.sin(rad)
        if self._smoothed_az_vec is None:
            self._smoothed_az_vec = [x, y]
        else:
            previous = math.degrees(
                math.atan2(self._smoothed_az_vec[1], self._smoothed_az_vec[0])
            )
            # Uses the shortest signed distance, so crossing the 0/360 seam
            # reads as a few degrees, not 360.
            deviation = (heading - previous + 540.0) % 360.0 - 180.0
            alpha = self._effective_alpha(deviation)
            self._smoothed_az_vec[0] = alpha * self._smoothed_az_vec[0] + (1.0 - alpha) * x
            self._smoothed_az_vec[1] = alpha * self._smoothed_az_vec[1] + (1.0 - alpha) * y
        heading = self._normalize_deg(
            math.degrees(math.atan2(self._smoothed_az_vec[1], self._smoothed_az_vec[0]))
        )

        if self._smoothed_el is None:
            self._smoothed_el = elevation
        else:
            alpha = self._effective_alpha(elevation - self._smoothed_el)
            self._smoothed_el = alpha * self._smoothed_el + (1.0 - alpha) * elevation
        elevation = self._smoothed_el

        return heading, elevation

    def read_orientation(self):
        ax, ay, az = self.read_accel_raw()
        mx, my, mz = self.read_mag_raw()

        mx = (mx - self.mag_offset_x) * self.mag_scale_x
        my = (my - self.mag_offset_y) * self.mag_scale_y
        mz = (mz - self.mag_offset_z) * self.mag_scale_z

        raw_heading = self._tilt_compensated_heading_deg((ax, ay, az), (mx, my, mz))
        if raw_heading is None:
            raw_heading = self._last_raw_heading
        else:
            self._last_raw_heading = raw_heading

        heading = raw_heading + self.mag_declination_deg + self.azimuth_offset_deg

        elevation = self._pair_angle_deg((ax, ay, az), self.elevation_axis_pair)
        elevation += self.elevation_offset_deg

        if self.invert_azimuth:
            heading = -heading
        if self.invert_elevation:
            elevation = -elevation

        heading = self._normalize_deg(heading)

        heading, elevation = self._apply_smoothing(heading, elevation)

        return heading, elevation

    # -- Declination/offset/invert/axis/filter calibration ----------------

    def get_calibration(self):
        return {
            "mag_declination_deg": self.mag_declination_deg,
            "azimuth_offset_deg": self.azimuth_offset_deg,
            "elevation_offset_deg": self.elevation_offset_deg,
            "invert_azimuth": self.invert_azimuth,
            "invert_elevation": self.invert_elevation,
            "mag_offset_x": self.mag_offset_x,
            "mag_offset_y": self.mag_offset_y,
            "mag_offset_z": self.mag_offset_z,
            "mag_scale_x": self.mag_scale_x,
            "mag_scale_y": self.mag_scale_y,
            "mag_scale_z": self.mag_scale_z,
            "azimuth_forward_axis": self.azimuth_forward_axis,
            "elevation_axis_pair": self.elevation_axis_pair,
            "filter_strength": self.filter_strength,
            "motion_threshold_deg": self.motion_threshold_deg,
        }

    def set_calibration(
        self,
        mag_declination_deg=None,
        azimuth_offset_deg=None,
        elevation_offset_deg=None,
        invert_azimuth=None,
        invert_elevation=None,
        azimuth_forward_axis=None,
        elevation_axis_pair=None,
        filter_strength=None,
        motion_threshold_deg=None,
    ):
        if mag_declination_deg is not None:
            self.mag_declination_deg = float(mag_declination_deg)
        if azimuth_offset_deg is not None:
            self.azimuth_offset_deg = float(azimuth_offset_deg)
        if elevation_offset_deg is not None:
            self.elevation_offset_deg = float(elevation_offset_deg)
        if invert_azimuth is not None:
            self.invert_azimuth = bool(invert_azimuth)
        if invert_elevation is not None:
            self.invert_elevation = bool(invert_elevation)
        if azimuth_forward_axis is not None and azimuth_forward_axis in self.VALID_FORWARD_AXES:
            self.azimuth_forward_axis = azimuth_forward_axis
        if elevation_axis_pair is not None and elevation_axis_pair in self.VALID_AXIS_PAIRS:
            self.elevation_axis_pair = elevation_axis_pair
        if filter_strength is not None:
            self.filter_strength = max(0.0, min(0.98, float(filter_strength)))
        if motion_threshold_deg is not None:
            self.motion_threshold_deg = max(0.0, float(motion_threshold_deg))

        # A calibration change can shift what the smoothed value means, so
        # the filter should restart.
        self._smoothed_az_vec = None
        self._smoothed_el = None

    # -- Magnetometer hard-iron and soft-iron calibration -------------------
    # Samples raw readings while the user sweeps the sensor. The midpoint of
    # each axis's min and max gives its hard-iron offset. The ratio of its
    # swing to the average swing gives its soft-iron scale factor.

    MIN_CAL_RANGE = 50  # raw counts, an axis below this is unswept

    def start_mag_calibration(self):
        self._mag_calibrating = True
        self._mag_cal_min = None
        self._mag_cal_max = None

    def cancel_mag_calibration(self):
        self._mag_calibrating = False
        self._mag_cal_min = None
        self._mag_cal_max = None

    def _track_mag_calibration(self, x, y, z):
        if self._mag_cal_min is None:
            self._mag_cal_min = [x, y, z]
            self._mag_cal_max = [x, y, z]
            return
        self._mag_cal_min[0] = min(self._mag_cal_min[0], x)
        self._mag_cal_min[1] = min(self._mag_cal_min[1], y)
        self._mag_cal_min[2] = min(self._mag_cal_min[2], z)
        self._mag_cal_max[0] = max(self._mag_cal_max[0], x)
        self._mag_cal_max[1] = max(self._mag_cal_max[1], y)
        self._mag_cal_max[2] = max(self._mag_cal_max[2], z)

    def mag_calibration_progress(self):
        if self._mag_cal_min is None:
            return {
                "sampling": self._mag_calibrating,
                "range_x": 0,
                "range_y": 0,
                "range_z": 0,
            }
        return {
            "sampling": self._mag_calibrating,
            "range_x": self._mag_cal_max[0] - self._mag_cal_min[0],
            "range_y": self._mag_cal_max[1] - self._mag_cal_min[1],
            "range_z": self._mag_cal_max[2] - self._mag_cal_min[2],
        }

    def finish_mag_calibration(self):
        if self._mag_cal_min is None:
            self._mag_calibrating = False
            raise ValueError("no calibration samples collected")

        cal_min = self._mag_cal_min
        cal_max = self._mag_cal_max

        self.mag_offset_x = (cal_max[0] + cal_min[0]) / 2.0
        self.mag_offset_y = (cal_max[1] + cal_min[1]) / 2.0
        self.mag_offset_z = (cal_max[2] + cal_min[2]) / 2.0

        range_x = cal_max[0] - cal_min[0]
        range_y = cal_max[1] - cal_min[1]
        range_z = cal_max[2] - cal_min[2]

        swept_ranges = [r for r in (range_x, range_y, range_z) if r >= self.MIN_CAL_RANGE]
        if swept_ranges:
            avg_range = sum(swept_ranges) / len(swept_ranges)
            self.mag_scale_x = avg_range / range_x if range_x >= self.MIN_CAL_RANGE else 1.0
            self.mag_scale_y = avg_range / range_y if range_y >= self.MIN_CAL_RANGE else 1.0
            self.mag_scale_z = avg_range / range_z if range_z >= self.MIN_CAL_RANGE else 1.0
        else:
            self.mag_scale_x = self.mag_scale_y = self.mag_scale_z = 1.0

        self._mag_calibrating = False
        self._mag_cal_min = None
        self._mag_cal_max = None

        return {
            "mag_offset_x": self.mag_offset_x,
            "mag_offset_y": self.mag_offset_y,
            "mag_offset_z": self.mag_offset_z,
            "mag_scale_x": self.mag_scale_x,
            "mag_scale_y": self.mag_scale_y,
            "mag_scale_z": self.mag_scale_z,
        }
