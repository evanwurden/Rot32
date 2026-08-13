import time
from machine import I2C, WDT, Pin

import config
import store
from lsm303 import LSM303
from rotator_controller import RotatorController
from rotctld_server import RotctldServer
from web_server import WebServer
from wifi_manager import attempt_reconnect, connect_wifi, get_wlan, is_connected

SENSOR_RETRY_MS = 5000

# Rate limits for messages that would otherwise repeat every loop tick and
# fill up the log buffer.
ACCEPT_ERROR_LOG_MS = 5000
LOG_REPEAT_MS = 30000

# How often to retry a listening socket that is not up yet.
SERVER_REBIND_RETRY_MS = 10000

# Recent status/diagnostic messages, served at GET /api/log.
LOG_BUFFER_SIZE = 40
log_buffer = []

# Runtime copy of config.SERIAL_LOGGING_ENABLED. It turns off for good the
# first time a console write raises.
_serial_logging = config.SERIAL_LOGGING_ENABLED

_repeat_state = {}


def _default_tuning():
    """Returns config.py's motion-tuning defaults, the baseline a saved
    tuning.json overrides. Rebuilt on each call so a caller cannot mutate
    it."""
    return {
        "position_tolerance_deg": float(config.POSITION_TOLERANCE_DEG),
        "approach_speed_percent": float(config.APPROACH_SPEED_PERCENT),
        "speed_ramp_deg": float(config.SPEED_RAMP_DEG),
        "speed_accel_percent_per_sec": float(config.SPEED_ACCEL_PERCENT_PER_SEC),
    }


def _sensor_settings(saved):
    """Returns LSM303 settings from a saved calibration dict. Falls back to
    config.py for any field the dict does not have."""
    return {
        "mag_declination_deg": saved.get("mag_declination_deg", config.MAG_DECLINATION_DEG),
        "azimuth_offset_deg": saved.get("azimuth_offset_deg", config.AZIMUTH_OFFSET_DEG),
        "elevation_offset_deg": saved.get("elevation_offset_deg", config.ELEVATION_OFFSET_DEG),
        "invert_azimuth": saved.get("invert_azimuth", config.INVERT_AZIMUTH),
        "invert_elevation": saved.get("invert_elevation", config.INVERT_ELEVATION),
        "mag_offset_x": saved.get("mag_offset_x", 0.0),
        "mag_offset_y": saved.get("mag_offset_y", 0.0),
        "mag_offset_z": saved.get("mag_offset_z", 0.0),
        "mag_scale_x": saved.get("mag_scale_x", 1.0),
        "mag_scale_y": saved.get("mag_scale_y", 1.0),
        "mag_scale_z": saved.get("mag_scale_z", 1.0),
        "azimuth_forward_axis": saved.get("azimuth_forward_axis", config.AZIMUTH_FORWARD_AXIS),
        "elevation_axis_pair": saved.get("elevation_axis_pair", config.ELEVATION_AXIS_PAIR),
        "filter_strength": saved.get("filter_strength", config.FILTER_STRENGTH),
        "motion_threshold_deg": saved.get("motion_threshold_deg", config.MOTION_THRESHOLD_DEG),
        "accel_odr_hz": config.ACCEL_ODR_HZ,
        "mag_odr_hz": config.MAG_ODR_HZ,
    }


def _log(*args):
    """Appends a message to the ring buffer, and echoes it to the console if
    serial logging is on. A console write that raises turns off serial
    output for the rest of the run."""
    global _serial_logging

    msg = " ".join(str(a) for a in args)
    log_buffer.append(msg)
    if len(log_buffer) > LOG_BUFFER_SIZE:
        del log_buffer[0]
    if _serial_logging:
        try:
            print(msg)
        except Exception:
            _serial_logging = False


def _log_repeating(key, msg):
    """Logs msg, but skips an identical consecutive messages for
    LOG_REPEAT_MS."""
    now = time.ticks_ms()
    previous = _repeat_state.get(key)
    if (
        previous is not None
        and previous[0] == msg
        and time.ticks_diff(now, previous[1]) < LOG_REPEAT_MS
    ):
        return
    _repeat_state[key] = (msg, now)
    _log(msg)


class _UnavailableController:
    """Stand-in used when RotatorController fails to construct."""
    target_az = None
    target_el = None
    manual_direction = None
    manual_speed = 0
    min_elevation_deg = 0.0
    max_elevation_deg = 0.0
    last_error = None

    # Borrowed from the real controller, so /api/tuning can still show its
    # bounds.
    TUNING_LIMITS = RotatorController.TUNING_LIMITS

    def __init__(self, reason):
        self.last_error = "Motor controller unavailable: {}".format(reason)

    def get_position(self):
        return None, None

    def update(self):
        return False

    def stop(self):
        pass

    def reset(self):
        pass

    def move(self, direction, speed_percent):
        raise OSError(self.last_error)

    def set_target(self, azimuth_deg, elevation_deg):
        raise OSError(self.last_error)

    def park(self):
        raise OSError(self.last_error)

    def get_tuning(self):
        return _default_tuning()

    def set_tuning(self, **values):
        raise OSError(self.last_error)


def main():
    try:
        i2c = I2C(
            0,
            sda=Pin(config.I2C_SDA_PIN),
            scl=Pin(config.I2C_SCL_PIN),
            freq=config.I2C_FREQ_HZ,
        )
    except Exception as e:
        i2c = None
        _log("I2C bus init failed, sensor will stay unavailable:", repr(e))

    saved_cal = store.load(store.CALIBRATION_FILE)
    saved_tuning = store.load(store.TUNING_FILE)

    # Retries with an empty calibration if the saved file itself is broken
    try:
        sensor = LSM303(i2c=i2c, **_sensor_settings(saved_cal))
    except Exception as e:
        _log("Saved calibration.json invalid, falling back to config.py defaults:", repr(e))
        sensor = LSM303(i2c=i2c, **_sensor_settings({}))

    # If the sensor does not initialize, log it
    last_sensor_attempt_ms = time.ticks_ms()
    try:
        sensor.begin()
        _log("LSM303 sensor ready")
    except Exception as e:
        _log("LSM303 sensor init failed (retrying):", repr(e))

    try:
        controller = RotatorController(
            sensor=sensor,
            az_plus_pin=config.AZ_PLUS_PIN,
            az_minus_pin=config.AZ_MINUS_PIN,
            el_plus_pin=config.EL_PLUS_PIN,
            el_minus_pin=config.EL_MINUS_PIN,
            pwm_freq_hz=config.PWM_FREQUENCY_HZ,
            position_tolerance_deg=config.POSITION_TOLERANCE_DEG,
            control_update_ms=config.CONTROL_UPDATE_MS,
            min_elevation_deg=config.MIN_ELEVATION_DEG,
            max_elevation_deg=config.MAX_ELEVATION_DEG,
            park_azimuth_deg=config.PARK_AZIMUTH_DEG,
            park_elevation_deg=config.PARK_ELEVATION_DEG,
            approach_speed_percent=config.APPROACH_SPEED_PERCENT,
            speed_ramp_deg=config.SPEED_RAMP_DEG,
            speed_accel_percent_per_sec=config.SPEED_ACCEL_PERCENT_PER_SEC,
        )
    except Exception as e:
        _log("Motor controller init failed.", repr(e))
        controller = _UnavailableController(repr(e))

    if saved_tuning and not isinstance(controller, _UnavailableController):
        try:
            controller.set_tuning(**saved_tuning)
            _log("motion tuning loaded from tuning.json")
        except Exception as e:
            _log("saved tuning.json invalid, using config.py defaults:", repr(e))

    try:
        wlan = get_wlan(
            power_save_disabled=config.WIFI_POWER_SAVE_DISABLED,
            txpower_dbm=config.WIFI_TX_POWER_DBM,
        )
    except Exception as e:
        wlan = None
        _log("Wi-Fi hardware init failed:", repr(e))
    try:
        ip = connect_wifi(
            config.WIFI_SSID,
            config.WIFI_PASSWORD,
            timeout_s=config.WIFI_CONNECT_TIMEOUT_S,
            power_save_disabled=config.WIFI_POWER_SAVE_DISABLED,
            txpower_dbm=config.WIFI_TX_POWER_DBM,
        )
        _log("Wi-Fi connected, IP:", ip)
    except Exception as e:
        _log("Wi-Fi connect failed at boot (will keep retrying):", repr(e))

    server = RotctldServer(
        controller=controller,
        host=config.ROTCTLD_HOST,
        port=config.ROTCTLD_PORT,
        client_idle_ms=config.ROTCTLD_CLIENT_IDLE_MS,
        max_clients=config.MAX_CLIENTS_PER_SERVER,
    )
    try:
        server.start()
        _log("rotctld server listening on {}:{}".format(config.ROTCTLD_HOST, config.ROTCTLD_PORT))
    except Exception as e:
        _log("rotctld server failed to start:", repr(e))

    web_server = WebServer(
        controller=controller,
        sensor=sensor,
        store=store,
        default_tuning=_default_tuning(),
        wlan=wlan,
        log_buffer=log_buffer,
        host=config.WEB_HOST,
        port=config.WEB_PORT,
        client_idle_ms=config.WEB_CLIENT_IDLE_MS,
        max_clients=config.MAX_CLIENTS_PER_SERVER,
    )
    try:
        web_server.start()
        _log("web UI listening on {}:{}".format(config.WEB_HOST, config.WEB_PORT))
    except Exception as e:
        _log("web server failed to start:", repr(e))

    last_wifi_attempt_ms = time.ticks_ms()
    wifi_was_connected = is_connected(wlan)
    wifi_failures = 0
    last_accept_log_ms = time.ticks_ms()
    last_bind_retry_ms = time.ticks_ms()
    bind_failure_logged = set()

    def _rebind(srv, name):
        """Reopens a server's listening socket after Wi-Fi comes back."""
        try:
            srv.restart()
            _log(name, "rebound after Wi-Fi reconnect")
        except Exception as e:
            _log(name, "failed to rebind after Wi-Fi reconnect:", repr(e))

    wdt = None
    if config.WDT_ENABLED:
        try:
            wdt = WDT(timeout=config.WDT_TIMEOUT_MS)
            _log("watchdog armed ({} ms)".format(config.WDT_TIMEOUT_MS))
        except Exception as e:
            _log("watchdog unavailable (continuing without it):", repr(e))

    try:
        while True:
            if wdt is not None:
                wdt.feed()

            now = time.ticks_ms()

            if wlan is not None and is_connected(wlan):
                if not wifi_was_connected:
                    _log("Wi-Fi connected, IP:", wlan.ifconfig()[0])
                    # A socket bound before the interface went down does not
                    # reliably keep accepting once the interface is rebuilt.
                    _rebind(server, "rotctld server")
                    _rebind(web_server, "web server")
                wifi_was_connected = True
                wifi_failures = 0
            else:
                if wifi_was_connected:
                    _log("Wi-Fi disconnected")
                wifi_was_connected = False
                if wlan is not None and time.ticks_diff(now, last_wifi_attempt_ms) >= config.WIFI_RECONNECT_MS:
                    last_wifi_attempt_ms = now
                    wifi_failures = attempt_reconnect(
                        wlan,
                        config.WIFI_SSID,
                        config.WIFI_PASSWORD,
                        consecutive_failures=wifi_failures,
                        hard_reset_after=config.WIFI_HARD_RESET_AFTER_ATTEMPTS,
                        power_save_disabled=config.WIFI_POWER_SAVE_DISABLED,
                        txpower_dbm=config.WIFI_TX_POWER_DBM,
                    )
                    _log("Wi-Fi reconnect attempt", wifi_failures)

            if time.ticks_diff(now, last_accept_log_ms) >= ACCEPT_ERROR_LOG_MS:
                last_accept_log_ms = now
                for srv, name in ((server, "rotctld"), (web_server, "web")):
                    if srv.last_accept_error is not None:
                        _log(
                            name,
                            "accept failed (socket pool exhausted?):",
                            srv.last_accept_error,
                        )
                        srv.last_accept_error = None

            # Retries any listening socket that is not up
            if time.ticks_diff(now, last_bind_retry_ms) >= SERVER_REBIND_RETRY_MS:
                last_bind_retry_ms = now
                for key, srv, name in (
                    ("rotctld", server, "rotctld server"),
                    ("web", web_server, "web server"),
                ):
                    if srv.server is not None:
                        continue
                    try:
                        srv.start()
                        bind_failure_logged.discard(key)
                        _log(name, "listening again")
                    except Exception as e:
                        if key not in bind_failure_logged:
                            bind_failure_logged.add(key)
                            _log(name, "not listening, will keep retrying:", repr(e))

            if not sensor.ready:
                if time.ticks_diff(now, last_sensor_attempt_ms) >= SENSOR_RETRY_MS:
                    last_sensor_attempt_ms = now
                    try:
                        sensor.begin()
                        _log("LSM303 sensor ready")
                    except Exception as e:
                        _log_repeating(
                            "sensor", "LSM303 sensor init retry failed: " + repr(e)
                        )

            try:
                if controller.update():
                    controller.last_error = None
            except Exception as e:
                controller.last_error = "{}: {}".format(type(e).__name__, e)
                _log_repeating(
                    "control", "controller update error: " + controller.last_error
                )

            try:
                server.poll_once()
            except Exception as e:
                _log("server poll error:", repr(e))
            try:
                web_server.poll_once()
            except Exception as e:
                _log("web server poll error:", repr(e))

            # Keep this below CONTROL_UPDATE_MS
            time.sleep_ms(10)
    finally:
        controller.stop()
        server.close()
        web_server.close()


if __name__ == "__main__":
    main()
