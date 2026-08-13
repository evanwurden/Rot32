import network
import time

try:
    _PM_NONE = network.WLAN.PM_NONE
except AttributeError:
    _PM_NONE = None


def apply_radio_settings(wlan, power_save_disabled=True, txpower_dbm=None):
    """Applies the radio settings"""
    if power_save_disabled and _PM_NONE is not None:
        try:
            wlan.config(pm=_PM_NONE)
        except Exception:
            pass

    if txpower_dbm is not None:
        try:
            wlan.config(txpower=txpower_dbm)
        except Exception:
            pass


def is_connected(wlan) -> bool:
    """Returns True only when the interface is associated and holds a
    usable address. isconnected() alone goes True before DHCP hands out a
    lease"""
    if wlan is None:
        return False
    try:
        if not wlan.isconnected():
            return False
        ip = wlan.ifconfig()[0]
    except Exception:
        return False
    return bool(ip) and ip != "0.0.0.0"


def connect_wifi(
    ssid: str,
    password: str,
    timeout_s: int = 20,
    power_save_disabled: bool = True,
    txpower_dbm=None,
) -> str:
    """Blocks up to timeout_s waiting for the initial connection, and raises
    RuntimeError on timeout. Callers should treat that as non-fatal, since
    Wi-Fi may come up later."""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    apply_radio_settings(wlan, power_save_disabled, txpower_dbm)

    if is_connected(wlan):
        return wlan.ifconfig()[0]

    wlan.connect(ssid, password)

    start = time.ticks_ms()
    timeout_ms = timeout_s * 1000

    # Uses is_connected(), so this does not return an ifconfig() of 0.0.0.0
    # and let the servers bind before DHCP finishes.
    while not is_connected(wlan):
        if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
            raise RuntimeError("Wi-Fi connection timeout")
        time.sleep_ms(200)

    return wlan.ifconfig()[0]


def get_wlan(power_save_disabled=True, txpower_dbm=None):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    apply_radio_settings(wlan, power_save_disabled, txpower_dbm)
    return wlan


def attempt_reconnect(
    wlan,
    ssid,
    password,
    consecutive_failures=0,
    hard_reset_after=3,
    power_save_disabled=True,
    txpower_dbm=None,
):
    """Starts a connection attempt without waiting for it to finish, and
    escalates to a full interface cycle after consecutive
    failures. Returns the updated failure count for the caller to carry
    forward."""
    consecutive_failures += 1

    try:
        # The cycle is the only blocking call here, for a couple hundred
        # milliseconds. It recovers a driver that ignores connect().
        if hard_reset_after > 0 and consecutive_failures % hard_reset_after == 0:
            wlan.active(False)
            time.sleep_ms(100)
            wlan.active(True)
            # Power save and TX power do not survive the cycle.
            apply_radio_settings(wlan, power_save_disabled, txpower_dbm)
        else:
            # Drops any half-open association first, since connect() on an
            # interface mid-attempt is a no-op on some builds.
            try:
                wlan.disconnect()
            except Exception:
                pass

        wlan.connect(ssid, password)
    except Exception:
        pass

    return consecutive_failures
