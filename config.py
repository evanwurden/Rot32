"""Rot32 configuration"""

# Wi-Fi settings
WIFI_SSID = "YOUR_SSID_HERE"
WIFI_PASSWORD = "YOUR_PASS_HERE"

# WIFI_CONNECT_TIMEOUT_S: how long the boot connect waits before it gives
# up. This does not stop startup.
WIFI_CONNECT_TIMEOUT_S = 20
# WIFI_RECONNECT_MS: how often it retries the connection while offline
WIFI_RECONNECT_MS = 10_000

WIFI_HARD_RESET_AFTER_ATTEMPTS = 5
WIFI_POWER_SAVE_DISABLED = True

# Transmit power cap in dBm. None leaves it uncapped
WIFI_TX_POWER_DBM = None

# Hardware watchdog
WDT_ENABLED = True
WDT_TIMEOUT_MS = 30000

# Logs to the serial console. Keep this off without a USB connection
SERIAL_LOGGING_ENABLED = False

# rotctld server settings
ROTCTLD_HOST = "0.0.0.0"
ROTCTLD_PORT = 4533

# Web control/calibration UI settings
WEB_HOST = "0.0.0.0"
WEB_PORT = 80

# Closes an idle client after no traffic for this long
WEB_CLIENT_IDLE_MS = 15_000
ROTCTLD_CLIENT_IDLE_MS = 300_000

MAX_CLIENTS_PER_SERVER = 4

# I2C pins for the LSM303 sensor. GPIO8 and GPIO9 are safe on ESP32-S3
I2C_SDA_PIN = 8
I2C_SCL_PIN = 9
I2C_FREQ_HZ = 400_000

# LSM303 output data rates.
# ACCEL_ODR_HZ (1/10/25/50/100/200/400)
ACCEL_ODR_HZ = 100
# MAG_ODR_HZ (15/30/75/220)
# Keep this above 1000/CONTROL_UPDATE_MS. Otherwise the control loop reads
# stale values.
MAG_ODR_HZ = 75

# Motor driver. GPIO4 to GPIO7 are safe on ESP32-S3
AZ_PLUS_PIN = 4
AZ_MINUS_PIN = 5
EL_PLUS_PIN = 6
EL_MINUS_PIN = 7
PWM_FREQUENCY_HZ = 500

# Control loop period, roughly 50 Hz
CONTROL_UPDATE_MS = 20

POSITION_TOLERANCE_DEG = 1.5
MAX_ELEVATION_DEG = 90.0
MIN_ELEVATION_DEG = -10.0

# Speed ramping. APPROACH_SPEED_PERCENT is the speed the rotator arrives at,
# SPEED_RAMP_DEG is how far out the taper down to it begins, and
# SPEED_ACCEL_PERCENT_PER_SEC caps how fast commanded speed may change.
APPROACH_SPEED_PERCENT = 15.0
SPEED_RAMP_DEG = 15.0
SPEED_ACCEL_PERCENT_PER_SEC = 200.0

# Optional sensor/orientation calibration
MAG_DECLINATION_DEG = 0.0
AZIMUTH_OFFSET_DEG = 0.0
ELEVATION_OFFSET_DEG = 0.0
INVERT_AZIMUTH = False
INVERT_ELEVATION = False

# AZIMUTH_FORWARD_AXIS: axis that lies parallel to the antenna. 
AZIMUTH_FORWARD_AXIS = "X"

# ELEVATION_AXIS_PAIR: axis pair ("XY", "XZ" or "YZ") that
# elevation reads from. Pick the pair that changes as you tilt the sensor.
ELEVATION_AXIS_PAIR = "XZ"

# Exponential-moving-average smoothing on the azimuth and elevation output
FILTER_STRENGTH = 0.95

# Deviation above which smoothing turns off and a reading is treated as a
# real move, not wind or noise
MOTION_THRESHOLD_DEG = 3.0

# Park position
PARK_AZIMUTH_DEG = 0.0
PARK_ELEVATION_DEG = 0.0
