import json

CALIBRATION_FILE = "calibration.json"
TUNING_FILE = "tuning.json"


def load(path):
    """Parsed JSON from path, or an empty dict if it is missing or corrupt."""
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save(path, data):
    with open(path, "w") as f:
        json.dump(data, f)
