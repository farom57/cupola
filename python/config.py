import json
import os
from pathlib import Path

from serial_com import DEFAULT_PORT

DEFAULTS = {
    "serial_port": DEFAULT_PORT,
    "pwi4_host": "localhost",
    "pwi4_port": 8220,
}


def config_path():
    # %APPDATA%\cupola\config.json sous Windows, ~/.config/cupola/config.json ailleurs
    base = os.environ.get("APPDATA") or Path.home() / ".config"
    return Path(base) / "cupola" / "config.json"


def load():
    config = dict(DEFAULTS)
    try:
        with open(config_path(), encoding="utf-8") as f:
            config.update(json.load(f))
    except FileNotFoundError:
        pass
    except (OSError, ValueError) as e:
        print(f"[CONFIG] Lecture impossible ({e}), valeurs par défaut utilisées")
    return config


def save(config):
    path = config_path()
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(config, f, indent=2)
        return True
    except OSError as e:
        print(f"[CONFIG] Écriture impossible ({e})")
        return False
