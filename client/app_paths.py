"""Where the client finds its files.

Everything is resolved relative to this folder, never the current directory, so the client behaves
the same whether it is started from the repo root or from client/.
"""
from pathlib import Path

APP_DIR = Path(__file__).resolve().parent

DATA_DIR = APP_DIR / 'data'
SENSOR_CONFIG_DIR = APP_DIR / 'sensor_configs'
SEQUENCES_DIR = APP_DIR / 'sequences'

# The firmware's own valve map, so the client always matches what clover was built with.
# If only the client folder was copied to a laptop, copy that file next to client-new.py too.
_REPO_VALVE_MAP = APP_DIR.parent / 'clover' / 'src' / 'sequencing' / 'valve_map.json'
VALVE_MAP_PATH = _REPO_VALVE_MAP if _REPO_VALVE_MAP.exists() else APP_DIR / 'valve_map.json'
