"""Shared valve readback display for the dashboard and named-sequence views."""
import json
from pathlib import Path
from rich.table import Table

MAP_PATH = Path(__file__).resolve().parents[1] / 'tests/valve_gpio/valve_map.json'


def valve_rows(info, mapping=None):
    if mapping is None:
        mapping = json.loads(MAP_PATH.read_text())
    readback = (info is not None and info.HasField('powered_channels')
                and info.HasField('known_channels'))
    for name, channel in mapping.items():
        state = 'UNKNOWN'
        if channel is None:
            state = 'UNASSIGNED'
        elif readback and info.known_channels & (1 << channel):
            state = 'ON' if info.powered_channels & (1 << channel) else 'OFF'
        yield name, str(channel) if channel is not None else '—', state


def valve_table(info):
    table = Table(title='Valve GPIO readback (not mechanical position)')
    table.add_column('Valve')
    table.add_column('Channel')
    table.add_column('Power')
    try:
        for name, channel, state in valve_rows(info):
            table.add_row(name, channel, state, style={'ON': 'green', 'OFF': 'dim'}.get(state, 'yellow'))
    except (OSError, ValueError):
        table.add_row('Valve map unavailable', '—', 'UNKNOWN')
    return table
