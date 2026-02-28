import json
import os
import time
from datetime import datetime, timedelta
from pathlib import Path
from zoneinfo import ZoneInfo

import clickhouse_connect
import polars as pl

PERSISTENT_PATH = Path('/opt/postfacto/ingest_legacy_daq_events_persist.json')
DATA_PATH = Path('/home/labtop/DAC/data')

READ_AT_ONCE = 1024 * 40

count = 0


def load_progress():
    if not PERSISTENT_PATH.exists():
        return {
            'dir': Path(),
            'bytes_read': 0,
            'col_names': [],
            'dir_mtime': datetime.now(),
        }
    with open(PERSISTENT_PATH) as f:
        p = json.load(f)
        p['dir'] = Path(p['dir'])
        p['dir_mtime'] = datetime.fromtimestamp(p['dir_mtime'], tz=ZoneInfo('America/Los_Angeles'))
        return p


def save_progress(p):
    p['dir'] = str(p['dir'])
    p['dir_mtime'] = p['dir_mtime'].timestamp()
    with open(PERSISTENT_PATH, 'w') as f:
        json.dump(p, f)


def next_data_dir(curr_dir: Path):
    """Return None if curr_Dir is the most recent. Otherwise, returns a Path for the next dir to process"""

    dirs = list(DATA_PATH.iterdir())
    dirs.sort(key=lambda p: p.stat().st_mtime)
    if curr_dir == Path():
        curr_dir = dirs[0]
    for i in range(len(dirs) - 1):
        if not dirs[i].joinpath('testData_log.txt').exists():
            continue
        if dirs[i] == curr_dir:
            return dirs[i + 1]

    return None


client = clickhouse_connect.get_client(
    host=os.environ['CLICKHOUSE_HOST'], username=os.environ['CLICKHOUSE_USERNAME'], password=os.environ['CLICKHOUSE_PASSWORD']
)

done_file = False
file_waiting = False

while True:
    progress = load_progress()

    # Find next file to parse through, if necessary.
    if done_file or progress['dir'] == Path():
        prev_dir = progress['dir']
        progress['dir'] = next_data_dir(progress['dir'])
        if progress['dir'] is not None:
            print(f'Found next dir: {progress["dir"]}')

            done_file = False
            file_waiting = False
            progress['dir_mtime'] = datetime.fromtimestamp(
                progress['dir'].stat().st_mtime, tz=ZoneInfo('America/Los_Angeles')
            )
            progress['bytes_read'] = 0
            data_path = progress['dir'].joinpath('testData_log.txt')
        else:
            progress['dir'] = prev_dir
    data_path = progress['dir'].joinpath('testData_log.txt')

    # Read a chunk, then trim to ensure it's terminated at a full line
    with open(data_path, 'rb') as f:
        f.seek(progress['bytes_read'], 0)
        buf = f.read(READ_AT_ONCE)
    if len(buf) == 0:
        done_file = True
        if not file_waiting:
            print(f'Waiting for file... (last read: {progress["dir"]})')
            file_waiting = True
        time.sleep(5)
        save_progress(progress)
        continue

    line_break_idx = buf.rfind(b'\n')
    if line_break_idx == -1:
        print("Didn't find a newline, how is this possible?")
        continue

    buf = buf[: line_break_idx + 1]
    progress['bytes_read'] += len(buf)

    times = []
    events = []
    for line in buf.decode().strip().split('\n'):
        line = line.strip()
        right_space = line.rfind(' ')
        time_sec = progress['dir_mtime'] + timedelta(seconds=float(line[right_space + 1 :]))

        left_space = line.find(' ')
        source = line[:left_space]

        event_name = line[left_space + 1 : right_space].lower()

        # Who cares about user side "regular" valve actuations
        if source == '[USER]' and ' -> ' in event_name:
            continue

        times.append(time_sec)
        events.append(event_name)

    df = pl.DataFrame({'time': times, 'event': events}).with_columns(
        sensor=pl.lit('event'), system=pl.lit('atlas'), source=pl.lit('dac_legacy')
    )
    print(df)
    client.insert_df_arrow('raw_sensors', df)

    save_progress(progress)

