import datetime
import os
import pickle
import uuid
from pathlib import Path

import clickhouse_connect
import polars as pl

PERSISTENT_DATA_PATH = Path('/var/extract_legacy_daq_tests.pickle')


def commit_last_processed_time(new_time: datetime.datetime):
    print(f'Committing last processed time: {new_time}')
    with open(PERSISTENT_DATA_PATH, 'w+b') as f:
        pickle.dump(new_time, f)


client = clickhouse_connect.get_client(
    host='localhost', username='admin', password=os.environ['CLICKHOUSE_ADMIN_PASSWORD']
)

# Restore persistent state
if PERSISTENT_DATA_PATH.exists():
    with open(PERSISTENT_DATA_PATH, 'rb') as f:
        last_processed_date = pickle.load(f)  # noqa: S301
        print(f'Restored last processed date: {last_processed_date}')
else:
    print('No persistent datetime found, setting it to the last known test datapoint.')

    df = client.query_df_arrow('SELECT max(`time`) FROM sensors', dataframe_library='polars')
    last_processed_date = df[0, 'max(time)']

    print(f'Found last processed datetime from max test datapoint: {last_processed_date}')

# Process data points.
while True:
    events = client.query_df_arrow(
        "SELECT `time`, `event` FROM raw_sensors WHERE `sensor` == 'event' AND `time` >= {t1:DateTime64(9, 'America/Los_Angeles')} AND `time` <= {t2:DateTime64(9, 'America/Los_Angeles')} ORDER BY `time`",
        parameters={
            't1': last_processed_date.timestamp(),
            't2': (last_processed_date + datetime.timedelta(minutes=5)).timestamp(),
        },
        dataframe_library='polars',
    )

    if events.height == 0:
        print(f'No new events found after {last_processed_date}')
        print('Checking for events after a long time jump...')
        gapped_events = client.query_df_arrow(
            "SELECT `time`, `event` FROM raw_sensors WHERE `sensor` == 'event' AND `time` >= {t:DateTime64(9, 'America/Los_Angeles')} ORDER BY `time` LIMIT 1",
            parameters={'t': last_processed_date.timestamp()},
            dataframe_library='polars',
        )
        if gapped_events.height > 0:
            print('Later event block found. Updating last processed date to start from this range.')
            last_processed_date = gapped_events[0, 'time']
            commit_last_processed_time(last_processed_date)
        else:
            print('No later event block found.')

        # time.sleep(3)
        continue

    # Sequence either goes ignition -> ignition_terminated OR ignition -> abort -> terminated.
    ignition_time = events.filter(pl.col('event') == 'ignition')[0, 'time']
    if type(ignition_time) is not datetime.datetime:
        print('No ignition event found, none of this data is within a sequence.')

        # Add an epsilon to prevent this event from being re-processed.
        last_processed_date = events[-1, 'time'] + datetime.timedelta(milliseconds=1)
        commit_last_processed_time(last_processed_date)
        continue

    # Do not match sequence events that are potentially cut off at the end with the current frame.
    events = events.filter(pl.col('time') >= ignition_time)

    # Match sequencer's end
    ignition_terminated_time = events.filter(pl.col('event') == 'ignition_terminated')[0, 'time']
    terminated_time = events.filter(pl.col('event') == 'terminated')[0, 'time']

    if (
        type(ignition_terminated_time) is not datetime.datetime
        and type(terminated_time) is not datetime.datetime
    ):
        print('No ignition termination events found, we may find it in the next frame.')
        last_processed_date = ignition_time
        commit_last_processed_time(last_processed_date)
        continue

    elif (
        type(ignition_terminated_time) is datetime.datetime
        and type(terminated_time) is datetime.datetime
    ):
        seq_end_time = min(ignition_terminated_time, terminated_time)

    elif type(ignition_terminated_time) is datetime.datetime:
        seq_end_time = ignition_terminated_time

    else:
        seq_end_time = terminated_time

    # Ignition event is sent ~10s before anything we care about happens, so for symmetry, let's capture another 10s
    # after the sequence ends.
    last_event_time = seq_end_time
    seq_end_time += datetime.timedelta(seconds=10)
    seq_start_time = ignition_time

    print(f'Test sequence discovered, from {seq_start_time} to {seq_end_time}')

    # Discover T=0 based on the sequencer events found. This also affects the detected test type.
    test_events = events.filter(
        (pl.col('time') >= seq_start_time) & (pl.col('time') <= seq_end_time)
    )

    # Detect t0.

    ignition_events = test_events.filter(pl.col('event') == 'ign-002 close -> open')
    lox_open_events = test_events.filter(pl.col('event') == 'pbv-101 close -> open')
    fuel_open_events = test_events.filter(pl.col('event') == 'pbv-201 close -> open')

    # Mark t0 from ignition, if it's there.
    if not ignition_events.is_empty():
        t0 = ignition_events[0, 'time']
        test_type = 'hotfire'

    # No valve actuations, we probably don't care about this sequence.
    elif lox_open_events.is_empty() and fuel_open_events.is_empty():
        print('No meaningful valve actuations discovered, discarding test.')
        last_processed_date = last_event_time
        commit_last_processed_time(last_processed_date)
        continue

    # If it's a fuel flow, mark from fuel open.
    elif lox_open_events.is_empty() and not fuel_open_events.is_empty():
        t0 = fuel_open_events[0, 'time']
        test_type = 'fuel_flow'

    # If it's a lox or dual flow, mark from lox open. There may be a pre-chill, so take the last lox open event.
    else:
        t0 = lox_open_events[-1, 'time']
        test_type = 'lox_flow' if fuel_open_events.is_empty() else 'dual_flow'

    # Insert new test entry!
    test_id = uuid.uuid4()
    client.insert(
        'tests',
        [[test_id, 'atlas', test_type, t0]],
        column_names=['id', 'system', 'type', 't0'],
    )
    print(f'Created new test entry with ID {test_id}')

    # Insert corresponding sensor data block
    sensors = client.query_df_arrow(
        "SELECT `time`, `sensor`, `system`, `value`, `event` FROM raw_sensors WHERE `system` == 'atlas' AND `time` >= {t1:DateTime64(9, 'America/Los_Angeles')} AND `time` <= {t2:DateTime64(9, 'America/Los_Angeles')} ORDER BY `sensor`, `time`",
        parameters={'t1': seq_start_time.timestamp(), 't2': seq_end_time.timestamp()},
        dataframe_library='polars',
    ).with_columns(test_id=pl.lit(str(test_id)))
    client.insert_df_arrow('sensors', sensors)
    print(f'Inserted sensor data:\n{sensors}')

    last_processed_date = last_event_time
    commit_last_processed_time(last_processed_date)
