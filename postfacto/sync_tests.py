import datetime
import os
import pickle
import uuid
from pathlib import Path
import time
from zoneinfo import ZoneInfo
import json

import clickhouse_connect
import polars as pl
from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError
from google.oauth2 import service_account

PERSISTENT_DATA_PATH = Path('/var/extract_legacy_daq_tests.pickle')
SCOPES = ['https://www.googleapis.com/auth/spreadsheets']
SERVICE_ACCOUNT_FILE = 'service_account.json'

CLICKHOUSE_HOST = os.environ['CLICKHOUSE_HOST']
CLICKHOUSE_USER = os.environ['CLICKHOUSE_USER']
CLICKHOUSE_PASSWORD = os.environ['CLICKHOUSE_PASSWORD']
SHEETS_ID = os.environ['SHEETS_ID']
SHEETS_SERVICE_ACCOUNT_FILE = os.environ['SHEETS_SERVICE_ACCOUNT_FILE']


# Make clickhouse client
clickhouse_client = clickhouse_connect.get_client(
    host=CLICKHOUSE_HOST, username=CLICKHOUSE_USER, password=CLICKHOUSE_PASSWORD
)

# Make google sheets client
sheets_creds = service_account.Credentials.from_service_account_file(SHEETS_SERVICE_ACCOUNT_FILE)
sheets_service = build('sheets', 'v4', credentials=sheets_creds)

sync_count = 0

# Process data points.
while True:
    # Get current tests in database
    db_tests_df = clickhouse_client.query_df_arrow(
        'SELECT toString(`id`) as "id", `test_type`, `notes`, toJSONString(`tags`) AS "tags" FROM tests',
        dataframe_library='polars',
    )
    db_tests_by_id = {}
    for row in db_tests_df.rows(named=True):
        db_tests_by_id[row['id']] = row

    # Get canonical tests from spreadsheet
    result = (
        sheets_service.spreadsheets()
        .values()
        .get(spreadsheetId=SHEETS_ID, range='exported_tests!A2:AA')
        .execute()
    )
    values = result['values']

    EXPECTED_INITIAL_HEADERS = [
        'test_id',
        't0_timestamp',
        'aborted',
        'source',
        'start_timestamp',
        'end_timestamp',
        'test_type',
        'notes',
    ]
    headers = values[0]
    values = values[1:]

    # First 8 columns must be identical, all others will be converted to JSON keys.
    if headers[0 : len(EXPECTED_INITIAL_HEADERS)] != EXPECTED_INITIAL_HEADERS:
        raise Exception(
            f'Unexpected initial headers: must be {EXPECTED_INITIAL_HEADERS}, but got: {headers}'
        )

    # Ensure database matches sheets.
    for row in values:
        if len(row) < 6:
            raise Exception(f'Found row with fewer entries that required headers, skipping: {row}')
        for _ in range(len(row), len(headers)):
            row.append('')

        test_id = row[0]
        if test_id == '':
            raise Exception(f'blank test ID: {row}')

        t0 = datetime.datetime.fromtimestamp(float(row[1]), tz=ZoneInfo('America/Los_Angeles'))

        if row[2] == 'TRUE':
            aborted = True
        elif row[2] == 'FALSE':
            aborted = False
        else:
            raise Exception(f'Bad value found in aborted field for row: {row}')

        source = row[3]
        start = datetime.datetime.fromtimestamp(float(row[4]), tz=ZoneInfo('America/Los_Angeles'))
        end = datetime.datetime.fromtimestamp(float(row[5]), tz=ZoneInfo('America/Los_Angeles'))
        test_type = row[6]
        notes = row[7]

        tags = {}
        for i in range(8, len(row)):
            if row[i] != '':
                try:
                    tags[headers[i]] = int(row[i])
                    continue
                except:  # noqa: E722
                    pass
                try:
                    tags[headers[i]] = float(row[i])
                    continue
                except:  # noqa: E722
                    tags[headers[i]] = row[i]

        # Insert a new test, if we have to.
        if test_id not in db_tests_by_id:
            df = pl.DataFrame(
                {
                    'id': test_id,
                    't0': t0,
                    'aborted': aborted,
                    'source': source,
                    'start': start,
                    'end': end,
                    'test_type': test_type,
                    'notes': notes,
                    'tags': json.dumps(tags),
                }
            )
            print(f'Inserting new row:\n{df}')
            clickhouse_client.insert_df_arrow(
                'tests',
                df,
            )

        # Otherwise, patch the existing test
        else:
            # Only update if notes, test_type, or tags have changed.
            db_test = db_tests_by_id[test_id]
            if (
                db_test['notes'] == notes
                and db_test['test_type'] == test_type
                and json.loads(db_test['tags']) == tags
            ):
                continue
            print(f'mismatch for test {test_id}')
            print(f'  notes:     {notes}')
            print(f'  test_type: {test_type}')
            print(f'  tags:      {tags}')
            result = clickhouse_client.command(
                'ALTER TABLE tests UPDATE `notes` = {notes:String}, `test_type` = {test_type:LowCardinality(String)}, `tags` = {tags:JSON} WHERE `id` == {test_id:UUID}',
                parameters={
                    'notes': notes,
                    'test_type': test_type,
                    'tags': tags,
                    'test_id': test_id,
                },
            )
            print(f'Updated test ID {test_id}:\n{result.summary}')

    sync_count += 1
    if sync_count % 1800 == 0:
        print(f'synced {sync_count} times; now = {datetime.datetime.now()}')

    time.sleep(2)
