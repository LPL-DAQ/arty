import os.path

from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError
from google.oauth2 import service_account

# If modifying these scopes, delete the file token.json.
SCOPES = ['https://www.googleapis.com/auth/spreadsheets']
SERVICE_ACCOUNT_FILE = 'service_account.json'

# The ID and range of a sample spreadsheet.
TESTS_SPREADSHEET_ID = '1Oq2oJ4aziSB8Ql_fTM8VF6au83S1JfWBH74enQstdhU'


def main():
    creds = service_account.Credentials.from_service_account_file(SERVICE_ACCOUNT_FILE)

    try:
        service = build('sheets', 'v4', credentials=creds)

        # Call the Sheets API
        sheet = service.spreadsheets().values().append(spreadsheetId=TESTS_SPREADSHEET_ID, range='')
        values = [
            []
        ]
        result = (
            sheet.values()
            .get(
                spreadsheetId=TESTS_SPREADSHEET_ID,
                range='raw_tests!A1:G',
                valueInputOptions='RAW',
                body={'values': values},
            )
            .execute()
        )
        print(f'sheet append result: {result}')


if __name__ == '__main__':
    main()
