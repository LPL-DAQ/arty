# postfacto

A unified platform for post-facto test data collation and analysis.

## Run

Ensure `postfacto/config` exists, and specifies the following:

```text
CLICKHOUSE_HOST=localhost
CLICKHOUSE_USER=writer
CLICKHOUSE_PASSWORD=<???>
SHEETS_ID=1Oq2oJ4aziSB8Ql_fTM8VF6au83S1JfWBH74enQstdhU
SHEETS_SERVICE_ACCOUNT_FILE=service_account.json
```

Also ensure `postfacto/service_account.json` exists, and has the contents of the service account
credentials for the spreadsheet bot.

We use Docker Compose to bring up the various required services on a single node.
After cloning the `arty` repo, run the following:

```shell
cd postfacto
docker compose up --detach
```
