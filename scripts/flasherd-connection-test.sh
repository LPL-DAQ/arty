#!/bin/bash

# Does a dummy run of flasherd-client to ensure it can talk to host flasherd.

echo "[flasherd-connection-test] Testing flasherd..."

# Runs dir in cmd shell on windows. For unix systems, those args are just passed to echo.
flasherd-client --command-windows cmd --command-macos echo --command-linux echo --arg "/c" --arg "dir"
status=$?

if [ $status -eq 0 ]; then
  echo "[flasherd-connection-test] flasherd is up!"
else
  echo "[flasherd-connection-test] flasherd test failed."
fi

exit $status
