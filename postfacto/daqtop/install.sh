#!/bin/bash

# Installer script for daqtop's required daemons.

set -euxo pipefail

if [ "$(whoami)" != "root" ]; then
    echo "Please run me as root"
    exit 1
fi

curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR="/opt/uv" sh

rm -rf /opt/postfacto
mkdir -p /opt/postfacto
cp -R /home/labtop/arty/postfacto/ /opt/

echo "Please ensure CLICKHOUSE_HOST, CLICKHOUSE_USERNAME, and CLICKHOUSE_PASSWORD are set in /etc/postfacto/db_config,"
echo "then press enter to install the systemd services"
read

chmod +r /opt/postfacto

cp /opt/postfacto/daqtop/ingest-legacy-daq-data.service /etc/systemd/system
cp /opt/postfacto/daqtop/ingest-legacy-daq-events.service /etc/systemd/system

systemctl daemon-reload

systemctl enable ingest-legacy-daq-data
systemctl enable ingest-legacy-daq-events

systemctl start ingest-legacy-daq-data
systemctl start ingest-legacy-daq-events
