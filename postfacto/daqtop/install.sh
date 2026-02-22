#!/bin/bash

# Installer script for daqtop's required daemons.

set -euxo pipefail

if [ "$(whoami)" != "root" ]; then
    echo "Please run me as root"
    exit 1
fi

postfacto_dir="$(dirname -- $0)/.."

curl -LsSf https://astral.sh/uv/install.sh | env UV_INSTALL_DIR="/opt/uv" sh

mkdir -p /opt/postfacto/daqtop
cp -R "$postfacto_dir/*" /opt/postfacto/

echo "Please ensure CLICKHOUSE_HOST, CLICKHOUSE_USERNAME, and CLICKHOUSE_PASSWORD are set in /opt/postfacto/db_config,"
echo "then press enter to install the systemd services"
read


