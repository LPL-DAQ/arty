FROM clickhouse/clickhouse-server@sha256:7f2126632cfa448bc73aa7dfadc6584b5500b8bc484f10e3af6d19e95adb03ad

COPY setup.sql /docker-entrypoint-initdb.d/

COPY users.xml /etc/clickhouse-server/users.d/
COPY server.xml /etc/clickhouse-server/config.d/
