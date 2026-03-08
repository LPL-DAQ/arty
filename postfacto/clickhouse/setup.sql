CREATE DATABASE IF NOT EXISTS lpl;

CREATE TABLE IF NOT EXISTS lpl.raw_sensors (
    `time` DateTime64(9, 'America/Los_Angeles'),
    `sensor` LowCardinality(String),
    `system` LowCardinality(String),
    `value` Float64,
    `event` LowCardinality(String),
    `source` LowCardinality(String),
) ENGINE = MergeTree()
ORDER BY
    (`time`, `sensor`) PARTITION BY `system`;

CREATE TABLE IF NOT EXISTS lpl.sensors (
    `time` DateTime64(9, 'America/Los_Angeles'),
    `sensor` LowCardinality(String),
    `value` Float64,
    `test_id` UUID,
    `event` LowCardinality(String)
) ENGINE = MergeTree()
ORDER BY
    (`test_id`, `time`, `sensor`);

CREATE TABLE IF NOT EXISTS lpl.tests (
    `id` UUID,
    `t0` DateTime64(9, 'America/Los_Angeles'),
    `aborted` Bool,
    `source` LowCardinality(String),
    `start` DateTime64(9, 'America/Los_Angeles'),
    `end` DateTime64(9, 'America/Los_Angeles'),
    `test_type` LowCardinality(String),
    `notes` String,
    `tags` JSON
) ENGINE = MergeTree()
ORDER BY
    (`t0`);
