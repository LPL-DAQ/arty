#pragma once

#include "Error.h"
#include "MutexGuard.h"
#include "config.h"
#include <expected>
#include <optional>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

// Placeholder for the two backup IMUs, which have no hardware yet. Same interface as VectornavIMU.h
// so a real driver can be added without changing callers.

enum class BackupImuKind { BACKUP_IMU_1, BACKUP_IMU_2 };

consteval static const char* kind_to_prefix(BackupImuKind kind)
{
    switch (kind) {
    case BackupImuKind::BACKUP_IMU_1:
        return "[BackupImu1]";
    case BackupImuKind::BACKUP_IMU_2:
        return "[BackupImu2]";
    default:
        return "[INVALID]";
    }
}

template <BackupImuKind kind> class BackupImu {
private:
    static inline ImuReading reading = ImuReading_init_default;
    static inline bool has_reading = false;

    // Initialized in init()
    static inline k_mutex reading_mutex;

public:
    static std::expected<void, Error> init();

    // Always std::nullopt until a driver exists.
    static std::optional<ImuReading> read();
};

template <BackupImuKind kind> std::expected<void, Error> BackupImu<kind>::init()
{
    LOG_MODULE_DECLARE(BackupImu);

    k_mutex_init(&reading_mutex);
    LOG_INF("%s Initialized (placeholder -- hardware not yet installed)", kind_to_prefix(kind));

    return {};
}

template <BackupImuKind kind> std::optional<ImuReading> BackupImu<kind>::read()
{
    MutexGuard reading_guard{&reading_mutex};

    if (!has_reading) {
        return std::nullopt;
    }

    // Consume reading
    has_reading = false;

    return {reading};
}

typedef BackupImu<BackupImuKind::BACKUP_IMU_1> BackupImu1;
typedef BackupImu<BackupImuKind::BACKUP_IMU_2> BackupImu2;
