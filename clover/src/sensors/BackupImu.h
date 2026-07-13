#pragma once

#include "Error.h"
#include "MutexGuard.h"
#include "config.h"
#include <expected>
#include <optional>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

// Placeholder interface for the two backup IMUs. Hardware does not exist yet -- this exists so the
// estimator has a stable interface to integrate against ahead of the real sensor drivers. Mirrors
// VectornavIMU.h's shape (static reading buffer + mutex + has_reading flag + read()), minus the
// UART/device plumbing there's no hardware to back yet.
//
// Why mirror the real sensor's shape rather than stub something simpler (e.g. a bare struct):
// StateEstimator.cpp and Controller.cpp call BackupImu1::read()/init() exactly like they call
// VectornavImu::read()/init() (see StateEstimator.cpp's VN300-vs-backup-IMU divergence check and
// Controller.cpp's read block). When real hardware and a real decode()/sense() thread eventually
// land, they only need to fill in this class's private internals -- the mutex-guarded
// reading/has_reading pattern is already in place -- without changing any call site. The mutex in
// particular exists now (even though nothing populates `reading` yet) so `read()`'s locking
// behavior won't change out from under callers once a sense() thread starts writing concurrently.

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

    // Returns the latest reading, if one is available. No hardware exists yet, so has_reading is
    // never set -- this will always return std::nullopt until a real driver lands.
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
