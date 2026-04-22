#ifndef ARTY_GNSS_H
#define ARTY_GNSS_H

#include <stdint.h>

// Solution type values from GREIS spec
#define GNSS_SOL_NONE       0
#define GNSS_SOL_STANDALONE 1
#define GNSS_SOL_DGPS       2
#define GNSS_SOL_FLOAT      3
#define GNSS_SOL_FIXED      4

// Populated from [mp], [VE], [SG], [ST], [~~]
struct GnssReading {
    // Position in local plane (from [mp])
    double north_m;
    double east_m;
    double up_m;
    float  pos_sigma_m;     // 3D position RMS [m]

    // Cartesian ECEF velocity (from [VE])
    float vx_ms;            // ECEF X velocity [m/s]
    float vy_ms;            // ECEF Y velocity [m/s]
    float vz_ms;            // ECEF Z velocity [m/s]
    float vel_sigma_ms;     // 3D velocity RMS [m/s]

    // RMS error breakdown (from [SG])
    float hrms_m;           // Horizontal position RMS [m]
    float vrms_m;           // Vertical position RMS [m]
    float hvel_rms_ms;      // Horizontal velocity RMS [m/s]
    float vvel_rms_ms;      // Vertical velocity RMS [m/s]

    // Timing
    uint32_t solution_time_ms;   // From [ST]: epoch time mod 1 day [ms]
    uint32_t receiver_time_ms;   // From [~~]: receiver time mod 1 day [ms]

    // Solution type (0=none, 1=standalone, 2=DGPS, 3=float, 4=fixed)
    uint8_t sol_type;
};

int gnss_init();

// Returns a copy of the latest reading. sol_type will be GNSS_SOL_NONE
// if no valid reading has been received yet.
GnssReading gnss_read();

#endif  // ARTY_GNSS_H