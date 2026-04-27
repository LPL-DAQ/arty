#include "RangerThrottle.h"
#include "Controller.h"
#include "LookupTable1D.h"
#include "MutexGuard.h"
// #include "../lut/thrust_to_fuel_1.5.h"
// #include "../lut/thrust_to_lox_1.5.h"
#include "../lut/thrust_to_fuel.h"
#include "../lut/thrust_to_lox.h"
#include "../lut/cea_lut.h"
#include <cmath>
#include <zephyr/kernel.h>

K_MUTEX_DEFINE(ranger_throttle_lock);



static constexpr float DEFAULT_FUEL_POS = 81.0f;
static constexpr float DEFAULT_LOX_POS = 74.0f;

static constexpr float FUEL_ENGINE_INLET_LINE_LOSS_PSI = 21.0f;
static constexpr float LOX_ENGINE_INLET_LINE_LOSS_PSI = 41.0f;

// Physics constants
static constexpr float EFFICIENCY = 0.93f;
static constexpr float LBF_CONVERSION = 0.224809f;
static constexpr float K_SLOPE = -1.132744863732548e-04f;
static constexpr float K_OFFSET = 0.123605503801193f;
static constexpr float ALPHA = 307.6704337316606f;
static constexpr float LOX_AREA_SI = 1.39154e-5f;
static constexpr float PSI_TO_PA = 6894.76f;
static constexpr float FUEL_CV_INJ = 0.5f;
static constexpr float FUEL_SG = 0.806f;
static constexpr float MIN_SAFE_OF = 0.5f;
static constexpr float MAX_SAFE_OF = 3.0f;
static constexpr float PTC401_ABORT_THRESHOLD = 10.0f;  //
static constexpr uint32_t PTC401_ABORT_THRESHOLD_TIME_MS = 500U;

// Controller constants (NOTE: Currently using OF = 1.4)
static inline float alpha = -1.0f;
static inline uint32_t low_ptc_start_time_ms = 0;
// 0.008283 this is for OF 1.5
static constexpr float THRUST_KP = 0.0076f;
static constexpr float MAX_CHANGE_ALPHA = 20.0f;
static constexpr float MIN_CHANGE_ALPHA = -MAX_CHANGE_ALPHA;
static constexpr float MIN_ALPHA = 0.0f;
static constexpr float MAX_ALPHA = 0.84f;
static constexpr float MIN_VALVE_POS = 25.0f;
static constexpr float MAX_VALVE_POS = 90.0f;

// Controller state variables
// TODO: James pls check
static constexpr float MAX_threshold_PT2k = 1900.0f; // Define a maximum value for sensor validation
static constexpr float MAX_threshold_PT1k = 950.0f; // Define a maximum value for sensor validation
static constexpr float MIN_threshold = 50.0f;// Define a maximum value for sensor validation

enum class CalPhase {
    SEEK_HARDSTOP,
    BACK_OFF,
    END_MOVEMENT,
    POWER_OFF,
    REPOWER,
    COMPLETE,
    MEASURE,
    ERROR,
};

static inline CalPhase cal_phase = CalPhase::SEEK_HARDSTOP;

static inline float cal_fuel_target_position = 0.0f;
static inline float cal_fuel_hardstop_position = 0.0f;
static inline bool cal_fuel_found_stop = false;
static inline float cal_lox_target_position = 0.0f;
static inline float cal_lox_hardstop_position = 0.0f;
static inline bool cal_lox_found_stop = false;
static inline float cal_step_size = 0.002f;
static inline int cal_rep_counter = 0;
static inline float cal_pos_error_limit = 0.2f;
static inline float cal_fuel_starting_error = 0.0f;
static inline float cal_lox_starting_error = 0.0f;
static inline uint32_t cal_power_cycle_timestamp = 0;

struct CalibrationValveRefs {
    float& target_position;
    float& hardstop_position;
    bool& found_stop;
    float& starting_error;
};

static bool is_supported_valve(ThrottleValveType valve)
{
    return valve == ThrottleValveType_FUEL || valve == ThrottleValveType_LOX;
}

static CalibrationValveRefs get_valve_refs(ThrottleValveType valve)
{
    if (valve == ThrottleValveType_FUEL) {
        return CalibrationValveRefs{cal_fuel_target_position, cal_fuel_hardstop_position, cal_fuel_found_stop, cal_fuel_starting_error};
    }
    return CalibrationValveRefs{cal_lox_target_position, cal_lox_hardstop_position, cal_lox_found_stop, cal_lox_starting_error};
}

static void calibration_seek_hardstop(ThrottleValveCommand& command, float valve_pos, float valve_pos_enc, CalibrationValveRefs refs)
{
    command.enable = true;

    if (!refs.found_stop && std::abs(valve_pos - (refs.starting_error + valve_pos_enc)) <= cal_pos_error_limit) {
        refs.target_position += cal_step_size / (cal_rep_counter + 1);
        command.set_pos = true;
        command.target_deg = refs.target_position;
    }
    else {
        refs.found_stop = true;
        refs.hardstop_position = valve_pos_enc;
        command.set_pos = true;
        command.target_deg = valve_pos_enc;
    }

    if (refs.found_stop) {
        refs.found_stop = false;
        cal_rep_counter++;
        cal_phase = CalPhase::END_MOVEMENT;
    }
}

static void calibration_end_movement(ThrottleValveCommand& command, uint32_t timestamp)
{
    command.enable = false;
    if (cal_power_cycle_timestamp == 0) {
        cal_power_cycle_timestamp = timestamp;
    }
    else if (timestamp - cal_power_cycle_timestamp >= 1000) {
        cal_phase = CalPhase::POWER_OFF;
    }
}

static void calibration_power_off(ThrottleValveCommand& command, uint32_t timestamp)
{
    command.enable = false;
    if (timestamp - cal_power_cycle_timestamp >= 4000) {
        cal_phase = CalPhase::REPOWER;
    }
}

static void calibration_repower(ThrottleValveCommand& command, uint32_t timestamp)
{
    command.enable = true;
    if (timestamp - cal_power_cycle_timestamp >= 5000) {
        cal_phase = CalPhase::COMPLETE;
    }
}

static void calibration_complete(ThrottleValveCommand& command, uint32_t timestamp, CalibrationValveRefs refs)
{
    command.enable = true;
    command.set_pos = true;
    command.target_deg = 95.0f;

    refs.found_stop = false;
    refs.starting_error = 0.0f;
    refs.target_position = 95.0f;

    if (timestamp - cal_power_cycle_timestamp >= 6500) {
        cal_phase = CalPhase::SEEK_HARDSTOP;
    }
}

static void calibration_error(ThrottleValveCommand& command, uint32_t timestamp)
{
    command.enable = false;

    if (cal_power_cycle_timestamp == 0) {
        cal_power_cycle_timestamp = timestamp;
    }
}

static void calibration_measure(ThrottleValveCommand& command, float valve_pos, float valve_pos_enc, CalibrationValveRefs refs)
{
    command.enable = true;

    if (valve_pos_enc > 10.0f) {
        refs.target_position -= cal_step_size * 3;
        command.set_pos = true;
        command.target_deg = refs.target_position;
    }
    else if (!refs.found_stop && std::abs(valve_pos - (refs.starting_error + valve_pos_enc)) <= cal_pos_error_limit) {
        refs.target_position -= cal_step_size;
        command.set_pos = true;
        command.target_deg = refs.target_position;
    }
    else {
        refs.found_stop = true;
        refs.hardstop_position = valve_pos_enc;
        command.set_pos = true;
        command.target_deg = valve_pos_enc;
    }

    if (refs.found_stop) {
        refs.target_position = refs.hardstop_position;
        cal_phase = CalPhase::SEEK_HARDSTOP;
    }
}

// Track duration of low chamber pressure for abort logic.
static inline float calculate_fuel_mass_flow(float p_inj_fuel, float p_ch)
{
    float dP = std::max(0.0f, p_inj_fuel - p_ch);
    return 0.06309f * FUEL_CV_INJ * std::sqrt(dP * FUEL_SG);
}

static inline float calculate_lox_mass_flow(float p_inj_lox, float p_ch)
{
    float dP_psi = std::max(0.0f, p_inj_lox - p_ch);
    float dP_Pa = dP_psi * PSI_TO_PA;
    float p_inj_safe = std::max(0.0f, p_inj_lox);
    float K_var = (K_SLOPE * p_inj_safe) + K_OFFSET;
    float rho_syn = 1141.0f + (ALPHA * p_inj_safe);
    return K_var * LOX_AREA_SI * std::sqrt(2.0f * rho_syn * dP_Pa);
}

// TODO: james pls turn into lookup table using rupin's csv

    // ISP lookup axes: chamber pressure (pc) vs O/F.
static constexpr int ISP_PC_LEN = 29;
static constexpr int ISP_OF_LEN = 34;

static constexpr float isp_pc_axis_internal[ISP_PC_LEN] = {1.0f,   16.0f,  31.0f,  46.0f,  61.0f,  76.0f,  91.0f,  100.0f, 106.0f, 115.0f,
                                                    130.0f, 145.0f, 160.0f, 175.0f, 190.0f, 205.0f, 220.0f, 235.0f, 250.0f, 265.0f,
                                                    280.0f, 295.0f, 310.0f, 325.0f, 340.0f, 355.0f, 370.0f, 385.0f, 400.0f};

static constexpr float isp_of_axis_internal[ISP_OF_LEN] = {0.1000f, 0.2000f, 0.3000f, 0.4000f, 0.5000f, 0.6000f, 0.7000f, 0.8000f, 0.9000f, 1.0000f, 1.1000f, 1.2000f,
                                                    1.2500f, 1.3000f, 1.3500f, 1.4000f, 1.4500f, 1.5000f, 1.5500f, 1.6000f, 1.7000f, 1.8000f, 1.9000f, 2.0000f,
                                                    2.1000f, 2.2000f, 2.3000f, 2.4000f, 2.5000f, 2.6000f, 2.7000f, 2.8000f, 2.9000f, 3.0000f};

static constexpr float isp_data_internal[] = {
    1210.9f, 1368.0f, 1484.1f, 1564.1f, 1626.5f, 1682.0f, 1737.2f, 1798.2f, 1865.5f, 1933.9f, 2000.1f, 2109.7f, 2189.1f, 2251.5f, 2302.1f, 2346.5f, 2385.0f,
    2417.8f, 2445.2f, 2467.5f, 2498.3f, 2513.1f, 2516.0f, 2511.5f, 2502.5f, 2491.0f, 2477.9f, 2464.2f, 2450.2f, 2436.1f, 2422.1f, 2408.3f, 2394.8f, 2381.5f,
    1251.7f, 1413.1f, 1530.5f, 1617.0f, 1685.7f, 1744.7f, 1798.7f, 1851.6f, 1906.0f, 1963.0f, 2020.5f, 2108.9f, 2177.2f, 2247.4f, 2303.5f, 2350.1f, 2391.5f,
    2428.2f, 2460.6f, 2488.8f, 2533.3f, 2563.6f, 2581.4f, 2588.3f, 2586.8f, 2579.7f, 2569.2f, 2556.6f, 2543.2f, 2529.5f, 2515.6f, 2501.7f, 2487.9f, 2474.3f,
    1259.9f, 1422.4f, 1540.7f, 1628.8f, 1699.1f, 1759.3f, 1813.6f, 1865.7f, 1918.0f, 1972.0f, 2027.3f, 2110.9f, 2176.0f, 2243.6f, 2302.9f, 2350.4f, 2392.2f,
    2429.5f, 2462.5f, 2491.6f, 2538.5f, 2571.9f, 2592.9f, 2603.2f, 2604.4f, 2599.2f, 2589.4f, 2577.1f, 2563.9f, 2550.1f, 2536.1f, 2522.0f, 2508.2f, 2494.4f,
    1264.6f, 1427.9f, 1547.0f, 1636.1f, 1707.5f, 1768.5f, 1823.4f, 1875.3f, 1926.4f, 1978.5f, 2032.0f, 2112.8f, 2175.7f, 2241.7f, 2302.1f, 2350.5f, 2392.5f,
    2430.0f, 2463.5f, 2493.0f, 2541.2f, 2576.3f, 2599.6f, 2612.3f, 2615.8f, 2612.2f, 2603.5f, 2591.7f, 2578.6f, 2564.9f, 2550.9f, 2536.9f, 2522.9f, 2509.1f,
    1267.9f, 1431.9f, 1551.6f, 1641.6f, 1713.9f, 1775.6f, 1830.9f, 1882.9f, 1933.5f, 1984.4f, 2036.3f, 2114.4f, 2175.7f, 2240.5f, 2301.4f, 2350.4f, 2392.7f,
    2430.4f, 2464.1f, 2493.9f, 2543.0f, 2579.2f, 2604.2f, 2618.9f, 2624.5f, 2622.5f, 2614.7f, 2603.6f, 2590.7f, 2577.1f, 2563.2f, 2549.2f, 2535.2f, 2521.3f,
    1270.4f, 1434.9f, 1555.2f, 1645.8f, 1718.8f, 1781.0f, 1836.6f, 1888.6f, 1939.2f, 1990.0f, 2041.4f, 2115.9f, 2175.9f, 2239.7f, 2300.7f, 2350.4f, 2392.8f,
    2430.6f, 2464.5f, 2494.5f, 2544.4f, 2581.4f, 2606.9f, 2622.0f, 2628.4f, 2628.0f, 2621.1f, 2610.7f, 2598.1f, 2584.8f, 2571.0f, 2557.1f, 2543.2f, 2529.3f,
    1272.4f, 1437.4f, 1558.0f, 1649.3f, 1722.9f, 1785.5f, 1841.5f, 1893.5f, 1943.8f, 1993.9f, 2044.5f, 2117.2f, 2176.2f, 2239.1f, 2300.1f, 2350.2f, 2392.9f,
    2430.8f, 2464.8f, 2495.0f, 2545.4f, 2583.1f, 2609.7f, 2625.9f, 2633.3f, 2632.4f, 2625.6f, 2615.2f, 2602.6f, 2589.2f, 2575.4f, 2561.4f, 2547.4f, 2533.5f,
    1273.2f, 1438.2f, 1559.0f, 1650.5f, 1724.2f, 1787.1f, 1843.2f, 1895.2f, 1945.5f, 1995.6f, 2046.1f, 2118.0f, 2176.4f, 2238.8f, 2299.8f, 2350.1f, 2392.9f,
    2430.9f, 2464.9f, 2495.2f, 2545.7f, 2583.6f, 2610.2f, 2626.6f, 2633.6f, 2633.0f, 2626.4f, 2615.9f, 2603.3f, 2589.9f, 2576.1f, 2562.1f, 2548.1f, 2534.2f,
    1273.8f, 1438.9f, 1559.8f, 1651.5f, 1725.4f, 1788.4f, 1844.6f, 1896.6f, 1946.9f, 1996.8f, 2047.2f, 2118.4f, 2176.5f, 2238.7f, 2299.6f, 2350.1f, 2393.0f,
    2431.0f, 2465.0f, 2495.4f, 2546.0f, 2584.1f, 2611.0f, 2627.6f, 2634.9f, 2634.5f, 2628.1f, 2617.7f, 2605.2f, 2591.8f, 2578.0f, 2564.0f, 2550.0f, 2536.1f,
    1274.8f, 1440.1f, 1561.2f, 1653.3f, 1727.4f, 1790.7f, 1847.0f, 1899.1f, 1949.2f, 1998.9f, 2049.0f, 2119.1f, 2176.7f, 2238.5f, 2299.3f, 2350.0f, 2393.0f,
    2431.0f, 2465.1f, 2495.6f, 2546.5f, 2584.8f, 2612.1f, 2629.2f, 2637.0f, 2637.1f, 2630.9f, 2620.7f, 2608.3f, 2594.9f, 2581.2f, 2567.2f, 2553.2f, 2539.3f,
    1276.4f, 1442.0f, 1563.4f, 1656.1f, 1730.6f, 1794.2f, 1850.8f, 1903.0f, 1952.9f, 2002.2f, 2052.0f, 2120.3f, 2177.0f, 2238.2f, 2298.9f, 2349.8f, 2393.0f,
    2431.1f, 2465.3f, 2495.8f, 2547.3f, 2586.0f, 2614.0f, 2631.8f, 2640.4f, 2641.1f, 2635.5f, 2625.5f, 2613.3f, 2599.9f, 2586.2f, 2572.2f, 2558.2f, 2544.3f,
    1277.9f, 1443.9f, 1565.6f, 1658.9f, 1733.8f, 1797.8f, 1854.6f, 1907.0f, 1956.6f, 2005.5f, 2054.9f, 2121.3f, 2177.4f, 2238.0f, 2298.5f, 2349.6f, 2393.0f,
    2431.2f, 2465.4f, 2496.1f, 2548.1f, 2587.2f, 2615.9f, 2634.3f, 2643.8f, 2645.2f, 2640.1f, 2630.3f, 2618.3f, 2604.9f, 2591.3f, 2577.3f, 2563.3f, 2549.4f,
    1279.5f, 1445.8f, 1567.8f, 1661.7f, 1737.0f, 1801.4f, 1858.4f, 1910.9f, 1960.3f, 2008.8f, 2057.8f, 2122.3f, 2177.7f, 2237.8f, 2298.2f, 2349.5f, 2393.0f,
    2431.3f, 2465.6f, 2496.3f, 2548.9f, 2588.4f, 2617.8f, 2636.9f, 2647.2f, 2649.3f, 2644.6f, 2635.0f, 2623.3f, 2609.9f, 2596.4f, 2582.4f, 2568.4f, 2554.5f,
    1281.0f, 1447.8f, 1570.0f, 1664.5f, 1740.2f, 1805.0f, 1862.1f, 1914.8f, 1964.0f, 2012.0f, 2060.7f, 2123.3f, 2178.1f, 2237.7f, 2297.8f, 2349.3f, 2393.0f,
    2431.4f, 2465.7f, 2496.4f, 2549.7f, 2589.6f, 2619.7f, 2639.5f, 2650.6f, 2653.4f, 2649.2f, 2639.8f, 2628.2f, 2614.9f, 2601.5f, 2587.5f, 2573.5f, 2559.6f,
    1282.5f, 1449.7f, 1572.2f, 1667.3f, 1743.4f, 1808.5f, 1865.9f, 1918.7f, 1967.8f, 2015.3f, 2063.6f, 2124.2f, 2178.4f, 2237.6f, 2297.6f, 2349.1f, 2393.0f,
    2431.4f, 2465.8f, 2496.6f, 2550.4f, 2590.8f, 2621.7f, 2642.1f, 2654.0f, 2657.4f, 2653.8f, 2644.6f, 2633.2f, 2620.0f, 2606.6f, 2592.6f, 2578.6f, 2564.7f,
    1284.1f, 1451.6f, 1574.4f, 1670.0f, 1746.5f, 1812.1f, 1869.7f, 1922.6f, 1971.5f, 2018.6f, 2066.5f, 2125.1f, 2178.8f, 2237.5f, 2297.3f, 2348.9f, 2393.0f,
    2431.5f, 2465.9f, 2496.7f, 2551.2f, 2592.0f, 2623.6f, 2644.6f, 2657.4f, 2661.5f, 2658.3f, 2649.3f, 2638.1f, 2625.0f, 2611.7f, 2597.7f, 2583.7f, 2569.8f,
    1285.6f, 1453.5f, 1576.6f, 1672.8f, 1749.7f, 1815.6f, 1873.5f, 1926.5f, 1975.2f, 2021.9f, 2069.4f, 2126.0f, 2179.1f, 2237.5f, 2297.1f, 2348.7f, 2393.0f,
    2431.5f, 2465.9f, 2496.8f, 2552.0f, 2593.2f, 2625.5f, 2647.2f, 2660.8f, 2665.6f, 2662.9f, 2654.1f, 2643.1f, 2630.0f, 2616.8f, 2602.8f, 2588.8f, 2574.9f,
    1287.1f, 1455.5f, 1578.8f, 1675.6f, 1752.9f, 1819.2f, 1877.3f, 1930.4f, 1978.9f, 2025.2f, 2072.3f, 2126.8f, 2179.4f, 2237.4f, 2296.9f, 2348.6f, 2393.0f,
    2431.5f, 2466.0f, 2497.0f, 2552.8f, 2594.5f, 2627.4f, 2649.8f, 2664.2f, 2669.7f, 2667.5f, 2658.9f, 2648.1f, 2635.0f, 2621.8f, 2607.9f, 2593.9f, 2580.0f,
    1288.7f, 1457.4f, 1580.9f, 1678.4f, 1756.1f, 1822.8f, 1881.0f, 1934.3f, 1982.6f, 2028.5f, 2075.2f, 2127.6f, 2179.8f, 2237.4f, 2296.7f, 2348.4f, 2393.0f,
    2431.6f, 2466.1f, 2497.1f, 2553.6f, 2595.7f, 2629.3f, 2652.3f, 2667.6f, 2673.7f, 2672.0f, 2663.7f, 2653.0f, 2640.0f, 2626.9f, 2613.0f, 2599.0f, 2585.1f,
    1290.2f, 1459.3f, 1583.1f, 1681.2f, 1759.3f, 1826.3f, 1884.8f, 1938.2f, 1986.3f, 2031.7f, 2078.1f, 2128.4f, 2180.1f, 2237.4f, 2296.5f, 2348.2f, 2392.9f,
    2431.6f, 2466.1f, 2497.1f, 2554.4f, 2596.9f, 2631.2f, 2654.9f, 2670.9f, 2677.8f, 2676.6f, 2668.4f, 2658.0f, 2645.1f, 2632.0f, 2618.1f, 2604.1f, 2590.2f,
    1291.7f, 1461.2f, 1585.3f, 1683.9f, 1762.5f, 1829.9f, 1888.6f, 1942.1f, 1990.0f, 2035.0f, 2081.0f, 2129.1f, 2180.4f, 2237.5f, 2296.3f, 2348.1f, 2392.9f,
    2431.6f, 2466.2f, 2497.2f, 2555.2f, 2598.1f, 2633.1f, 2657.5f, 2674.3f, 2681.9f, 2681.2f, 2673.2f, 2663.0f, 2650.1f, 2637.1f, 2623.2f, 2609.2f, 2595.3f,
    1293.2f, 1463.1f, 1587.5f, 1686.7f, 1765.7f, 1833.5f, 1892.4f, 1946.0f, 1993.7f, 2038.3f, 2083.9f, 2129.9f, 2180.8f, 2237.5f, 2296.2f, 2347.9f, 2392.8f,
    2431.6f, 2466.2f, 2497.3f, 2555.9f, 2599.3f, 2635.0f, 2660.1f, 2677.7f, 2686.0f, 2685.7f, 2678.0f, 2668.0f, 2655.1f, 2642.1f, 2628.3f, 2614.3f, 2600.4f,
    1294.8f, 1465.1f, 1589.7f, 1689.5f, 1768.8f, 1837.0f, 1896.2f, 1949.9f, 1997.4f, 2041.6f, 2086.8f, 2130.6f, 2181.1f, 2237.5f, 2296.1f, 2347.8f, 2392.8f,
    2431.7f, 2466.3f, 2497.4f, 2556.7f, 2600.5f, 2636.9f, 2662.6f, 2681.1f, 2690.0f, 2690.3f, 2682.8f, 2672.9f, 2660.1f, 2647.2f, 2633.4f, 2619.4f, 2605.5f,
    1296.3f, 1467.0f, 1591.9f, 1692.3f, 1772.0f, 1840.6f, 1899.9f, 1953.8f, 2001.1f, 2044.9f, 2089.7f, 2131.3f, 2181.4f, 2237.6f, 2295.9f, 2347.6f, 2392.8f,
    2431.7f, 2466.3f, 2497.4f, 2557.5f, 2601.7f, 2638.8f, 2665.2f, 2684.5f, 2694.1f, 2694.9f, 2687.5f, 2677.9f, 2665.2f, 2652.3f, 2638.4f, 2624.5f, 2610.6f,
    1297.8f, 1468.9f, 1594.1f, 1695.1f, 1775.2f, 1844.2f, 1903.7f, 1957.6f, 2004.8f, 2048.2f, 2092.6f, 2131.9f, 2181.7f, 2237.6f, 2295.8f, 2347.5f, 2392.7f,
    2431.7f, 2466.4f, 2497.5f, 2558.3f, 2602.9f, 2640.7f, 2667.8f, 2687.9f, 2698.2f, 2699.4f, 2692.3f, 2682.9f, 2670.2f, 2657.4f, 2643.5f, 2629.6f, 2615.7f,
    1299.3f, 1470.8f, 1596.3f, 1697.8f, 1778.4f, 1847.7f, 1907.5f, 1961.5f, 2008.5f, 2051.5f, 2095.5f, 2132.6f, 2182.0f, 2237.7f, 2295.7f, 2347.3f, 2392.7f,
    2431.7f, 2466.4f, 2497.6f, 2559.1f, 2604.1f, 2642.6f, 2670.4f, 2691.3f, 2702.3f, 2704.0f, 2697.1f, 2687.8f, 2675.2f, 2662.4f, 2648.6f, 2634.7f, 2620.8f,
    1300.9f, 1472.7f, 1598.5f, 1700.6f, 1781.6f, 1851.3f, 1911.3f, 1965.4f, 2012.2f, 2054.8f, 2098.4f, 2133.3f, 2182.3f, 2237.7f, 2295.6f, 2347.2f, 2392.6f,
    2431.7f, 2466.4f, 2497.6f, 2559.8f, 2605.3f, 2644.5f, 2672.9f, 2694.6f, 2706.3f, 2708.6f, 2701.8f, 2692.8f, 2680.2f, 2667.5f, 2653.7f, 2639.8f, 2625.9f,
    1302.4f, 1474.6f, 1600.7f, 1703.4f, 1784.8f, 1854.9f, 1915.1f, 1969.3f, 2015.9f, 2058.1f, 2101.3f, 2133.9f, 2182.7f, 2237.8f, 2295.5f, 2347.1f, 2392.6f,
    2431.7f, 2466.5f, 2497.7f, 2560.6f, 2606.5f, 2646.4f, 2675.5f, 2698.0f, 2710.4f, 2713.2f, 2706.6f, 2697.8f, 2685.3f, 2672.6f, 2658.8f, 2644.9f, 2631.0f,
    1303.9f, 1476.5f, 1602.8f, 1706.2f, 1787.9f, 1858.4f, 1918.9f, 1973.2f, 2019.6f, 2061.4f, 2104.2f, 2134.5f, 2183.0f, 2237.8f, 2295.4f, 2346.9f, 2392.5f,
    2431.7f, 2466.5f, 2497.7f, 2561.4f, 2607.7f, 2648.3f, 2678.1f, 2701.4f, 2714.5f, 2717.7f, 2711.4f, 2702.7f, 2690.3f, 2677.7f, 2663.9f, 2650.0f, 2636.1f,
};

// MPrime lookup axes: target thrust (lbf) vs O/F.
static constexpr int THRUST_AXIS_LEN = 100;
static constexpr int OF_AXIS_LEN = 100;

static constexpr float thrust_axis_internal[] = {
    350.000000f, 354.040404f, 358.080808f, 362.121212f, 366.161616f, 370.202020f, 374.242424f, 378.282828f, 382.323232f, 386.363636f, 390.404040f, 394.444444f,
    398.484848f, 402.525253f, 406.565657f, 410.606061f, 414.646465f, 418.686869f, 422.727273f, 426.767677f, 430.808081f, 434.848485f, 438.888889f, 442.929293f,
    446.969697f, 451.010101f, 455.050505f, 459.090909f, 463.131313f, 467.171717f, 471.212121f, 475.252525f, 479.292929f, 483.333333f, 487.373737f, 491.414141f,
    495.454545f, 499.494949f, 503.535354f, 507.575758f, 511.616162f, 515.656566f, 519.696970f, 523.737374f, 527.777778f, 531.818182f, 535.858586f, 539.898990f,
    543.939394f, 547.979798f, 552.020202f, 556.060606f, 560.101010f, 564.141414f, 568.181818f, 572.222222f, 576.262626f, 580.303030f, 584.343434f, 588.383838f,
    592.424242f, 596.464646f, 600.505051f, 604.545455f, 608.585859f, 612.626263f, 616.666667f, 620.707071f, 624.747475f, 628.787879f, 632.828283f, 636.868687f,
    640.909091f, 644.949495f, 648.989899f, 653.030303f, 657.070707f, 661.111111f, 665.151515f, 669.191919f, 673.232323f, 677.272727f, 681.313131f, 685.353535f,
    689.393939f, 693.434343f, 697.474747f, 701.515152f, 705.555556f, 709.595960f, 713.636364f, 717.676768f, 721.717172f, 725.757576f, 729.797980f, 733.838384f,
    737.878788f, 741.919192f, 745.959596f, 750.000000f};


/// Reset internal state before an active control trace
void RangerThrottle::reset()
{
    MutexGuard ranger_throttle_guard{&ranger_throttle_lock};
    low_ptc_start_time_ms = 0;
    alpha = -1.0f;
}

// TODO: test the re-done single valve calibration. Also, make it not super slow
std::expected<ThrottleValveCommand, Error>
RangerThrottle::calibration_tick(ThrottleValveType valve, uint32_t timestamp, float valve_pos, float valve_pos_enc)
{
    MutexGuard ranger_throttle_guard{&ranger_throttle_lock};

    if (!is_supported_valve(valve)) {
        return std::unexpected(Error::from_cause("unknown valve passed to calibration_tick"));
    }

    auto refs = get_valve_refs(valve);
    ThrottleValveCommand command = ThrottleValveCommand_init_default;
    command.enable = true;

    switch (cal_phase) {
    case CalPhase::SEEK_HARDSTOP:
        calibration_seek_hardstop(command, valve_pos, valve_pos_enc, refs);
        break;
    case CalPhase::BACK_OFF:
        break;
    case CalPhase::END_MOVEMENT:
        calibration_end_movement(command, timestamp);
        break;
    case CalPhase::POWER_OFF:
        calibration_power_off(command, timestamp);
        break;
    case CalPhase::REPOWER:
        calibration_repower(command, timestamp);
        break;
    case CalPhase::COMPLETE:
        calibration_complete(command, timestamp, refs);
        break;
    case CalPhase::MEASURE:
        calibration_measure(command, valve_pos, valve_pos_enc, refs);
        break;
    case CalPhase::ERROR:
        calibration_error(command, timestamp);
        break;
    default:
        break;
    }

    return command;
}

void RangerThrottle::calibration_reset(ThrottleValveType valve, float valve_pos, float valve_pos_enc)
{
    MutexGuard ranger_throttle_guard{&ranger_throttle_lock};

    if (!is_supported_valve(valve)) {
        return;
    }

    auto refs = get_valve_refs(valve);

    cal_phase = CalPhase::SEEK_HARDSTOP;
    cal_rep_counter = 0;
    refs.found_stop = false;
    refs.hardstop_position = 0.0f;
    cal_power_cycle_timestamp = 0;

    refs.target_position = valve_pos_enc;

    refs.starting_error = valve_pos - valve_pos_enc;
}

    static std::expected<float, Error> thrust_predictor(AnalogSensorReadings& analog_sensors, RangerThrottleMetrics& metrics)
{
    // Safety: abort if PTC401 is below threshold for too long.
    uint32_t current_time = (uint32_t)k_uptime_get();
    if (analog_sensors.ptc1 <= PTC401_ABORT_THRESHOLD) {
        if (low_ptc_start_time_ms == 0) {
            low_ptc_start_time_ms = current_time;
        }
        else if (current_time - low_ptc_start_time_ms > PTC401_ABORT_THRESHOLD_TIME_MS) {
            return std::unexpected(Error::from_cause("PTC401 < %f for >%u ms in THRUST_SEQ, aborting.", (double)PTC401_ABORT_THRESHOLD, PTC401_ABORT_THRESHOLD_TIME_MS));
        }
    }
    else {
        low_ptc_start_time_ms = 0;
    }

    // 2. Read pressures
    // TODO: these were also battery voltage? That seems wrong
    // TODO: ARRE THESE THE CORRECT PTS? NEEDS TO BE UPDATED FOR RANGER
    // TODO: james, rupin says he doesnt know what to do - He cant find the P&ID
    float ptc401_val = analog_sensors.ptc1;
    float pto401_val = analog_sensors.pto1 + LOX_ENGINE_INLET_LINE_LOSS_PSI;  // Adjusted value
    float pt103_val = analog_sensors.pt101;
    float ptf401_val = analog_sensors.ptf1 + FUEL_ENGINE_INLET_LINE_LOSS_PSI; // Adjusted value
    float pt203_val = analog_sensors.pt201;
    float ptc402_val = analog_sensors.ptc2;
    bool pt203_valid = (analog_sensors.pt201 >= MIN_threshold && analog_sensors.pt201 <= MAX_threshold_PT2k);
    bool ptf401_valid = (analog_sensors.ptf1 >= MIN_threshold && analog_sensors.ptf1 <= MAX_threshold_PT2k);
    bool pt103_valid = (analog_sensors.pt101 >= MIN_threshold && analog_sensors.pt101 <= MAX_threshold_PT2k);
    bool pto401_valid = (analog_sensors.pto1 >= MIN_threshold && analog_sensors.pto1 <= MAX_threshold_PT2k);
    bool ptc401_valid = (analog_sensors.ptc1 >= MIN_threshold && analog_sensors.ptc1 <= MAX_threshold_PT1k);
    bool ptc402_valid = (analog_sensors.ptc2 >= MIN_threshold && analog_sensors.ptc2 <= MAX_threshold_PT1k);

    float p_inj_fuel;
    float p_inj_lox;
    float p_ch;
    if (ptc401_valid && ptc402_valid) {
        // Both are healthy: Take the average
        p_ch = (ptc401_val + ptc402_val) / 2.0f;
    }
    else if (ptc401_valid) {
        // Only primary is healthy
        p_ch = ptc401_val;
    }
    else if (ptc402_valid) {
        // Only backup is healthy
        p_ch = ptc402_val;
    }
    else {
        return std::unexpected(Error::from_cause("NO PTC 401 / PTC402"));
    }

    if (pt203_valid && ptf401_valid) {
        // Both are healthy: Take the average
        p_inj_fuel = (pt203_val + ptf401_val) / 2.0f;
    }
    else if (pt203_valid) {
        // Only primary is healthy
        p_inj_fuel = pt203_val;
    }
    else if (ptf401_valid) {
        // Only backup is healthy
        p_inj_fuel = ptf401_val;
    }
    else {
        // Both failed: Trigger Abort
        return std::unexpected(Error::from_cause("NO PT 203 / PTF401"));
    }
    if (pt103_valid && pto401_valid) {
        // Both are healthy: Take the average
        p_inj_lox = (pt103_val + pto401_val) / 2.0f;
    }
    else if (pt103_valid) {
        // Only primary is healthy
        p_inj_lox = pt103_val;
    }
    else if (pto401_valid) {
        // Only backup is healthy
        p_inj_lox = pto401_val;
    }
    else {
        // Both failed: Trigger Abort
        return std::unexpected(Error::from_cause("NO PT 103 / PTO401"));
    }

    // 3. Calculate mass flows
    float mdot_f = calculate_fuel_mass_flow(p_inj_fuel, p_ch);
    float mdot_lox = calculate_lox_mass_flow(p_inj_lox, p_ch);

    // 4. Clamp fuel mass flow to avoid division by zero
    float mdot_f_safe = std::max(mdot_f, 0.001f);

    // 5. Calculate O/F
    float predicted_of = mdot_lox / mdot_f_safe;

    // 6. Clamp O/F for lookup
    [[maybe_unused]] float of_safe = std::clamp(predicted_of, MIN_SAFE_OF, MAX_SAFE_OF);

    // TODO: add lut for cea
    // 7. Predict Isp using chamber pressure and O/F
    float predicted_isp = PcOfCea::sample(p_ch, predicted_of);

    // 8. Predict thrust (convert to lbf-equivalent)
    float predicted_thrust_lbf = (mdot_f + mdot_lox) * predicted_isp * EFFICIENCY * LBF_CONVERSION;

    metrics.predicted_thrust_lbf = 0;
    metrics.predicted_thrust_lbf = predicted_thrust_lbf;

    metrics.predicted_of = predicted_of;
    metrics.mdot_fuel = mdot_f;
    metrics.mdot_lox = mdot_lox;

    // return predicted_thrust_lbf;
    return 0.0f;
}

static std::tuple<ThrottleValveCommand, ThrottleValveCommand> active_control(float& alpha_state, float predicted_thrust_lbf, float thrust_command_lbf, RangerThrottleMetrics& metrics)
{
    float dt = Controller::SEC_PER_CONTROL_TICK;
    float thrust_error = thrust_command_lbf - predicted_thrust_lbf;
    float change_alpha_cmd = THRUST_KP * thrust_error;
    change_alpha_cmd *= dt;
    float clamped_change_alpha_cmd = std::clamp(change_alpha_cmd, MIN_CHANGE_ALPHA, MAX_CHANGE_ALPHA);

    // 10. Integrate PID to get alpha
    if (alpha_state == -1.0f) {
        // Initialize alpha to starting guess based on Mprime
        alpha_state = (thrust_command_lbf - thrust_axis_internal[0]) / (thrust_axis_internal[100 - 1] - thrust_axis_internal[0]);
    }
    alpha_state += clamped_change_alpha_cmd;
    alpha_state = std::clamp(alpha_state, MIN_ALPHA, MAX_ALPHA);

    // 11. Plug alpha into Mprime contour
    float thrust_from_alpha_lbf = alpha_state * (thrust_axis_internal[100 - 1] - thrust_axis_internal[0]) + thrust_axis_internal[0];
    float fuel_valve_command_deg = ThrustToFuelAxis::sample(thrust_from_alpha_lbf);
    float lox_valve_command_deg = ThrustToLoxAxis::sample(thrust_from_alpha_lbf);

    // 12. Clamp valve commands to safe ranges
    fuel_valve_command_deg = std::clamp(fuel_valve_command_deg, MIN_VALVE_POS, MAX_VALVE_POS);
    lox_valve_command_deg = std::clamp(lox_valve_command_deg, MIN_VALVE_POS, MAX_VALVE_POS);

    metrics.change_alpha_cmd = change_alpha_cmd;
    metrics.clamped_change_alpha_cmd = clamped_change_alpha_cmd;
    metrics.alpha = alpha_state;
    metrics.thrust_from_alpha_lbf = thrust_from_alpha_lbf;

    return {
        ThrottleValveCommand{.enable = true, .set_pos = true, .target_deg = fuel_valve_command_deg},
        ThrottleValveCommand{.enable = true, .set_pos = true, .target_deg = lox_valve_command_deg}};
}

/// Generate a comomand for the fuel and lox valve positions in degrees.
std::expected<std::tuple<ThrottleValveCommand, ThrottleValveCommand, RangerThrottleMetrics>, Error> RangerThrottle::tick(AnalogSensorReadings& analog_sensors, float thrust_command_lbf)
{
    MutexGuard ranger_throttle_guard{&ranger_throttle_lock};
    RangerThrottleMetrics metrics = RangerThrottleMetrics_init_default;

    auto predicted_thrust = thrust_predictor(analog_sensors, metrics);
    if (!predicted_thrust) {
        return std::unexpected(predicted_thrust.error());
    }

    auto [fuel_command, lox_command] = active_control(alpha, *predicted_thrust, thrust_command_lbf, metrics);

    return {{fuel_command, lox_command, metrics}};
}

#if CONFIG_TEST
std::tuple<ThrottleValveCommand, ThrottleValveCommand> RangerThrottle::active_control_test(float& alpha_state, float predicted_thrust_lbf, float thrust_command_lbf, RangerThrottleMetrics& metrics)
{
    return ::active_control(alpha_state, predicted_thrust_lbf, thrust_command_lbf, metrics);
}
#endif
