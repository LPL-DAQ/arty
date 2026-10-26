// Bare-Teensy TC check: AD8495 breakout OUT -> A0, Vin -> 3.3V, GND -> GND. NEVER power the amp from 5 V here:
// Teensy 4.x analog pins only tolerate 3.3 V.
// Prints counts, volts, and degC two ways, ~10 lines/s over USB serial.
// The temperature conversion is the same as Ranger firmware (clover AnalogSensors.cpp + NIST ITS-90 K inverse),
// so slopes compare; offsets don't (different ADC).
#include <Arduino.h>

// Teensy ADC reference is its 3.3 V rail. Meter the 3.3V pin and put the real value here.
static const float VREF = 3.29f;  // metered 2026-10-20
static const int PIN = A0;
static const int BITS = 12;

// NIST ITS-90 type K inverse, E in mV -> degC. -200..0 C and 0..500 C ranges.
static float k_mv_to_c(float e)
{
    static const float neg[] = {0.0f, 2.5173462e1f, -1.1662878f, -1.0833638f, -8.9773540e-1f,
                                -3.7342377e-1f, -8.6632643e-2f, -1.0450598e-2f, -5.1920577e-4f};
    static const float pos[] = {0.0f, 2.508355e1f, 7.860106e-2f, -2.503131e-1f, 8.315270e-2f,
                                -1.228034e-2f, 9.804036e-4f, -4.413030e-5f, 1.057734e-6f, -1.052755e-8f};
    const float* d = e < 0 ? neg : pos;
    int n = e < 0 ? 9 : 10;
    float t = 0, p = 1;
    for (int i = 0; i < n; ++i) {
        t += d[i] * p;
        p *= e;
    }
    return t;
}

void setup()
{
    Serial.begin(115200);
    analogReadResolution(BITS);
    analogReadAveraging(32);
}

void loop()
{
    // Average 64 more reads to beat the Teensy ADC noise down
    uint32_t sum = 0;
    for (int i = 0; i < 64; ++i) sum += analogRead(PIN);
    float counts = sum / 64.0f;
    float v = counts / ((1 << BITS) - 1) * VREF;
    float simple_c = (v - 1.25f) / 0.005f;                    // datasheet 5 mV/C
    float tc_mv = (v - 1.25f - 0.00125f) / 122.4f * 1000.0f;  // same as Ranger firmware
    float nist_c = k_mv_to_c(tc_mv);
    Serial.printf("counts=%7.1f  V=%.4f  simple=%6.2f C  nist=%6.2f C\n", counts, v, simple_c, nist_c);
    delay(100);
}
