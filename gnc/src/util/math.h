#pragma once
#include <cstdint>

namespace util
{
  template <typename T>
  inline T clamp(T v, T lo, T hi)
  {
    return (v < lo) ? lo : (v > hi) ? hi
                                    : v;
  }

  // declaration only
  float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

  // declaration only
  uint16_t angleDegToUs(float deg, float minDeg, float maxDeg,
                        uint16_t minUs, uint16_t maxUs);
}
