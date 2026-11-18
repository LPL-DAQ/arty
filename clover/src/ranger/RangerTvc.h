#pragma once

#include "Error.h"
#include "clover.pb.h"
#include <expected>
#include <optional>
#include <tuple>

namespace RangerTvc {

struct TravelRange {
    float min_rev; 
    float max_rev;
};    

std::expected<TravelRange, Error> home(); //home sequence
std::optional<TravelRange> travel(); //returns last home range
void reset();
std::expected<std::tuple<TvcActuatorCommand, TvcActuatorCommand, RangerTvcMetrics>, Error> tick(float position);
}  // namespace RangerTvc
