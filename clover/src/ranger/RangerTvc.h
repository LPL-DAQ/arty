#pragma once

#include "Error.h"
#include "clover.pb.h"
#include <expected>
#include <tuple>

namespace RangerTvc {

    struct TravelRange{
        float min_rev;
        float max_rev;
    };

    void reset();

    std::expected<TravelRange, Error> home();
    std::expected<std::tuple<TvcActuatorCommand, TvcActuatorCommand, RangerTvcMetrics>, Error> tick(float position);
}  // namespace RangerTvc
