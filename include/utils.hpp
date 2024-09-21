#pragma once

#include <vector>
#include <utility>
#include "autonomy_simulator/RoverPose.h"

std::vector<std::vector<int8_t>> generateMap(size_t);

inline std::pair<int8_t, int8_t> rotateRight(const std::pair<int8_t, int8_t> xy) {
    return { xy.second, -xy.first };
}

inline std::pair<int8_t, int8_t> deltaInDirection(
    std::pair<int8_t, int8_t> xy,
    const autonomy_simulator::RoverPose::_orientation_type direciton) {
    for(int8_t i = 0; i < direciton; ++i) {
        xy = rotateRight(xy);
    }
    return xy;
}
