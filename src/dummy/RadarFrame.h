#pragma once
#define RADAR_FRAME_H

#include <vector>
#include <cstdint>
#include "RadarPoint.h"

struct RadarFrame
{
    uint32_t timestamp;
    std::vector<RadarPoint> points;
};
