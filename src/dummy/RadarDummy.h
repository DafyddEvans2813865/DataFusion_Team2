#pragma once

#include "RadarDetection.h"
#include <vector>
#include <utility>

class RadarDummy {
public:
    // Constructor
    RadarDummy(int numPoints = 60);
    // Generate dummy radar data: returns vector of (x, y) coordinates
    //"A time-stamped frame containing a variable-length list of detected points, each with position, velocity, and signal strength."
    std::vector<RadarDetection> generateData() const; 

private:
    int numPoints; 
};

