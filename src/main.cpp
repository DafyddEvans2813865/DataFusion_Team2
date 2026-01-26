#include <iostream>
#include <iomanip>
#include <vector>

#include "dummy/RadarDummy.h"

int main() {
    // Create a RadarDummy with 10 points
    RadarDummy radar(10);

    // Generate dummy radar data
    std::vector<RadarDetection> data = radar.generateData();

    // Print the points (clean + readable)
    std::cout << "Radar Dummy Data\n";
    std::cout << "----------------\n";

    for (const auto& d : data) {
        std::cout << std::fixed << std::setprecision(2)
                  << "(" << d.x << ", " << d.y << ")\n";
    }

    return 0;
}
