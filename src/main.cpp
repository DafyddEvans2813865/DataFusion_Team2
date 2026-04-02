#include <iostream>
#include <iomanip>
#include <vector>

#include "dummy/RadarDummy.h"
#include "imu/IMU.h"
#include "imu/IMUBuffer.h"

int main() {

    // Create an IMU buffer to collect continuous data
    IMUBuffer imuBuffer;
    
    // Simulate incoming IMU data (adding multiple readings)
    imuBuffer.addReading(IMU(257400, 257.4, 0.1742, -0.0766,
                             -0.0126, 0.0046, 0.0399,
                             -0.035, -0.0353, -9.7809, 3, 0, 0));
    
    imuBuffer.addReading(IMU(257410, 257.41, 0.1742, -0.077,
                             -0.0179, 0.0121, 0.0854,
                             -0.0429, -0.0418, -9.759, 3, 0, 0));
    
    imuBuffer.addReading(IMU(257420, 257.42, 0.1737, -0.0767,
                             -0.0627, 0.0408, 0.0658,
                             -0.0379, -0.0266, -9.7689, 3, 0, 0));
    
    std::cout << "IMU Data Stream\n";
    std::cout << "===============\n";
    imuBuffer.printAll();
    std::cout << "\n";
    
    // Create a RadarDummy with 10 points
    RadarDummy radar(10);

    // Generate dummy radar data
    std::vector<RadarPoint> data = radar.generateData();

    // Print the points (clean + readable)
    std::cout << "Radar Dummy Data\n";
    std::cout << "----------------\n";

    for (const auto& d : data) {
        std::cout << std::fixed << std::setprecision(2)
                  << "(" << d.x << ", " << d.y << ")\n";
    }

    return 0;
}
