#include "IMU.h"
#include <iostream>
#include <iomanip>

IMU::IMU() 
    : timeITOW(0), time(0.0), roll(0.0), pitch(0.0),
      xRate(0.0), yRate(0.0), zRate(0.0),
      xAccel(0.0), yAccel(0.0), zAccel(0.0),
      opMode(0), linAccSw(0), turnSw(0) {}

IMU::IMU(long timeITOW_val, double time_val, double roll_val, double pitch_val,
         double xRate_val, double yRate_val, double zRate_val,
         double xAccel_val, double yAccel_val, double zAccel_val,
         int opMode_val, int linAccSw_val, int turnSw_val)
    : timeITOW(timeITOW_val), time(time_val), roll(roll_val), pitch(pitch_val),
      xRate(xRate_val), yRate(yRate_val), zRate(zRate_val),
      xAccel(xAccel_val), yAccel(yAccel_val), zAccel(zAccel_val),
      opMode(opMode_val), linAccSw(linAccSw_val), turnSw(turnSw_val) {}

void IMU::print() const {
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "IMU Data:\n";
    std::cout << "  Time: " << time << "s (ITOW: " << timeITOW << "ms)\n";
    std::cout << "  Orientation: roll=" << roll << "°, pitch=" << pitch << "°\n";
    std::cout << "  Gyroscope: x=" << xRate << "°/s, y=" << yRate << "°/s, z=" << zRate << "°/s\n";
    std::cout << "  Accelerometer: x=" << xAccel << "m/s², y=" << yAccel << "m/s², z=" << zAccel << "m/s²\n";
    std::cout << "  Status: opMode=" << opMode << ", linAccSw=" << linAccSw << ", turnSw=" << turnSw << "\n";
}
