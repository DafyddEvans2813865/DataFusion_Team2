#include "IMUBuffer.h"

IMUBuffer::IMUBuffer(size_t capacity) : maxCapacity(capacity) {}

void IMUBuffer::addReading(const IMU& imu) {
    readings.push_back(imu);
    
    // If we have a capacity limit and exceed it, remove oldest reading
    if (maxCapacity > 0 && readings.size() > maxCapacity) {
        readings.erase(readings.begin());
    }
}

size_t IMUBuffer::size() const {
    return readings.size();
}

const IMU& IMUBuffer::getReading(size_t index) const {
    if (index >= readings.size()) {
        throw std::out_of_range("IMU reading index out of range");
    }
    return readings[index];
}

const IMU& IMUBuffer::getLatest() const {
    if (readings.empty()) {
        throw std::runtime_error("IMU buffer is empty");
    }
    return readings.back();
}

void IMUBuffer::clear() {
    readings.clear();
}

void IMUBuffer::printAll() const {
    if (readings.empty()) {
        std::cout << "IMU Buffer is empty\n";
        return;
    }
    
    std::cout << "IMU Buffer (" << readings.size() << " readings)\n";
    std::cout << "============================================\n";
    for (size_t i = 0; i < readings.size(); ++i) {
        std::cout << "\nReading #" << (i + 1) << ":\n";
        readings[i].print();
    }
    std::cout << "============================================\n";
}
