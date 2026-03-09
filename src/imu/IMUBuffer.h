#ifndef IMU_BUFFER_H
#define IMU_BUFFER_H

#include "IMU.h"
#include <vector>
#include <iostream>

/**
 * @class IMUBuffer
 * @brief Buffer for collecting continuous IMU sensor data streams
 * 
 * Stores multiple IMU readings and provides methods to add new data
 * and retrieve statistics
 */
class IMUBuffer {
private:
    std::vector<IMU> readings;      // Storage for all IMU readings
    size_t maxCapacity;             // Maximum number of readings to store (0 = unlimited)
    
public:
    /**
     * @brief Constructor with optional capacity limit
     * @param capacity Maximum readings to store (0 = unlimited)
     */
    IMUBuffer(size_t capacity = 0);
    
    /**
     * @brief Add a new IMU reading to the buffer
     * @param imu The IMU data to add
     */
    void addReading(const IMU& imu);
    
    /**
     * @brief Get the number of readings in the buffer
     * @return Number of readings
     */
    size_t size() const;
    
    /**
     * @brief Get a reading at a specific index
     * @param index The index of the reading
     * @return Reference to the IMU reading
     */
    const IMU& getReading(size_t index) const;
    
    /**
     * @brief Get the latest reading
     * @return Reference to the most recent IMU reading
     */
    const IMU& getLatest() const;
    
    /**
     * @brief Clear all readings from the buffer
     */
    void clear();
    
    /**
     * @brief Print all readings in the buffer
     */
    void printAll() const;
};

#endif // IMU_BUFFER_H
