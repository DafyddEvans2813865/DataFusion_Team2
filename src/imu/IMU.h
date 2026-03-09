#ifndef IMU_H
#define IMU_H

/**
 * @class IMU
 * @brief Represents Inertial Measurement Unit (IMU) sensor data
 * 
 * Contains orientation, angular velocity, linear acceleration, and mode information
 */
class IMU {
public:
    // Timestamps
    long timeITOW;      // Millisecond time of week
    double time;        // Time in seconds
    
    // Orientation
    double roll;        // Roll angle (degrees)
    double pitch;       // Pitch angle (degrees)
    
    // Angular velocity (gyroscope)
    double xRate;       // X-axis rotation rate (deg/s)
    double yRate;       // Y-axis rotation rate (deg/s)
    double zRate;       // Z-axis rotation rate (deg/s)
    
    // Linear acceleration (accelerometer)
    double xAccel;      // X-axis acceleration (m/s^2)
    double yAccel;      // Y-axis acceleration (m/s^2)
    double zAccel;      // Z-axis acceleration (m/s^2)
    
    // Status/Mode
    int opMode;         // Operation mode (0-15)
    int linAccSw;       // Linear acceleration switch (0 or 1)
    int turnSw;         // Turn switch (0 or 1)
    
    /**
     * @brief Default constructor
     */
    IMU();
    
    /**
     * @brief Parameterized constructor
     */
    IMU(long timeITOW_val, double time_val, double roll_val, double pitch_val,
        double xRate_val, double yRate_val, double zRate_val,
        double xAccel_val, double yAccel_val, double zAccel_val,
        int opMode_val, int linAccSw_val, int turnSw_val);
    
    /**
     * @brief Print IMU data to console
     */
    void print() const;
};

#endif // IMU_H
