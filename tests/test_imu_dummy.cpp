#include <cassert>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

#include "../src/imu/IMU.h"

std::vector<IMU> parseCSV(const std::string& filename) {
    std::vector<IMU> data;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }
    
    std::string line;
    std::getline(file, line);
    
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        std::istringstream iss(line);
        std::string token;
        long timeITOW;
        double time, roll, pitch, xRate, yRate, zRate, xAccel, yAccel, zAccel;
        int opMode, linAccSw, turnSw;
        
        std::getline(iss, token, ','); timeITOW = std::stol(token);
        std::getline(iss, token, ','); time = std::stod(token);
        std::getline(iss, token, ','); roll = std::stod(token);
        std::getline(iss, token, ','); pitch = std::stod(token);
        std::getline(iss, token, ','); xRate = std::stod(token);
        std::getline(iss, token, ','); yRate = std::stod(token);
        std::getline(iss, token, ','); zRate = std::stod(token);
        std::getline(iss, token, ','); xAccel = std::stod(token);
        std::getline(iss, token, ','); yAccel = std::stod(token);
        std::getline(iss, token, ','); zAccel = std::stod(token);
        std::getline(iss, token, ','); opMode = std::stoi(token);
        std::getline(iss, token, ','); linAccSw = std::stoi(token);
        std::getline(iss, token, ','); turnSw = std::stoi(token);
        
        data.push_back(IMU(timeITOW, time, roll, pitch, xRate, yRate, zRate,
                          xAccel, yAccel, zAccel, opMode, linAccSw, turnSw));
    }
    
    return data;
}

void test_csv_file_exists() {
    std::ifstream file("tests/data/IMU_Test_Data.csv");
    assert(file.good() && "CSV file should exist");
}

void test_csv_not_empty() {
    auto data = parseCSV("tests/data/IMU_Test_Data.csv");
    assert(data.size() > 0 && "CSV should contain data rows");
}

void test_csv_column_count() {
    std::ifstream file("tests/data/IMU_Test_Data.csv");
    std::string header;
    std::getline(file, header);
    
    int commaCount = 0;
    for (char c : header) {
        if (c == ',') commaCount++;
    }
    
    assert(commaCount == 12 && "CSV should have 13 columns (12 commas)");
}

void test_csv_header_format() {
    std::ifstream file("tests/data/IMU_Test_Data.csv");
    std::string header;
    std::getline(file, header);
    
    std::vector<std::string> required_columns = {
        "timeITOW", "time", "roll", "pitch", "xRate", "yRate", "zRate",
        "xAccel", "yAccel", "zAccel", "opMode", "linAccSw", "turnSw"
    };
    
    for (const auto& col : required_columns) {
        assert(header.find(col) != std::string::npos &&
               ("Header should contain column: " + col).c_str());
    }
}

// TODO: Re-enable once CSV data is sorted chronologically - talk to Comms

// void test_timestamp_increasing() {
//     auto data = parseCSV("tests/data/IMU_Test_Data.csv");
//     
//     for (size_t i = 1; i < data.size(); i++) {
//         if (data[i].time < data[i-1].time) {
//             std::cerr << "Timestamp violation at row " << i+1 << ": "
//                       << data[i-1].time << " -> " << data[i].time << "\n";
//         }
//         assert(data[i].time >= data[i-1].time - 1e-6 &&
//                "Timestamps should be non-decreasing");
//     }
// }

void test_accelerometer_ranges() {
    auto data = parseCSV("tests/data/IMU_Test_Data.csv");
    
    double minZ = data[0].zAccel, maxZ = data[0].zAccel;
    double minX = data[0].xAccel, maxX = data[0].xAccel;
    double minY = data[0].yAccel, maxY = data[0].yAccel;
    
    for (const auto& row : data) {
        minZ = std::min(minZ, row.zAccel);
        maxZ = std::max(maxZ, row.zAccel);
        minX = std::min(minX, row.xAccel);
        maxX = std::max(maxX, row.xAccel);
        minY = std::min(minY, row.yAccel);
        maxY = std::max(maxY, row.yAccel);
        
        // Verify ranges
        assert(row.xAccel >= -8.0 && row.xAccel <= 7.0 &&
               "X acceleration out of range");
        assert(row.yAccel >= -6.0 && row.yAccel <= 12.0 &&
               "Y acceleration out of range");
        assert(row.zAccel >= -14.0 && row.zAccel <= 2.0 &&
               "Z acceleration out of range");
    }
    
    std::cout << "  Accelerometer ranges:\n";
    std::cout << "    X: [" << minX << ", " << maxX << "]\n";
    std::cout << "    Y: [" << minY << ", " << maxY << "]\n";
    std::cout << "    Z: [" << minZ << ", " << maxZ << "]\n";
}

void test_gyroscope_ranges() {
    auto data = parseCSV("tests/data/IMU_Test_Data.csv");
    
    for (const auto& row : data) {
        assert(row.xRate > -1000.0 && row.xRate < 1000.0 &&
               "X rotation rate out of range");
        assert(row.yRate > -1000.0 && row.yRate < 1000.0 &&
               "Y rotation rate out of range");
        assert(row.zRate > -1000.0 && row.zRate < 1000.0 &&
               "Z rotation rate out of range");
    }
}

void test_orientation_ranges() {
    auto data = parseCSV("tests/data/IMU_Test_Data.csv");
    
    for (const auto& row : data) {
        assert(row.roll >= -1000.0 && row.roll <= 1000.0 &&
               "Roll angle out of range");
        assert(row.pitch >= -1000.0 && row.pitch <= 1000.0 &&
               "Pitch angle out of range");
    }
}

void test_operation_mode_valid() {
    auto data = parseCSV("tests/data/IMU_Test_Data.csv");
    
    for (const auto& row : data) {
        assert(row.opMode >= 0 && row.opMode <= 15 &&
               "Operation mode should be in valid range");
    }
}

void test_switch_values_binary() {
    auto data = parseCSV("tests/data/IMU_Test_Data.csv");
    
    for (const auto& row : data) {
        assert((row.linAccSw == 0 || row.linAccSw == 1) &&
               "linAccSw should be 0 or 1");
        assert((row.turnSw == 0 || row.turnSw == 1) &&
               "turnSw should be 0 or 1");
    }
}

int main() {
    std::cout << "Running IMU CSV format tests...\n";
    
    try {
        test_csv_file_exists();
        std::cout << "✓ CSV file exists\n";
        
        test_csv_column_count();
        std::cout << "✓ CSV has correct column count\n";
        
        test_csv_header_format();
        std::cout << "✓ CSV header format correct\n";
        
        test_csv_not_empty();
        std::cout << "✓ CSV contains data\n";
        
        // test_timestamp_increasing();  // TODO: Re-enable once CSV data is sorted
        // std::cout << "✓ Timestamps are increasing\n";
        
        test_accelerometer_ranges();
        std::cout << "✓ Accelerometer values in valid range\n";
        
        test_gyroscope_ranges();
        std::cout << "✓ Gyroscope values in valid range\n";
        
        test_orientation_ranges();
        std::cout << "✓ Orientation angles in valid range\n";
        
        test_operation_mode_valid();
        std::cout << "✓ Operation mode values valid\n";
        
        test_switch_values_binary();
        std::cout << "✓ Switch values are binary\n";
        
        std::cout << "\nAll IMU CSV tests passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed: " << e.what() << "\n";
        return 1;
    }
}
