#include "../src/radar/RadarParser.h"
#include <iostream>

int main() {
    RadarParser parser;
    
    std::cout << "Parsing radar test data..." << std::endl;
    
    if (!parser.parseFile("tests/data/Radar_Test_Data.txt")) {
        std::cerr << "Failed to parse radar test data file" << std::endl;
        return 1;
    }
    
    std::cout << "Exporting to CSV..." << std::endl;
    
    if (!parser.exportToCSV("tests/data/radar_parsing")) {
        std::cerr << "Failed to export CSV files" << std::endl;
        return 1;
    }
    
    std::cout << "CSV files saved to tests/data/radar_parsing/" << std::endl;
    std::cout << "Total frames parsed: " << parser.getFrameCount() << std::endl;
    
    return 0;
}
