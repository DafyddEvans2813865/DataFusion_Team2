#include "RadarDummy.h"
#include <cstdlib>  
#include <ctime>  


RadarDummy::RadarDummy(int numPoints) : numPoints(numPoints) {
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
}
 
std::vector<RadarPoint> RadarDummy::generateData() const {
    std::vector<RadarPoint> data;
    for(int i = 0; i < numPoints; i++) {
        RadarPoint d;
        d.x = static_cast<float>(std::rand()) / RAND_MAX * 100.0f; // random x between 0-100
        d.y = static_cast<float>(std::rand()) / RAND_MAX * 100.0f; // random y between 0-100
        /* 
        TO-DO 
        Fix this to represent realistic Data
        */

        data.push_back(d);
    }
    return data;
}
