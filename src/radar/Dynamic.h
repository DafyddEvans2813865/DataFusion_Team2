#pragma once

#include <cstdint>

class Dynamic {
public:
    Dynamic(uint32_t frameNum, 
            uint32_t timestamp,
            float range, 
            float angle, 
            float elev, 
            float doppler,
            uint16_t snr, 
            uint16_t noise)
        : frame(frameNum), 
          timestamp(timestamp),
          range(range), 
          angle(angle), 
          elev(elev), 
          doppler(doppler),
          snr(snr), 
          noise(noise) {}

    // Getters
    uint32_t getFrame() const { return frame; }
    uint32_t getTimestamp() const { return timestamp; }
    float getRange() const { return range; }
    float getAngle() const { return angle; }
    float getElev() const { return elev; }
    float getDoppler() const { return doppler; }
    uint16_t getSNR() const { return snr; }
    uint16_t getNoise() const { return noise; }

private:

    uint32_t frame;
    uint32_t timestamp;

    float range;
    float angle;
    float elev;
    float doppler;

    uint16_t snr;
    uint16_t noise;
};
