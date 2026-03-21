#pragma once

#include <cstdint>

class Static {
public:
    Static(uint32_t frameNum, 
           uint32_t timestamp,
           float x, 
           float y, 
           float z, 
           float doppler,
           uint16_t snr, 
           uint16_t noise)
        : frame(frameNum), 
          timestamp(timestamp),
          x(x), 
          y(y), 
          z(z), 
          doppler(doppler),
          snr(snr), 
          noise(noise) {}

    // Getters
    uint32_t getFrame() const { return frame; }
    uint32_t getTimestamp() const { return timestamp; }
    float getX() const { return x; }
    float getY() const { return y; }
    float getZ() const { return z; }
    float getDoppler() const { return doppler; }
    uint16_t getSNR() const { return snr; }
    uint16_t getNoise() const { return noise; }

private:
    uint32_t frame;
    uint32_t timestamp;

    float x;
    float y;
    float z;
    float doppler;

    uint16_t snr;
    uint16_t noise;
};
