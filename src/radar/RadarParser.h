#pragma once

#include <vector>
#include <cstdint>
#include <string>

// Data structures for parsed radar frame data

struct DynamicPoint {
    float range;
    float angle;
    float elev;
    float doppler;
};

struct SideInfo {
    uint16_t snr;
    uint16_t noise;
};

struct StaticPoint {
    float x;
    float y;
    float z;
    float doppler;
};

struct Vec3 {
    float x, y, z;
};

struct TrackedObject {
    uint32_t id;
    Vec3 pos;
    Vec3 vel;
    Vec3 acc;
};

struct RadarFrameData {
    // Header information
    uint32_t version;
    uint32_t packetLength;
    uint32_t platform;
    uint32_t frameNumber;
    uint32_t time;
    uint32_t numObjects;
    uint32_t numTLV;
    uint32_t subframeNum;
    uint32_t numStaticObjects;

    // TLV Data
    std::vector<DynamicPoint> dynamic_points;
    std::vector<SideInfo> dynamic_sideinfo;
    std::vector<StaticPoint> static_points;
    std::vector<SideInfo> static_sideinfo;
    std::vector<TrackedObject> tracks;
    std::vector<uint8_t> associations;
};

class RadarParser {
public:
    RadarParser();
    
    // Parse binary radar data from file
    bool parseFile(const std::string& filename);
    
    // Parse binary data from buffer
    bool parseBuffer(const uint8_t* data, size_t length);
    
    // Export parsed frames to CSV files
    bool exportToCSV(const std::string& outputDir = ".");
    
    // Getters
    const std::vector<RadarFrameData>& getFrames() const { return frames; }
    size_t getFrameCount() const { return frames.size(); }

private:
    static constexpr size_t MAGIC_WORD_LEN = 8;
    static constexpr uint8_t MAGIC_WORD[] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};
    
    std::vector<RadarFrameData> frames;
    std::vector<uint8_t> rawData;
    
    // Helper functions
    uint32_t readU32(const uint8_t* data, size_t offset) const;
    float readF32(const uint8_t* data, size_t offset) const;
    
    // Find magic words in data
    std::vector<size_t> findMagicWords(const uint8_t* data, size_t length) const;
    
    // Parse individual frame
    bool parseFrame(const uint8_t* data, size_t length, RadarFrameData& frame);
    
    // Parse TLV structures
    bool parseTLV(const uint8_t* packet, size_t tlvStart, uint32_t tlvType, 
                  uint32_t tlvLength, RadarFrameData& frame);
    
    // CSV export helpers
    bool exportDynamicPointsCSV(const std::string& filename) const;
    bool exportStaticPointsCSV(const std::string& filename) const;
    bool exportTracksCSV(const std::string& filename) const;
    bool exportAssociationsCSV(const std::string& filename) const;
};
