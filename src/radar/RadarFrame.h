#pragma once

#include <vector>
#include <cstdint>
#include "Dynamic.h"
#include "Static.h"

class RadarFrame {
public:
    RadarFrame(uint32_t frameNumber, uint32_t timestamp)
        : frame_number(frameNumber), timestamp(timestamp) {}

    // Frame metadata
    uint32_t getFrameNumber() const { return frame_number; }
    uint32_t getTimestamp() const { return timestamp; }

    // Add a dynamic detection to this frame
    void addDynamic(const Dynamic& detection) {
        dynamic_points.push_back(detection);
    }

    // Add a static detection to this frame
    void addStatic(const Static& detection) {
        static_points.push_back(detection);
    }

    // Getters for detections
    const std::vector<Dynamic>& getDynamicPoints() const {
        return dynamic_points;
    }

    const std::vector<Static>& getStaticPoints() const {
        return static_points;
    }

    // Count getters
    size_t getDynamicCount() const {
        return dynamic_points.size();
    }

    size_t getStaticCount() const {
        return static_points.size();
    }

    // Check if frame has dynamic or static points
    bool hasDynamicPoints() const {
        return !dynamic_points.empty();
    }

    bool hasStaticPoints() const {
        return !static_points.empty();
    }

private:
    // Frame metadata
    uint32_t frame_number;
    uint32_t timestamp;

    // Detections in this frame
    std::vector<Dynamic> dynamic_points;
    std::vector<Static> static_points;

    // TODO: Add tracks and associations in next update
};
