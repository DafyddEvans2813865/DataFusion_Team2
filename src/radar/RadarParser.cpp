#include "RadarParser.h"
#include <fstream>
#include <iostream>
#include <cstring>
#include <iomanip>
#include <sstream>

constexpr uint8_t RadarParser::MAGIC_WORD[];

RadarParser::RadarParser() {
}

uint32_t RadarParser::readU32(const uint8_t* data, size_t offset) const {
    return (static_cast<uint32_t>(data[offset]) |
            (static_cast<uint32_t>(data[offset + 1]) << 8) |
            (static_cast<uint32_t>(data[offset + 2]) << 16) |
            (static_cast<uint32_t>(data[offset + 3]) << 24));
}

float RadarParser::readF32(const uint8_t* data, size_t offset) const {
    uint32_t u = readU32(data, offset);
    float f;
    std::memcpy(&f, &u, sizeof(float));
    return f;
}

std::vector<size_t> RadarParser::findMagicWords(const uint8_t* data, size_t length) const {
    std::vector<size_t> positions;
    
    for (size_t i = 0; i < length - MAGIC_WORD_LEN + 1; ++i) {
        bool match = true;
        for (size_t j = 0; j < MAGIC_WORD_LEN; ++j) {
            if (data[i + j] != MAGIC_WORD[j]) {
                match = false;
                break;
            }
        }
        if (match) {
            positions.push_back(i);
        }
    }
    
    return positions;
}

bool RadarParser::parseFile(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }
    
    // Read entire file as hex string
    file.seekg(0, std::ios::end);
    size_t fileSize = file.tellg();
    file.seekg(0, std::ios::beg);
    
    std::string hexData(fileSize, ' ');
    file.read(&hexData[0], fileSize);
    file.close();
    
    // Convert space-separated hex string to bytes
    rawData.clear();
    std::istringstream iss(hexData);
    std::string hexByte;
    
    while (iss >> hexByte) {
        try {
            uint8_t byte = static_cast<uint8_t>(std::stoi(hexByte, nullptr, 16));
            rawData.push_back(byte);
        } catch (const std::exception& e) {
            std::cerr << "Failed to parse hex byte: " << hexByte << std::endl;
            return false;
        }
    }
    
    return parseBuffer(rawData.data(), rawData.size());
}

bool RadarParser::parseBuffer(const uint8_t* data, size_t length) {
    frames.clear();
    
    // Find all magic words
    auto magicPositions = findMagicWords(data, length);
    
    std::cout << "Found " << magicPositions.size() << " magic words" << std::endl;
    
    // Extract packets based on magic word positions and packet length
    std::vector<std::pair<size_t, size_t>> packets;
    
    for (size_t x = 0; x < magicPositions.size(); ++x) {
        size_t start = magicPositions[x];
        
        if (start + 16 > length) {
            continue;
        }
        
        // Read packet length from offset 12 (relative to magic word position)
        uint32_t pktLen = readU32(data, start + 12);
        
        if (start + pktLen <= length) {
            packets.push_back({start, pktLen});
        }
    }
    
    std::cout << "Extracted " << packets.size() << " packets" << std::endl;
    
    // Parse each packet
    for (const auto& [packetStart, packetLength] : packets) {
        RadarFrameData frame;
        if (parseFrame(data + packetStart, packetLength, frame)) {
            frames.push_back(frame);
        }
    }
    
    std::cout << "Parsed " << frames.size() << " frames" << std::endl;
    return !frames.empty();
}

bool RadarParser::parseFrame(const uint8_t* data, size_t length, RadarFrameData& frame) {
    if (length < 44) {
        std::cerr << "Packet too short for header" << std::endl;
        return false;
    }
    
    // Parse header (offsets relative to start of packet)
    frame.version = readU32(data, 8);
    frame.packetLength = readU32(data, 12);
    frame.platform = readU32(data, 16);
    frame.frameNumber = readU32(data, 20);
    frame.time = readU32(data, 24);
    frame.numObjects = readU32(data, 28);
    frame.numTLV = readU32(data, 32);
    frame.subframeNum = readU32(data, 36);
    frame.numStaticObjects = readU32(data, 40);
    
    std::cout << "Frame: version=" << frame.version 
              << ", numTLV=" << frame.numTLV 
              << std::endl;
    
    size_t idx = 44;
    
    // Parse TLVs
    for (uint32_t tlv = 0; tlv < frame.numTLV; ++tlv) {
        if (idx + 8 > length) {
            std::cerr << "Not enough data for TLV header" << std::endl;
            break;
        }
        
        uint32_t tlvType = readU32(data, idx);
        uint32_t tlvLength = readU32(data, idx + 4);
        
        std::cout << "TLV: type=" << tlvType << ", length=" << tlvLength << std::endl;
        
        size_t tlvStart = idx;
        size_t tlvEnd = tlvStart + 8 + tlvLength;
        
        // Sanity check
        if (tlvLength < 8 || tlvEnd > length) {
            std::cerr << "Invalid TLV detected, stopping parse" << std::endl;
            break;
        }
        
        idx += 8;  // Move to TLV payload
        
        if (!parseTLV(data, idx, tlvType, tlvLength, frame)) {
            std::cerr << "Failed to parse TLV type " << tlvType << std::endl;
        }
        
        idx = tlvEnd;
    }
    
    return true;
}

bool RadarParser::parseTLV(const uint8_t* packet, size_t tlvStart, uint32_t tlvType,
                           uint32_t tlvLength, RadarFrameData& frame) {
    size_t idx = tlvStart;
    size_t tlvEnd = tlvStart + tlvLength;
    
    switch (tlvType) {
        case 1: {  // Dynamic detected points
            uint32_t count = tlvLength / 16;
            for (uint32_t k = 0; k < count; ++k) {
                if (idx + 16 > tlvEnd) break;
                
                DynamicPoint pt;
                pt.range = readF32(packet, idx);
                pt.angle = readF32(packet, idx + 4);
                pt.elev = readF32(packet, idx + 8);
                pt.doppler = readF32(packet, idx + 12);
                
                frame.dynamic_points.push_back(pt);
                idx += 16;
            }
            break;
        }
        
        case 7: {  // Side info (dynamic)
            while (idx + 4 <= tlvEnd) {
                SideInfo info;
                info.snr = (static_cast<uint16_t>(packet[idx]) |
                           (static_cast<uint16_t>(packet[idx + 1]) << 8));
                info.noise = (static_cast<uint16_t>(packet[idx + 2]) |
                             (static_cast<uint16_t>(packet[idx + 3]) << 8));
                
                frame.dynamic_sideinfo.push_back(info);
                idx += 4;
            }
            break;
        }
        
        case 8: {  // Static detected points
            while (idx + 16 <= tlvEnd) {
                StaticPoint pt;
                pt.x = readF32(packet, idx);
                pt.y = readF32(packet, idx + 4);
                pt.z = readF32(packet, idx + 8);
                pt.doppler = readF32(packet, idx + 12);
                
                frame.static_points.push_back(pt);
                idx += 16;
            }
            break;
        }
        
        case 9: {  // Static side info
            while (idx + 4 <= tlvEnd) {
                SideInfo info;
                info.snr = (static_cast<uint16_t>(packet[idx]) |
                           (static_cast<uint16_t>(packet[idx + 1]) << 8));
                info.noise = (static_cast<uint16_t>(packet[idx + 2]) |
                             (static_cast<uint16_t>(packet[idx + 3]) << 8));
                
                frame.static_sideinfo.push_back(info);
                idx += 4;
            }
            break;
        }
        
        case 10: {  // Tracked objects
            while (idx + 40 <= tlvEnd) {
                TrackedObject obj;
                obj.id = readU32(packet, idx);
                obj.pos.x = readF32(packet, idx + 4);
                obj.pos.y = readF32(packet, idx + 8);
                obj.vel.x = readF32(packet, idx + 12);
                obj.vel.y = readF32(packet, idx + 16);
                obj.acc.x = readF32(packet, idx + 20);
                obj.acc.y = readF32(packet, idx + 24);
                obj.pos.z = readF32(packet, idx + 28);
                obj.vel.z = readF32(packet, idx + 32);
                obj.acc.z = readF32(packet, idx + 36);
                
                frame.tracks.push_back(obj);
                idx += 40;
            }
            break;
        }
        
        case 11: {  // Point to track association
            while (idx < tlvEnd) {
                frame.associations.push_back(packet[idx]);
                idx += 1;
            }
            break;
        }
        
        default:
            std::cout << "Unhandled TLV type " << tlvType << ", skipping" << std::endl;
            break;
    }
    
    return true;
}

bool RadarParser::exportToCSV(const std::string& outputDir) {
    std::cout << "Exporting " << frames.size() << " frames to CSV..." << std::endl;
    
    bool success = true;
    success &= exportDynamicPointsCSV(outputDir + "/dynamic_points.csv");
    success &= exportStaticPointsCSV(outputDir + "/static_points.csv");
    success &= exportTracksCSV(outputDir + "/tracks.csv");
    success &= exportAssociationsCSV(outputDir + "/associations.csv");
    
    if (success) {
        std::cout << "CSV export complete" << std::endl;
    }
    
    return success;
}

bool RadarParser::exportDynamicPointsCSV(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << " for writing" << std::endl;
        return false;
    }
    
    file << "frame,range,angle,elev,doppler,snr,noise\n";
    
    for (size_t frameIdx = 0; frameIdx < frames.size(); ++frameIdx) {
        const auto& frame = frames[frameIdx];
        const auto& dyn = frame.dynamic_points;
        const auto& side = frame.dynamic_sideinfo;
        
        for (size_t i = 0; i < dyn.size(); ++i) {
            file << frameIdx << ","
                 << std::fixed << std::setprecision(6) << dyn[i].range << ","
                 << dyn[i].angle << ","
                 << dyn[i].elev << ","
                 << dyn[i].doppler;
            
            if (i < side.size()) {
                file << "," << side[i].snr << "," << side[i].noise;
            } else {
                file << ",,";
            }
            file << "\n";
        }
    }
    
    file.close();
    return true;
}

bool RadarParser::exportStaticPointsCSV(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << " for writing" << std::endl;
        return false;
    }
    
    file << "frame,x,y,z,doppler,snr,noise\n";
    
    for (size_t frameIdx = 0; frameIdx < frames.size(); ++frameIdx) {
        const auto& frame = frames[frameIdx];
        const auto& stat = frame.static_points;
        const auto& side = frame.static_sideinfo;
        
        for (size_t i = 0; i < stat.size(); ++i) {
            file << frameIdx << ","
                 << std::fixed << std::setprecision(6) << stat[i].x << ","
                 << stat[i].y << ","
                 << stat[i].z << ","
                 << stat[i].doppler;
            
            if (i < side.size()) {
                file << "," << side[i].snr << "," << side[i].noise;
            } else {
                file << ",,";
            }
            file << "\n";
        }
    }
    
    file.close();
    return true;
}

bool RadarParser::exportTracksCSV(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << " for writing" << std::endl;
        return false;
    }
    
    file << "frame,track_id,px,py,pz,vx,vy,vz,ax,ay,az\n";
    
    for (size_t frameIdx = 0; frameIdx < frames.size(); ++frameIdx) {
        const auto& frame = frames[frameIdx];
        
        for (const auto& track : frame.tracks) {
            file << frameIdx << ","
                 << track.id << ","
                 << std::fixed << std::setprecision(6)
                 << track.pos.x << ","
                 << track.pos.y << ","
                 << track.pos.z << ","
                 << track.vel.x << ","
                 << track.vel.y << ","
                 << track.vel.z << ","
                 << track.acc.x << ","
                 << track.acc.y << ","
                 << track.acc.z << "\n";
        }
    }
    
    file.close();
    return true;
}

bool RadarParser::exportAssociationsCSV(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << " for writing" << std::endl;
        return false;
    }
    
    file << "frame,target_id\n";
    
    for (size_t frameIdx = 0; frameIdx < frames.size(); ++frameIdx) {
        const auto& frame = frames[frameIdx];
        
        for (uint8_t tid : frame.associations) {
            file << frameIdx << "," << static_cast<int>(tid) << "\n";
        }
    }
    
    file.close();
    return true;
}
