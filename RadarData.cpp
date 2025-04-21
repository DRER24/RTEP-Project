#include "RadarData.h"
#include <iostream>


RadarData::RadarData() 
    : fault_(0), count_(0) {
}

uint8_t RadarData::getFault() const {
    return fault_;
}

void RadarData::setFault(uint8_t fault) {
    fault_ = fault;
}

uint8_t RadarData::getCount() const {
    return count_;
}

void RadarData::setCount(uint8_t count) {
    count_ = count;
}

const std::vector<Target>& RadarData::getTargets() const {
    return targets_;
}

void RadarData::addTarget(const Target& target) {
    targets_.push_back(target);
}

void RadarData::clearTargets() {
    targets_.clear();
}

RadarData& RadarData::parseFrame(const uint8_t* p, size_t len) {
    // Clear any existing targets
    clearTargets();
    
    // Set fault code and target count from the frame
    fault_ = p[4];
    count_ = p[5];
    
    // Parse individual targets
    for (int i = 0; i < count_; ++i) {
        int base = 4 + 8 + i * 8;
        
        Target t;
        t.setId(p[base]);
        t.setDistance(p[base + 1] * 0.1f);
        t.setPitch(static_cast<int8_t>(p[base + 2]));
        t.setYaw(static_cast<int8_t>(p[base + 3]));
        t.setX(static_cast<int8_t>(p[base + 6]) * 0.1f);
        t.setY(static_cast<int8_t>(p[base + 7]) * 0.1f);
        t.setValid(true);
        
        addTarget(t);
    }
    
    return *this;
}

Target RadarData::findBestTarget() const {
    // If there are no targets, return an invalid target
    if (targets_.empty()) {
        Target t;
        t.setValid(false);
        return t;
    }
    
    // Find the closest target
    const Target* closest = &targets_[0];
    float minDist = closest->getDistance();
    
    for (const auto& t : targets_) {
        if (t.getDistance() < minDist) {
            closest = &t;
            minDist = t.getDistance();
        }
    }
    
    // Return a copy of the closest target
    return *closest;
}
