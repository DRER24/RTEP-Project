#include "Target.h"

Target::Target() 
    : id_(0), dist_(0), pitch_(0), yaw_(0), x_(0), y_(0), valid_(false) {
}

uint8_t Target::getId() const {
    return id_;
}

void Target::setId(uint8_t id) {
    id_ = id;
}

float Target::getDistance() const {
    return dist_;
}

void Target::setDistance(float dist) {
    dist_ = dist;
}

int8_t Target::getPitch() const {
    return pitch_;
}

void Target::setPitch(int8_t pitch) {
    pitch_ = pitch;
}

int8_t Target::getYaw() const {
    return yaw_;
}

void Target::setYaw(int8_t yaw) {
    yaw_ = yaw;
}

float Target::getX() const {
    return x_;
}

void Target::setX(float x) {
    x_ = x;
}

float Target::getY() const {
    return y_;
}

void Target::setY(float y) {
    y_ = y;
}

bool Target::isValid() const {
    return valid_;
}

void Target::setValid(bool valid) {
    valid_ = valid;
}
