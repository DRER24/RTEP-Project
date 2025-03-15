#include "radar_sensor.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <unistd.h> 

void RadarSensor::initialize() {
    std::cout << "Initializing Radar Sensor..." << std::endl;
}

std::vector<RadarData> RadarSensor::getData() {
    std::lock_guard<std::mutex> lock(radarMutex);
    return latestData;
}

void RadarSensor::readRadarData() {
    std::ifstream radar("/dev/ttyUSB0");
    std::string line;
    while (running) {
        if (std::getline(radar, line)) {
            std::istringstream iss(line);
            RadarData data;
            iss >> data.distance >> data.angle >> data.elevation; 
            
            std::lock_guard<std::mutex> lock(radarMutex);
            latestData.push_back(data);
        }
        usleep(50000); 
    }
}

void RadarSensor::startListening() {
    running = true;
    radarThread = std::thread(&RadarSensor::readRadarData, this);
}

void RadarSensor::stopListening() {
    running = false;
    if (radarThread.joinable()) radarThread.join();
}