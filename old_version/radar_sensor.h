#ifndef RADAR_SENSOR_H
#define RADAR_SENSOR_H

#include <vector>
#include <thread>
#include <mutex>

struct RadarData {
    float distance;
    float angle;
    float elevation;
};

class RadarSensor {
public:
    void initialize();
    std::vector<RadarData> getData();
    void startListening();
    void stopListening();

private:
    void readRadarData();
    bool running = false;
    std::thread radarThread;
    std::mutex radarMutex;
    std::vector<RadarData> latestData;
};

#endif
