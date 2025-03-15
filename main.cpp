#include "radar_sensor.h"
#include "pcl_visualization.h"
#include "tou.h"
#include <gpiod.h>
#include <thread>
#include <vector>

void updateProcessing(RadarSensor &radar, PCLVisualization &pclViz) {
    while (true) {
        std::vector<RadarData> radarData = radar.getData();
        pclViz.updatePointCloud(radarData);
    }
}

void gpioMonitoring(gpiod_line *shock, gpiod_line *red, gpiod_line *green) {
    std::thread sensor_thread(sensor_monitor_thread, shock, [=]() {
        led_control_callback(red, green);
    });
    sensor_thread.join();
}

int main() {
    RadarSensor radar;
    PCLVisualization pclViz;
    radar.initialize();
    radar.startListening();
    
    std::thread processingThread(updateProcessing, std::ref(radar), std::ref(pclViz));
    
    
    gpiod_chip *chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip) return -1;

    gpiod_line *shock = gpiod_chip_get_line(chip, 17);
    gpiod_line *red   = gpiod_chip_get_line(chip, 27);
    gpiod_line *green = gpiod_chip_get_line(chip, 18);

    if (!shock || !red || !green) {
        gpiod_chip_close(chip);
        return -1;
    }

    if (gpiod_line_request_both_edges_events(shock, "vibration_sensor") < 0 ||
        gpiod_line_request_output(red, "led_red", 0) < 0 ||
        gpiod_line_request_output(green, "led_green", 1) < 0) {
        gpiod_chip_close(chip);
        return -1;
    }

    std::thread gpioThread(gpioMonitoring, shock, red, green);
    
    pclViz.run(); 
    
    processingThread.join();
    gpioThread.join();

    radar.stopListening();
    gpiod_line_release(shock);
    gpiod_line_release(red);
    gpiod_line_release(green);
    gpiod_chip_close(chip);
    
    return 0;
}
