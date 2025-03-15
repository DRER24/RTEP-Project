#ifndef TOU_H
#define TOU_H

#include <gpiod.h>
#include <functional>
#include "radar_sensor.h"
#include "pcl_visualization.h"
#include "fft_processor.h"

// Function declarations for GPIO event handling
void sensor_monitor_thread(gpiod_line *sensor_line, const std::function<void(void)> &callback);
void led_control_callback(gpiod_line *led_red_line, gpiod_line *led_green_line);

// Function declarations for radar processing
void process_radar_data(RadarSensor &radar, PCLVisualization &pclViz, FFTProcessor &fftProc);

#endif // TOU_H
