#ifndef tou_H
#define tou_H

#include <gpiod.h>
#include <functional>


void sensor_monitor_thread(gpiod_line *sensor_line, const std::function<void(void)> &callback);
void led_control_callback(gpiod_line *led_red_line, gpiod_line *led_green_line);

#endif // SENSOR_H
