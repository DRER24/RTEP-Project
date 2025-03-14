#include "tou.h"
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>

static std::mutex led_mutex;
static std::atomic<bool> is_led_active{false};



void sensor_monitor_thread(gpiod_line *sensor_line, const std::function<void(void)> &callback) {
    while (true) 
    {
        struct timespec timeout = {5, 0};
        if (gpiod_line_event_wait(sensor_line, &timeout) <= 0)
            continue;

        struct gpiod_line_event event;

        if (gpiod_line_event_read(sensor_line, &event) < 0)
            continue;

        if (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE && !is_led_active.load())
            std::thread(callback).detach();
    }
}





void led_control_callback(gpiod_line *led_red_line, gpiod_line *led_green_line) 
{
    if (is_led_active.exchange(true))
        return;
    
    {
        std::lock_guard<std::mutex> lock(led_mutex);
        gpiod_line_set_value(led_green_line, 0);
        gpiod_line_set_value(led_red_line, 1);
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    {
        std::lock_guard<std::mutex> lock(led_mutex);
        gpiod_line_set_value(led_red_line, 0);
        gpiod_line_set_value(led_green_line, 1);
    }
    is_led_active = false;
}
