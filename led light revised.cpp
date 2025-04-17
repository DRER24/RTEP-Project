#include <gpiod.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <functional>

using namespace std;

class LEDController {
public:
    LEDController(gpiod_line* red, gpiod_line* green)
        : red_line(red), green_line(green), is_active(false) {}

    void trigger() {
        if (is_active.exchange(true)) return;

        {
            lock_guard<mutex> lock(mtx);
            gpiod_line_set_value(green_line, 0);
            gpiod_line_set_value(red_line, 1);
            cout << "LED status: red" << endl;
        }

        this_thread::sleep_for(chrono::seconds(1));

        {
            lock_guard<mutex> lock(mtx);
            gpiod_line_set_value(red_line, 0);
            gpiod_line_set_value(green_line, 1);
            cout << "LED status: green" << endl;
        }

        is_active = false;
    }

private:
    gpiod_line* red_line;
    gpiod_line* green_line;
    atomic<bool> is_active;
    mutex mtx;
};

class SensorMonitor {
public:
    SensorMonitor(gpiod_line* line, function<void()> callback)
        : sensor_line(line), event_callback(callback) {}

    void start() {
        monitor_thread = thread([this]() {
            while (true) {
                struct timespec timeout = {5, 0};
                int ret = gpiod_line_event_wait(sensor_line, &timeout);
                if (ret <= 0) continue;

                struct gpiod_line_event event;
                if (gpiod_line_event_read(sensor_line, &event) == 0 &&
                    event.event_type == GPIOD_LINE_EVENT_RISING_EDGE)
                {
                    cout << "Vibration detected!" << endl;
                    thread(event_callback).detach();
                }
            }
        });
    }

    void join() {
        if (monitor_thread.joinable())
            monitor_thread.join();
    }

private:
    gpiod_line* sensor_line;
    function<void()> event_callback;
    thread monitor_thread;
};

int main() {
    gpiod_chip* chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip) {
        cerr << "Failed to open GPIO chip." << endl;
        return -1;
    }

    gpiod_line* sensor = gpiod_chip_get_line(chip, 17);
    gpiod_line* red_led = gpiod_chip_get_line(chip, 27);
    gpiod_line* green_led = gpiod_chip_get_line(chip, 18);

    if (!sensor || !red_led || !green_led) {
        cerr << "Failed to get GPIO lines." << endl;
        gpiod_chip_close(chip);
        return -1;
    }

    if (gpiod_line_request_both_edges_events(sensor, "vibration_sensor") < 0 ||
        gpiod_line_request_output(red_led, "led_red", 0) < 0 ||
        gpiod_line_request_output(green_led, "led_green", 1) < 0)
    {
        cerr << "Failed to configure GPIO lines." << endl;
        gpiod_chip_close(chip);
        return -1;
    }

    cout << "System initialized. Waiting for sensor events..." << endl;

    LEDController led_control(red_led, green_led);
    SensorMonitor monitor(sensor, [&led_control]() {
        led_control.trigger();
    });

    monitor.start();
    monitor.join();  
、
    gpiod_line_release(sensor);
    gpiod_line_release(red_led);
    gpiod_line_release(green_led);
    gpiod_chip_close(chip);

    return 0;
}
