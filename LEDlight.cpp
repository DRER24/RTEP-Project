#include <gpiod.h>     
#include <iostream>     
#include <unistd.h>     
#include <chrono>      
#include <thread>       
#include <mutex>       
#include <functional>   
#include <atomic>       

using namespace std;


std::mutex led_mutex;
std::atomic<bool> is_led_active{false};

void sensor_monitor(gpiod_line *sensor_line, const function<void(void)>& callback) {
    while (true) {

        struct timespec timeout = {5, 0};
        int ret = gpiod_line_event_wait(sensor_line, &timeout);
        struct gpiod_line_event event;
        ret = gpiod_line_event_read(sensor_line, &event);
      
        
        if (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
            cout << "run!!!" << endl;  
            std::thread(callback).detach();
        }
    }
}

int main() {
  
    struct gpiod_chip *chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip) {
        cout << "EORROR" << endl;
        return -1;
    }

    struct gpiod_line *sensor_line = gpiod_chip_get_line(chip, 17);
    struct gpiod_line *led_red_line = gpiod_chip_get_line(chip, 27);
    struct gpiod_line *led_green_line = gpiod_chip_get_line(chip, 18);
    
    if (!sensor_line || !led_red_line || !led_green_line) {
        cout << "ERROR" << endl;
        gpiod_chip_close(chip);
        return -1;
    }
    
    if (gpiod_line_request_both_edges_events(sensor_line, "vibration_sensor") < 0) {
        cout << "Failed to request a vibration sensor GPIO event!" << endl;
        gpiod_chip_close(chip);
        return -1;
    }

    if (gpiod_line_request_output(led_red_line, "led_red", 0) < 0) {
        cout << "Request for red LED GPIO output failed!" << endl;
        gpiod_chip_close(chip);
        return -1;
    }
    if (gpiod_line_request_output(led_green_line, "led_green", 1) < 0) {
        cerr << "Request for green LED GPIO output failed!" << endl;
        gpiod_chip_close(chip);
        return -1;
    }
    
    cout << "System initialization is complete, waiting for vibration sensor events..." << endl;

    auto led_callback = [led_red_line, led_green_line]() {
        if (is_led_active.exchange(true)) {
            return;
        }
        
        {
            std::lock_guard<std::mutex> lock(led_mutex);
            gpiod_line_set_value(led_green_line, 0);
            gpiod_line_set_value(led_red_line, 1);
            cout << "LED status: red" << endl;
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
        
        {
            std::lock_guard<std::mutex> lock(led_mutex);
            gpiod_line_set_value(led_red_line, 0);
            gpiod_line_set_value(led_green_line, 1);
            cout << "LED status: green" << endl;
        }
        is_led_active = false;
    };
    

    std::thread sensor_thread(sensor_monitor, sensor_line, led_callback);
    
  
    sensor_thread.join();

    gpiod_line_release(sensor_line);
    gpiod_line_release(led_red_line);
    gpiod_line_release(led_green_line);
    gpiod_chip_close(chip);
    
    return 0;
}