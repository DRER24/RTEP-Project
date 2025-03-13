# RTEP-Project 
Real Time Embedded Programming using C++ AND Raspberry Pi

    #include "tou.h"
    #include <gpiod.h>
    #include <iostream>
    #include <thread>

    using namespace std;

    int main() {
    gpiod_chip *chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip) {
        cerr << "Failed to open GPIO chip!" << endl;
        return -1;
    }
    

    gpiod_line *shock = gpiod_chip_get_line(chip, 17);
    gpiod_line *red = gpiod_chip_get_line(chip, 27);
    gpiod_line *green = gpiod_chip_get_line(chip, 18);
    
    if (!shock || !red || !green) {
        cerr << "Failed to get GPIO line!" << endl;
        gpiod_chip_close(chip);
        return -1;
    }
    
    if (gpiod_line_request_both_edges_events(shock, "vibration_sensor") < 0) {
        cerr << “Request for vibration sensor GPIO event failed!” << endl;
        gpiod_chip_close(chip);
        return -1;
    }
    
    if (gpiod_line_request_output(red, "led_red", 0) < 0) {
        cerr << “Request for red LED GPIO output failed!” << endl;
        gpiod_chip_close(chip);
        return -1;
    }
    if (gpiod_line_request_output(green, "led_green", 1) < 0) {
        cerr << “Request for green LED GPIO output failed!” << endl;
        gpiod_chip_close(chip);
        return -1;
    }
    
    cout << “System initialization complete, waiting for vibration sensor event...” << endl;
    
    std::thread sensor_thread(sensor_monitor_thread, shock, [=]() {
        led_control_callback(red, green);
    });
    
    sensor_thread.join();
    
    gpiod_line_release(shock);
    gpiod_line_release(red);
    gpiod_line_release(green);
    gpiod_chip_close(chip);
    
    return 0;
