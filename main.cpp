#include "tou.h"
#include <gpiod.h>
#include <iostream>
#include <thread>

using namespace std;

int main() {
    // 打开默认GPIO芯片（"/dev/gpiochip0"）
    gpiod_chip *chip = gpiod_chip_open_by_name("gpiochip0");
    if (!chip) {
        cerr << "打开GPIO芯片失败！" << endl;
        return -1;
    }
    
    // 获取对应的GPIO线
    // 振动传感器连接在GPIO 17，
    // 双色LED红色通道连接在GPIO 27，绿色通道连接在GPIO 18
    gpiod_line *shock = gpiod_chip_get_line(chip, 17);
    gpiod_line *red = gpiod_chip_get_line(chip, 27);
    gpiod_line *green = gpiod_chip_get_line(chip, 18);
    
    if (!shock || !red || !green) {
        cerr << "获取GPIO线失败！" << endl;
        gpiod_chip_close(chip);
        return -1;
    }
    
    // 请求振动传感器GPIO为事件输入模式（双边沿模式）
    if (gpiod_line_request_both_edges_events(shock, "vibration_sensor") < 0) {
        cerr << "请求振动传感器GPIO事件失败！" << endl;
        gpiod_chip_close(chip);
        return -1;
    }
    
    // 请求LED GPIO为输出模式
    // 初始状态：红色LED关闭，绿色LED点亮
    if (gpiod_line_request_output(red, "led_red", 0) < 0) {
        cerr << "请求红色LED GPIO输出失败！" << endl;
        gpiod_chip_close(chip);
        return -1;
    }
    if (gpiod_line_request_output(green, "led_green", 1) < 0) {
        cerr << "请求绿色LED GPIO输出失败！" << endl;
        gpiod_chip_close(chip);
        return -1;
    }
    
    cout << "系统初始化完成，等待振动传感器事件..." << endl;
    
    // 创建传感器监控线程，当检测到振动时调用LED控制回调函数
    std::thread sensor_thread(sensor_monitor_thread, shock, [=]() {
        led_control_callback(red, green);
    });
    
    sensor_thread.join();
    
    // 释放GPIO资源
    gpiod_line_release(shock);
    gpiod_line_release(red);
    gpiod_line_release(green);
    gpiod_chip_close(chip);
    
    return 0;
}
