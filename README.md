# RTEP-Project
Real time radar data processing system using UART-based MMwave Sensor and 2D visualization with HeatMap based visualization integrating both camera and mmwave sensor for detection

# Introduction

This project implements a real-time sensor system that
- Reads radar data via UART
- 
- Uses GPIO for vibration sensor-based event detection & LED control

  
# Installation & Compilation
 Install dependencies
 
 sudo apt update
 
 sudo apt install cmake libpcl-dev libgpiod-dev

 # Building the Project
 mkdir build && 
 cd build
 cmake .
 make 

 # Dependencies
 - C++ 17 
 - CMake 
 - PCL
 - libgpiod
 - POSIX Threads
 
# Authors
- Abishek Srinivasan Moorthy(3043860S)
- Derrick Roy Edgar Rajappan(3023903R)
- Dheemanth S Naidu(3049973N)
- Yuxin Du(3008171)
- Zhuolin Li(3021316)
