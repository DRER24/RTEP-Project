# RTEP-Project
Real time radar data processing system using UART-based MMwave Sensor with FFT signal analysis and 3D visualization with Point Cloud Library (PCL) and sensor based detection

# Introduction

This project implements a real-time sensor system that
- Reads radar data via UART
- Processes radar signals using Fast Fourier Transform
- Visualizes data in 3D using Point Cloud Library (PCL)
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
# Social Media Platforms
https://www.instagram.com/rtep_uofg?igsh=YWxrM3R1cHRkd2Q5
 
# Authors
- Abishek Srinivasan Moorthy(3043860S)
- Derrick Roy Edgar Rajappan(3023903R)
- Dheemanth S Naidu(3049973N)
- Yuxin Du(3008171)
- Zhuolin Li(3021316)
