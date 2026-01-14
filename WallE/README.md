# WallE - VEX V5 PushBack AI Robot

This subdirectory contains a V5 C++ project designed to showcase how the Jetson/Raspberry Pi runs inference on game objects and sends detection data to the V5 Brain, which uses that data to play the game. 

This project contains a basic implementation to get you started and is designed to be ran on the 2025-26 Pushback HeroBot with a ball pre-loaded

When the program is ran, the robot will search for a blue ball and once found, will drive to it and intake it. The robot will then calculate which of the 4 ends of the long goals it is closest to, drive to that end, and score the ball.

#### Program Structure
- `main.cpp`- Entry point of the program. Configures devices, sets up the main callback and polls the Jetson/Raspberry Pi for data
- `ai_functions.cpp` - Contains helper functions that allow the robot to navigate the field, and find and interact with objects
- `ai_jetson.cpp` - Code that handles receiving data from the Jetson/Raspberry Pi over serial
- `ai_robot_link` - Code that handles robot-to-robot communications
- `dashboard.cpp` - Presents data and status information for Jetson/Raspberry Pi and VEXLink communications on the Brain screen

***Detailed documentation is listed below:***
        <li> <a href="https://kb.vex.com/hc/en-us/articles/360049619171-Coding-the-VEX-AI-Robot
">Coding the VEX AI Robot</a></li>
