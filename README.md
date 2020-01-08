# RPi_Arduino-autonomous-robot
autonomous robot controlled by RPi, with sensor data from Arduino

wiring 1.png - basic hardware component connection diagram
robot control commands rev01.jpg - drawing of robot highlighting servos and motors
system description.xlsx - some basic component notes
robot_control_3.ino - arduino (Leonardo) sketch that passes sensor values out to RPi (serial/i2c) and accepts motion commands (serial)
robo3.py - simple python script (3.x, for RPi) to test robot_control_3.ino function
