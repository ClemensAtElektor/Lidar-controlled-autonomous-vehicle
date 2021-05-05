# Lidar-controlled-autonomous-vehicle
2D-Lidar-controlled autonomous vehicle with ESP32 and Bluetooth

Arduino ESP32 sketch for an autonomous vehicle that finds it way around a room without bumping into objects.

The lidar is the X4 from Ydlidar.

Uses the [Dabble](https://github.com/STEMpedia/Dabble) Bluetooth library for Bluetooth (duh).

Also uses the very nice ESP32 motor driver library [ESP32MotorControl](https://github.com/JoaoLopesF/ESP32MotorControl) that does not use ledc like everybody else but instead uses the real motor driver peripheral built into the ESP32.

The vehicle is a simple design with two motorized wheels in the center and two stand-offs on either end for remaining balanced. This arrangement makes the vehicle very agile.

The motors are controlled by an ESP32 Pico Kit, which in turn is controlled over Bluetooth by a smartphone. The lidar data is processed by the ESP32 and used to steer the vehicle in the direction of the greatest distance, i.e. the object that is furthest away.

Video: https://youtu.be/BmNelv_gR9Q
