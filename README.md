# Line-Follower using DSPIC30F4011
This program controls a line-following robot using the DSPIC30F4011 microcontroller. It reads sensor values from an array of line sensors and uses a PID algorithm to control the motors and follow a line.

## Description
The program uses 16 sensors to detect the line and control the motors. The code includes functions for reading the sensors, mapping the values to a useful range, and controlling the motors.

## Requirements
The program was developed using MPLAB IDE and CCS compiler. Additionally, the following pins are used:

* Button: RB6
* Left LED: RB7
* Right LED: RB8
* Line change sensor: RF5
* RX: RF4
* Potentiometer: AN4
* Motors:
    * Motor A PWM: OC3
    * Motor B PWM: OC4
    * Motor A direction: RE5
    * Motor B direction: RE4

## Instalation
1. Clone the repository to your local machine.
2. Open the project in MPLAB X IDE.
3. Compile the project.
4. Program the microcontroller using your preferred programmer, recomended pickit 3.

## Usage
* main file is [main.c](/Seguidor_2019/main.c)
* Power on the robot.
* Place the robot on a line.
* Calibrate the robot.
* The robot should follow the line.

## Notes
The implementation and source of this code is based on the next repository thaks to the help of the Professor and his Student:

* [Harold Murcia](https://github.com/HaroldMurcia)
* [Yeison Daniel Tapiero]()
* Link project [Line Follower](https://github.com/HaroldMurcia/Line_Follower)

## License
This code is released under the MIT License. Feel free to modify and use it in your own projects.