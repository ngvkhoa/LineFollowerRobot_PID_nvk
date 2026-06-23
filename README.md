# High-Speed PID Line Follower Robot 🏎️

This repository contains the Arduino source code and hardware documentation for a custom-built, high-speed line following robot. The system utilizes a Proportional-Integral-Derivative (PID) control algorithm for smooth and aggressive track navigation.

 🌟 Key Features
* Custom PID Algorithm: Highly tuned Proportional and Derivative control for rapid error correction without oscillations.
* Auto-Calibration Mode: A 5-second dynamic scanning phase that automatically normalizes the white/black reflectance thresholds based on environmental lighting.
* Active Alignment (Standby Mode): A pre-race holding mode where the robot uses a limited-torque PID loop to "lock" itself onto the center of the line while waiting for the launch signal.
* Soft-Start (Anti-Wheelie): Gradually ramps up the base speed upon launch to prevent the high-torque N20 motors from lifting the front sensor array off the track.
* Emergency Brake & Reset: Instantly cuts power mid-race via a push button and seamlessly returns to Active Alignment mode without needing a reboot.

 🛠️ Hardware Specifications
* Microcontroller: LGT8F328P (Chosen for its higher clock speed and processing efficiency over standard Arduino Nano).
* Motors: 2x N20 Micro Metal Gearmotors (1:30 Gear Ratio).
* Motor Driver: Dual H-Bridge Module.
* Sensors: 8-Channel Analog Infrared (IR) Reflectance Sensor Array.
* Power Supply: 2x 18650 High-Discharge Li-ion Batteries.
* Power Regulation: Mini 360 Buck Converter (Steps down 7.4V-8.4V to a stable 5V for logic circuits).
* User Interface: 1x Push Button, 1x Active Buzzer, 1x Status LED, 1x Voltmeter.

 ⚙️ Pin Configuration
| Component | Arduino Pin |
| :--- | :--- |
| Right Motor PWM (PWMA) | 9 |
| Right Motor Dir 1 (AIN1) | 7 |
| Right Motor Dir 2 (AIN2) | 8 |
| Left Motor PWM (PWMB) | 6 |
| Left Motor Dir 1 (BIN1) | 5 |
| Left Motor Dir 2 (BIN2) | 4 |
| IR Sensor Array (S1 -> S8)| A7, A6, A5, A4, A3, A2, A1, A0 |
| Push Button | 12 (INPUT_PULLUP) |
| Active Buzzer | 11 |
| Status LED | 13 |

 🚀 How to Run
1. Power On: Turn on the main switch.
2. Auto-Calibrate: Press the button once. The buzzer will beep twice, and the LED will flash. Sweep the front sensor array back and forth across the black line and white background for 5 seconds.
3. Standby Mode: Once calibration is complete (indicated by a long beep), the robot enters Active Alignment mode. Place it on the line; it will make micro-adjustments to hold its center position.
4. Launch: Press the button again. The robot will emit a 3-beep countdown and launch using the Soft-Start sequence.
5. Emergency Stop: Press the button anytime during the run to stop the motors instantly and return to Standby Mode.

 💻 Software Setup
No external libraries are required! The code is written in standard C++ for the Arduino IDE. 
Just select your board (LGT8F328P or standard Arduino Nano/Uno), compile, and upload.

*Note: This project is open-source. Feel free to use, modify, and optimize the code for your own robotics builds!*

Nguyen Van Khoa
