# CENG2400(2024 Fall) Final Project - Laser Turret

A gyroscope-controlled laser turret project that uses MPU6050 to direct laser movement. The system consists of two main components communicating via UART:

- **Master End**: Collects and processes gyro/accelerometer data from MPU6050
- **Slave End**: Receives angles and controls servo motors via PWM signals

## ⚠️ Disclaimer
This repository contains academic work and is published for record and reference purposes only. **Do not copy or reuse the code** as it may constitute academic misconduct. The code is specific to our course project and should be used solely for learning and understanding the concepts. For your own good, please do your project with your own understanding and knowledge. I am not responsible for any academic misconduct caused by the code in this repository.

All project specifications, manuals, and related course materials are the intellectual property of The Chinese University of Hong Kong or Texas Instruments. I do not own or claim any copyright over these materials. They are provided for reference only.

## Project Structure

### Documentation
- [Project Specification](docs/Specification.pdf)
- [User Manual](docs/)
- [Project Report](report/report.pdf)

### Source Code
- [Source Files](src/)
  - [MPU code](src/main_mpu.c)
  - [Servo code](src/main_servo.c)
  - [DEMO code](src/demo/)
- CCS Project Packages:
  - [MPU Project](Project_MPU.zip)
  - [Servo Project](Project_SERVO.zip)

## Implementation Details

The project implements several key features:
1. MPU6050 data collection and processing
2. UART wireless communication
3. PWM servo motor control
4. Data integrity checking

For detailed implementation challenges and solutions, please refer to the [Project Report](report/report.pdf).

## Team Members
- Yu Ching Hei (chyu2@cse.cuhk.edu.hk)
- Tam Yiu Hei (1155223226@link.cuhk.edu.hk)
- Leung Chung Wang (1155194650@link.cuhk.edu.hk)

## Acknowledgments
- Professor: Prof. Ming-Chang YANG
- Teaching Assistant: 
  - Mr. Chenchen ZHAO
  - Mr. Kezhi LI
  - Mr. Han ZHAO
  - Mr. Zhirui ZHANG

## Project History
Full source code and development history available on [GitHub](https://github.com/Jellyfish227/CENG2400_Embedded_System_Design.git)



