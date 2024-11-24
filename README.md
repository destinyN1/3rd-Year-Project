# Real-Time Ball Balancing System

This repository contains the source code and implementation details of a real-time ball balancing system. The project demonstrates the use of Arduino to dynamically stabilize a ball on a plate using servo motors, sensors, and PID (Proportional-Integral-Derivative) control.

## Project Overview

The goal of the project is to:

1. Read the ball's position on a plate using sensors.
2. Compute control signals using PID controllers to adjust the plate's angle dynamically.
3. Stabilize the ball at a desired position (setpoint) on the plate.

This project highlights the integration of control systems, embedded programming, and practical mechatronics applications.

## Features

- **PID Control**: Fine-tuned PID controllers for X and Y axes to stabilize the ball.
- **Real-Time Processing**: Dynamic adjustments to maintain the ball's position at the desired setpoint.
- **Servo Motor Control**: Precise angular displacement for plate stabilization.
- **Sensing System**: Uses analog readings to determine the ball's position on the plate.

## Components Used

- **Arduino Uno**: Microcontroller for computation and signal processing.
- **Servo Motors**: For tilting the plate along the X and Y axes.
- **Sensors**: Analog sensors to detect the ball's position.
- **Plate with Corner Pins**: Mechanical system to support and stabilize the ball.

## Code Overview

### Main Functionalities

1. **PID Control**:
   - PID controllers compute the required servo angles based on the ball's position and the setpoint.
   - Gains (`Kp`, `Ki`, `Kd`) are tuned for optimal performance.

2. **Servo Control**:
   - Servo motors adjust the plate's tilt based on PID output.

3. **Ball Sensing**:
   - Analog readings are averaged for accurate position detection.
   - Pins are toggled to read X and Y coordinates separately.

4. **Stability Logic**:
   - If no ball movement is detected, the plate resets to a flat position.

### Variables and Constants

- **PID Variables**:
  - `SetPointX`, `SetPointY`: Desired position of the ball.
  - `pvX`, `pvY`: Process variables (current position of the ball).
  - `OutputX`, `OutputY`: Control signals sent to the servo motors.

- **Servo Parameters**:
  - `thetaXflat`, `thetaYflat`: Flat position angles.
  - `thetaXmax`, `thetaYmax`: Maximum angular displacement.

- **Timing**:
  - `samplingInterval`: Ensures consistent control loop timing.

## Setup and Usage

1. **Hardware Setup**:
   - Connect servos to pins 7 and 8 on the Arduino.
   - Connect sensors to the `sensePin` (A0).
   - Connect the corner pins of the plate to pins 2, 3, 4, and 5.

2. **Software Setup**:
   - Install the required libraries: [PID_v1](https://github.com/br3ttb/Arduino-PID-Library), [Servo](https://www.arduino.cc/reference/en/libraries/servo/).
   - Upload the provided code to the Arduino board.

3. **Running the System**:
   - Power on the Arduino.
   - The system will stabilize the ball in real-time.
   - Use the serial monitor to observe sensor readings and PID outputs.

## Libraries Used

- [PID_v1](https://github.com/br3ttb/Arduino-PID-Library): For PID control.
- [Servo](https://www.arduino.cc/reference/en/libraries/servo/): For controlling servo motors.

## Key Parameters

- **PID Gains**:
  - X-axis: `KpX = 0.5`, `KiX = 0.5`, `KdX = 0.00075`
  - Y-axis: `KpY = 0.5`, `KiY = 0.5`, `KdY = 0.00075`

- **Setpoints**:
  - `SetPointX = 490`
  - `SetPointY = 512`

- **Servo Limits**:
  - Maximum angular displacement: `90 degrees`


