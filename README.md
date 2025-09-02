<<<<<<< HEAD
# autonomous_wall_e_robot
=======
# 🤖 Obstacle Avoiding Robot

An Arduino-based obstacle avoiding robot that uses an **HC-SR04 ultrasonic sensor**, a **servo motor** to scan, and an **Adafruit Motor Shield** to drive 4 DC motors.

---

## What this project does

- Drives forward until an obstacle is detected within 12 cm.
- Stops, backs up slightly, scans left and right with a servo-mounted ultrasonic sensor.
- Turns toward the side with more free space and continues.

---

## Files

- `Obstacle_Avoiding_Robot.ino` — Arduino sketch (main code)

---

## Parts / Components

# 🤖 Obstacle Avoiding Robot

An Arduino-based obstacle avoiding robot that uses an **HC-SR04 ultrasonic sensor**, a **servo motor** to scan, and an **Adafruit Motor Shield** to drive 4 DC motors.

---

## What this project does

- Drives forward until an obstacle is detected within 12 cm.
- Stops, backs up slightly, scans left and right with a servo-mounted ultrasonic sensor.
- Turns toward the side with more free space and continues.

---

## Files

- `Obstacle_Avoiding_Robot.ino` — Arduino sketch (main code)

---

## Parts / Components

- Arduino Uno (or compatible)
- Adafruit Motor Shield (v1) + 4 DC motors
- HC-SR04 Ultrasonic Sensor (Trig → A1, Echo → A0)
- SG90 (or similar) Servo (signal → pin 10)
- External motor power supply (recommended)
- Jumper wires, chassis, battery pack

---

## Wiring (overview)

- Adafruit Motor Shield mounted on Arduino (motors connected to M1..M4 on the shield).
- HC-SR04:
  - Vcc → 5V
  - GND → GND
  - Trig → A1
  - Echo → A0
- Servo:
  - Signal → D10
  - Vcc → 5V (if powered from Arduino) or separate 5V supply (recommended)
  - GND → common ground with Arduino and motor supply

Important: Use a separate power supply for motors and ensure common ground between motor supply and Arduino to avoid brownouts.

---

## How to use

1. Open `Obstacle_Avoiding_Robot.ino` in the Arduino IDE.
2. Select the correct board and port.
3. Install the `AFMotor` library (Adafruit Motor Shield v1) and the `Servo` library (usually included).
4. Upload the sketch to the Arduino.
5. Place the robot on the ground and power the motors with the external supply. The robot should start moving forward and avoid obstacles automatically.

---

## Tips & tuning

- Adjust `baseSpeed` in the sketch (default 170) to tune motor speed.
- Adjust `midAngle` if your servo's straight position differs.
- Reduce `delay(...)` durations to make reactions faster — test carefully.

---

## License

This project is provided under the MIT License. Include a `LICENSE` file in your repo if you want to publish it publicly.

---

## Attribution

Original logic provided by the user; refactored and documented for GitHub upload.
