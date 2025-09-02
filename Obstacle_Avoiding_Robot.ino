/*
  Obstacle Avoiding Robot with Ultrasonic Sensor and Servo
  ---------------------------------------------------------
  - Uses Adafruit Motor Shield (AFMotor library)
  - Ultrasonic sensor (HC-SR04) for obstacle detection
  - Servo motor to scan left/right for free path
  - 4 DC motors for robot movement

  Author: Your Name
  GitHub: https://github.com/YourUsername/Obstacle-Avoiding-Robot
  License: MIT
*/

#include <Servo.h>
#include <AFMotor.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>

// Pin definitions (keep same pins as original)
#define ECHO_PIN A0       // Ultrasonic sensor Echo pin
#define TRIG_PIN A1       // Ultrasonic sensor Trigger pin
#define SERVO_PIN 10      // Servo motor pin (for scanning)

// DFPlayer Mini (voice alerts) pins
static const uint8_t MP3_TX = 2; // Connects to module RX
static const uint8_t MP3_RX = 3; // Connects to module TX

// Configuration — tune these values for your robot
#define BASE_SPEED 170         // Motor speed (0-255)
#define MID_ANGLE 103          // Servo center angle (straight)
#define MIN_DISTANCE_CM 12     // Obstacle threshold (cm)
#define BACKUP_MS 150          // Backup duration (milliseconds)
#define TURN_MS 450            // Turn duration when deciding direction (milliseconds)
#define SCAN_SETTLE_MS 300     // Wait after moving servo before reading distance
#define DIST_SAMPLES 3         // Number of samples to average for distance
#define PULSE_TIMEOUT 20000UL  // pulseIn timeout in microseconds (~20ms -> ~3.4m)

// Distance measurement variables
int dist = 0;               // Distance measured straight ahead
int leftDist = 0;           // Distance measured to the left
int rightDist = 0;          // Distance measured to the right

Servo scanServo;            // Servo object (scanner)

// Hand servos for gestures
Servo leftHandServo;
Servo rightHandServo;

// DFPlayer objects
SoftwareSerial mp3Serial(MP3_RX, MP3_TX);
DFRobotDFPlayerMini player;

// Motors (Adafruit Motor Shield)
AF_DCMotor motor1(1);
AF_DCMotor motor2(2);
AF_DCMotor motor3(3);
AF_DCMotor motor4(4);

// Helper: safe delay that keeps serial responsive (small wrapper)
void safeDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    // allow other background tasks if needed
    delay(1);
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Attach servo to pin
  scanServo.attach(SERVO_PIN);
  scanServo.write(MID_ANGLE);
  safeDelay(200);

  // Attach hand servos (pins 6 and 7 are used for gestures)
  leftHandServo.attach(6);
  rightHandServo.attach(7);
  // Default hand positions
  leftHandServo.write(160);
  rightHandServo.write(0);

  // Initialize DFPlayer serial and module
  mp3Serial.begin(9600);
  if (!player.begin(mp3Serial)) {
    Serial.println("DFPlayer Mini initialization failed!");
    // continue without audio
  } else {
    Serial.println("DFPlayer Mini initialized.");
    player.volume(25); // 0..30
  }

  // seed random using floating analog pin (if available)
  randomSeed(analogRead(A2));

  // Set motor speeds
  motor1.setSpeed(BASE_SPEED);
  motor2.setSpeed(BASE_SPEED);
  motor3.setSpeed(BASE_SPEED);
  motor4.setSpeed(BASE_SPEED);

  Serial.println("Obstacle Avoiding Robot - ready");
}

void loop() {
  avoidObstacles();  // Continuously check and avoid obstacles
}

// Main obstacle avoidance logic (non-blocking-ish)
void avoidObstacles() {
  dist = getDistanceAverage(DIST_SAMPLES);   // Measure distance ahead
  Serial.print("Ahead: "); Serial.print(dist); Serial.println(" cm");

  if (dist > 0 && dist <= MIN_DISTANCE_CM) {  // If obstacle within threshold
    Serial.println("Obstacle detected — backing up and scanning");
    halt();               // Stop
    moveBackward();       // Move back slightly
    safeDelay(BACKUP_MS);
    halt();

    // Look left
    leftDist = scanLeft();
    scanServo.write(MID_ANGLE);
    safeDelay(SCAN_SETTLE_MS);

    // Look right
    rightDist = scanRight();
    scanServo.write(MID_ANGLE);
    safeDelay(SCAN_SETTLE_MS);

    Serial.print("Scan left: "); Serial.print(leftDist); Serial.print(" cm, right: "); Serial.print(rightDist); Serial.println(" cm");

    // Play a random voice alert (if DFPlayer initialized)
    if (player.available()) {
      int track = random(1, 12); // 1..11
      Serial.print("Playing audio track: "); Serial.println(track);
      player.play(track);
      // short delay to allow track start; long delay follows for safety
      safeDelay(200);
    }

    // Gesture: move hands when obstacle detected
    leftHandServo.write(90);  // gesture pose
    rightHandServo.write(90);
    safeDelay(1000);

    // Decide turning direction (prefer larger distance)
    if (leftDist <= 0 && rightDist <= 0) {
      // both failed -> rotate in place to the right a bit
      Serial.println("Both scans failed — rotating in place");
      turnRight(); safeDelay(TURN_MS); halt(); safeDelay(200);
    } else if (leftDist < rightDist) {
      Serial.println("Turning right — more space on right");
      turnRight(); safeDelay(TURN_MS); halt(); safeDelay(200);
    } else {
      Serial.println("Turning left — more space on left");
      turnLeft(); safeDelay(TURN_MS); halt(); safeDelay(200);
    }
  // After decision, return hands to default
  leftHandServo.write(160);
  rightHandServo.write(0);
  // If DFPlayer started playback, wait a safe period to avoid rapid retriggers
  safeDelay(4000);
  } else {
    // No close obstacle → move forward
    moveForward();
  // Keep hands in idle pose
  leftHandServo.write(160);
  rightHandServo.write(0);
  }
}

// Read distance multiple times and average (ignores zero readings)
int getDistanceAverage(int samples) {
  long sum = 0;
  int count = 0;
  for (int i = 0; i < samples; ++i) {
    int d = getDistance();
    if (d > 0) { sum += d; ++count; }
    safeDelay(30);
  }
  if (count == 0) return -1; // indicate failure
  return (int)(sum / count);
}

// Ultrasonic distance measurement (single reading)
int getDistance() {
  // Send trigger pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(4);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long echoTime = pulseIn(ECHO_PIN, HIGH, PULSE_TIMEOUT);   // Time for echo (with timeout)
  if (echoTime == 0) {
    // timeout or no pulse
    return -1;
  }

  long distanceCm = echoTime / 29 / 2;      // Convert to cm
  // Basic sanity clamp
  if (distanceCm > 400) return -1;
  return (int)distanceCm;
}

// Movement functions
void moveForward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void moveBackward() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void turnRight() {
  // pivot: left side wheels backward, right side forward
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void turnLeft() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void halt() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

// Servo scanning functions — return distance (or -1 on failure)
int scanRight() {
  scanServo.write(20);         // Rotate servo to right
  safeDelay(SCAN_SETTLE_MS);
  int d = getDistanceAverage(max(1, DIST_SAMPLES));   // average a few samples
  return d;
}

int scanLeft() {
  scanServo.write(180);        // Rotate servo to left
  safeDelay(SCAN_SETTLE_MS);
  int d = getDistanceAverage(max(1, DIST_SAMPLES));
  return d;
}
