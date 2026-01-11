C++
/*
 * Project: Autonomous & WiFi Robot Car
 * Platform: Arduino UNO R3
 * Sensors: HC-SR04 (Ultrasonic), IR (Infrared), Servo SG90
 * Actuators: L298N Motor Driver, Buzzer, LEDs
 * Communication: SoftwareSerial to ESP32-CAM
 */

#include <Servo.h>
#include <SoftwareSerial.h>

// --- Pin Definitions ---

// Motor Control Pins (L298N)
const int ENA = 3;  // PWM Speed Control for Left Motor
const int IN1 = 4;  // Direction Pin 1 Left
const int IN2 = 5;  // Direction Pin 2 Left
const int IN3 = 6;  // Direction Pin 1 Right
const int IN4 = 7;  // Direction Pin 2 Right
const int ENB = 9;  // PWM Speed Control for Right Motor

// Sensor Pins
const int IR_LEFT = 2;      // Left IR Sensor (Digital Input)
const int IR_RIGHT = 8;     // Right IR Sensor (Digital Input)
const int TRIG_PIN = A0;    // Ultrasonic Trigger
const int ECHO_PIN = A1;    // Ultrasonic Echo

// Servo, Buzzer & Communication Pins
const int SERVO_PIN = 10;   // SG90 Control Pin (PWM)
const int BUZZER_PIN = 13;  // Active Buzzer
const int RX_PIN = 11;      // SoftwareSerial RX (Connects to ESP32 TX)
const int TX_PIN = 12;      // SoftwareSerial TX (Connects to ESP32 RX via voltage divider)

// Indicator LEDs
const int LED_BRAKE = A2;   // Red LEDs (Rear)
const int LED_LEFT = A3;    // Yellow LED (Left Signal)
const int LED_RIGHT = A4;   // Yellow LED (Right Signal)

// --- Objects ---
Servo headServo;
SoftwareSerial espSerial(RX_PIN, TX_PIN); // Create serial object for ESP32 comms

// --- Global Variables ---
int distance = 0;
const int SAFE_DISTANCE = 30; // Obstacle threshold in cm
const int SPEED_MAX = 180;    // Forward speed (0-255)
const int SPEED_TURN = 150;   // Turning speed (0-255)
bool autonomousMode = true;   // Flag to toggle between Autonomous and Manual mode

void setup() {
  // Initialize Motor Pins
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  
  // Initialize Sensors & Actuators
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  pinMode(LED_BRAKE, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);

  // Initialize Communication
  Serial.begin(9600);    // For debugging via USB
  espSerial.begin(9600); // For communication with ESP32

  // Initialize Servo
  headServo.attach(SERVO_PIN);
  headServo.write(90); // Look forward (Center)

  // Startup Sound
  beep(100, 2);
}

void loop() {
  // 1. Check for incoming commands from ESP32 (Wi-Fi Control)
  if (espSerial.available()) {
    char cmd = espSerial.read();
    autonomousMode = false; // Disable autonomous mode when manual command is received
    handleRemoteCommand(cmd);
  }

  // 2. Run Autonomous Logic (if enabled)
  if (autonomousMode) {
    runAutonomousLogic();
  }
}

// --- Autonomous Logic Core ---
void runAutonomousLogic() {
  // Check IR Sensors (Reflex action for drop-offs or close obstacles)
  // Logic: LOW usually means obstacle detected for many IR sensors
  if (digitalRead(IR_LEFT) == LOW |

| digitalRead(IR_RIGHT) == LOW) {
    emergencyStop();
    moveBackward();
    delay(400);
    stopMotors();
    return;
  }

  // Check Ultrasonic Distance
  distance = readUltrasonic();
  
  if (distance < SAFE_DISTANCE) {
    obstacleAvoidanceRoutine();
  } else {
    moveForward();
  }
}

// --- Obstacle Avoidance Routine ---
void obstacleAvoidanceRoutine() {
  stopMotors();
  digitalWrite(LED_BRAKE, HIGH); // Turn on brake lights
  beep(200, 1); // Warning sound
  
  // Back up slightly
  moveBackward();
  delay(300);
  stopMotors();
  
  // Scan surroundings
  headServo.write(170); // Look Left
  delay(500);
  int leftDist = readUltrasonic();
  
  headServo.write(10);  // Look Right
  delay(500);
  int rightDist = readUltrasonic();
  
  headServo.write(90);  // Reset to Center
  delay(500);
  
  digitalWrite(LED_BRAKE, LOW);

  // Decision Making
  if (leftDist > rightDist) {
    flashIndicator(LED_LEFT);
    turnLeft();
    delay(500); // Duration of turn
  } else {
    flashIndicator(LED_RIGHT);
    turnRight();
    delay(500);
  }
  stopMotors();
}

// --- Helper Functions ---

// Measure distance using HC-SR04
int readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  if (duration == 0) return 200; // Assume clear path if timeout
  return duration * 0.034 / 2;
}

// Handle commands from ESP32
void handleRemoteCommand(char cmd) {
  switch (cmd) {
    case 'F': moveForward(); break;
    case 'B': moveBackward(); break;
    case 'L': turnLeft(); break;
    case 'R': turnRight(); break;
    case 'S': stopMotors(); break;
    case 'A': autonomousMode = true; stopMotors(); break; // Re-enable Auto Mode
    case 'U': beep(500,1); break; // Horn
  }
}

// --- Motor Control Functions ---
void moveForward() {
  setMotors(SPEED_MAX, SPEED_MAX, HIGH, LOW, HIGH, LOW);
  digitalWrite(LED_BRAKE, LOW);
}

void moveBackward() {
  setMotors(SPEED_MAX, SPEED_MAX, LOW, HIGH, LOW, HIGH);
  digitalWrite(LED_BRAKE, HIGH);
  beep(50, 0); // Beeping while backing up
}

void turnLeft() {
  setMotors(SPEED_TURN, SPEED_TURN, LOW, HIGH, HIGH, LOW); // Tank turn left
}

void turnRight() {
  setMotors(SPEED_TURN, SPEED_TURN, HIGH, LOW, LOW, HIGH); // Tank turn right
}

void emergencyStop() {
  stopMotors();
  digitalWrite(LED_BRAKE, HIGH);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  digitalWrite(LED_BRAKE, HIGH);
}

// Low-level function to set motor pins
void setMotors(int speedA, int speedB, int d1, int d2, int d3, int d4) {
  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);
  digitalWrite(IN1, d1); digitalWrite(IN2, d2);
  digitalWrite(IN3, d3); digitalWrite(IN4, d4);
}

void beep(int duration, int count) {
  for (int i=0; i<count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
    delay(50);
  }
}

void flashIndicator(int pin) {
  for(int i=0; i<3; i++){
    digitalWrite(pin, HIGH); delay(100);
    digitalWrite(pin, LOW); delay(100);
  }
}
