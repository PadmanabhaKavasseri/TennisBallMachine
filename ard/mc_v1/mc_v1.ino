#include <Servo.h>
#include <AccelStepper.h>

Servo linearActuator;
Servo topMotor;
Servo bottomMotor;

const int LA_PIN = 6;
const int TOP_MOTOR_PIN = 3;
const int BOTTOM_MOTOR_PIN = 2;

// Direction macros - adjust these if motor goes wrong way
#define CW   LOW   // Clockwise
#define CCW  HIGH  // Counter-clockwise

// Stepper Motor - Step/Dir mode
#define STEP_PIN     44
#define DIR_PIN      46
#define ENABLE_PIN   30
#define LIMIT_PIN    52

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

const int STEPS_PER_REV = 200 * 16;
const int MAX_SPEED = 1500;
const int ACCELERATION = 800;
const int HOMING_SPEED = 800;

bool homingComplete = false;

void setup() {
  Serial.begin(9600);
  
  linearActuator.attach(LA_PIN);
  topMotor.attach(TOP_MOTOR_PIN);
  bottomMotor.attach(BOTTOM_MOTOR_PIN);

  // Set initial positions
  int initPitchAngle = map(70, 0, 100, 0, 180);  // 70 -> ~126°
  linearActuator.write(initPitchAngle);

  int initWheelAngle = map(0, 0, 100, 0, 180);  // 0 -> 0° (stopped)
  // Send absolute minimum throttle to arm ESCs
  topMotor.writeMicroseconds(1000);
  bottomMotor.writeMicroseconds(1000);
  
  Serial.println("Arming ESCs...");
  Serial.println("Sending 1000us throttle signal");
  
  // Wait for ESCs to recognize signal and arm
  for (int i = 0; i < 10; i++) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Done!");
  
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(LIMIT_PIN, INPUT_PULLUP);
  
  digitalWrite(ENABLE_PIN, HIGH);  // Start disabled (active LOW)
  
  stepper.setMaxSpeed(MAX_SPEED);
  stepper.setAcceleration(ACCELERATION);
  stepper.setCurrentPosition(0);

  delay(1000);
  Serial.println("Ready!");
}

void loop() {
  stepper.run();
  
  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    message.trim();

    if (message.startsWith("MSG_START") && message.endsWith("MSG_END")) {
      processMessage(message);
    }
  }
}

void processMessage(String msg) {
  msg.replace("MSG_START|", "");
  msg.replace("|MSG_END", "");

  int lastChecksumIndex = msg.lastIndexOf("|#");
  String core = msg.substring(0, lastChecksumIndex);

  String parts[10];
  int count = splitString(core, '|', parts, 10);

  if (count < 3) return;

  String msgId = parts[1];

  for (int i = 2; i < count; i++) {
    int eqIndex = parts[i].indexOf('=');
    if (eqIndex != -1) {
      String motor = parts[i].substring(0, eqIndex);
      String value = parts[i].substring(eqIndex + 1);
      routeMotorCommand(motor, value, msgId);
    }
  }
}

void routeMotorCommand(String motor, String value, String msgId) {
  if (motor == "TOP_MOTOR") {
    handleTopMotor(value, msgId);
  } else if (motor == "BOTTOM_MOTOR") {
    handleBottomMotor(value, msgId);
  } else if (motor == "STEPPER") {
    handleStepper(value, msgId);
  } else if (motor == "LA") {
    handleLinearActuator(value, msgId);
  }
}

void handleLinearActuator(String pitch, String msgId) {
  int pitchValue = pitch.toInt();
  int servoAngle = map(pitchValue, 0, 100, 0, 180);
  linearActuator.write(servoAngle);
  Serial.println("LA: " + pitch + "° [" + msgId + "]");
}

void handleTopMotor(String speed, String msgId) {
  int speedValue = speed.toInt();  // 0-100
  int pulseWidth = map(speedValue, 0, 100, 1000, 2000);  // Map to microseconds
  topMotor.writeMicroseconds(pulseWidth);
  Serial.println("TOP: " + speed + " (" + String(pulseWidth) + "us) [" + msgId + "]");
}

void handleBottomMotor(String speed, String msgId) {
  int speedValue = speed.toInt();  // 0-100
  int pulseWidth = map(speedValue, 0, 100, 1000, 2000);  // Map to microseconds
  bottomMotor.writeMicroseconds(pulseWidth);
  Serial.println("BOT: " + speed + " (" + String(pulseWidth) + "us) [" + msgId + "]");
}

void handleStepper(String command, String msgId) {
  if (command == "ENABLE") {
    digitalWrite(ENABLE_PIN, LOW);
    Serial.println("✅ Stepper enabled [" + msgId + "]");
    return;
  }

  if (command == "DISABLE") {
    digitalWrite(ENABLE_PIN, HIGH);
    Serial.println("⛔ Stepper disabled [" + msgId + "]");
    return;
  }

  if (command == "HOME") {
    Serial.println("🔄 Homing stepper... [" + msgId + "]");
    
    digitalWrite(ENABLE_PIN, LOW);
    delay(10);
    
    if (digitalRead(LIMIT_PIN) == LOW) {
      Serial.println("Already at limit, backing off...");
      digitalWrite(DIR_PIN, CCW);  // Move away from limit
      for (int i = 0; i < 200; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(500);
      }
      delay(500);
    }
    
    digitalWrite(DIR_PIN, CCW);  // Move toward limit
    Serial.println("Moving toward limit switch...");
    
    unsigned long startTime = millis();
    while (digitalRead(LIMIT_PIN) == HIGH) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(500);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(500);
      
      if (millis() - startTime > 10000) {
        Serial.println("⚠️ Homing timeout");
        return;
      }
    }
    
    Serial.println("Limit pressed! Backing off...");
    delay(200);
    
    digitalWrite(DIR_PIN, CW);  // Back away from limit
    for (int i = 0; i < 840; i++) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(500);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(500);
    }
    
    Serial.println("✅ Homing complete! [" + msgId + "]");
    homingComplete = true;
    stepper.setCurrentPosition(0);
    return;
  }

  // Handle step testing commands (STEP_CW_100 or STEP_CCW_100)
  if (command.startsWith("STEP_")) {
    digitalWrite(ENABLE_PIN, LOW);  // Enable motor
    delay(10);
    
    int firstUnderscore = command.indexOf('_');
    int secondUnderscore = command.indexOf('_', firstUnderscore + 1);
    
    String direction = command.substring(firstUnderscore + 1, secondUnderscore);
    int steps = command.substring(secondUnderscore + 1).toInt();
    
    Serial.println("Moving " + String(steps) + " steps " + direction + " [" + msgId + "]");
    
    if (direction == "CW") {
      digitalWrite(DIR_PIN, CW);
    } else {
      digitalWrite(DIR_PIN, CCW);
    }
    
    for (int i = 0; i < steps; i++) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(500);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(500);
    }
    
    Serial.println("✅ Move complete [" + msgId + "]");
    return;
  }

  if (command == "STOP") {
    stepper.stop();
    Serial.println("STEPPER: Stop [" + msgId + "]");
    return;
  }

  int angle = command.toInt();
  if (angle >= 0 && angle <= 180) {
    digitalWrite(ENABLE_PIN, LOW);
    delay(10);
    
    long targetSteps = map(angle, 0, 180, 0, STEPS_PER_REV / 2);
    stepper.moveTo(targetSteps);
    Serial.println("STEPPER: " + String(angle) + "° [" + msgId + "]");
  }
}

int splitString(String input, char delimiter, String output[], int maxParts) {
  int count = 0;
  int start = 0;
  int end = input.indexOf(delimiter);

  while (end != -1 && count < maxParts) {
    output[count++] = input.substring(start, end);
    start = end + 1;
    end = input.indexOf(delimiter, start);
  }

  if (count < maxParts) {
    output[count++] = input.substring(start);
  }

  return count;
}

/*
WIRING:
Arduino Pin 42 -> STEP on TMC2209
Arduino Pin 43 -> DIR on TMC2209
Arduino Pin 32 -> ENABLE on TMC2209
Arduino Pin 24 -> Limit Switch NO
GND -> Limit Switch C, TMC2209 GND

TMC2209 Power:
VM -> 18V
VIO -> 5V

VREF: Adjust potentiometer for motor current
Higher clockwise = more current = more torque

INSTALL LIBRARY:
arduino-cli lib install AccelStepper

COMPILE & UPLOAD:
cd /Users/padmanabha/Projects/TennisBallMachineUI/ard/mc_v1/
arduino-cli compile --fqbn arduino:avr:mega mc_v1.ino
arduino-cli upload -p /dev/cu.usbmodem14101 --fqbn arduino:avr:mega mc_v1.ino
*/