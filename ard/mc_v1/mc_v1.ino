#include <Servo.h>
#include <TMCStepper.h>

Servo linearActuator;
Servo topMotor;
Servo bottomMotor;

const int LA_PIN = 6;
const int TOP_MOTOR_PIN = 3;
const int BOTTOM_MOTOR_PIN = 2;

// Stepper Motor
#define DIAG_PIN       23
#define DRIVER_ADDRESS 0b00
#define R_SENSE        0.11f
#define LIMIT_PIN      28

TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);
bool homingComplete = false;
bool backingOff = false;
unsigned long backoffStartTime = 0;

void setup() {
  Serial.begin(9600);
  
  // Setup Linear Actuator
  linearActuator.attach(LA_PIN);
  topMotor.attach(TOP_MOTOR_PIN);
  bottomMotor.attach(BOTTOM_MOTOR_PIN);
  
  setupStepper();

  delay(2000);
  Serial.println("Arduino ready!");
}

void loop() {
  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    message.trim();

    if (message.startsWith("MSG_START") && message.endsWith("MSG_END")) {
      processMessage(message);
    } else {
      Serial.println("Invalid message format: " + message);
    }
  }
}

void setupStepper(){
  Serial1.begin(9600);

  pinMode(DIAG_PIN, INPUT_PULLUP);
  pinMode(LIMIT_PIN, INPUT_PULLUP);

  driver.begin();
  driver.toff(5);
  driver.rms_current(1500);
  driver.microsteps(16);
  driver.pwm_autoscale(true);
  driver.en_spreadCycle(false);
  driver.TCOOLTHRS(0xFFFFF);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);

  Serial.println("Stepper Setup Complete");
}

void processMessage(String msg) {
  msg.replace("MSG_START|", "");
  msg.replace("|MSG_END", "");

  int lastChecksumIndex = msg.lastIndexOf("|#");
  String core = msg.substring(0, lastChecksumIndex);
  String checksum = msg.substring(lastChecksumIndex + 2);

  String parts[10];
  int count = splitString(core, '|', parts, 10);

  if (count < 3) {
    Serial.println("Malformed message");
    return;
  }

  String version = parts[0];
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
  } else {
    Serial.println("Unknown motor: " + motor);
  }
}

void handleLinearActuator(String pitch, String msgId) {
  int pitchValue = pitch.toInt();  // 0-100 from your interface
  
  // Map 0-100 to servo range 0-180 (internally maps to 1000-2000μs)
  int servoAngle = map(pitchValue, 0, 100, 0, 180);
  
  linearActuator.write(servoAngle);
  
  Serial.println("LA set to " + pitch + " (Servo angle: " + String(servoAngle) + "°) [MSG_ID: " + msgId + "]");
}

void handleTopMotor(String speed, String msgId) {
  int speedValue = speed.toInt();
  int servoAngle = map(speedValue, 0, 100, 0, 180);
  topMotor.write(servoAngle);
  Serial.println("TOP_MOTOR set to " + speed + " [MSG_ID: " + msgId + "]");
}

void handleBottomMotor(String speed, String msgId) {
  int speedValue = speed.toInt();
  int servoAngle = map(speedValue, 0, 100, 0, 180);
  bottomMotor.write(servoAngle);
  Serial.println("BOTTOM_MOTOR set to " + speed + " [MSG_ID: " + msgId + "]");
}

void handleStepper(String command, String msgId) {
  if (command == "HOME") {
    Serial.println("🔄 Homing stepper... [MSG_ID: " + msgId + "]");
    driver.VACTUAL(-2000);

    while (digitalRead(LIMIT_PIN) != LOW) {
      delay(1);
    }

    driver.VACTUAL(0);
    delay(500);

    driver.VACTUAL(2000);
    delay(500);
    driver.VACTUAL(0);

    Serial.println("✅ Homing complete.");
    homingComplete = true;
    return;
  }

  if (command == "ENABLE") {
    Serial.println("✅ Stepper driver enabled [MSG_ID: " + msgId + "]");
    return;
  }

  if (command == "DISABLE") {
    Serial.println("⛔ Stepper driver disabled [MSG_ID: " + msgId + "]");
    return;
  }

  Serial.println("STEPPER set to " + command + "° [MSG_ID: " + msgId + "]");
  // Add actual motor control logic here
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
cd /Users/padmanabha/Projects/TennisBallMachineUI/ard/mc_v1/
compile:
arduino-cli compile --fqbn arduino:avr:mega mc_v1.ino

flash:
arduino-cli upload -p /dev/cu.usbmodem14101 --fqbn arduino:avr:mega mc_v1.ino

arduino-cli lib install Servo
*/