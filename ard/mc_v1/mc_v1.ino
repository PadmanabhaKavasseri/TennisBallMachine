#include <TMCStepper.h>

// #define EN_PIN         6
#define DIAG_PIN       3
#define DRIVER_ADDRESS 0b00
#define R_SENSE        0.11f
#define LIMIT_PIN      10           // Physical limit switch input

TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);
bool homingComplete = false;
bool backingOff = false;
unsigned long backoffStartTime = 0;

void setup() {
  Serial.begin(9600);

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
  //rxtx1 for stepper motor
  Serial1.begin(9600);

  // pinMode(EN_PIN, OUTPUT);
  pinMode(DIAG_PIN, INPUT_PULLUP);
  pinMode(LIMIT_PIN, INPUT_PULLUP); // ✅ Added this line to stabilize input
  // digitalWrite(EN_PIN, LOW);        // Enable driver

  driver.begin();
  driver.toff(5);                   // Enable driver timing
  driver.rms_current(1500);         // Set motor current
  driver.microsteps(16);            // Microstepping for smooth motion
  driver.pwm_autoscale(true);       // Maintain torque at low speeds
  driver.en_spreadCycle(false);     // StealthChop for quiet operation
  driver.TCOOLTHRS(0xFFFFF);        // StallGuard velocity threshold
  driver.semin(5);                  // StallGuard sensitivity
  driver.semax(2);                  // StallGuard upper threshold
  driver.sedn(0b01);                // StallGuard filter setting

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

void handleTopMotor(String rpm, String msgId) {
  Serial.println("TOP_MOTOR set to " + rpm + " RPM [MSG_ID: " + msgId + "]");
  // Add actual motor control logic here
}

void handleBottomMotor(String rpm, String msgId) {
  Serial.println("BOTTOM_MOTOR set to " + rpm + " RPM [MSG_ID: " + msgId + "]");
  // Add actual motor control logic here
}

void handleStepper(String angle, String msgId) {
  if (angle == "HOME") {
    Serial.println("🔄 Homing stepper... [MSG_ID: " + msgId + "]");
    driver.VACTUAL(-2000); // Move toward limit

    while (digitalRead(LIMIT_PIN) != LOW) {
      delay(1); // crude polling
    }

    driver.VACTUAL(0); // Stop
    delay(500);

    driver.VACTUAL(2000); // Back off
    delay(1000);          // Adjust duration as needed
    driver.VACTUAL(0);

    Serial.println("✅ Homing complete.");
    homingComplete = true;
    return;
  }
  Serial.println("STEPPER set to " + angle + "° [MSG_ID: " + msgId + "]");
  // Add actual motor control logic here
}

void handleLinearActuator(String angle, String msgId) {
  Serial.println("LA set to " + angle + "° [MSG_ID: " + msgId + "]");
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
arduino-cli upload -p /dev/cu.usbmodem14501 --fqbn arduino:avr:mega mc_v1.ino
*/