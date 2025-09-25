
void setup() {
  Serial.begin(9600);
  delay(2000);  // Wait for serial connection
  Serial.println("Arduino ready!");
}

void loop() {
  if (Serial.available()) {
    String message = Serial.readStringUntil('\n');
    message.trim();
    Serial.println("Echo: " + message);
  }
}

/*
cd /Users/padmanabha/Projects/TennisBallMachineUI/ard/echo/
compile:
arduino-cli compile --fqbn arduino:avr:mega echo.ino

flash
arduino-cli upload -p /dev/cu.usbmodem14501 --fqbn arduino:avr:mega echo.ino
*/