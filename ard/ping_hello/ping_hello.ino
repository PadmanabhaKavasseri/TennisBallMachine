int counter = 1;

void setup() {
  Serial.begin(9600);
  delay(2000);  // Wait for serial connection to establish
  Serial.println("Arduino started!");
}

void loop() {
  Serial.print("Hello ");
  Serial.println(counter);
  counter++;
  delay(2000);  // Wait 2 seconds between messages
}

/*
cd /Users/padmanabha/Projects/TennisBallMachineUI/ard/ping_hello/
compile:
arduino-cli compile --fqbn arduino:avr:mega ping_hello.ino

flash
arduino-cli upload -p /dev/cu.usbmodem14501 --fqbn arduino:avr:mega ping_hello.ino
*/

