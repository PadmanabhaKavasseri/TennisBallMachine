void setup() {
  Serial.begin(9600);
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

void processMessage(String msg) {
  // Remove wrappers
  msg.replace("MSG_START|", "");
  msg.replace("|MSG_END", "");

  // Split into parts
  int lastChecksumIndex = msg.lastIndexOf("|#");
  String core = msg.substring(0, lastChecksumIndex);
  String checksum = msg.substring(lastChecksumIndex + 2);

  // Optionally validate checksum here...

  // Split core message
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
      setMotor(motor, value, msgId);
    }
  }
}

void setMotor(String name, String value, String msgId) {
  Serial.println("SET " + name + " TO " + value + " [MSG_ID: " + msgId + "]");
}

// Utility: split string by delimiter
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

flash
arduino-cli upload -p /dev/cu.usbmodem14501 --fqbn arduino:avr:mega mc_v1.ino
*/