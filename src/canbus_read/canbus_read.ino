/* Code for reading CANbus data and sending it to a Python program that
 * processes it for displaying or controlling the system.
 */

#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>
#include <mcp2515_defs.h>

void setup() {
  // Blink led on pin 13 for debugging
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
  // Establish connection with Python program
  Serial.begin(115200);
  byte read_byte;
  bool not_connected = true;
  while (not_connected) {
    read_byte = Serial.read();
    if (read_byte == 48) { // 48 = ascii number for '0' character
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      not_connected = false;
      Serial.write('1'); // return signal to python program
    }
  }

  // Setup canbus connection
  Serial.println("CAN Read: Testing receival of CAN bus message");
  if (Canbus.init(CANSPEED_500)) {
    Serial.println("CAN Init ok");
  }
  else {
    Serial.println("Can't init CAN");
  }

  delay(1000);
}

void loop() {
  // Read CANbus message
  tCAN message;
  if (mcp2515_check_message()) {
    if (mcp2515_get_message(&message)) {
      Serial.print("ID: ");
      Serial.print(message.id, HEX);
      Serial.print(", ");
      Serial.print("Data: ");
      Serial.print(message.header.length, DEC);
      for (int i = 0; i < message.header.length; i++) {
        Serial.print(message.data[i], HEX);
        Serial.print(" ");
      }
      Serial.print("\n");
    }
  }
  
  // Write message over serial for reading

}
