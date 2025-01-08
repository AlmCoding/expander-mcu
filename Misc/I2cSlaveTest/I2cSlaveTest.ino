#include <avr/io.h>
#include <avr/interrupt.h>

#define SLAVE_ADDRESS 0x03
#define NACK_AT_BYTE 3 // Number of bytes to ACK before sending a NACK

volatile uint8_t byteCount = 0;
bool inProgress = false;
uint32_t startTime = 0;
uint32_t resetDuration = 100; // Time in ms

void setup() {
  // Set the slave address
  TWAR = (SLAVE_ADDRESS << 1); // Load the slave address into the TWI Address Register

  // Enable TWI, ACK, and interrupt
  TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE);

  Serial.begin(9600);
  Serial.println("I2C Slave ready to send immediate NACK.");
}

void loop() {
  // Main loop does nothing; everything is handled by the ISR

  if (byteCount > 0) {
    if (inProgress == false) {
      inProgress = true;
      startTime = millis();
    } else if (millis() - startTime >= resetDuration) {
      Serial.println("Reset byte count.");
      inProgress = false;
      byteCount = 0;
    }
  }
}

// ISR for TWI (Two-Wire Interface)
ISR(TWI_vect) {
  uint8_t status = TWSR & 0xF8; // Mask the status register

  switch (status) {
    case 0x60: // SLA+W received, ACK sent
      byteCount = 0; // Reset byte count on new transmission
      if (NACK_AT_BYTE == 0) {
       TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT); // Disable ACK, send NACK for the next byte
      } else {
       TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE) | (1 << TWINT);
      }
      break;

    case 0x80: // Data byte received, ACK sent
      byteCount++;
      if (byteCount >= NACK_AT_BYTE) {
        TWCR = (1 << TWEN) | (1 << TWIE) | (1 << TWINT); // Disable ACK, send NACK for the next byte
        //Serial.println("Sending immediate NACK...");
      } else {
        TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE) | (1 << TWINT); // Continue to ACK
        //Serial.print("Received byte: ");
        //Serial.println(TWDR); // Print received byte
      }
      break;

    case 0xA0: // Stop or repeated start condition received
      TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE) | (1 << TWINT); // Reset for the next transfer
      //Serial.println("Stop condition received.");
      break;

    default: // Catch-all for unexpected states
      //Serial.print("Unexpected TWI state: 0x");
      //Serial.println(status, HEX);
      TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWIE) | (1 << TWINT); // Recover gracefully
      break;
  }
}
