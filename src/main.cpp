#include <Arduino.h>
#include <BluetoothSerial.h>
#include <TMCStepper.h>
#include <HardwareSerial.h>


BluetoothSerial serialBT;
char cmd;

// put function declarations here:
#define TMC_SERIAL_PORT Serial2
#define TMC_UART_RX_PIN 16
#define TMC_UART_TX_PIN 17

#define DRIVER_ADDRESS 0b00 // TMC2209 driver address
#define R_SENSE 0.11f // Sense resistor value in ohms

TMC2209Stepper driver(&TMC_SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Starting TMC2209 UART Control Example (EN hardwired)");

  // No pinMode(EN_PIN, OUTPUT); or digitalWrite(EN_PIN, HIGH); needed

  TMC_SERIAL_PORT.begin(115200, SERIAL_8N1, TMC_UART_RX_PIN, TMC_UART_TX_PIN);

  delay(100);

  if (driver.test_connection()) {
    Serial.println("TMC2209 connection successful!");
  } else {
    Serial.println("TMC2209 connection FAILED!");
    while(1);
  }

  // Configure the driver via UART
  driver.toff(3);
  driver.rms_current(800);
  driver.pwm_mode(true);
  driver.microsteps(256);
  driver.pwm_autoscale(true);

  // Driver is already enabled due to hardwired EN to GND
  Serial.println("TMC2209 configured and always enabled.");
}

void loop() {
  Serial.println("Moving forward...");
  driver.VACTUAL(20000);
  delay(3000);

  Serial.println("Stopping (holding position)...");
  driver.VACTUAL(0); // This is how you "stop" movement and hold position
  delay(2000);

  Serial.println("Moving backward...");
  driver.VACTUAL(-20000);
  delay(3000);

  Serial.println("Stopping (holding position)...");
  driver.VACTUAL(0);
  delay(2000);
}

