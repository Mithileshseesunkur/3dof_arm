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
  // put your setup code here, to run once:
  serialBT.begin("esp32-bt");
  pinMode(2, OUTPUT); // Set GPIO 2 as output for LED control

}

void loop() {
  // put your main code here, to run repeatedly:
  if (serialBT.available()){

    cmd=serialBT.read();

  }
  if(cmd=='1'){

    digitalWrite(2, HIGH);
  }
  if(cmd=='0'){

    digitalWrite(2, LOW);
  }

  delay(100);
  
}

