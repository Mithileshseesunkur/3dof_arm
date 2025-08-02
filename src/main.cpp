// #include <Arduino.h>
// #include <BluetoothSerial.h>
// #include <TMCStepper.h>
// #include <HardwareSerial.h>


// BluetoothSerial serialBT;
// char cmd;

// // put function declarations here:
// #define TMC_SERIAL_PORT Serial1
// #define TMC_UART_RX_PIN 16
// #define TMC_UART_TX_PIN 17

// #define DRIVER_ADDRESS 0b00 // TMC2209 driver address
// #define R_SENSE 0.11f // Sense resistor value in ohms

// TMC2209Stepper driver(&TMC_SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);

// void setup() {
//   Serial.begin(115200);
//   while (!Serial);
//   Serial.println("Starting TMC2209 UART Control Example (EN hardwired)");

//   // No pinMode(EN_PIN, OUTPUT); or digitalWrite(EN_PIN, HIGH); needed

//   TMC_SERIAL_PORT.begin(115200, SERIAL_8N1, TMC_UART_RX_PIN, TMC_UART_TX_PIN);

//   delay(100);

//   if (driver.test_connection()) {
//     Serial.println("TMC2209 connection successful!");
//   } else {
//     Serial.println("TMC2209 connection FAILED!");
//     while(1);
//   }

//   // Configure the driver via UART
//   driver.toff(3);
//   driver.rms_current(800);
//   driver.pwm_mode(TMC2209_MODE_STEALTHCHOP);
//   driver.microsteps(256);
//   driver.pwm_autoscale(true);

//   // Driver is already enabled due to hardwired EN to GND
//   Serial.println("TMC2209 configured and always enabled.");
// }

// void loop() {
//   Serial.println("Moving forward...");
//   driver.VACTUAL(20000);
//   delay(3000);

//   Serial.println("Stopping (holding position)...");
//   driver.VACTUAL(0); // This is how you "stop" movement and hold position
//   delay(2000);

//   Serial.println("Moving backward...");
//   driver.VACTUAL(-20000);
//   delay(3000);

//   Serial.println("Stopping (holding position)...");
//   driver.VACTUAL(0);
//   delay(2000);
// }

#include <TMCStepper.h>
#include <AccelStepper.h>

#define SW_RX             17 // TMC2208/TMC2224 SoftwareSerial receive pin
#define SW_TX             16 // TMC2208/TMC2224 SoftwareSerial transmit pin
#define EN_PIN            32  // LOW: Driver enabled. HIGH: Driver disabled
#define STEP_PIN          12  // Step on rising edge
#define DIR_PIN           13  // Step on rising edge
#define SERIAL_PORT       Serial2 // TMC2208/TMC2224 HardwareSerial port
#define R_SENSE           0.11  // SilentStepStick series use 0.11, Watterott TMC5160 uses 0.075
#define DRIVER_ADDRESS    0b00 // TMC2209 Driver address according to MS1 and MS2
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

#define EN_PIN_2          26  // LOW: Driver enabled. HIGH: Driver disabled
#define STEP_PIN_2        27  // Step on rising edge
#define DIR_PIN_2         14  // Step on rising edge
#define R_SENSE_2         0.11  // SilentStepStick series use 0.11, Watterott TMC5160 uses 0.075
#define DRIVER_ADDRESS_2  0b10 // TMC2209 Driver address according to MS1 and MS2
TMC2209Stepper driver_2(&SERIAL_PORT, R_SENSE_2, DRIVER_ADDRESS_2);
AccelStepper stepper_2 = AccelStepper(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);

void setup() {
  Serial.begin(115200); 

  // Initialize UART for TMC2209
  SERIAL_PORT.begin(115200, SERIAL_8N1, SW_RX, SW_TX);
  driver.begin();
  driver_2.begin();
  // Enable drivers
  pinMode(EN_PIN, OUTPUT);
  pinMode(EN_PIN_2, OUTPUT);
  digitalWrite(EN_PIN, LOW);   // Enable Yaw stepper
  digitalWrite(EN_PIN_2, LOW); // Enable Pitch stepper

  // Configure steppers
  // Enable stealthChop mode for the first driver (Yaw)
  driver.en_spreadCycle(false);
  driver_2.en_spreadCycle(false);

  driver.microsteps(64); // Set microstepping to 64
  driver_2.microsteps(16); // Set microstepping to 64 for the second driver (Pitch)
  driver.rms_current(900); // Set RMS current for Yaw stepper
  driver_2.rms_current(900); // Set RMS current for Pitch stepper
  stepper.setMaxSpeed(5000); // Set max speed (steps per seco nd)
  stepper.setAcceleration(1000); // Set acceleration (steps per second^2)
  stepper_2.setMaxSpeed(2000);
  stepper_2.setAcceleration(1000);

  // Read the GCONF register to confirm the setting
  uint32_t gconf_yaw = driver.GCONF();
  Serial.print("GCONF (Yaw): 0x");
  Serial.println(gconf_yaw, HEX);
}

void loop() {
  Serial.println("Moving Yaw & Pitch...");

  // Move Yaw stepper forward & backward
  stepper.move(10000);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  delay(500);
  stepper.move(-10000);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  //Move Pitch stepper forward & backward
  stepper_2.move(5000);
  while (stepper_2.distanceToGo() != 0) {
    stepper_2.run();
  }
  delay(500);
  stepper_2.move(-5000);
  while (stepper_2.distanceToGo() != 0) {
    stepper_2.run();
  }

  Serial.println("Completed one movement cycle.");
  delay(2000);
}