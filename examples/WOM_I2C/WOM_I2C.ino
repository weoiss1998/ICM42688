#include "ICM42688.h"
// TODO: Need to test this with the ICM42688

// an ICM42688 object with the ICM42688 sensor on I2C bus 0 with address 0x68
ICM42688 IMU(Wire,0x68);
int status;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  // attaching the interrupt to microcontroller pin 1
  pinMode(1,INPUT);
  attachInterrupt(1,wakeUp,RISING);
}

void loop() {}

void wakeUp() {
  Serial.println("Awake!");
}
