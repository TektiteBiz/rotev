#include <TektiteRotEv.h>

RotEv rotev;

void setup() {
  rotev.begin();
  Serial.println("Hello, RotEv!");  // rotev.begin() automatically starts Serial
                                    // at 115200 baud
}

// Use a boolean variable to trigger the GO functionality only when the button
// is released
bool goPressed = false;

// Variable to see if the GO functionality is currently active
bool going = false;

// Main loop
void loop() {
  // Handle button presses
  if (rotev.goButtonPressed()) {
    goPressed = true;

    rotev.ledWrite(0.0f, 0.1f, 0.0f);  // 10% green
  } else if (goPressed && !rotev.goButtonPressed()) {
    goPressed = false;

    // Trigger the GO functionality
    going = true;
    rotev.motorEnable(true);  // Enable the motor drivers
  } else if (rotev.stopButtonPressed()) {
    going = false;
    rotev.motorEnable(false);  // Disable the motor drivers
    rotev.servoDetach();       // Disables the servo

    rotev.ledWrite(0.1f, 0.0f, 0.0f);  // 10% red, since stop is pressed
  } else {
    rotev.ledWrite(0.0f, 0.0f, 0.1f);  // 10% blue
  }

  // Motor writes
  if (going) {
    rotev.motorWrite1(0.5f);  // Set motor 1 speed to 50%

    float voltage = rotev.getVoltage();  // Returns the voltage of the battery
    rotev.motorWrite2(4.0f / voltage);   // Set motor 2 voltage to 4V

    rotev.servoWrite(90.0f);  // Move the servo to the middle (range: 0-180).
                              // This method automatically attaches the servo.
  }
}

// SECOND CORE: The RP2350 processor has two cores. You can use the FIFO bus to
// transfer data or use global variables. If using global variables you may want
// to use mutexes.
void setup1() {
  // You can put setup code for the second core here
}

void loop2() {
  // I recommend doing sensor fusion on the second core, since this needs to
  // occur at a high rate. In this example it is printing out sensor data.
  float yawRateDeg = rotev.readYawRateDegrees();
  float enc1Angle = rotev.enc1Angle();
  float enc2Angle = rotev.enc2Angle();
  float curr1 = rotev.motorCurr1();
  float curr2 = rotev.motorCurr2();
  float voltage = rotev.getVoltage();

  // If you print in this format, you can use Arduino's Serial monitor to plot
  // the data over time
  Serial.print("yawRate:");
  Serial.print(yawRateDeg);

  Serial.print(",enc1:");
  Serial.print(enc1Angle);
  Serial.print(",enc2:");
  Serial.print(enc2Angle);

  Serial.print(",curr1:");
  Serial.print(curr1);
  Serial.print(",curr2:");
  Serial.print(curr2);

  Serial.print(",voltage:");
  Serial.print(voltage);

  Serial.println();

  delay(50);  // This slows down the printing rate. Note that you would not want
              // a delay in your actual code, since sensor fusion must occur at
              // a high frequency
}
