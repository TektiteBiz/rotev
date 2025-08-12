# Tektite RotEv
This library provides code to use the hardware built into the [Tektite RotEv Controller](https://tektitebiz.com/product/tektite-rotev-controller-beta/)! Read below for documentation on the functions.

**Use the examples! See the `full` example for how to use the board's full capabilities, and make a new project using the `empty` example!**

## Initialization
Instantiate a "RotEv" library by including `TektiteRotev.h` and making a global variable:
```c
RotEv rotev;

void setup() {
  rotev.begin();
}
```

Use the `empty` example to get a blank program!

If the LEDs become red, look at the serial monitor! The `begin` method detects and prints out errors.

## Buttons
Use the GO (green) and STOP (red) buttons to start and stop your code. To see if the button is pressed, use the following functions that return booleans:
```cpp
rotev.goButtonPressed()
rotev.stopButtonPressed()
```

## RGB LED
The board includes an RGB LED, for which I recommend showing button states or the status within a motion profile. Use floating-point values from 0-1 for red, green, and blue. 

```cpp
rotev.ledWrite(r, g, b);
```

## Built-in IMU
A MPU-6500 IMU is built into the board. While it supports reading acceleration and gyroscope on all axes, yaw ("z-axis") rotation rate is the only useful one. To read it, use: 
```cpp
rotev.readYawRate() // Returns yaw rate in rad/s
rotev.readYawRateDegrees() // Returns yaw rate in deg/s
```

## MT6701 Encoder Connectors
There are two SPI connectors on the board meant for Tektite Slim Encoders. To read the angle of the magnet measured by these encoders, use:

```cpp
// Read angle in radians for encoder 1 and 2:
rotev.enc1Angle()
rotev.enc2Angle()

// Read angle in degrees for encoder 1 and 2:
rotev.enc1AngleDegrees()
rotev.enc2AngleDegrees()
```

## Servo Port
You can write an angle from 0-180 degrees, which corresponds to the range from 1000 to 2000uS PWM. To move the servo to an angle in this range, use:

```cpp
rotev.servoWrite(angle) // angle must be a float in the range 0-180
```

This method automatically enables the servo. By default, the servo is disabled.

To re-disable the attached servo, use:
```cpp
rotev.servoDetach()
```

## Battery Voltage
To read the battery voltage as a float, use:
```cpp
rotev.getVoltage()
```

If USB is connected but a battery isn't, this value will be close to zero.

## Motor Control
There are two built-in DRV8873 drivers that can do 2.5A continuous and 5A peak each. By default, these are disabled and the motors will coast. To use the drivers, you must first enable the motors:
```cpp
rotev.motorEnable(true) // Use false to disable the drivers
```

To write a duty cycle (percent of time the motor is connected to the battery), use:
```cpp
rotev.motorWrite1(0.50f) // Writes a 50% duty cycle to Motor 1 port
rotev.motorWrite2(0.25f) // Writes a 25% duty cycle to Motor 2 port
```

**Pro Tip!** You can use the `getVoltage` method to write voltages instead of duty cycles, which will make the robot perform more consistently across battery charge states. For example, to write a voltage of 6V you could do:

```cpp
float vbat = rotev.getVoltage();
rotev.motorWrite1(6.0f / vbat); // Writes 6V to Motor 1 port
```

If the battery voltage is 12V, then `vbat` will be 12, so the duty cycle will become 50%. 50% of 12V is 6V, so the motor will have 6V across it. 

If the battery voltage has dropped to 10V, then the the duty cycle will become higher to account for this (at 10V, it would be 60%), so the motor will still move roughly the same speed as if the battery was at 12V.

Note that you should limit the duty cycle to a max of `1.0f`, so that if the voltage drops to 9V and you're trying to write a voltage of 10V it still functions normally. You can achieve this with something like:
```cpp
rotev.motorWrite1(min(6.0f / vbat, 1.0f));
```
