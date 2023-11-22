// Last Update 11/21/23 
//
// Emergency Automation superimposition on manual motor control.
// Created by Nathan Fikes for use in controlling motors from various Ultrasound sensors.
// Logic is unfinished. Thomas Tran
//
// Description: Program hosted on the Arduino NANO to handle all the data between the 4 ultrasound sensors, commands from the BLE peripheral, and handle motor control.

#include <Servo.h>
#include <math.h>    //round
#include <string.h>  //strcmp
#include <stdlib.h>  //abs

#define RATE_MICROS 16000
#define SERIAL_REFRESH 32  //How many cycles to skip until reload SERIAL

// If we need to show the sample rate for the ultrasounds, we can store the value here.
static int sample_rate = 1 / (RATE_MICROS / 1000000);

// Initialization of Ultrasound Variables

// Threshold at which the Vehicle will produce certain actions. 5cm past lower bound.
static int upperDistanceThreshold = 7;  // cm
static int lowerDistanceThreshold = 3;  // cm
int cm = 0;                             // Initialization of cm as a variable

static int motor_steering_angle = 0;    // -90 to 90 steering angle
static int motor_power = 0;             // -100% to 100% motor power

// For the driving motor. Pins used:
int PWM_pin_out = 11;                   // Needs to be a ~ pin on the NANO
int Dir_pin_out = 10;                   // Pin for pulling low/high for the nand gate h-bridge "fooling" circuit.

// For the steering motor. Pin used:
int Steering_servo_pin_out = 9;         // The steering motor data line should be connected to D11
int steer_max = 90;                     // Max of 90 degree steer angle.

Servo steering_servo;

long readUltrasonicDistance(int triggerPin, int echoPin) {
  //Code from TinkerCad, useful for driving any Ultrasound Sensor:

  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  // Sets the trigger pin to HIGH state for 10 microseconds
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(triggerPin, LOW);

  // Reads the echo pin, and returns the sound wave travel time in microseconds
  // We need to use the RATE_MICROS here to make sure to ignore signals that are not useful.
  return pulseIn(echoPin, HIGH, RATE_MICROS / 2);
}

// Function to handle the driving motor. Takes a power from -100 to 100
long motorPowerPWMandDir(int power) {
  // Function that should help set PWM pin without uneccesary math
  // Function should also set direction, aka H-Bridge control.
  float pwm_decimal = abs(power) * 2.25;          // Transforms power from 0 - +-100 to 0 - 225 all scaled in the positive dir

  analogWrite(PWM_pin_out, round(pwm_decimal));   // PWM pin takes this number as an int.

  if (power >= 0) {
    digitalWrite(Dir_pin_out, LOW);               // If the power is greater than 0 we tell the H-Bridge to go forward.
  } else if (power < 0) {
    digitalWrite(Dir_pin_out, HIGH);              // If the power is less than 0 we need to fool the H-Bridge to go backwards.
  } else {
    Serial.print("Error in the motor direction, can't resolve direction. Check motorPowerPWMandDir function."); // I doubt this line can be reached.
  }
}

// Function to handle the steering servo. Takes an angle.
long motorSteeringHandler(int steering_angle) {
  if (abs(steering_angle) >= steer_max) {
    Serial.print("Error in motor steering, past motor max.");
  } else {
    steering_servo.write(steering_angle + 90);
  }
}

// BAUD rate for serial communication using UART.
void setupSerial() {
  Serial.begin(115200);
}

// For each sensor we need to set the pins accordingly, the echo pin must be an input and the trigger must be an output.
void setupUltrasonicPins(int triggerPin, int echoPin) {
  pinMode(triggerPin, OUTPUT);  // Clear the trigger
  pinMode(echoPin, INPUT);
}

// Setup the ultrasonics using the pin configurations below.
void setupUltrasonic() {
  //setupUltrasonicPins(1, 2);
  setupUltrasonicPins(3, 4);
  //setupUltrasonicPins(5, 6);
  //setupUltrasonicPins(7, 8);
}

// Everything in this section will run only once. We can use this to set up some parameters.
void setup() {
  setupSerial();
  setupUltrasonic();
  steering_servo.attach(Steering_servo_pin_out, 500, 2500);
  pinMode(PWM_pin_out, OUTPUT);
  pinMode(Dir_pin_out, OUTPUT);
  analogWrite(PWM_pin_out, 0);
  digitalWrite(Dir_pin_out, LOW);
}

// This is useful for reading serial commands.
static String buffer = "";

void loop() {
  // Ultrasound sensor used [Contactless Distance Measurement]: HC-SR04
  // Datasheet declared accuracy: 0.03 cm
  // Datasheet declared range: 2cm - 400cm

  // measure the ping time in cm [TinkerCad calculation. May have to tune this.]
  // Ultrasound sensors occupy 2 ports. Need to reroute pin 1 as it is now a TX line.

  //float UF_cm = 0.01723 * readUltrasonicDistance(1, 2);
  //float UL_cm = 0.01723 * readUltrasonicDistance(3, 4);
  //float UR_cm = 0.01723 * readUltrasonicDistance(5, 6);
  //float UB_cm = 0.01723 * readUltrasonicDistance(7, 8);

  int UL_cm = 0;
  int UF_cm = 0;
  int UB_cm = 0;
  int UR_cm = 0;

  static int loop_counter;
  if (loop_counter++ % SERIAL_REFRESH == 0) {
    //Serial.print("\u0001");  // Control character for the ESPN32-S2 board to refresh the screen.
    /*
    Serial.print("\nUltraSound Sample Rate: ");
    Serial.print(sample_rate);
    Serial.print("\n[F], ");
    Serial.print(UF_cm);
    Serial.print("\n[L], ");
    Serial.print(UL_cm);
    Serial.print("\n[R], ");
    Serial.print(UR_cm);
    Serial.print("\n[B], ");
    Serial.print(UB_cm);
    Serial.print("\n\n");
    */
  }

  // We can read serial commands and interpret them to do certain tasks. In this case we will just consider motor functions. The lights will be handeled by the BLE server module. ESP32-WROVER-E.
  if (Serial.available()) {
    char receivedChar = Serial.read();            // Grab a character from the serial communication RX line.
    if (receivedChar == '\n') {
      Serial.print("Accumulated buffer: ");       // After awhile of accumulating things we will print it only if the recieved character is a special escape character \n.
      Serial.println(buffer.c_str());             // Display this buffer string. This will be displayed on the OLED screen as a serial back out.
      char message_type = buffer[0];              // The message type or tag is always the first character in the sequence.
      String message_value = buffer.substring(1); // The value is everything afterwards. This is everything after the index 0 denoted as substring(1).

      switch (message_type) {
        case 'P':
          {
            int motor_power = message_value.toInt();          // Reads the motor power off this tag. "P"
            motorPowerPWMandDir(motor_power);
            break;
          }
        case 'S':
          {
            int motor_steering_angle = message_value.toInt(); // Reads the steering angle off this tag. "S"
            motorSteeringHandler(motor_steering_angle);
            break;
          }
        default:
          {
            // Nothing in here.
          }
      }
      buffer = "";              // clear buffer after reading the value and using it.
    } else {
      
      buffer += receivedChar;   // Accumulate stuff into the buffer as long as the recieved char is not '\n'
    }
  }

  //   _______   _____ ______   _______   ________  ________  _______   ________   ________      ___    ___
  //  |\  ___ \ |\   _ \  _   \|\  ___ \ |\   __  \|\   ____\|\  ___ \ |\   ___  \|\   ____\    |\  \  /  /|
  //  \ \   __/|\ \  \\\__\ \  \ \   __/|\ \  \|\  \ \  \___|\ \   __/|\ \  \\ \  \ \  \___|    \ \  \/  / /
  //   \ \  \_|/_\ \  \\|__| \  \ \  \_|/_\ \   _  _\ \  \  __\ \  \_|/_\ \  \\ \  \ \  \        \ \    / /
  //    \ \  \_|\ \ \  \    \ \  \ \  \_|\ \ \  \\  \\ \  \|\  \ \  \_|\ \ \  \\ \  \ \  \____    \/  /  /
  //     \ \_______\ \__\    \ \__\ \_______\ \__\\ _\\ \_______\ \_______\ \__\\ \__\ \_______\__/  / /
  //      \|_______|\|__|     \|__|\|_______|\|__|\|__|\|_______|\|_______|\|__| \|__|\|_______|\___/ /
  //                                                                                            \___|/
  //  ________  ________  ________  _________  ________  ________  ________  ___       ________  ___
  //  |\   __  \|\   __  \|\   __  \|\___   ___\\   __  \|\   ____\|\   __  \|\  \     |\   ____\|\  \      
  //  \ \  \|\  \ \  \|\  \ \  \|\  \|___ \  \_\ \  \|\  \ \  \___|\ \  \|\  \ \  \    \ \  \___|\ \  \     
  //   \ \   ____\ \   _  _\ \  \\\  \   \ \  \ \ \  \\\  \ \  \    \ \  \\\  \ \  \    \ \_____  \ \  \    
  //    \ \  \___|\ \  \\  \\ \  \\\  \   \ \  \ \ \  \\\  \ \  \____\ \  \\\  \ \  \____\|____|\  \ \__\   
  //     \ \__\    \ \__\\ _\\ \_______\   \ \__\ \ \_______\ \_______\ \_______\ \_______\____\_\  \|__|
  //      \|__|     \|__|\|__|\|_______|    \|__|  \|_______|\|_______|\|_______|\|_______|\_________\  ___
  //                                                                                      \|_________| |\__\
  //                                                                                                   \|__|



  // Protocols for each emercency case [What should the car do when something is too close?]:

  delayMicroseconds(RATE_MICROS - micros()%RATE_MICROS);
}