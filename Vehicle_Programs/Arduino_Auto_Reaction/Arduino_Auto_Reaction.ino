// 11/20/23 Emergency Automation superimposition on manual motor control.
// Created by Nathan Fikes for use in controlling motors from various Ultrasound sensors.
// Logic is unfinished. Thomas Tran

#include <Servo.h>
#include <math.h>    //round
#include <string.h>  //strcmp
#include <stdlib.h>  //abs

#define RATE_MICROS 16000
#define SERIAL_REFRESH 32  //How many cycles to skip until reload SERIAL

// Initialization of Ultrasound Variables

// Threshold at which the Vehicle will produce certain actions. 5cm past lower bound.
static int distanceThreshold = 7;  //cm
int cm = 0;
static double sample_rate = 40;  // 1/S how many samples per second.

static int motor_steering_angle = 0;  // -90 to 90 steering angle
static int motor_power = 0;           // -100% to 100% motor power

// For the driving motor.
int PWM_pin_out = 11;
int Dir_pin_out = 10;

// For the steering motor.
int Steering_servo_pin_out = 9;  // The steering motor data line should be connected to D11
int steer_max = 90;              // Max of 90 degree steer angle for now.

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
  return pulseIn(echoPin, HIGH, RATE_MICROS / 2);
}

long motorPowerPWMandDir(int power) {
  // Function that should help set PWM pin without uneccesary math
  // Function should also set direction, aka H-Bridge control.
  float pwm_decimal = abs(power) * 2.25;

  analogWrite(PWM_pin_out, round(pwm_decimal));

  if (power >= 0) {
    digitalWrite(Dir_pin_out, LOW);
  } else if (power < 0) {
    digitalWrite(Dir_pin_out, HIGH);
  } else {
    Serial.print("Error in the motor direction, can't resolve direction. Check motorPowerPWMandDir function.");
  }
}

long motorSteeringHandler(int steering_angle) {
  if (abs(steering_angle) >= steer_max) {
    Serial.print("Error in motor steering, past motor max. Change steer_max global if more range is needed.");
  } else {
    steering_servo.write(steering_angle + 90);
  }
}

void setupSerial() {
  Serial.begin(115200);
}

void setupUltrasonicPins(int triggerPin, int echoPin) {
  pinMode(triggerPin, OUTPUT);  // Clear the trigger
  pinMode(echoPin, INPUT);
}

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

static String buffer = "";

void loop() {
  // Threshold at which the Vehicle will produce certain actions. 5cm past lower bound.
  distanceThreshold = 7;  //cm

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

  if (Serial.available()) {
    char receivedChar = Serial.read();
    if (receivedChar == '\n') {
      Serial.print("Accumulated buffer: ");
      Serial.println(buffer.c_str());

      char message_type = buffer[0];
      String message_value = buffer.substring(1);

      switch (message_type) {
        case 'P':
          {
            int motor_power = message_value.toInt();
            motorPowerPWMandDir(motor_power);

            break;
          }
        case 'S':
          {
            int motor_steering_angle = message_value.toInt();
            motorSteeringHandler(motor_steering_angle);
            break;
          }
        default:
          {
          }
      }
      buffer = "";  // clear buffer
    } else {
      // accumulate
      buffer += receivedChar;
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

  /*
  if (motor_power > 0){
    // If we are driving, we can't assume any value for motor_power, be careful when assigning values to the motor functions.

    if (UF_cm < distanceThreshold){
      // We can't set this to 0, what should motor power be if there is something in front that is slower?
      // Protip: scale motor power based on UF_cm reading.
    } else if (UB_cm < distanceThreshold){
      // We shouldn't accelerate crazy, what should motor power be if there is something behind moving faster?
      // Protip: scale motor power based on BF_cm reading.
    } else if (UB_cm < distanceThreshold && UF_cm < distanceThreshold){
      // Uh oh, we are in a traffic jam. We don't want to speed up or slow down. In this case we want to maximize space.
      // Protip: We should speed up if UF_cm < UB_cm or slow down if UF_cm > UB_cm. Scale these on U measurements.

    } else if (UL_cm < distanceThreshold){
      // Someone is too close to the left, which way should we turn?
      // Protip: set steeringHandler() to a specific steering angle. Negative for left, positive for right.
    } else if (UR_cm < distanceThreshold){
      // Man now someone is on the right side, which way should we turn?
      // Protip: set steeringHandler() to a specific steering angle. Negative for left, positive for right.
    } else if (UL_cm < distanceThreshold && UR_cm < distanceThreshold){
      // This is quite the pickle, we have two people on either side.
      // Minimizing space would not be good due to sensitivity. If this occurs we should just drive straight.
    }
  } else if (motor_power == 0){
    // If we are NOT driving, we are safe to assume motor_power is currently at 0

    if (UF_cm < distanceThreshold){
      // In this case someone is approaching from the front while we are stationary, where should we drive?
      // Protip: scale motor power based on UF_cm reading.
    } else if (UB_cm < distanceThreshold){
      // Now someone is approaching the back, which direction should we drive?
      // Protip: scale motor power based on BF_cm reading.
    } else if (UB_cm < distanceThreshold && UF_cm < distanceThreshold){
      // This is a real traffic jam where everyone is stopped. We should be careful where we drive.
      // Protip: We should speed up if UF_cm < UB_cm or slow down if UF_cm > UB_cm. Scale these on U measurements.

    } else if (UL_cm < distanceThreshold){
      // Someone is too close to the left, which way should we turn? Also which direction do we drive to make space?
      // Protip: This requires a special maneuver called a Jack-Knife generally used by Semi Trucks. The same concept applies here.
      // Protip on Jack-Knife: To perform a Jack-Knife, turn toward the incoming obstacle, drive backwards.
    } else if (UR_cm < distanceThreshold){
      // Someone is too close to the right now, which way should we turn? Also which direction do we drive to make space?
      // Protip: This requires a special maneuver called a Jack-Knife generally used by Semi Trucks. The same concept applies here.
      // Protip on Jack-Knife: To perform a Jack-Knife, turn toward the incoming obstacle, drive backwards.
    } else if (UL_cm < distanceThreshold && UR_cm < distanceThreshold){
      // This is quite the pickle, we have two people on either side.
      // Minimizing space would not be good due to sensitivity. If this occurs we should just brake [not move at all].
    }
  } else {
    // If no emergency protocols are activated, we can just default the driving parameters into the motor functions.
    //motorPowerPWMandDir(motor_power, motor_direction);
    //motorSteeringHandler(motor_steering_angle);
  }
  */

  /*
  for (int i = 255; i >= 0; i--) {
    analogWrite(PWM_pin_out, i);
    delay(50);
    digitalWrite(Dir_pin_out, HIGH);
  }

  for (int i = 0; i <= 255; i++) {
    analogWrite(PWM_pin_out, i);
    delay(50);
    digitalWrite(Dir_pin_out, LOW);
  }
  */

  /*
  for (int i = -90; i <= 90; i++) {
  motorSteeringHandler(i);
  delay(10);
  }
  for (int i = 90; i >= -90; i--) {
  motorSteeringHandler(i);
  delay(10);
  }
  */
  //motorPowerPWMandDir(-10);
  //motorPowerPWMandDir(motor_power);
  //motorSteeringHandler(motor_steering_angle);

  //digitalWrite(PWM_pin_out, HIGH);

  //delayMicroseconds(RATE_MICROS - micros()%RATE_MICROS);
  //delay(round((1/sample_rate) * 1000));
}