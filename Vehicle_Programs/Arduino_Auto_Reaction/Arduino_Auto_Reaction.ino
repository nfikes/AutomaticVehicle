// Last Update 11/25/23 
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
static int UF_cm = 0;
static int UL_cm = 0;
static int UR_cm = 0;
static int UB_cm = 0;

// Threshold at which the Vehicle will produce certain actions. 5cm past lower bound.
static int upperDistanceThreshold = 20;  // cm
static int lowerDistanceThreshold = 3;  // cm
int cm = 0;                             // Initialization of cm as a variable

static int distanceThreshold = 20;       // Was not declared earlier to allow for compatability with older code.

static int motor_steering_angle = 0;    // -90 to 90 steering angle
static int motor_power = 0;             // -100% to 100% motor power

// For the driving motor. Pins used:
int PWM_pin_out = 11;                   // Needs to be a ~ pin on the NANO (D11)
int Dir_pin_out = 10;                   // Pin for pulling low/high for the nand gate h-bridge "fooling" circuit. (D10)

// For the steering motor. Pin used:
int Steering_servo_pin_out = 9;         // The steering motor data line should be (D9)
int steer_max = 95;                     // Max of 90 degree steer angle. Stepper goes max 90

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
  steering_servo.write(constrain(steering_angle, -90, 90) + 90);
}

float ultrasoundFixer(float ultrasoundSig) {
  if (ultrasoundSig < 3.00){
    return 100.0;
  } else {
    return ultrasoundSig;
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
  setupUltrasonicPins(2, 3);    //D2 and D3
  setupUltrasonicPins(4, 5);    //D4 and D5
  setupUltrasonicPins(6, 7);    //D6 and D7
  setupUltrasonicPins(8, 12);   //D8 and D12
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

  float UF_cm = 0.01723 * readUltrasonicDistance(2, 3);
  float UL_cm = 0.01723 * readUltrasonicDistance(4, 5);
  float UR_cm = 0.01723 * readUltrasonicDistance(6, 7);
  float UB_cm = 0.01723 * readUltrasonicDistance(8, 12);

  UF_cm = ultrasoundFixer(UF_cm);
  UL_cm = ultrasoundFixer(UL_cm);
  UR_cm = ultrasoundFixer(UR_cm);
  UB_cm = ultrasoundFixer(UB_cm);

  static int loop_counter;
  if (loop_counter++ % SERIAL_REFRESH == 0) {
    Serial.print("\u0001");  // Control character for the ESPN32-S2 board to refresh the screen.
    Serial.print(UF_cm);
    Serial.print("\n");
    Serial.print(UL_cm);
    Serial.print("\n");
    Serial.print(UR_cm);
    Serial.print("\n");
    Serial.print(UB_cm);
    Serial.print("\n");
    
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

  static float reaction_power = 100;

  if (abs(motor_power) > 10){
    // If we are driving, we can't assume any value for motor_power, be careful when assigning values to the motor functions.

    if (UF_cm < distanceThreshold){
      // We can't set this to 0, what should motor power be if there is something in front that is slower?
      // Protip: scale motor power based on UF_cm reading.
      //Assuming motor power is postiive (moving forward), things should start happening within that 7cm. Front Ultrasonic is detecting something front.
      //float reaction_power = motor_power / 5;
      //Scaling down how strong influence is based on how fast we're going.

      float influence = (-UF_cm * ((reaction_power)/(distanceThreshold-3)) + ((reaction_power)/(distanceThreshold-3)) - 3 + reaction_power );
      //Crude version of Nathan's desmos formula. When UF_cm = 3, influence value will be 0. When UF_cm = 7, influence value will be in this case, 20.
      motorPowerPWMandDir(motor_power - influence);
      //What this is doing is if the UF is reading 7, the infuence value will be 0, therefore 0 changes to MP. 
      //Could possible wrap influence as a function for other if statements.

      //Superimposing motor_reaction on top of motor_power, we want to have a high value for a lower reading. 1/7 = smaller value than 1/3.
      //Scale the UF_cm reading to better influence the motor_power. 


    } else if (UB_cm < distanceThreshold){
      // We shouldn't accelerate crazy, what should motor power be if there is something behind moving faster?
      // Protip: scale motor power based on BF_cm reading.
      //float reaction_power = motor_power / 5;
      //This float can be defined as a global later.


      float influence = (-UB_cm * ((reaction_power)/(distanceThreshold-3)) + ((reaction_power)/(distanceThreshold-3)) - 3 + reaction_power );
      motorPowerPWMandDir(motor_power + influence);      
      //If car reads something from BEHIND, then you want to add influence instead of subtracting.
      
    } else if (UB_cm < distanceThreshold && UF_cm < distanceThreshold){
      // Uh oh, we are in a traffic jam. We don't want to speed up or slow down. In this case we want to maximize space.
      // Protip: We should speed up if UF_cm < UB_cm or slow down if UF_cm > UB_cm. Scale these on U measurements.
      //Different between UB and UF sensors. Take the difference between the two sensors, if one of higher, then that sensor has to infleunce the motor more.
      //float reaction_power = motor_power / 5;
      float influence = (-(UB_cm - UF_cm ) * ((reaction_power)/(distanceThreshold-3)) + ((reaction_power)/(distanceThreshold-3)) - 3 + reaction_power );
      motorPowerPWMandDir(motor_power + influence);       


    } else if (UL_cm < distanceThreshold){
      // Someone is too close to the left, which way should we turn?
      // Protip: set motorSteeringHandler() to a specific steering angle. Negative for left, positive for right.
      //Imagine a jackknife, if something approaches 
      //float reaction_power = motor_power / 5;
      float influence = (-UL_cm * ((reaction_power)/(distanceThreshold-3)) + ((reaction_power)/(distanceThreshold-3)) - 3 + reaction_power );

      motorSteeringHandler(motor_steering_angle + influence);

    } else if (UR_cm < distanceThreshold){
      // Man now someone is on the right side, which way should we turn?
      // Protip: set motorSteeringHandler() to a specific steering angle. Negative for left, positive for right.
      //float reaction_power = motor_power / 5;
      float influence = (-UR_cm * ((reaction_power)/(distanceThreshold-3)) + ((reaction_power)/(distanceThreshold-3)) - 3 + reaction_power );

      motorSteeringHandler(motor_steering_angle - influence);

    } else if (UL_cm < distanceThreshold && UR_cm < distanceThreshold){
      // This is quite the pickle, we have two people on either side.
      // Minimizing space would not be good due to sensitivity. If this occurs we should just drive straight.
      motorSteeringHandler(motor_steering_angle);
      // Is this just enough or would slowing down also be a needed measure?

    }
  } else if (abs(motor_power) < 10){
    // If we are NOT driving, we are safe to assume motor_power is currently at 0

    if (UF_cm < distanceThreshold){
      // In this case someone is approaching from the front while we are stationary, where should we drive?
      // Protip: scale motor power based on UF_cm reading.
      //float reaction_power = motor_power / 5;
      float influence = (-UF_cm * ((reaction_power)/(distanceThreshold-3)) + ((reaction_power)/(distanceThreshold-3)) - 3 + reaction_power );
      motorPowerPWMandDir(motor_power - influence);  
      

    } else if (UB_cm < distanceThreshold){
      // Now someone is approaching the back, which direction should we drive?
      // Protip: scale motor power based on BF_cm reading.
      //float reaction_power = motor_power / 5;
      float influence = (-UB_cm * ((reaction_power)/(distanceThreshold-3)) + ((reaction_power)/(distanceThreshold-3)) - 3 + reaction_power );
      motorPowerPWMandDir(motor_power + influence);  


    } else if (UB_cm < distanceThreshold && UF_cm < distanceThreshold){
      // This is a real traffic jam where everyone is stopped. We should be careful where we drive.
      // Protip: We should speed up if UF_cm < UB_cm or slow down if UF_cm > UB_cm. Scale these on U measurements.
      //float reaction_power = motor_power / 5;
      float influence = (-UB_cm * ((reaction_power)/(distanceThreshold-3)) + ((reaction_power)/(distanceThreshold-3)) - 3 + reaction_power );
      float influence2 = (-UF_cm * ((reaction_power)/(distanceThreshold-3)) + ((reaction_power)/(distanceThreshold-3)) - 3 + reaction_power );
      motorPowerPWMandDir(motor_power + (influence2 - influence));  



    } else if (UL_cm < distanceThreshold){
      // Someone is too close to the left, which way should we turn? Also which direction do we drive to make space?
      // Protip: This requires a special maneuver called a Jack-Knife generally used by Semi Trucks. The same concept applies here.
      // Protip on Jack-Knife: To perform a Jack-Knife, turn toward the incoming obstacle, drive backwards.
      //float reaction_power = motor_power / 5;
      float influence = (-UL_cm * ((reaction_power)/(distanceThreshold-3)) + ((reaction_power)/(distanceThreshold-3)) - 3 + reaction_power );

      motorPowerPWMandDir(motor_power - influence);  
      motorSteeringHandler(motor_steering_angle - influence);

    } else if (UR_cm < distanceThreshold){
      // Someone is too close to the right now, which way should we turn? Also which direction do we drive to make space?
      // Protip: This requires a special maneuver called a Jack-Knife generally used by Semi Trucks. The same concept applies here.
      // Protip on Jack-Knife: To perform a Jack-Knife, turn toward the incoming obstacle, drive backwards.
      //float reaction_power = motor_power / 5;
      float influence = (-UR_cm * ((reaction_power)/(distanceThreshold-3)) + ((reaction_power)/(distanceThreshold-3)) - 3 + reaction_power );

      motorPowerPWMandDir(motor_power - influence);  
      motorSteeringHandler(motor_steering_angle + influence);


    } else if (UL_cm < distanceThreshold && UR_cm < distanceThreshold){
      // This is quite the pickle, we have two people on either side.
      // Minimizing space would not be good due to sensitivity. If this occurs we should just brake [not move at all].
      motorPowerPWMandDir(0);
      //Would superimposing matter due to the fact we want this to be a sudden brake? Or would we want a deceleration to be put into place?
    }
  } else {
    // If no emergency protocols are activated, we can just default the driving parameters into the motor functions.
    motorPowerPWMandDir(motor_power);
    //Be very careful with the inputs, whatever changes to input must consider the existing input. Try to superimpose reactions that ALTER.
    //motorPowerPW is going to override motor_power and motor_direction.
    motorSteeringHandler(motor_steering_angle);
  }

  // Protocols for each emercency case [What should the car do when something is too close?]:

  delayMicroseconds(RATE_MICROS - micros()%RATE_MICROS);
}