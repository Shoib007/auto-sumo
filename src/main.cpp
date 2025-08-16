#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h> // Include ESP32Servo for servo control
#include <IRremote.h>   // Include IRremote for IR remote control

// IR Remote Pin
#define IR_REMOTE_PIN 4 // Pin for IR remote control

// Servo Pins
#define SERVO_PIN 23
Servo flagServo; // Servo for flag control

// Rhino Motor Driver Pins
#define DIR1 12
#define PWM1 14
#define DIR2 27
#define PWM2 15

// Channel for PWM
#define PWM_FREQUENCY 10000 // 10 kHz frequency
#define PWM_RESOLUTION 8    // 8-bit resolution (0-255)

// IR Pins
#define IR_LEFT 33
#define IR_RIGHT 32
#define IR_BACK 34

// VL53L0X Pins
#define VL53L0X_SDA 21
#define VL53L0X_SCL 22
#define VL53L0X_XSHUT1 19 // For Left ToF sensor
#define VL53L0X_XSHUT2 13 // For Front Left ToF sensor
#define VL53L0X_XSHUT4 26 // For Front Right ToF sensor
#define VL53L0X_XSHUT3 18 // For Right ToF sensor

VL53L0X FLToF;
VL53L0X FRToF;
VL53L0X LToF;
VL53L0X RToF;

bool debug = true;        // Set to true for debugging
#define TURN_SPEED 100    // Speed for turning
#define FORWARD_SPEED 250 // Speed for moving forward
#define SEARCH_SPEED 30   // Speed for searching
#define TURN_DELAY 100    // Delay for turning in milliseconds
#define BACK_SPEED 150    // Speed for moving backward
int SEARCH_RANGE = 500;  // Range to search for opponent in mm
int turnDirection = 0;    // 0: left, 1: right

#define BLE_SEND_DELAY 100

// Non-blocking timing variables
unsigned long lastEdgeAvoidTime = 0;
unsigned long lastTurnTime = 0;
bool isAvoidingEdge = false;

uint16_t FLToF_val, FRToF_val, LToF_val, RToF_val;

bool LIR_val, RIR_val, BIR_val;

// Global variables to add
unsigned long lastIRCheckTime = 0;
const int IR_CHECK_INTERVAL = 5;     // Check IR sensors every 5ms (200Hz)
int edgeAvoidanceSpeed = BACK_SPEED; // Default edge avoidance speed
int edgeAvoidanceTime = 400;         // Increased avoidance time (ms)
bool wasNearEdge = false;            // Track if we were previously near edge

// Variable to keep track if the robot is running or not
bool isRunning = false;

int angle = 90;
// function to return a random angle of 8 or 160 for servo angle
int getRandomServoAngle()
{
  // 8 for left, 160 for right
  return random(0, 2) * 152 + 8; // Returns either 8 or 160
}

struct ToFResult
{
  bool inRange;    // True if any sensor is in range
  bool FL_inRange; // True if front left sensor is in range
  bool FR_inRange; // True if front right sensor is in range
  bool L_inRange;  // True if left sensor is in range
  bool R_inRange;  // True if right sensor is in range
  uint16_t FL;     // Distance from front left sensor
  uint16_t FR;     // Distance from front right sensor
  uint16_t L;      // Distance from left sensor
  uint16_t R;      // Distance from right sensor
};

ToFResult tof;

// Function prototypes
void Motor1(int speed);
void Motor2(int speed);
void motorControl(int leftSpeed, int rightSpeed);
void avoidEdge();
void searchOpponent();
void attackTarget(const ToFResult &tof);

// Initialize ToF sensors
void initSensor(VL53L0X &sensor, uint8_t address, int xshutPin)
{
  pinMode(xshutPin, OUTPUT);
  digitalWrite(xshutPin, LOW);
  delay(100);
  digitalWrite(xshutPin, HIGH);
  delay(100);
  sensor.init();
  sensor.setAddress(address);
  sensor.startContinuous();
  delay(100); // Allow sensor to stabilize
}

ToFResult checkToFSensors(uint16_t rangeLimit)
{
  ToFResult result;

  result.FL = FLToF.readRangeContinuousMillimeters();
  result.FR = FRToF.readRangeContinuousMillimeters();
  result.L = LToF.readRangeContinuousMillimeters();
  result.R = RToF.readRangeContinuousMillimeters();

  result.FL_inRange = result.FL <= rangeLimit;
  result.FR_inRange = result.FR <= rangeLimit;
  result.L_inRange = result.L <= rangeLimit;
  result.R_inRange = result.R <= rangeLimit;

  result.inRange = (result.FL_inRange || result.FR_inRange || result.L_inRange || result.R_inRange);

  return result;
}

void Motor1(int speed)
{
  if (speed > 0)
  {
    digitalWrite(DIR1, HIGH); // Set direction forward
    ledcWrite(PWM1, speed);   // Set speed
  }
  else if (speed < 0)
  {
    digitalWrite(DIR1, LOW); // Set direction backward
    ledcWrite(PWM1, -speed); // Set speed (convert to positive value)
  }
  else
  {
    digitalWrite(DIR1, LOW); // Stop the motor
    ledcWrite(PWM1, 0);      // Set speed to 0
  }
}

void Motor2(int speed)
{
  if (speed > 0)
  {
    digitalWrite(DIR2, LOW); // Set direction forward
    ledcWrite(PWM2, speed);  // Set speed
  }
  else if (speed < 0)
  {
    digitalWrite(DIR2, HIGH); // Set direction backward
    ledcWrite(PWM2, -speed);  // Set speed (convert to positive value)
  }
  else
  {
    digitalWrite(DIR2, LOW); // Stop the motor
    ledcWrite(PWM2, 0);      // Set speed to 0
  }
}

void motorControl(int leftSpeed, int rightSpeed)
{
  Motor1(rightSpeed);
  Motor2(leftSpeed);
}

void avoidEdge() {
  // Fast IR sampling - check IR sensors more frequently than other sensors
  if (millis() - lastIRCheckTime < IR_CHECK_INTERVAL && !isAvoidingEdge)
  {
    return; // Skip if we checked recently and aren't in avoidance mode
  }
  lastIRCheckTime = millis();

  // Read all IR sensors
  LIR_val = digitalRead(IR_LEFT);
  RIR_val = digitalRead(IR_RIGHT);
  BIR_val = digitalRead(IR_BACK);

  // Record previous state for transition detection
  bool wasAvoiding = isAvoidingEdge;

  // Edge avoidance in progress
  if (isAvoidingEdge)
  {
    // Use dynamic timing based on severity
    unsigned long avoidanceTime = (LIR_val == LOW && RIR_val == LOW) ? edgeAvoidanceTime * 1.5 : edgeAvoidanceTime;

    if (millis() - lastEdgeAvoidTime >= avoidanceTime)
    {
      isAvoidingEdge = false;
      motorControl(0, 0); // Brief stop before next action

      // Always turn toward center after edge avoidance
      // If both front sensors detected edge, pick opposite of last turn direction
      if (LIR_val == LOW && RIR_val == LOW)
      {
        turnDirection = !turnDirection; // Switch directions
      }
    }

    // Continue checking sensors during avoidance for adaptive response
    if (LIR_val == LOW || RIR_val == LOW)
    {
      // Still on/near edge during avoidance - extend the avoidance time
      lastEdgeAvoidTime = millis() - (avoidanceTime / 2); // Reset halfway through
    }
    return; // Continue current avoidance maneuver
  }

  // All sensors on black (safe)
  if (LIR_val == HIGH && RIR_val == HIGH && BIR_val == HIGH)
  {
    // Reset wasNearEdge if we're safely in the arena
    wasNearEdge = false;
    return;
  }

  // Start edge avoidance maneuver
  isAvoidingEdge = true;
  lastEdgeAvoidTime = millis();

  // Edge detection with dynamic speed - more aggressive reversal when more sensors detect edge
  int reversePower = BACK_SPEED;
  if (LIR_val == LOW && RIR_val == LOW)
  {
    // Both front sensors on white - critical situation!
    reversePower = BACK_SPEED + 50; // More aggressive backup
    motorControl(-reversePower, -reversePower);
    Serial.println("CRITICAL: Both front sensors detect edge!");
    return;
  }

  // Left sensor on white (left wheel outside)
  if (LIR_val == LOW)
  {
    motorControl(-reversePower, -(reversePower / 2)); // More aggressive turn
    Serial.println("Edge detected on LEFT");
    return;
  }

  // Right sensor on white (right wheel outside)
  if (RIR_val == LOW)
  {
    motorControl(-(reversePower / 2), -reversePower); // More aggressive turn
    Serial.println("Edge detected on RIGHT");
    return;
  }

  // Back sensor on white
  if (BIR_val == LOW)
  {
    // If back sensor is on white and front sensors are on black, move forward
    if (LIR_val == HIGH && RIR_val == HIGH)
    {
      motorControl(FORWARD_SPEED, FORWARD_SPEED);
      Serial.println("Edge detected on BACK only - moving forward");
    }
    else
    {
      // Some front sensor also detects white - emergency stop and reverse direction
      motorControl(-reversePower, -reversePower);
      Serial.println("Edge detected on BACK and FRONT - emergency reverse");
    }
    return;
  }
}

void searchOpponent()
{
  // Rotate in place to search for opponent
  if (turnDirection == 0)
  {
    // Turn left
    motorControl(-TURN_SPEED, TURN_SPEED);
  }
  else
  {
    // Turn right
    motorControl(TURN_SPEED, -TURN_SPEED);
  }
}

void attackTarget(const ToFResult &tof)
{
  if (tof.FL_inRange || tof.FR_inRange)
  {
    // Opponent detected in front, move forward
    motorControl(FORWARD_SPEED, FORWARD_SPEED);
  }
  else if (tof.L_inRange && tof.R_inRange)
  {
    // Opponent detected on both sides means he is opened the flag
    // so run straight forward to hit the opponent
    motorControl(FORWARD_SPEED, FORWARD_SPEED);
  }
  else if (tof.L_inRange)
  {
    // Opponent detected on left, turn left
    turnDirection = 0;
    motorControl(-TURN_SPEED, TURN_SPEED);
  }
  else if (tof.R_inRange)
  {
    // Opponent detected on right, turn right
    turnDirection = 1;
    motorControl(TURN_SPEED, -TURN_SPEED);
  }
}

// Global variables for core syncronization
TaskHandle_t IR_ServoHandler;

void IR_Servo_Task(void *pvParameters)
{
  for (;;) {
    if (IrReceiver.decode()) {
      if (IrReceiver.decodedIRData.command == 69) {
        isRunning = !isRunning; // Toggle running state
      }
      IrReceiver.resume(); // Prepare to receive the next value
    }
    if (isRunning) {
      flagServo.write(angle); // Move flag to the current angle
      if (tof.FL_inRange && tof.FR_inRange)
      {
        // look for the ir signal to stop it
        if (IrReceiver.decode())
        {
          if (IrReceiver.decodedIRData.command == 69) {
            isRunning = !isRunning; // Toggle running state
          }
          IrReceiver.resume(); // Prepare to receive the next value
        }

        // If both front sensors detect opponent, lift the flag
        flagServo.write(90); // Lift flag to 90 degrees
      }
      // if left ToF detects and flag angle is 8 then lift the flag to 90
      if (tof.L_inRange && angle == 8) {
        flagServo.write(90);
        delay(2000);
        angle = getRandomServoAngle(); // Get a new random angle for the next time
      }
      // if right ToF detects and flag angle is 160 then lift the flag to 90
      else if (tof.R_inRange && angle == 160) {
        flagServo.write(90);
        delay(2000);
        angle = getRandomServoAngle(); // Get a new random angle for the next time
      }
    }
    else {
      flagServo.write(90);           // flag will be up
      angle = getRandomServoAngle(); // Get a new random angle for the next time
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  // Initialize serial communication for debugging
  Serial.begin(115200);
  IrReceiver.begin(IR_REMOTE_PIN); // Initialize IR receiver

  angle = getRandomServoAngle(); // Get initial random servo angle

  // FIRST: Allocate specific timers for servo (before any other timer usage)
  ESP32PWM::allocateTimer(2); // Use timer 2 for servo (avoid 0,1 used by motors)
  ESP32PWM::allocateTimer(3); // Use timer 3 as backup

  Wire.begin(VL53L0X_SDA, VL53L0X_SCL);   // Initialize I2C for VL53L0X sensors
  Wire.setClock(400000);                  // Increase I2C speed to 400kHz for faster sensor reading
  flagServo.setPeriodHertz(50);           // Set servo frequency to 50Hz
  flagServo.attach(SERVO_PIN, 500, 2400); // Attach servo to control flag
  flagServo.write(angle);                 // Initialize flag position which should be standing initially

  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_BACK, INPUT);

  // Configure PWM for the motor driver
  ledcAttachChannel(PWM1, PWM_FREQUENCY, PWM_RESOLUTION, 0); // Attach PWM1 to channel 0
  ledcAttachChannel(PWM2, PWM_FREQUENCY, PWM_RESOLUTION, 1); // Attach PWM2 to channel 1

  // Create the task for another core
  xTaskCreatePinnedToCore(
      IR_Servo_Task, // Task function
      "IRServoTask",
      2048,             // Stack size
      NULL,             // Task input parameter
      1,                // Task priority
      &IR_ServoHandler, // Task handle
      0                 // Core ID
  );

  // Initialize ToF sensors
  initSensor(LToF, 0x31, VL53L0X_XSHUT1);
  initSensor(FLToF, 0x30, VL53L0X_XSHUT2);
  initSensor(RToF, 0x33, VL53L0X_XSHUT3);
  initSensor(FRToF, 0x34, VL53L0X_XSHUT4);
  delay(100); // Allow sensors to stabilize
}

void loop()
{
  if (!isRunning)
  {
    motorControl(0, 0); // Stop the motors
    return;             // Exit the loop if robot is not running
  }

  // PRIORITY 1: Edge avoidance (always check first)
  avoidEdge();

  // If we're avoiding edge, don't do anything else
  if (isAvoidingEdge)
  {
    return;
  }

  // PRIORITY 2: Attack or search (only if not avoiding edge)
  tof = checkToFSensors(SEARCH_RANGE);

  if (tof.inRange)
  {
    // Check edge sensors one more time before aggressive attack moves
    // This double-check helps prevent missing the edge when chasing opponents
    LIR_val = digitalRead(IR_LEFT);
    RIR_val = digitalRead(IR_RIGHT);
    if (LIR_val == LOW || RIR_val == LOW)
    {
      avoidEdge(); // Re-run edge avoidance if we detect an edge
      return;
    }

    attackTarget(tof); // Only attack if edge is clear
  }
  else
  {
    searchOpponent();
  }
}


