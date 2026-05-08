#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <ArduinoJson.h>
// #include <ESP32Servo.h> // Include ESP32Servo for servo control
// #include <IRremote.h>   // Include IRremote for IR remote control

// // IR Remote Pin
// #define IR_REMOTE_PIN 4 // Pin for IR remote control

// Servo Pins
// #define SERVO_PIN 23
// Servo flagServo; // Servo for flag control

// RF Remote pins
#define RF_ON_PIN 4        // Pin for RF remote control (same as IR remote pin for simplicity)
#define RF_OFF_PIN 23      // Pin for RF remote control (not used in this code, but can be added for future functionality)
#define ROBOT_STATUS_LED 5 // LED to indicate robot status (on/off)

// Rhino Motor Driver Pins
#define DIR1 14
#define PWM1 15 // 14 on New robot
#define DIR2 12 // 27 on New robot
#define PWM2 27

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

bool debug = true;    // Set to true for debugging
#define TURN_SPEED 30 // Speed for turning
// #define FORWARD_SPEED 250 // Speed for moving forward
#define SEARCH_SPEED 30  // Speed for searching
#define TURN_DELAY 100   // Delay for turning in milliseconds
#define BACK_SPEED 150   // Speed for moving backward
int SEARCH_RANGE = 1500; // Range to search for opponent in mm
int turnDirection = 0;   // 0: left, 1: right
int LEFT_MOTOR_SPEED = 255;
int RIGHT_MOTOR_SPEED = 255;
bool isRushing = true;
unsigned long rushLimit = 500; // Time limit for rushing (ms)

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
bool isRunning = false; // Start in running state, can be toggled with IR remote

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

// Initialize ToF sensors
void initSensor(VL53L0X &sensor, uint8_t address, int xshutPin)
{
  // Assumes other XSHUTs are held LOW before calling this function.
  digitalWrite(xshutPin, HIGH); // bring this sensor out of reset
  delay(50);                    // allow boot
  Serial.print("Initializing sensor at xshut pin ");
  Serial.println(xshutPin);

  // Attempt init with a timeout to avoid blocking forever
  unsigned long start = millis();
  bool ok = false;
  while (millis() - start < 1000)
  { // 1 second timeout
    if (sensor.init())
    { // many VL53L0X libs return bool; if yours doesn't, this still calls init once
      ok = true;
      break;
    }
    delay(50);
  }

  if (!ok)
  {
    Serial.print("ERROR: sensor.init() timed out at xshut pin ");
    Serial.println(xshutPin);
    return;
  }

  Serial.print("Initialized Sensor at xshut pin ");
  Serial.println(xshutPin);
  sensor.setAddress(address);
  sensor.startContinuous();
  delay(20); // allow continuous mode to start
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
    digitalWrite(DIR1, LOW); // Set direction forward
    ledcWrite(PWM1, speed);   // Set speed
  }
  else if (speed < 0)
  {
    digitalWrite(DIR1, HIGH); // Set direction backward
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
    digitalWrite(DIR2, HIGH); // Set direction forward
    ledcWrite(PWM2, speed);  // Set speed
  }
  else if (speed < 0)
  {
    digitalWrite(DIR2, LOW); // Set direction backward
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

void avoidEdge()
{
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
      motorControl(LEFT_MOTOR_SPEED, RIGHT_MOTOR_SPEED);
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
  Serial.println("Searching for opponent...");
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
  const int BIAS = 100;
  int leftSpeed = LEFT_MOTOR_SPEED;
  int rightSpeed = RIGHT_MOTOR_SPEED;

  if (tof.FL_inRange || tof.FR_inRange)
  {
    if (turnDirection == 0)
    {
      // bias to left: slow left, keep right full
      leftSpeed = (LEFT_MOTOR_SPEED > BIAS) ? (LEFT_MOTOR_SPEED - BIAS) : 0;
      rightSpeed = RIGHT_MOTOR_SPEED;
      Serial.println("Attacking target - biasing left");
    }
    else
    {
      // bias to right: slow right, keep left full
      leftSpeed = LEFT_MOTOR_SPEED;
      rightSpeed = (RIGHT_MOTOR_SPEED > BIAS) ? (RIGHT_MOTOR_SPEED - BIAS) : 0;
      Serial.println("Attacking target - biasing right");
    }
    motorControl(leftSpeed, rightSpeed);
  }
  else if (tof.L_inRange && tof.R_inRange)
  {
    // opponent centered — go straight
    motorControl(LEFT_MOTOR_SPEED, RIGHT_MOTOR_SPEED);
    Serial.println("Attacking target - centered");
  }
  else if (tof.L_inRange)
  {
    // opponent on left: turn in place left to reacquire
    turnDirection = 0;
    motorControl(-TURN_SPEED, TURN_SPEED);
    Serial.println("Attacking target - turning left");
  }
  else if (tof.R_inRange)
  {
    // opponent on right: turn in place right to reacquire
    turnDirection = 1;
    motorControl(TURN_SPEED, -TURN_SPEED);
    Serial.println("Attacking target - turning right");
  }
}

// Global variables for core syncronization
TaskHandle_t IR_ServoHandler;

void IR_Servo_Task(void *pvParameters)
{
  for (;;)
  {
    // if (IrReceiver.decode()) {
    //   if (IrReceiver.decodedIRData.command == 69) {
    //     isRunning = !isRunning; // Toggle running state
    //   }
    //   IrReceiver.resume(); // Prepare to receive the next value
    // }
    if (isRunning)
    {
      // flagServo.write(angle); // Move flag to the current angle
      if (tof.FL_inRange || tof.FR_inRange)
      {
        // If both front sensors detect opponent, lift the flag
        // flagServo.write(90); // Lift flag to 90 degrees
      }
      // if left ToF detects and flag angle is 8 then lift the flag to 90
      if (tof.L_inRange && angle == 8)
      {
        // flagServo.write(90);
        delay(2000);
        angle = getRandomServoAngle(); // Get a new random angle for the next time
      }
      // if right ToF detects and flag angle is 160 then lift the flag to 90
      else if (tof.R_inRange && angle == 160)
      {
        // flagServo.write(90);
        delay(2000);
        angle = getRandomServoAngle(); // Get a new random angle for the next time
      }
    }
    else
    {
      // flagServo.write(90);           // flag will be up
      angle = getRandomServoAngle(); // Get a new random angle for the next time
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  // Initialize serial communication for debugging
  Serial.begin(115200);
  // IrReceiver.begin(IR_REMOTE_PIN); // Initialize IR receiver

  // angle = getRandomServoAngle(); // Get initial random servo angle

  // FIRST: Allocate specific timers for servo (before any other timer usage)
  // ESP32PWM::allocateTimer(2); // Use timer 2 for servo (avoid 0,1 used by motors)
  // ESP32PWM::allocateTimer(3); // Use timer 3 as backup

  Wire.begin(VL53L0X_SDA, VL53L0X_SCL); // Initialize I2C for VL53L0X sensors
  Wire.setClock(400000);                // Increase I2C speed to 400kHz for faster sensor reading

  // --- Ensure all VL53L0X XSHUT lines are held LOW first (reset all sensors) ---
  pinMode(VL53L0X_XSHUT1, OUTPUT);
  pinMode(VL53L0X_XSHUT2, OUTPUT);
  pinMode(VL53L0X_XSHUT4, OUTPUT);
  pinMode(VL53L0X_XSHUT3, OUTPUT);

  digitalWrite(VL53L0X_XSHUT1, LOW);
  digitalWrite(VL53L0X_XSHUT2, LOW);
  digitalWrite(VL53L0X_XSHUT4, LOW);
  digitalWrite(VL53L0X_XSHUT3, LOW);
  delay(20); // all sensors in reset

  // Initialize each sensor
  initSensor(LToF, 0x31, VL53L0X_XSHUT1);
  initSensor(FLToF, 0x30, VL53L0X_XSHUT2);
  initSensor(RToF, 0x33, VL53L0X_XSHUT3);
  initSensor(FRToF, 0x34, VL53L0X_XSHUT4);
  delay(100); // Allow sensors to stabilize

  // flagServo.setPeriodHertz(50);           // Set servo frequency to 50Hz
  // flagServo.attach(SERVO_PIN, 500, 2400); // Attach servo to control flag
  // flagServo.write(angle);                 // Initialize flag position which should be standing initially

  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);

  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_BACK, INPUT);

  // RF remote pins (same as IR for simplicity)
  pinMode(RF_ON_PIN, INPUT);
  pinMode(RF_OFF_PIN, INPUT);
  pinMode(ROBOT_STATUS_LED, OUTPUT);
  digitalWrite(ROBOT_STATUS_LED, LOW); // Turn OFF status LED to indicate robot

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
}

void loop()
{

  if (digitalRead(RF_ON_PIN) == HIGH)
    isRunning = true; // Start the robot if RF ON signal is received
  if (digitalRead(RF_OFF_PIN) == HIGH)
    isRunning = false;                                    // Stop the robot if RF OFF signal is received
  digitalWrite(ROBOT_STATUS_LED, isRunning ? HIGH : LOW); // Update status LED based on running state

  if (!isRunning)
  {
    motorControl(0, 0); // Stop the motors
    return;             // Exit the loop if robot is not running
  }

  static unsigned long startRushing = millis();

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
    // rush for 500ms using millis() and make rush = false
    // use rushLimit variable
    // this should be asyncronous so that it does'nt block other function
    if (isRushing)
    {
      if (millis() - startRushing < rushLimit)
      {
        motorControl(LEFT_MOTOR_SPEED, RIGHT_MOTOR_SPEED);
      }
      else
      {
        isRushing = false; // Stop rushing after the limit
      }
    }
    else
    {
      searchOpponent();
    }
  }
}


