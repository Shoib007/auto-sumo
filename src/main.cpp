#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <BluetoothSerial.h>
#include <ArduinoJson.h>

// Bluetooth Serial
BluetoothSerial SerialBT;

// Rhino Motor Driver Pins
#define DIR1 12
#define PWM1 13
#define DIR2 14
#define PWM2 15
#define SLEEP1 26
#define SLEEP2 27

// Channel for PWM
#define PWM_FREQUENCY 10000 // 10 kHz frequency
#define PWM_RESOLUTION 8    // 8-bit resolution (0-255)

// IR Pins
#define IR_LEFT 25
#define IR_RIGHT 4
#define IR_BACK 34

// VL53L0X Pins
#define VL53L0X_SDA 21
#define VL53L0X_SCL 22
#define VL53L0X_XSHUT1 32
#define VL53L0X_XSHUT2 33
#define VL53L0X_XSHUT4 19
#define VL53L0X_XSHUT3 18

VL53L0X FLToF;
VL53L0X FRToF;
VL53L0X LToF;
VL53L0X RToF;

bool debug = true;        // Set to true for debugging
#define TURN_SPEED 100    // Speed for turning
#define FORWARD_SPEED 255 // Speed for moving forward
#define SEARCH_SPEED 50
#define TURN_DELAY 150
#define BACK_SPEED 150 // Speed for moving backward
int SEARCH_RANGE = 500; // Range to search for opponent in mm

#define BLE_SEND_DELAY 100

// Millies to check delay to send bluetooth data
unsigned long sendTime = millis();

uint16_t FLToF_val, FRToF_val, LToF_val, RToF_val;

bool LIR_val, RIR_val, BIR_val;

// Variable to keep track if the robot is running or not
bool isRunning = false;

struct ToFResult
{
  bool inRange;
  bool FL_inRange;
  bool FR_inRange;
  bool L_inRange;
  bool R_inRange;
  uint16_t FL;
  uint16_t FR;
  uint16_t L;
  uint16_t R;
};

// Function prototypes
void Motor1(int speed);
void Motor2(int speed);
void motorControl(int leftSpeed, int rightSpeed);
void avoidEdge();
void searchOpponent();
void attackTarget(const ToFResult &tof);

// Initialize ToF sensors
void initSensor(VL53L0X &sensor, uint8_t address, int xshutPin) {
  pinMode(xshutPin, OUTPUT);
  digitalWrite(xshutPin, LOW);
  delay(10);
  digitalWrite(xshutPin, HIGH);
  delay(10);
  sensor.init();
  sensor.setAddress(address);
  sensor.startContinuous();
  delay(10); // Allow sensor to stabilize
}

// Function to send debug values to Bluetooth
void sendDatatoBluetooth(const ToFResult &tof) {
  JsonDocument doc;
  doc["FL"] = tof.FL_inRange;
  doc["FR"] = tof.FR_inRange;
  doc["L"] = tof.L_inRange;
  doc["R"] = tof.R_inRange;
  doc["IR_R"] = RIR_val;
  doc["IR_L"] = LIR_val;
  doc["IR_B"] = BIR_val;
  serializeJson(doc, SerialBT); // Send JSON data over Bluetooth
  SerialBT.println();
}

ToFResult checkToFSensors(uint16_t rangeLimit) {
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

void Motor1(int speed) {
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

void Motor2(int speed) {
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

void motorControl(int leftSpeed, int rightSpeed) {
  Motor1(leftSpeed);
  Motor2(rightSpeed);
}

void avoidEdge() {
  LIR_val = digitalRead(IR_LEFT);
  RIR_val = digitalRead(IR_RIGHT);
  BIR_val = digitalRead(IR_BACK);

  // All sensors on black (safe)
  if (LIR_val == HIGH && RIR_val == HIGH && BIR_val == HIGH) {
    return;
  }

  // Both front sensors on white (danger: fully outside)
  if (LIR_val == LOW && RIR_val == LOW) {
    motorControl(-FORWARD_SPEED, -FORWARD_SPEED); // Reverse straight
    delay(200);
    return;
  }

  // Left sensor on white (left wheel outside)
  if (LIR_val == LOW) {
    motorControl(-FORWARD_SPEED, -100); // Reverse right
    delay(200);
    return;
  }

  // Right sensor on white (right wheel outside)
  if (RIR_val == LOW) {
    motorControl(-100, -FORWARD_SPEED); // Reverse left
    delay(200);
    return;
  }

  // Back sensor on white and at least one front sensor on white (almost out)
  if (BIR_val == LOW && (LIR_val == LOW || RIR_val == LOW)) {
    motorControl(-BACK_SPEED, -BACK_SPEED); // Reverse
    delay(200);
    return;
  }

  // Only back sensor on white (back is out, front is in)
  if (BIR_val == LOW && LIR_val == HIGH && RIR_val == HIGH) {
    motorControl(FORWARD_SPEED, FORWARD_SPEED); // Move forward to get back in
    delay(200);
    return;

  }
}

void searchOpponent() {
  // Rotate in place to search for opponent
  Serial.println("Searching for opponent...");
  motorControl(SEARCH_SPEED, -SEARCH_SPEED);
}

void attackTarget(const ToFResult &tof) {
  if (tof.FL_inRange || tof.FR_inRange) {
    // Opponent detected in front, move forward
    motorControl(FORWARD_SPEED, FORWARD_SPEED);
  } else if (tof.L_inRange) {
    // Opponent detected on left, turn left
    motorControl(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DELAY);
  } else if (tof.R_inRange) {
    // Opponent detected on right, turn right
    motorControl(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DELAY);
  }
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  while (!Serial)
  {
    ; // Wait for serial port to connect. Needed for native USB
  }

  Wire.begin(VL53L0X_SDA, VL53L0X_SCL); // Initialize I2C for VL53L0X sensors
  SerialBT.begin("SumoBot");            // Start Bluetooth with device name "SumoBot"

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

  // Initialize ToF sensors
  initSensor(FLToF, 0x31, VL53L0X_XSHUT1);
  initSensor(FRToF, 0x30, VL53L0X_XSHUT2);
  initSensor(LToF, 0x33, VL53L0X_XSHUT3);
  initSensor(RToF, 0x34, VL53L0X_XSHUT4);

}




void loop() {
  // Check if Bluetooth is connected
  if(SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    command.trim(); // Remove any leading/trailing whitespace
    if(command == "1") {
      isRunning = true; // Start the robot
      SerialBT.println("Robot started");
    } else if(command == "0") {
      isRunning = false;
      SerialBT.println("Robot stopped");
    }
  }

    if(!isRunning) {
      motorControl(0, 0); // Stop the motors
      return; // Exit the loop if robot is not running
    }

  avoidEdge();

  // check all ToF sensors
  ToFResult tof = checkToFSensors(SEARCH_RANGE);
  
  // Attach target if any sensor detects an opponent
  if(tof.inRange) {
    attackTarget(tof); // Attack if any sensor detects an opponent
  }else {
    searchOpponent(); // Search for opponent if not detected
  }

  // Send ToF data to Bluetooth
  if (millis() - sendTime >= BLE_SEND_DELAY) {
    sendDatatoBluetooth(tof);
    sendTime = millis(); // Update the last send time
  }
  
}


// void loop() {
//   // Test all three IR sensors
//   LIR_val = digitalRead(IR_LEFT);
//   RIR_val = digitalRead(IR_RIGHT);
//   BIR_val = digitalRead(IR_BACK);
//   Serial.print("Left IR: ");
//   Serial.print(LIR_val);
//   Serial.print(" | Right IR: ");
//   Serial.print(RIR_val);
//   Serial.print(" | Back IR: ");
//   Serial.println(BIR_val);
// }

