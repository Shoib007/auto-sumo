# Autonomous Sumo Robot - AI Coding Guidelines

## Project Overview
This is an ESP32-based autonomous sumo robot using PlatformIO. The robot combines distance sensing (VL53L0X ToF), edge detection (IR sensors), motor control (Rhino driver), and Bluetooth communication for remote control and debugging.

## Architecture & Key Components

### Hardware Abstraction Layer
- **Motor Control**: Uses `Motor1()`, `Motor2()`, and `motorControl(leftSpeed, rightSpeed)` for differential drive
- **Sensor Management**: Four VL53L0X ToF sensors (FL, FR, L, R) with unique I2C addresses via XSHUT pins
- **Edge Detection**: Three IR sensors (LEFT, RIGHT, BACK) for ring boundary detection

### Core State Machine
The main loop follows this priority-based behavior:
1. **Bluetooth Commands**: Check for "1" (start) / "0" (stop) commands
2. **Edge Avoidance**: `avoidEdge()` - highest priority, uses IR sensors to stay in ring
3. **Target Acquisition**: `attackTarget()` if ToF sensors detect opponent
4. **Search Pattern**: `searchOpponent()` rotates to find targets when none detected

### Critical Patterns

#### Sensor Initialization Sequence
```cpp
// VL53L0X sensors require specific initialization with XSHUT pin manipulation
initSensor(FLToF, 0x31, VL53L0X_XSHUT1);  // Address assignment via XSHUT
```

#### ToF Data Structure
```cpp
struct ToFResult {
    bool inRange;           // Any sensor detected target
    bool FL_inRange, FR_inRange, L_inRange, R_inRange;  // Per-sensor flags
    uint16_t FL, FR, L, R;  // Raw distance values
};
```

#### Motor Direction Convention
- **Motor1**: Positive = forward, uses `DIR1` HIGH for forward
- **Motor2**: Positive = forward, uses `DIR2` LOW for forward (reversed wiring)
- **Turning**: `motorControl(-TURN_SPEED, TURN_SPEED)` = left turn

#### Edge Detection Logic
IR sensors read HIGH on black (safe), LOW on white (danger):
- Both front sensors LOW = reverse straight
- Single front sensor LOW = reverse away from edge
- Back sensor LOW + front sensor(s) LOW = emergency reverse

## Development Workflows

### Building & Uploading
```bash
# Use PlatformIO CLI or VS Code PlatformIO extension
pio run                    # Build
pio run --target upload    # Upload to ESP32
pio device monitor         # Serial monitor (115200 baud)
```

### Debugging Approaches
- **Serial Monitor**: Primary debug output at 115200 baud
- **Bluetooth Debug**: JSON telemetry via `sendDatatoBluetooth()` (currently commented out)
- **Remote Control**: Send "1"/"0" commands via Bluetooth to start/stop robot

### Testing Patterns
- **Sensor Testing**: Uncomment the debug loop at bottom of `main.cpp` for IR sensor values
- **Motor Testing**: Use `motorControl()` with known speeds to verify direction/wiring
- **ToF Calibration**: Adjust `SEARCH_RANGE` (default 500mm) for opponent detection sensitivity

## Project-Specific Conventions

### Pin Definitions
All hardware pins are `#define`d at the top - modify here for hardware changes:
- VL53L0X: Custom I2C pins (SDA=21, SCL=22) + 4 XSHUT pins for addressing
- Motors: PWM + DIR pins with ESP32 LEDC for PWM generation
- IR sensors: Digital input pins (25, 4, 34)

### Speed Constants
Robot behavior tuned via speed constants:
- `FORWARD_SPEED=200`, `TURN_SPEED=100`, `BACK_SPEED=150`
- All speeds are 0-255 PWM values (8-bit resolution)

### Communication Protocol
- Bluetooth device name: "SumoBot"
- Command protocol: Single character commands ("1"=start, "0"=stop)
- Debug JSON: `{"FL": bool, "FR": bool, "L": bool, "R": bool, "IR_R": bool, "IR_L": bool, "IR_B": bool}`

## Integration Points
- **ArduinoJson**: Used for Bluetooth telemetry serialization
- **VL53L0X Library**: Continuous ranging mode for all ToF sensors
- **ESP32 LEDC**: Hardware PWM for smooth motor control
- **Wire Library**: I2C communication with custom SDA/SCL pins

When modifying sensor logic, always consider the priority hierarchy: edge avoidance overrides everything, then attack, then search.
