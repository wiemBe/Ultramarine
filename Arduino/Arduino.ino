#include <Arduino.h>
// #include <PID_v1.h> // Highly recommended: Install PID library by Brett Beauregard

// --- Pin Definitions (MODIFY THESE!) ---
// L298N Motor Driver Pins
const int ENA = 5;  // Left Motor Speed (PWM)
const int IN1 = 7;  // Left Motor Direction 1
const int IN2 = 8;  // Left Motor Direction 2
const int ENB = 6;  // Right Motor Speed (PWM)
const int IN3 = 9;  // Right Motor Direction 1
const int IN4 = 11; // Right Motor Direction 2

// Encoder Pins (Using Interrupts)
const int ENCODER_LEFT_A = 2;  // Interrupt 0 (Pin 2 on Uno/Mega) - MUST be interrupt pin
const int ENCODER_LEFT_B = 4;  // Non-interrupt pin (used for direction)
const int ENCODER_RIGHT_A = 3; // Interrupt 1 (Pin 3 on Uno/Mega) - MUST be interrupt pin
const int ENCODER_RIGHT_B = 12; // Non-interrupt pin (used for direction)

// Ultrasonic Sensor Pins
const int TRIG_PIN = 10;
const int ECHO_PIN = 13;


// --- NEW: Robot Orientation Constants (Matching ESP/Server) ---
const int OrientationNorth = 0;
const int OrientationEast  = 1;
const int OrientationSouth = 2;
const int OrientationWest  = 3;

// --- Calibration Constants (MODIFY THESE!) ---
const double WHEEL_DIAMETER_CM = 6.5;         // Diameter of your robot wheels in cm
const double WHEEL_SEPARATION_CM = 15.0;      // Distance between the centers of the two wheels in cm
const double ENCODER_COUNTS_PER_REV = 400.0; // Pulses/counts from ONE encoder for a full wheel revolution
const double CM_PER_GRID_CELL = 20.0;         // How many cm corresponds to one grid unit (e.g., FORWARD(1))

const double TARGET_DISTANCE_PER_FORWARD = CM_PER_GRID_CELL; // Target distance in CM for a FORWARD(1) command
const double TARGET_RADIANS_PER_TURN = HALF_PI; // Target angle in radians for a 90-degree turn (PI/2)

const double WHEEL_CIRCUMFERENCE_CM = PI * WHEEL_DIAMETER_CM;
const double CM_PER_ENCODER_COUNT = WHEEL_CIRCUMFERENCE_CM / ENCODER_COUNTS_PER_REV;

// --- Movement & Control ---
const int MAX_PWM = 200; // Max motor PWM value (0-255) - Limit for smoother control
const int BASE_SPEED_PWM = 100; // Default speed when moving straight/turning
const unsigned long SENSOR_READ_INTERVAL_MS = 50; // How often to check obstacle sensor (ms)
const double OBSTACLE_THRESHOLD_CM = 15.0; // Stop if obstacle closer than this (cm)

// --- PID Constants (PLACEHOLDER - TUNING REQUIRED!) ---
// double Kp = 2.0, Ki = 0.5, Kd = 1.0;
// You'd typically have separate PID for:
// 1. Maintaining heading while moving forward (adjusting left/right speeds)
// 2. Reaching target distance (controlling overall speed)
// 3. Reaching target angle during turns (controlling differential speed)
// For simplicity here, we'll use basic speed control.

// --- Odometry Variables ---
volatile long encoderLeftCount = 0;
volatile long encoderRightCount = 0;
long lastLeftCount = 0;
long lastRightCount = 0;
double currentX_Grid = 0.0; // Robot position in grid units
double currentY_Grid = 0.0;
double currentTheta_Rad = 0.0; // Robot orientation in radians (0 = East/X+, PI/2 = North/Y+)

// --- State Machine ---
enum RobotState {
    STATE_IDLE,
    STATE_MOVING_FORWARD,
    STATE_TURNING
};
RobotState currentState = STATE_IDLE;

// --- Target State for Movement ---
double targetDistanceCm = 0.0;
double targetThetaRad = 0.0;
double distanceMovedCm = 0.0;
double angleTurnedRad = 0.0;
bool targetReached = false;

// --- Sensor Timing ---
unsigned long lastSensorReadTime = 0;

// --- Serial Communication ---
String incomingEspCmd = "";
bool commandComplete = false;

// --- Function Declarations ---
void handleSerial();
void parseAndExecuteCommand(String cmd);
void startMoveForward(double distanceGridUnits);
void startTurn(bool turnLeft); // true for left, false for right
void updateMovement();
void updateOdometry();
void stopMotors();
void setMotorSpeeds(int leftSpeed, int rightSpeed); // Handles direction based on sign
double readUltrasonicCm();
void reportStatus(const char* status, double x, double y, double theta);
void encoderLeftISR();
void encoderRightISR();


// --- Setup ---
void setup() {
    Serial.begin(115200); // Match ESP baud rate
    Serial.println("Arduino Booting...");

    // Motor Pins
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    stopMotors();

    // Encoder Pins & Interrupts
    pinMode(ENCODER_LEFT_A, INPUT_PULLUP);
    pinMode(ENCODER_LEFT_B, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_A, INPUT_PULLUP);
    pinMode(ENCODER_RIGHT_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), encoderLeftISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), encoderRightISR, RISING);

    // Sensor Pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    // Initialize odometry
    currentX_Grid = 0.0; // Or load from EEPROM if needed
    currentY_Grid = 0.0;
    currentTheta_Rad = 0.0; // Start facing along positive X-axis (East)
    encoderLeftCount = 0;
    encoderRightCount = 0;
    lastLeftCount = 0;
    lastRightCount = 0;


    // PID Setup (if using library)
    // pidDistance = new PID(&distanceInput, &distanceOutput, &distanceSetpoint, Kp_dist, Ki_dist, Kd_dist, DIRECT);
    // pidAngle = new PID(&angleInput, &angleOutput, &angleSetpoint, Kp_angle, Ki_angle, Kd_angle, DIRECT);
    // pidDistance.SetMode(AUTOMATIC);
    // pidAngle.SetMode(AUTOMATIC);

    currentState = STATE_IDLE;
    Serial.println("Arduino Ready.");

    // **IMPORTANT**: Removed sending INIT_POS from setup as ESP now uses fixed start.
    // Arduino now just starts tracking from its internal 0,0,0.
    // The ESP's first reportStatusToServer(StatusInit, FIXED_START_X, ...)
    // establishes the initial link between Arduino's relative tracking and the server's absolute grid.
}

// --- Main Loop ---
void loop() {
    handleSerial(); // Check for commands from ESP

    if (currentState != STATE_IDLE) {
        updateMovement(); // Handle ongoing movement, sensor checks, PID, etc.
    }
}

// --- Serial Communication ---
void handleSerial() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n') {
            // Process complete command
            if (incomingEspCmd.length() > 0) {
                // Only execute if IDLE, otherwise ignore new commands during movement
                if (currentState == STATE_IDLE) {
                    parseAndExecuteCommand(incomingEspCmd);
                } else {
                    Serial.print("Warn: Ignoring command '");
                    Serial.print(incomingEspCmd);
                    Serial.println("' while busy.");
                }
                incomingEspCmd = ""; // Clear buffer
            }
        } else if (c >= 0) {
            incomingEspCmd += c;
            if (incomingEspCmd.length() > 100) { // Prevent buffer overflow
                 Serial.println("ERR: Incoming command too long!");
                 incomingEspCmd = "";
            }
        }
    }
}

void parseAndExecuteCommand(String cmd) {
    Serial.print("Received Command: ");
    Serial.println(cmd);

    if (cmd.startsWith("CMD:")) {
        String actualCmd = cmd.substring(4); // Remove "CMD:" prefix

        if (actualCmd.startsWith("FORWARD")) {
            // Assuming format FORWARD(1) - extract the number if needed later
            startMoveForward(1.0); // Start moving forward 1 grid unit
        } else if (actualCmd.startsWith("TURN_LEFT")) {
            startTurn(true); // Start turning left 90 deg
        } else if (actualCmd.startsWith("TURN_RIGHT")) {
            startTurn(false); // Start turning right 90 deg
        } else {
            Serial.print("ERR: Unknown command received: ");
            Serial.println(actualCmd);
            // Optionally report an error back? Or just ignore.
        }
    } else {
         Serial.print("ERR: Invalid command format: ");
         Serial.println(cmd);
    }
}

// --- Movement Control ---

void startMoveForward(double distanceGridUnits) {
    Serial.print("Starting Move Forward: ");
    Serial.print(distanceGridUnits);
    Serial.println(" grid units");

    // Calculate target distance in CM based on grid units
    targetDistanceCm = distanceGridUnits * TARGET_DISTANCE_PER_FORWARD;
    distanceMovedCm = 0.0; // Reset distance counter
    targetReached = false;

    // Reset encoder diffs for accurate distance tracking for this segment
    noInterrupts(); // Temporarily disable interrupts to read volatile vars safely
    lastLeftCount = encoderLeftCount;
    lastRightCount = encoderRightCount;
    interrupts(); // Re-enable interrupts

    currentState = STATE_MOVING_FORWARD;
    setMotorSpeeds(BASE_SPEED_PWM, BASE_SPEED_PWM); // Start moving forward
}

void startTurn(bool turnLeft) {
    Serial.print("Starting Turn: ");
    Serial.println(turnLeft ? "Left" : "Right");

    // Calculate target angle change - add/subtract from current theta
    if (turnLeft) {
        targetThetaRad = currentTheta_Rad + TARGET_RADIANS_PER_TURN;
    } else {
        targetThetaRad = currentTheta_Rad - TARGET_RADIANS_PER_TURN;
    }
    // Normalize angle if needed (keep within -PI to PI or 0 to 2PI)
    // targetThetaRad = fmod(targetThetaRad + PI, 2.0*PI) - PI; // Example normalization to -PI to PI

    angleTurnedRad = 0.0; // Reset angle counter
    targetReached = false;

    // Reset encoder diffs for accurate angle tracking for this segment
    noInterrupts();
    lastLeftCount = encoderLeftCount;
    lastRightCount = encoderRightCount;
    interrupts();

    currentState = STATE_TURNING;

    // Set motors to turn in place
    if (turnLeft) {
        setMotorSpeeds(-BASE_SPEED_PWM, BASE_SPEED_PWM); // Left reverse, Right forward
    } else {
        setMotorSpeeds(BASE_SPEED_PWM, -BASE_SPEED_PWM); // Left forward, Right reverse
    }
}

// This function is called repeatedly during movement
void updateMovement() {
    // 1. Update Odometry based on encoder counts since last update
    updateOdometry();

    // 2. Check for Obstacles (periodically)
    unsigned long now = millis();
    if (now - lastSensorReadTime >= SENSOR_READ_INTERVAL_MS) {
        lastSensorReadTime = now;
        double distance = readUltrasonicCm();
        // Only check obstacles when moving forward
        if (currentState == STATE_MOVING_FORWARD && distance < OBSTACLE_THRESHOLD_CM && distance > 0) {
            Serial.print("OBSTACLE Detected! Dist: ");
            Serial.println(distance);
            stopMotors();
            reportStatus("OBSTACLE", currentX_Grid, currentY_Grid, currentTheta_Rad);
            currentState = STATE_IDLE; // Stop current action
            return; // Exit update function
        }
    }

    // 3. Update PID and Motor Speeds (Simplified - just checks target)
    if (currentState == STATE_MOVING_FORWARD) {
        // Check if target distance reached
        if (abs(distanceMovedCm) >= abs(targetDistanceCm)) {
            targetReached = true;
        }
        // ** Placeholder for real PID **
        // - Calculate error based on distanceMovedCm vs targetDistanceCm
        // - Calculate error based on difference in left/right counts (to go straight)
        // - Compute PID output
        // - setMotorSpeeds(baseSpeed + correction, baseSpeed - correction);
        if (!targetReached) {
             setMotorSpeeds(BASE_SPEED_PWM, BASE_SPEED_PWM); // Continue moving (simplified)
        }

    } else if (currentState == STATE_TURNING) {
        // Check if target angle reached
        // Need careful handling of angle wrap-around
        double angleDiff = targetThetaRad - currentTheta_Rad;
        // Normalize diff if needed, e.g., handle turn from 350deg to 10deg
        // ... normalization logic ...
        // This simplified check uses the total angle turned
        if (abs(angleTurnedRad) >= abs(TARGET_RADIANS_PER_TURN)) {
            targetReached = true;
        }

        // ** Placeholder for real PID **
        // - Calculate error based on angleTurnedRad vs TARGET_RADIANS_PER_TURN
        // - Compute PID output for differential speed
        // - setMotorSpeeds(-turnSpeed, turnSpeed) or setMotorSpeeds(turnSpeed, -turnSpeed);
        // Keep turning if target not reached (simplified)
    }

    // 4. Check for Completion
    if (targetReached) {
        Serial.println("Target Reached.");
        stopMotors();
        // Report DONE status with final calculated position
        reportStatus("DONE", currentX_Grid, currentY_Grid, currentTheta_Rad);
        currentState = STATE_IDLE; // Finished action
    }
}


// --- Odometry ---
void updateOdometry() {
    long currentLeftCount;
    long currentRightCount;

    // Safely read volatile encoder counts
    noInterrupts();
    currentLeftCount = encoderLeftCount;
    currentRightCount = encoderRightCount;
    interrupts();

    // Calculate change since last update
    long deltaLeft = currentLeftCount - lastLeftCount;
    long deltaRight = currentRightCount - lastRightCount;

    // Convert counts to distance in CM
    double deltaLeftCm = (double)deltaLeft * CM_PER_ENCODER_COUNT;
    double deltaRightCm = (double)deltaRight * CM_PER_ENCODER_COUNT;

    // Calculate average distance moved and change in angle
    double deltaDistanceCm = (deltaLeftCm + deltaRightCm) / 2.0;
    double deltaThetaRad = (deltaRightCm - deltaLeftCm) / WHEEL_SEPARATION_CM;

    // Update position (using midpoint angle approximation)
    currentX_Grid += (deltaDistanceCm / CM_PER_GRID_CELL) * cos(currentTheta_Rad + deltaThetaRad / 2.0);
    currentY_Grid += (deltaDistanceCm / CM_PER_GRID_CELL) * sin(currentTheta_Rad + deltaThetaRad / 2.0);
    currentTheta_Rad += deltaThetaRad;

    // Normalize angle (optional but good practice, e.g., keep between -PI and PI)
    while (currentTheta_Rad > PI) currentTheta_Rad -= TWO_PI;
    while (currentTheta_Rad <= -PI) currentTheta_Rad += TWO_PI;


    // Update cumulative distance/angle for current movement segment
    distanceMovedCm += deltaDistanceCm;
    angleTurnedRad += deltaThetaRad;

    // Store current counts for the next calculation
    lastLeftCount = currentLeftCount;
    lastRightCount = currentRightCount;

     // Debug Odometry Printing (optional - can generate lots of output)
     // Serial.print("Odom: X="); Serial.print(currentX_Grid);
     // Serial.print(" Y="); Serial.print(currentY_Grid);
     // Serial.print(" T="); Serial.print(currentTheta_Rad * RAD_TO_DEG); // Print theta in degrees
     // Serial.print(" Lc="); Serial.print(currentLeftCount);
     // Serial.print(" Rc="); Serial.println(currentRightCount);
}

// --- Motor Control ---
void stopMotors() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

// Sets speeds. Positive = forward, Negative = reverse. Max = +/- 255
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    // Left Motor
    if (leftSpeed > 0) { // Forward
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, min(abs(leftSpeed), 255));
    } else if (leftSpeed < 0) { // Reverse
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(ENA, min(abs(leftSpeed), 255));
    } else { // Stop
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 0);
    }

    // Right Motor
    if (rightSpeed > 0) { // Forward
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, min(abs(rightSpeed), 255));
    } else if (rightSpeed < 0) { // Reverse
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        analogWrite(ENB, min(abs(rightSpeed), 255));
    } else { // Stop
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, 0);
    }
}

// --- Sensors ---
double readUltrasonicCm() {
    // Standard HC-SR04 Ping sequence
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Read echo time in microseconds
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Timeout 30ms

    // Calculate distance in cm (speed of sound ~343 m/s or 0.0343 cm/us)
    // Divide by 2 for round trip
    double distance = (duration * 0.0343) / 2.0;

    if (duration == 0) { // Timeout or error
        return 999.0; // Return a large value indicating no object detected / error
    }
    return distance;
}

// --- Reporting ---
void reportStatus(const char* status, double x, double y, double theta) {
    // Convert radians back to orientation index for reporting (0=N, 1=E, 2=S, 3=W)
    // This depends heavily on how your 0 angle relates to N/E/S/W
    // Assuming 0 rad = East (positive X)
    int thetaIndex = 0;
    double angleDeg = fmod(theta * RAD_TO_DEG + 360.0, 360.0); // Normalize to 0-360 deg
    if (angleDeg >= 45 && angleDeg < 135) thetaIndex = OrientationNorth; // North (Y+) approx 90 deg
    else if (angleDeg >= 135 && angleDeg < 225) thetaIndex = OrientationWest;  // West (X-) approx 180 deg
    else if (angleDeg >= 225 && angleDeg < 315) thetaIndex = OrientationSouth; // South (Y-) approx 270 deg
    else thetaIndex = OrientationEast; // East (X+) approx 0/360 deg

    // Convert double X/Y grid coordinates to nearest integer for reporting
    int reportX = round(x);
    int reportY = round(y);

    Serial.print("STATUS:");
    Serial.print(status);
    Serial.print(":X:");
    Serial.print(reportX);
    Serial.print(":Y:");
    Serial.print(reportY);
    Serial.print(":T:");
    Serial.print(thetaIndex);
    Serial.println(); // Send newline
}

// --- Interrupt Service Routines (ISRs) ---
// Keep ISRs as short and fast as possible!
void encoderLeftISR() {
    // Basic counting - Read B pin to determine direction
    if (digitalRead(ENCODER_LEFT_B) == HIGH) {
        encoderLeftCount++;
    } else {
        encoderLeftCount--;
    }
}

void encoderRightISR() {
    // Basic counting - Read B pin to determine direction
    if (digitalRead(ENCODER_RIGHT_B) == HIGH) {
        encoderRightCount++;
    } else {
        encoderRightCount--;
    }
}