// =====================================================================
// ==  INTEGRATED ROBOT CONTROLLER FOR ARDUINO MEGA                   ==
// ==  Listens to commands from ESP, controls motors, and reports back ==
// =====================================================================

// --- Communication with ESP8266 ---
#define ESP_SERIAL Serial2
const unsigned long ESP_BAUD_RATE = 9600; // MUST MATCH ESP8266 CODE
String espSerialBuffer = "";
bool commandReady = false;
bool isInitialized = false; 
unsigned long lastInitSendTime = 0;

// --- Robot State (Grid Coordinates) ---
struct RobotState {
  int x;
  int y;
  int theta; // 0:North, 1:East, 2:South, 3:West
};
RobotState robotState = {0, 0, 0}; // Initial position at (0,0) facing North

// --- Physical Constants (IMPORTANT: TUNE THESE FOR YOUR ROBOT!) ---
const float GRID_CELL_DISTANCE_MM = 200.0; // <<<--- TUNE: How many mm is one "FORWARD(1)" move?
const float DEGREES_TO_MM_90_TURN = 175.0; // <<<--- TUNE: How many mm must one wheel travel for a 90-degree turn?
const float ENCODER_TICK_TO_MM = 4.45;     // Your 'step_dist'. (mm traveled per encoder tick)

// --- Motor, Encoder, and Sensor Pins ---
const int encoder_left = 2;   // Interrupt Pin
const int encoder_right = 3;  // Interrupt Pin
const int in1 = 6;
const int in2 = 7;
const int in3 = 8;
const int in4 = 9;
const int pwm_left = 5;
const int pwm_right = 10;
#define TRIG_PIN 12
#define ECHO_PIN 11
const float OBSTACLE_DISTANCE_CM = 15.0;

// --- Motor Control & PID Variables ---
volatile long left_counter = 0;
volatile long right_counter = 0;
float Kp = 2.0;
float Kd = 1.0;
int base_pwm = 150;

// --- Status Codes (To send back to ESP) ---
const char* STATUS_INIT = "INIT";
const char* STATUS_DONE = "DONE";
const char* STATUS_OBSTACLE = "OBSTACLE";

// ======================================================
// ==                      SETUP                       ==
// ======================================================
void setup() {
  Serial.begin(115200); // For PC debugging
  while (!Serial);
  Serial.println("\n\n-- Integrated Arduino Mega Robot Controller --");

  ESP_SERIAL.begin(ESP_BAUD_RATE);
  Serial.print("Listening to ESP8266 on Serial2 at ");
  Serial.print(ESP_BAUD_RATE);
  Serial.println(" baud.");

  // Pin Configurations
  pinMode(encoder_left, INPUT_PULLUP);
  pinMode(encoder_right, INPUT_PULLUP);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(pwm_left, OUTPUT); pinMode(pwm_right, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);

  // Attach Interrupts for Encoders
  attachInterrupt(digitalPinToInterrupt(encoder_left), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_right), rightISR, CHANGE);
}

// ======================================================
// ==                   MAIN LOOP                      ==
// ======================================================
void loop() {
  readFromEsp();

  if (!isInitialized) {
    if (millis() - lastInitSendTime > 2000) {
      Serial.println("Handshake: Sending INIT to ESP...");
      send_status_report(STATUS_INIT);
      lastInitSendTime = millis();
    }
  }

  if (commandReady) {
    commandReady = false;
    processCommand(espSerialBuffer);
    espSerialBuffer = "";
  }
}

// ======================================================
// ==            COMMAND & STATUS HANDLING             ==
// ======================================================

void readFromEsp() {
  while (ESP_SERIAL.available() > 0) {
    char c = ESP_SERIAL.read();
    if (c == '\n') { commandReady = true; break; } 
    else { espSerialBuffer += c; }
  }
}

void processCommand(String command) {
  isInitialized = true;
  command.trim();
  Serial.print("Received from ESP: "); Serial.println(command);

  char cmd_type[20];
  int value;
  int parsed = sscanf(command.c_str(), "CMD:%[^(](%d)", cmd_type, &value);

  if (parsed >= 1) {
    String cmdStr = String(cmd_type);
    const char* resultStatus;

    if (cmdStr.equalsIgnoreCase("FORWARD")) {
      resultStatus = go_straight(value > 0 ? value : 1);
    } else if (cmdStr.equalsIgnoreCase("TURN_LEFT")) {
      resultStatus = turn_left();
    } else if (cmdStr.equalsIgnoreCase("TURN_RIGHT")) {
      resultStatus = turn_right();
    } else {
      Serial.print("Unknown command type: "); Serial.println(cmdStr);
      isInitialized = false; // Re-enable handshake if command is bad
      return;
    }
    
    send_status_report(resultStatus);
  } else {
    Serial.print("Failed to parse command: "); Serial.println(command);
    isInitialized = false; // Re-enable handshake if command is bad
  }
}

void send_status_report(const char* status) {
  String report = "STATUS:" + String(status) +
                  ":X:" + String(robotState.x) + ":Y:" + String(robotState.y) +
                  ":T:" + String(robotState.theta) + "\n";
  Serial.print("Sending to ESP: "); Serial.print(report);
  ESP_SERIAL.print(report);
}

// ======================================================
// ==              MOVEMENT FUNCTIONS                  ==
// ======================================================

const char* go_straight(int grid_cells) {
  float target_dist_mm = grid_cells * GRID_CELL_DISTANCE_MM;
  reset_encoders();

  // Set motor direction to FORWARD
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);

  long previous_count_L = 0;
  long previous_count_R = 0;
  
  while (true) {
    if (check_obstacle()) {
      stop_motors();
      Serial.println("!!! OBSTACLE DETECTED !!!");
      return STATUS_OBSTACLE;
    }

    int error = left_counter - right_counter;
    int derivative = (left_counter - previous_count_L) - (right_counter - previous_count_R);
    int correction = Kp * error + Kd * derivative;

    int pwm_L = base_pwm - correction;
    int pwm_R = base_pwm + correction;
    analogWrite(pwm_left, constrain(pwm_L, 0, 255));
    analogWrite(pwm_right, constrain(pwm_R, 0, 255));

    previous_count_L = left_counter;
    previous_count_R = right_counter;
    
    float current_dist_mm = ((left_counter + right_counter) / 2.0) * ENCODER_TICK_TO_MM;
    if (current_dist_mm >= target_dist_mm) {
      break; // Exit the loop when distance is reached
    }
    delay(10); // Small delay to prevent starving other processes
  }

  stop_motors();

  switch(robotState.theta) {
    case 0: robotState.y += grid_cells; break; // North
    case 1: robotState.x += grid_cells; break; // East
    case 2: robotState.y -= grid_cells; break; // South
    case 3: robotState.x -= grid_cells; break; // West
  }
  return STATUS_DONE;
}

const char* turn_right() {
  reset_encoders();
  // Left motor forward, right motor backward
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);

  analogWrite(pwm_left, base_pwm);
  analogWrite(pwm_right, base_pwm);

  while( (left_counter * ENCODER_TICK_TO_MM < DEGREES_TO_MM_90_TURN) && 
         (right_counter * ENCODER_TICK_TO_MM < DEGREES_TO_MM_90_TURN) ) {
    delay(1);
  }
  stop_motors();
  robotState.theta = (robotState.theta + 1) % 4;
  return STATUS_DONE;
}

const char* turn_left() {
  reset_encoders();
  // Left motor backward, right motor forward
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);

  analogWrite(pwm_left, base_pwm);
  analogWrite(pwm_right, base_pwm);
  
  while( (left_counter * ENCODER_TICK_TO_MM < DEGREES_TO_MM_90_TURN) && 
         (right_counter * ENCODER_TICK_TO_MM < DEGREES_TO_MM_90_TURN) ) {
    delay(1);
  }
  stop_motors();
  robotState.theta = (robotState.theta + 3) % 4; // Wraps around from 0 to 3
  return STATUS_DONE;
}

// ======================================================
// ==              UTILITY FUNCTIONS                   ==
// ======================================================
void leftISR() { left_counter++; }
void rightISR() { right_counter++; }

void reset_encoders() {
  stop_motors();
  delay(50);
  noInterrupts();
  left_counter = 0;
  right_counter = 0;
  interrupts();
}

void stop_motors() {
  analogWrite(pwm_left, 0);
  analogWrite(pwm_right, 0);
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}

bool check_obstacle() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 25000); // 25ms timeout
  float distance_cm = (duration / 2.0) * 0.0343;
  return (distance_cm > 0 && distance_cm < OBSTACLE_DISTANCE_CM);
}