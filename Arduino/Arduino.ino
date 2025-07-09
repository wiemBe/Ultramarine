// ======================================================================
// ==  INTEGRATED ROBOT CONTROLLER FOR ARDUINO UNO                    ==
// ==  Listens to commands from ESP, controls motors, and reports back ==
// ======================================================================

// --- UNO-SPECIFIC CHANGES: Include and configure SoftwareSerial ---
#include <SoftwareSerial.h>

// We will create a virtual serial port on pins 4 and 7 for the ESP
// You can change these pins if needed, as long as they are free.
const byte ESP_RX_PIN = 4; // Uno RX pin, connect to ESP TX pin (via level shifter)
const byte ESP_TX_PIN = 7; // Uno TX pin, connect to ESP RX pin (via level shifter)
SoftwareSerial espSerial(ESP_RX_PIN, ESP_TX_PIN);

// --- Communication with ESP8266 ---
#define ESP_SERIAL espSerial // Use our new software serial port
const unsigned long ESP_BAUD_RATE = 9600; // MUST MATCH ESP8266 CODE
String espSerialBuffer = "";
bool commandReady = false;
bool isInitialized = false; 
unsigned long lastInitSendTime = 0;
bool firstMovementDone = false;

// --- Robot State (Grid Coordinates) ---
struct RobotState {
  int x;
  int y;
  int theta; // 0:North, 1:East, 2:South, 3:West
};
RobotState robotState = {2, 20, 1};

// --- Physical Constants (IMPORTANT: TUNE THESE FOR YOUR ROBOT!) ---
const float GRID_CELL_DISTANCE_MM = 200.0;
const float DEGREES_TO_MM_90_TURN = 262.5; // 262.5
const float ENCODER_TICK_TO_MM = 6.2;  //6.2

// --- Pin Assignments (All are valid on Uno) ---
const int encoder_left = 2;   // Interrupt Pin 0 on Uno
const int encoder_right = 3;  // Interrupt Pin 1 on Uno
const int in1 = 8;
const int in2 = 6; // Changed from 7 to avoid conflict with new ESP_TX_PIN
const int in3 = 10;
const int in4 = 9;
const int pwm_left = 5;
const int pwm_right = 11;
#define TRIG_PIN 12
#define ECHO_PIN 13
const float OBSTACLE_DISTANCE_CM = 15.0;

// --- Motor Control & PID Variables ---
volatile long left_counter = 0;
volatile long right_counter = 0;
float Kp = 11.335;
//önceki 11.25 sağ sapma
float Kd = 0.11;
int base_pwm = 150;

long  left_s_counter = 0; 
long  right_s_counter = 0; 
int startup;

// --- Status Codes (To send back to ESP) ---
const char* STATUS_INIT = "INIT";
const char* STATUS_DONE = "DONE";
const char* STATUS_OBSTACLE = "OBSTACLE";


// ======================================================
// ==                      SETUP                       ==
// ======================================================
void setup() {
  Serial.begin(115200); 
  while (!Serial);
  Serial.println("\n\n-- Integrated Arduino UNO Robot Controller --");

  // Start the SoftwareSerial port for the ESP
  ESP_SERIAL.begin(ESP_BAUD_RATE);
  Serial.print("Listening to ESP8266 on SoftwareSerial at ");
  Serial.print(ESP_BAUD_RATE);
  Serial.println(" baud.");

  // Pin Configurations
  pinMode(encoder_left, INPUT_PULLUP);
  pinMode(encoder_right, INPUT_PULLUP);
  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(pwm_left, OUTPUT); pinMode(pwm_right, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);

  // Attach Interrupts (Pins 2 and 3 are the only interrupt pins on the Uno)
  attachInterrupt(digitalPinToInterrupt(encoder_left), leftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_right), rightISR, CHANGE);
  startup = 1;
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


//
// THE REST OF THE CODE (command handling, movement functions, utilities)
// IS IDENTICAL TO THE MEGA VERSION AND IS OMITTED HERE FOR BREVITY.
// COPY AND PASTE ALL THE FUNCTIONS FROM YOUR MEGA CODE FROM THIS POINT DOWN.
//
// void readFromEsp() { ... }
// void processCommand(String command) { ... }
// void send_status_report(const char* status) { ... }
// const char* go_straight(int grid_cells) { ... }
// const char* turn_right() { ... }
// const char* turn_left() { ... }
// void leftISR() { ... }
// void rightISR() { ... }
// void reset_encoders() { ... }
// void stop_motors() { ... }
// bool check_obstacle() { ... }
//

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
      if(!firstMovementDone){
        startup = 1;
      }else{
        startup = 0;
      }
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

    if(String(resultStatus) != STATUS_OBSTACLE){
      firstMovementDone = true;
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

  unsigned long prev_mil = 0;
  unsigned long cur_mil = 0;
  unsigned long movementStartTime = millis();

  while (true) {
    if (check_obstacle()) {
      stop_motors();
      Serial.println("!!! OBSTACLE DETECTED !!!");
      return STATUS_OBSTACLE;
    }
    if (startup == 1 && (millis() - movementStartTime < 500)) {
   // ilk saniye düz git
    analogWrite(pwm_left, 170);
    analogWrite(pwm_right, 200);
    }else {
    startup = 0;
    int cur_mil = millis();
    float delta_time = (cur_mil  - prev_mil) / 1000;
    int error = (left_s_counter - right_s_counter) / 2;
    int derivative = (right_s_counter - previous_count_R) / delta_time;
    int correction = Kp * error + Kd * derivative;
    prev_mil = cur_mil;

    int pwm_L = base_pwm;
    int pwm_R = (base_pwm + correction);
    pwm_L = constrain(pwm_L, 0, 255);
    pwm_R = constrain(pwm_R, 0, 255);
    analogWrite(pwm_left, constrain(pwm_L, 0, 255));
    analogWrite(pwm_right, constrain(pwm_R, 0, 255));
    /*Serial.print(left_s_counter);
    Serial.print(" ");
    Serial.print(right_s_counter);
    Serial.print(" ");
    Serial.print(error);
    Serial.print(" ");
    Serial.print(pwm_L);
    Serial.print(" ");
    Serial.print(pwm_R);
    Serial.println("");*/

    previous_count_L = left_s_counter;
    previous_count_R = right_s_counter;
  }
    
    float current_dist_mm = ((left_counter + right_counter) / 2.0) * ENCODER_TICK_TO_MM;
    if (current_dist_mm >= target_dist_mm) {
      break; // Exit the loop when distance is reached
    }
    delay(10); // Small delay to prevent starving other processes
  }

  stop_motors();

  switch(robotState.theta) {
    case 0: robotState.y -= grid_cells; break; // North
    case 1: robotState.x += grid_cells; break; // East
    case 2: robotState.y += grid_cells; break; // South
    case 3: robotState.x -= grid_cells; break; // West
  }
  return STATUS_DONE;
}

const char* turn_right() {
  firstMovementDone = true;
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
  firstMovementDone = true;
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
void leftISR() {
   left_counter++;
  if(left_s_counter > 6000) left_s_counter = 0;
   left_s_counter++; 
  }
void rightISR() { 
  right_counter++;
  if(right_s_counter > 6000) right_s_counter = 0;
  right_s_counter++;
}

void reset_encoders() {
  stop_motors();
  delay(50);
  noInterrupts();
  left_counter = 0;
  right_counter = 0;
  interrupts();
}

void stop_motors() {
  digitalWrite(pwm_left, LOW);
  digitalWrite(pwm_right, LOW);
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