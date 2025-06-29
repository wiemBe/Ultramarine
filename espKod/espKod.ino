#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h> // Use v6+
// #include <SoftwareSerial.h> // Include if using SoftwareSerial for Arduino comms

// --- Configuration ---
const char* ssid = "esptel";        // <<<--- REPLACE
const char* password = "kdjn5860";  // <<<--- REPLACE

// IMPORTANT: Replace with the actual IP address of your Go server
const char* serverIp = "192.168.135.239"; // <<<--- REPLACE (Example IP)
const uint16_t serverPort = 8080;

const unsigned long WIFI_CONNECT_TIMEOUT = 30000; // 30 seconds
const unsigned long HTTP_TIMEOUT_MS = 10000;      // HTTP request timeout (10 seconds)
const unsigned long IDLE_CHECK_INTERVAL_MS = 5000; // Check for tasks every 5s when idle
const unsigned long ARDUINO_RESPONSE_TIMEOUT_MS = 7000; // Max time to wait for Arduino status after sending command (7 seconds)
const unsigned long GET_CMD_RETRY_DELAY_MS = 2000; // Delay before retrying get command if server says wait
const unsigned long REPORT_RETRY_DELAY_MS = 3000; // Delay before retrying report status if HTTP fails

// --- Arduino Serial Communication ---
// Choose ONE option and ensure it matches your hardware connection:
// Option 1: Hardware Serial1 (NodeMCU V3 TX=GPIO2(D4)) - Recommended if available & not conflicting
// #define ARDUINO_SERIAL Serial1

// Option 2: Software Serial
#include <SoftwareSerial.h>
const byte ARDUINO_RX_PIN = D5; // GPIO14
const byte ARDUINO_TX_PIN = D6; // GPIO12
SoftwareSerial ArduinoSerial(ARDUINO_RX_PIN, ARDUINO_TX_PIN);
#define ARDUINO_SERIAL ArduinoSerial

// Option 3: Default Hardware Serial (Serial) - Shared with USB Monitor! Use with caution.
//#define ARDUINO_SERIAL Serial
const unsigned long ARDUINO_BAUD_RATE = 9600;

// --- Robot Command/Status Constants (Mirrored from Go Server) ---
// Server Commands (Received)
const char* CmdForward     = "FORWARD(1)";
const char* CmdTurnLeft    = "TURN_LEFT(90)";
const char* CmdTurnRight   = "TURN_RIGHT(90)";
const char* CmdNone        = "NO_COMMAND";
const char* CmdComplete    = "PATH_COMPLETE";
const char* CmdError       = "ERROR";
// Arduino Statuses (Received from Arduino / Sent to Server)
const char* StatusInit     = "INIT";
const char* StatusDone     = "DONE";
const char* StatusObstacle = "OBSTACLE";
const char* StatusTimeout  = "TIMEOUT"; // Internal status if Arduino doesn't respond

// --- Robot Orientation Constants ---
const int OrientationNorth = 0;
const int OrientationEast  = 1;
const int OrientationSouth = 2;
const int OrientationWest  = 3;

// --- Robot State Machine ---
enum RobotState {
    STATE_BOOTING,              // Initial state, waiting for Arduino INIT_POS
    STATE_IDLE,                 // Connected, waiting for task assignment
    STATE_INITIALIZING_TASK,    // Task assigned, reporting initial pos to server
    STATE_GETTING_COMMAND,      // Ready for next navigation command from server
    STATE_SENDING_TO_ARDUINO,   // Sending received command to Arduino
    STATE_WAITING_FOR_ARDUINO,  // Waiting for DONE/OBSTACLE status from Arduino
    STATE_REPORTING_TO_SERVER,  // Reporting Arduino status (DONE/OBSTACLE) + pos to server
    STATE_TASK_FINALIZING,      // Calling /complete or /fail endpoint (Optional, server handles internally)
    STATE_ERROR                 // Unrecoverable error state
};

RobotState currentState = STATE_BOOTING;
long currentTaskID = -1;
String lastServerCommand = ""; // Last command received from server
String lastArduinoStatus = ""; // Last status received from Arduino ("DONE", "OBSTACLE", "INIT", "TIMEOUT")

// Last known position/orientation reported by Arduino
int lastReportedX = 0;
int lastReportedY = 0;
int lastReportedTheta = 0;
bool initialPositionReceived = false;

unsigned long lastStateChangeTime = 0;
unsigned long lastIdleCheckTime = 0;
unsigned long arduinoCmdSentTime = 0;

// Buffer for reading Arduino Serial data
String arduinoSerialBuffer = "";

// --- Function Declarations ---
void connectWiFi();
void handleStateMachine();
void setState(RobotState newState);
String makeHttpRequest(String method, String url, String payload);
bool sendCommandToArduino(String command);
bool readAndParseArduinoResponse();
void reportStatusToServer(const char* status);
void getNextCommandFromServer();
void getNextTaskFromServer();
// void finalizeTask(bool success); // This is now optional, server handles final state

// --- Setup ---
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) { ; }
    Serial.println("\n\nESP8266 Robot Coordinator Booting...");

    ARDUINO_SERIAL.begin(ARDUINO_BAUD_RATE);
    Serial.println("Arduino Serial (using SoftwareSerial on D5/D6) initialized.");
    Serial.print("Baud rate set to: ");
    Serial.println(ARDUINO_BAUD_RATE);

    connectWiFi();
    setState(STATE_BOOTING);
}

// --- Main Loop ---
void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi Disconnected. Attempting reconnect...");
        connectWiFi();
        delay(1000);
        return;
    }

    // Non-blocking Arduino Serial Read
    while (ARDUINO_SERIAL.available() > 0) {
        char c = ARDUINO_SERIAL.read();
        if (c == '\n') {
            if (currentState == STATE_BOOTING || currentState == STATE_WAITING_FOR_ARDUINO) {
                if (!readAndParseArduinoResponse()) {
                    Serial.println("Failed to parse buffered Arduino data.");
                }
            } else {
                Serial.print("Ignoring unexpected Arduino data in state ");
                Serial.print(currentState); Serial.print(": "); Serial.println(arduinoSerialBuffer);
            }
            arduinoSerialBuffer = "";
        } else if (c >= 0) {
            arduinoSerialBuffer += c;
            if (arduinoSerialBuffer.length() > 200) {
                Serial.println("Warning: Arduino Serial Buffer overflow. Clearing.");
                arduinoSerialBuffer = "";
            }
        }
    }

    handleStateMachine();
    delay(50);
}

// --- State Machine Handler ---
void handleStateMachine() {
    switch (currentState) {
        case STATE_BOOTING:
            if (millis() - lastStateChangeTime > 15000 && !initialPositionReceived) {
                Serial.println("Timeout waiting for initial position from Arduino.");
                lastStateChangeTime = millis(); // Reset timer
                // Consider entering error state if this repeats
            }
            // State transition happens in readAndParseArduinoResponse
            break;

        case STATE_IDLE:
            if (millis() - lastIdleCheckTime >= IDLE_CHECK_INTERVAL_MS) {
                lastIdleCheckTime = millis();
                getNextTaskFromServer();
            }
            break;

        case STATE_INITIALIZING_TASK:
            reportStatusToServer(StatusInit);
            break;

        case STATE_GETTING_COMMAND:
            getNextCommandFromServer();
            break;

        case STATE_SENDING_TO_ARDUINO:
            if (sendCommandToArduino(lastServerCommand)) {
                arduinoCmdSentTime = millis();
                setState(STATE_WAITING_FOR_ARDUINO);
            } else {
                Serial.println("Failed to send cmd to Arduino. Retrying...");
                delay(500);
            }
            break;

        case STATE_WAITING_FOR_ARDUINO:
            if (millis() - arduinoCmdSentTime > ARDUINO_RESPONSE_TIMEOUT_MS) {
                Serial.println("Timeout waiting for Arduino response!");
                lastArduinoStatus = StatusTimeout;
                // Report timeout as obstacle to server to force check/replan
                reportStatusToServer(StatusObstacle);
            }
            // State transition happens in readAndParseArduinoResponse
            break;

        case STATE_REPORTING_TO_SERVER:
            reportStatusToServer(lastArduinoStatus.c_str());
            break;

        case STATE_TASK_FINALIZING: // This state is now less critical
             Serial.println("Server indicated task complete/error. Returning to Idle.");
             currentTaskID = -1;
             setState(STATE_IDLE);
            break;

        case STATE_ERROR:
            Serial.println("Entered ERROR state. Halting operations.");
            // Add LED blink or other indicator
            delay(10000);
            // Attempt recovery maybe?
             connectWiFi(); // Try reconnecting WiFi first
             if(WiFi.status() == WL_CONNECTED) {
                  setState(STATE_BOOTING); // Try to restart flow if WiFi ok
             }
            break;
    }
}

// --- State Transition Helper ---
void setState(RobotState newState) {
    if (currentState != newState) {
        Serial.print("State Changing: "); Serial.print(currentState);
        Serial.print(" -> "); Serial.println(newState);
        currentState = newState;
        lastStateChangeTime = millis();
        if (newState == STATE_IDLE) {
            lastIdleCheckTime = millis();
            currentTaskID = -1; // Clear task ID when becoming idle
        }
         if (newState == STATE_BOOTING) {
             initialPositionReceived = false; // Reset flag when rebooting/restarting flow
         }
    }
}

// --- Server Communication ---

void getNextTaskFromServer() {
    Serial.println("Requesting next task from server...");
    String url = "http://" + String(serverIp) + ":" + String(serverPort) + "/tasks/next";
    String response = makeHttpRequest("POST", url, String("")); // Empty payload for POST

    if (response == "ERROR" || response == "TIMEOUT") {
        Serial.println("Failed to get task assignment from server.");
        return; // Stay IDLE, retry later
    }

    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, response);

    if (error) {
        Serial.print("Failed to parse task assignment JSON: "); Serial.println(error.c_str());
        if (response.indexOf("No pending tasks available") != -1) { Serial.println("Server: No tasks."); }
        else { Serial.println("Unexpected server response format."); Serial.println("Response: " + response); }
        return; // Stay IDLE
    }

    if (!doc.containsKey("id") || !doc.containsKey("station")) {
        Serial.println("Error: Assignment JSON missing 'id' or 'station'.");
        Serial.println("Response: " + response);
        return; // Stay IDLE
    }

    currentTaskID = doc["id"].as<long>();
    int station = doc["station"].as<int>();
    Serial.printf("Received Task ID: %ld for Station: %d\n", currentTaskID, station);
    setState(STATE_INITIALIZING_TASK); // Need to report initial position
}

void reportStatusToServer(const char* status) {
    if (currentTaskID < 0) {
        Serial.println("Error: Cannot report status, no active Task ID.");
        setState(STATE_IDLE); return;
    }
    // Ensure we have a position before reporting, especially for INIT
    if (!initialPositionReceived && strcmp(status, StatusInit) == 0) {
        Serial.println("Error: Trying to report INIT status before receiving position from Arduino.");
        setState(STATE_BOOTING); // Go back to waiting for Arduino
        return;
    }


    Serial.printf("Reporting status '%s' for Task %ld (Pos: %d,%d Theta: %d)...\n",
        status, currentTaskID, lastReportedX, lastReportedY, lastReportedTheta);

    String url = "http://" + String(serverIp) + ":" + String(serverPort) + "/tasks/" + String(currentTaskID) + "/report_status";

    StaticJsonDocument<200> payloadDoc;
    payloadDoc["x"] = lastReportedX; payloadDoc["y"] = lastReportedY; payloadDoc["theta"] = lastReportedTheta;
    payloadDoc["last_cmd_status"] = status;
    String payload; serializeJson(payloadDoc, payload);

    String response = makeHttpRequest("POST", url, payload);

    if (response == "ERROR" || response == "TIMEOUT") {
        Serial.println("Failed to report status to server. Will retry...");
        delay(REPORT_RETRY_DELAY_MS);
        // Stay in REPORTING_TO_SERVER to retry
        return;
    }

    StaticJsonDocument<128> doc; // Check server response
    DeserializationError error = deserializeJson(doc, response);
    if (error || !doc.containsKey("message")) { Serial.print("Warning: Could not parse OK response after reporting status. Assuming OK. Resp: " + response); Serial.println();}
    else { Serial.print("Server response: "); Serial.println(doc["message"].as<String>()); }

    // Successfully reported status, now request the next command
    setState(STATE_GETTING_COMMAND);
}

void getNextCommandFromServer() {
    if (currentTaskID < 0) { Serial.println("Error: Cannot get command, no Task ID."); setState(STATE_IDLE); return; }

    Serial.printf("Requesting next command for Task %ld...\n", currentTaskID);
    String url = "http://" + String(serverIp) + ":" + String(serverPort) + "/tasks/" + String(currentTaskID) + "/next_command";
    String response = makeHttpRequest("GET", url, String("")); // Empty payload for GET

    if (response == "ERROR" || response == "TIMEOUT") { Serial.println("Failed to get cmd. Retrying..."); delay(GET_CMD_RETRY_DELAY_MS); return; } // Stay in state

    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, response);

    if (error || !doc.containsKey("command")) {
        Serial.print("Failed parse command JSON: "); if(error) Serial.println(error.c_str()); else Serial.println("Key missing");
        Serial.println("Response: " + response); Serial.println("Retrying..."); delay(GET_CMD_RETRY_DELAY_MS); return; // Stay in state
    }

    lastServerCommand = doc["command"].as<String>();
    Serial.print("Received command: "); Serial.println(lastServerCommand);

    if (lastServerCommand == CmdComplete) { Serial.println("Server: Path Complete!"); setState(STATE_TASK_FINALIZING); }
    else if (lastServerCommand == CmdError) { Serial.println("Server reported ERROR."); setState(STATE_TASK_FINALIZING); }
    else if (lastServerCommand == CmdNone) { Serial.println("Server: NO_COMMAND (Waiting?). Retrying..."); delay(GET_CMD_RETRY_DELAY_MS); /* Stay in state */ }
    else if (lastServerCommand == CmdForward || lastServerCommand == CmdTurnLeft || lastServerCommand == CmdTurnRight) { setState(STATE_SENDING_TO_ARDUINO); }
    else { Serial.print("Unknown command: "); Serial.println(lastServerCommand); setState(STATE_TASK_FINALIZING); /* Treat as error */ }
}

// finalizeTask function is removed as it's optional and server handles the DB state internally now.

// --- Arduino Communication ---

bool sendCommandToArduino(String command) {
    if (command == "") { Serial.println("Err: Empty cmd to Arduino."); return false; }
    String formattedCmd = "CMD:" + command + "\n";
    Serial.print("Sending Arduino: "); Serial.print(formattedCmd);
    size_t bytesSent = ARDUINO_SERIAL.print(formattedCmd); ARDUINO_SERIAL.flush();
    if (bytesSent != formattedCmd.length()) { Serial.println("Warn: Bytes mismatch sending?"); }
    return bytesSent > 0;
}


bool readAndParseArduinoResponse() {
    // We are now looking for a "STATUS:" message in both BOOTING and WAITING states.
    if ((currentState == STATE_BOOTING || currentState == STATE_WAITING_FOR_ARDUINO) && arduinoSerialBuffer.startsWith("STATUS:")) {
        
        Serial.print("Parsing valid STATUS message from Arduino: "); Serial.println(arduinoSerialBuffer);
        
        char statusStr[20] = {0}; // Buffer for the status text (e.g., "INIT", "DONE")
        int x = -1, y = -1, t = -1;
        
        // Parse the incoming string
        int parsedCount = sscanf(arduinoSerialBuffer.c_str(), "STATUS:%19[^:]:X:%d:Y:%d:T:%d", statusStr, &x, &y, &t);

        if (parsedCount == 4) {
            String tempStatus = String(statusStr);
            tempStatus.toUpperCase();
            lastArduinoStatus = tempStatus;
            
            lastReportedX = x;
            lastReportedY = y;
            lastReportedTheta = t;
            Serial.printf("Parsed Status: %s, Pos: X=%d, Y=%d, Theta=%d\n", lastArduinoStatus.c_str(), x, y, t);

            // If we are booting and we get the INIT message, we are ready to move to IDLE
            if (currentState == STATE_BOOTING && lastArduinoStatus == StatusInit) {
                initialPositionReceived = true;
                Serial.println("Initial position handshake complete! Moving to IDLE state.");
                setState(STATE_IDLE);
                return true;
            }
            
            // If we were waiting for a command to finish, and we got DONE or OBSTACLE
            if (currentState == STATE_WAITING_FOR_ARDUINO && (lastArduinoStatus == StatusDone || lastArduinoStatus == StatusObstacle)) {
                setState(STATE_REPORTING_TO_SERVER); // Got a valid status, report it
                return true;
            }
            
            // If we get here, the message was parsed but wasn't the one we expected for the current state.
            Serial.println("Warning: Parsed status message, but it was not expected in the current state.");
            return false;

        } else {
            Serial.println("Error: Message started with STATUS: but format was incorrect.");
            return false;
        }
    }
    
    // If the message didn't start with STATUS: or we weren't in the right state, it's a failure.
    return false;
}


// --- HTTP Helper ---
String makeHttpRequest(String method, String url, String payload = "") {
    WiFiClient client; HTTPClient http; String response = "ERROR";
    Serial.print("HTTP "); Serial.print(method); Serial.print(": "); Serial.println(url);
    if (payload != "") { Serial.print("Payload: "); Serial.println(payload); }
    http.setTimeout(HTTP_TIMEOUT_MS);
    if (http.begin(client, url)) {
        int httpCode = 0;
        if (method == "GET") { httpCode = http.GET(); }
        else if (method == "POST") { http.addHeader("Content-Type", "application/json"); httpCode = http.POST(payload); }
        else { Serial.println("Unsupported HTTP method"); http.end(); return "ERROR"; }
        if (httpCode > 0) {
            response = http.getString();
            Serial.print("HTTP Resp Code: "); Serial.println(httpCode);
            if (response.length() < 200) { Serial.print("HTTP Resp Body: "); Serial.println(response); }
            else { Serial.println("HTTP Resp Body (truncated)..."); }
            if (httpCode < 200 || httpCode >= 300) { Serial.print("HTTP req potential fail code: "); Serial.println(httpCode); }
        } else {
            Serial.printf("[HTTP] %s fail, err: %s\n", method.c_str(), http.errorToString(httpCode).c_str());
            if (httpCode == HTTPC_ERROR_CONNECTION_REFUSED || httpCode == HTTPC_ERROR_SEND_HEADER_FAILED || httpCode == HTTPC_ERROR_CONNECTION_FAILED) { response = "ERROR"; }
            else if (httpCode == HTTPC_ERROR_READ_TIMEOUT) { response = "TIMEOUT"; }
            else { response = "ERROR"; }
        }
        http.end();
    } else { Serial.printf("[HTTP] Unable to connect %s\n", url.c_str()); response = "ERROR"; }
    return response;
}

// --- WiFi Connection ---
void connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) return;
    Serial.print("Connecting WiFi: "); Serial.print(ssid);
    WiFi.mode(WIFI_STA); WiFi.begin(ssid, password);
    unsigned long startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_CONNECT_TIMEOUT) { Serial.print("."); delay(500); }
    if (WiFi.status() != WL_CONNECTED) { Serial.println("\nWiFi Fail!"); setState(STATE_ERROR); }
    else {
        Serial.println("\nWiFi connected!"); Serial.print("IP: "); Serial.println(WiFi.localIP());
        if (currentState == STATE_BOOTING || currentState == STATE_ERROR) {
             Serial.println("WiFi ok, -> BOOTING (wait Arduino Init)...");
             setState(STATE_BOOTING); // Go back to waiting for Arduino init
        }
    }
}