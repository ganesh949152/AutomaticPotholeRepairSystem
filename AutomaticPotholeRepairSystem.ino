#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>
#include <ESP32Servo.h> // Use ESP32Servo for ESP32
#include <math.h>     // For M_PI and pow
#include <freertos/FreeRTOS.h> // Include FreeRTOS headers
#include <freertos/task.h>     // For task creation
#include <freertos/queue.h>    // For queue management
#include <WebServer.h>         // Include WebServer library for ESP32
#include <ArduinoJson.h>       // For sending JSON responses from ESP32 web server

// --- WiFi Credentials ---
const char* ssid = "Unknown";
const char* password = "@@##$$__";

// --- Python Anywhere Hostname and Endpoints ---
const char* serverName = "ganeshganji.pythonanywhere.com";
const String dataEndpoint = "/receive_data";
const String statusEndpoint = "/update_status";

// --- Ultrasonic Sensor Pins ---
const int trigPinFront = 23; // Front Sensor (Obstacle Detection)
const int echoPinFront = 22; // Front Sensor (Obstacle Detection)
const int trigPinBottom = 21; // Bottom Sensor (Pothole Detection)
const int echoPinBottom = 19; // Bottom Sensor (Pothole Detection)

// --- L298N Motor Driver Pins ---
// These pins are for the main robot movement motors
const int motor1_IN1_MAIN = 25; // Motor 1 Forward (Main Movement)
const int motor1_IN2_MAIN = 26; // Motor 1 Backward (Main Movement)
const int motor2_IN3_MAIN = 27; // Motor 2 Forward (Main Movement)
const int motor2_IN4_MAIN = 14; // Motor 2 Backward (Main Movement)

// New DC Motor Pins for Mixer
const int MIXER_MOTOR_IN1 = 17; // Connected to OUT1 on L298N
const int MIXER_MOTOR_IN2 = 16; // Connected to OUT2 on L298N


// --- Servo Pins ---
Servo cementServo;
Servo waterServo;
Servo releaseServo; // Reintroducing releaseServo

const int cementServoPin = 18;
const int waterServoPin = 5;
const int releaseServoPin = 4; // Pin 4 for Under (Release) Servo

// --- Servo Angles (CALIBRATE THESE!) ---
// IMPORTANT: These are example values. YOU MUST CALIBRATE THESE FOR YOUR SPECIFIC SERVOS.
// All servo sliders now range from 3 to 88 degrees.
const int CEMENT_SERVO_CLOSED_ANGLE = 3;
const int CEMENT_SERVO_OPEN_ANGLE = 88;

const int WATER_SERVO_CLOSED_ANGLE = 3;
const int WATER_SERVO_OPEN_ANGLE = 88;

const int RELEASE_SERVO_CLOSED_ANGLE = 3; // New for Release Servo
const int RELEASE_SERVO_OPEN_ANGLE = 88;  // New for Release Servo


// --- Water Level Sensor Pin ---
// !!! IMPORTANT: Pin 4 is now used for releaseServoPin. This is a conflict.
// If you need the water level sensor, you MUST change waterLevelPin to a different GPIO.
// For now, it remains commented out and checkMaterialLevel() returns true.
// const int waterLevelPin = 4; // Digital input, HIGH = OK, LOW = Empty

// --- Neo-6M GPS Module ---
HardwareSerial gpsSerial(1); // Use Serial1: RX2 (GPIO16), TX2 (GPIO17) by default on many ESP32s
TinyGPSPlus gps;
const int GPS_RX_PIN = 32; // Connect GPS TX to this ESP32 RX
const int GPS_TX_PIN = 33; // Connect GPS TX to this ESP32 RX (using RX2, TX2)

// --- System Constants & Calibration ---
const float POTHOLE_THRESHOLD_DISTANCE_CM = 5.0; // Distance above this is considered a pothole
const float OBSTACLE_DETECTION_DISTANCE_CM = 20.0; // Distance below this is considered an obstacle
const float VEHICLE_SPEED_CM_PER_SEC = 30.0; // Calibrated speed of vehicle
const unsigned long INITIAL_BOOT_DELAY_MS = 5000; // 5 seconds delay after boot
const unsigned long SERVO_ACTION_DURATION_PER_CM3_MS = 100; // 100ms per cm3 of volume for servos

// --- Global Variables for Flow Control ---
unsigned long systemStartTime = 0;
bool systemInitialized = false;

// --- Global Variables for Data ---
float currentPotholeLengthCm = 0;
float currentPotholeVolumeCm3 = 0;
float totalDistanceTravelledCm = 0;
int potholesFixedCount = 0;
float lastKnownLat = 0.0;
float lastKnownLon = 0.0;
unsigned long lastDistanceUpdateTime = 0;

// --- Debouncing Variables ---
const int DEBOUNCE_READINGS = 5;
const unsigned long DEBOUNCE_DELAY_MS = 100;
float potholeReadings[DEBOUNCE_READINGS];
int potholeReadingIndex = 0;
int potholeReadingsCount = 0;
unsigned long lastPotholeReadTime = 0;
bool isPotholeDetected = false; // Flag for debounced pothole state

// --- FreeRTOS Queue for HTTP Requests ---
// Queue will store String pointers (payload + endpoint)
// Max 5 items in queue, each item is a pointer to a String object
QueueHandle_t httpQueue;

// --- Global WiFiClientSecure instance for the HTTP task ---
WiFiClientSecure client;

// --- WebServer instance for manual control ---
WebServer server(80); // Web server on port 80

// --- Global variables for web control and logging ---
enum MotorCommand { STOP, FORWARD, BACKWARD, LEFT, RIGHT };
volatile MotorCommand currentMotorCommand = STOP; // Use volatile for shared variable (accessed by loop and web handlers)
volatile bool manualControlMode = false; // Controlled only by web page toggle
String currentAlertMessage = "System Ready";

// Flags to trigger actions from web handlers to main loop
volatile bool triggerMeasureCenterFlag = false;
volatile bool triggerRepairFlag = false;


// Log buffer for web-based serial monitor
String logBuffer = "";
const int MAX_LOG_SIZE = 4000; // Max characters in log buffer for the web page

// --- Helper function to log messages to the serial monitor AND the web buffer ---
void logToWeb(String message) {
  Serial.println(message); // Always print to actual serial for debugging
  // Append message to log buffer, ensure it's doesn't grow indefinitely
  if (logBuffer.length() + message.length() + 1 > MAX_LOG_SIZE) {
    // If adding message exceeds limit, truncate oldest part of log
    int truncateIndex = logBuffer.length() + message.length() + 1 - MAX_LOG_SIZE;
    logBuffer = logBuffer.substring(truncateIndex);
  }
  logBuffer += message + "\n";
}

// --- HTML for the control page (served by ESP32) ---
const char* controlPageHtml = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Robot Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: 'Inter', sans-serif; text-align: center; margin: 0; padding: 20px; background-color: #f0f2f5; color: #333; }
        .container { max-width: 600px; margin: 20px auto; background-color: #fff; padding: 25px; border-radius: 12px; box-shadow: 0 6px 12px rgba(0,0,0,0.15); }
        h1 { color: #2c3e50; margin-bottom: 25px; }
        .status-box { background-color: #e0f7fa; border: 1px solid #b2ebf2; padding: 15px; border-radius: 8px; margin-bottom: 20px; font-size: 1.1em; color: #00796b; font-weight: bold; }
        .controls-grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 15px; margin-top: 20px; }
        .button {
            padding: 15px 20px; font-size: 1.1em; border: none; border-radius: 8px; cursor: pointer;
            background-color: #28a745; color: white; transition: all 0.2s ease;
            box-shadow: 0 4px #218838; position: relative; top: 0;
        }
        .button:active { background-color: #218838; box-shadow: 0 2px #1e7e34; top: 2px; }
        .button.stop { background-color: #dc3545; box-shadow: 0 4px #c82333; }
        .button.stop:active { background-color: #c82333; box-shadow: 0 2px #bd2130; }
        .button.action { background-color: #007bff; box-shadow: 0 4px #0069d9; }
        .button.action:active { background-color: #0069d9; box-shadow: 0 2px #0062cc; }

        .mode-toggle { margin-top: 30px; padding-top: 20px; border-top: 1px solid #eee; }
        .mode-button { padding: 10px 18px; font-size: 1em; border: none; border-radius: 6px; cursor: pointer;
            background-color: #6c757d; color: white; transition: background-color 0.2s ease;
            box-shadow: 0 3px #5a6268; position: relative; top: 0; }
        .mode-button:active { background-color: #5a6268; box-shadow: 0 1px #4e555b; top: 1px; }

        .servo-control { margin-top: 30px; padding-top: 20px; border-top: 1px solid #eee; text-align: left;}
        .servo-item { margin-bottom: 15px; display: flex; align-items: center; gap: 15px; }
        .servo-item label { flex: 1; font-weight: bold; color: #555; }
        .servo-item input[type="range"] { flex: 3; -webkit-appearance: none; width: 100%; height: 8px; background: #ddd; border-radius: 5px; outline: none; }
        .servo-item input[type="range"]::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 20px; height: 20px; border-radius: 50%; background: #007bff; cursor: pointer; box-shadow: 0 2px 4px rgba(0,0,0,0.2); }
        .servo-item input[type="range"]::-moz-range-thumb { width: 20px; height: 20px; border-radius: 50%; background: #007bff; cursor: pointer; box-shadow: 0 2px 4px rgba(0,0,0,0.2); }
        .servo-item span { flex: 0.5; text-align: right; font-weight: bold; color: #007bff; }

        .action-buttons { margin-top: 30px; padding-top: 20px; border-top: 1px solid #eee; display: flex; justify-content: space-around; gap: 15px; }

        .log-container {
            margin-top: 30px; padding-top: 20px; border-top: 1px solid #eee;
            text-align: left; background-color: #333; color: #0f0;
            font-family: 'Consolas', 'Monaco', monospace; font-size: 0.9em;
            height: 250px; overflow-y: scroll; border-radius: 8px; padding: 15px;
            white-space: pre-wrap; /* Preserve whitespace and wrap long lines */
        }

        @media (max-width: 480px) {
            .controls-grid { grid-template-columns: 1fr; }
            .action-buttons { flex-direction: column; }
            .button { width: 100%; }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Robot Control Panel</h1>
        <div class="status-box" id="alertStatus">Status: Initializing...</div>

        <div class="mode-toggle">
            <p>Current Mode: <span id="currentMode" style="color: #007bff;">Loading...</span></p>
            <button class="mode-button" onclick="toggleMode()">Toggle Mode</button>
        </div>

        <h2>Manual Movement</h2>
        <div class="controls-grid">
            <div></div><button class="button" onmousedown="startMove('forward')" onmouseup="stopMove()" ontouchstart="startMove('forward')" ontouchend="stopMove()">Forward</button><div></div>
            <button class="button" onmousedown="startMove('left')" onmouseup="stopMove()" ontouchstart="startMove('left')" ontouchend="stopMove()">Left</button>
            <button class="button stop" onclick="sendCommand('/stop')">Stop All</button>
            <button class="button" onmousedown="startMove('right')" onmouseup="stopMove()" ontouchstart="startMove('right')" ontouchend="stopMove()">Right</button>
            <div></div><button class="button" onmousedown="startMove('backward')" onmouseup="stopMove()" ontouchstart="startMove('backward')" ontouchend="stopMove()">Backward</button><div></div>
        </div>

        <div class="servo-control">
            <h2>Dispenser Servos</h2>
            <div class="servo-item">
                <label for="cementServo">Cement Servo</label>
                <input type="range" id="cementServo" min="3" max="88" value="3" onmouseup="setServo('cement', this.value)" ontouchend="setServo('cement', this.value)">
                <span id="cementServoValue">3</span>
            </div>
            <div class="servo-item">
                <label for="waterServo">Water Servo</label>
                <input type="range" id="waterServo" min="3" max="88" value="3" onmouseup="setServo('water', this.value)" ontouchend="setServo('water', this.value)">
                <span id="waterServoValue">3</span>
            </div>
            <div class="servo-item">
                <label for="releaseServo">Under (Release) Servo</label>
                <input type="range" id="releaseServo" min="3" max="88" value="3" onmouseup="setServo('release', this.value)" ontouchend="setServo('release', this.value)">
                <span id="releaseServoValue">3</span>
            </div>
        </div>

        <div class="action-buttons">
            <button class="button action" onclick="triggerAction('/triggerMeasureCenter')">Measure & Center Pothole</button>
            <button class="button action" onclick="triggerAction('/triggerRepair')">Start Repair</button>
        </div>

        <div class="log-container" id="logOutput">
            Robot Log:
        </div>
    </div>

    <script>
        const servoMap = {
            'cement': { id: 'cementServo', valueId: 'cementServoValue' },
            'water': { id: 'waterServo', valueId: 'waterServoValue' },
            'release': { id: 'releaseServo', valueId: 'releaseServoValue' } // Added new servo
        };

        let currentMode = 'loading'; // 'manual' or 'autonomous'
        let moveInterval;
        let logPollInterval;

        // Update servo value display as slider moves
        document.querySelectorAll('input[type="range"]').forEach(slider => {
            slider.oninput = function() {
                document.getElementById(slider.id + 'Value').innerText = this.value;
            };
        });

        function sendCommand(command) {
            fetch(command)
                .then(response => response.text())
                .then(data => {
                    document.getElementById('alertStatus').innerText = 'Command: ' + command.substring(1).replace('?dir=', '') + ' | Response: ' + data;
                })
                .catch(error => {
                    document.getElementById('alertStatus').innerText = 'Error: ' + error;
                    console.error('Error:', error);
                });
        }

        function startMove(direction) {
            if (currentMode === 'manual') {
                if (moveInterval) clearInterval(moveInterval);
                sendCommand(`/move?dir=${direction}`); // Send initial command
                moveInterval = setInterval(() => sendCommand(`/move?dir=${direction}`), 200); // Send repeatedly
            } else {
                document.getElementById('alertStatus').innerText = 'Error: Must be in Manual Mode to control movement.';
            }
        }

        function stopMove() {
            if (moveInterval) {
                clearInterval(moveInterval);
                moveInterval = null;
            }
            if (currentMode === 'manual') {
                sendCommand('/stop'); // Send stop command once
            }
        }

        function toggleMode() {
            fetch('/toggleMode')
                .then(response => response.text())
                .then(data => {
                    document.getElementById('alertStatus').innerText = 'Mode Toggled | Response: ' + data;
                    updateModeDisplay(); // Update display and manage log polling
                })
                .catch(error => {
                    document.getElementById('alertStatus').innerText = 'Error toggling mode: ' + error;
                    console.error('Error toggling mode:', error);
                });
        }

        function updateModeDisplay() {
            fetch('/getMode')
                .then(response => response.text())
                .then(data => {
                    currentMode = (data === '1' ? 'manual' : 'autonomous');
                    document.getElementById('currentMode').innerText = currentMode === 'manual' ? 'Manual' : 'Autonomous';

                    // Manage log polling based on mode
                    if (currentMode === 'autonomous') {
                        if (!logPollInterval) {
                            logPollInterval = setInterval(fetchLogAndDisplay, 1000); // Start polling
                            document.getElementById('logOutput').style.display = 'block'; // Show log
                            document.getElementById('logOutput').innerText = 'Robot Log:\n'; // Clear log on mode switch
                        }
                    } else {
                        if (logPollInterval) {
                            clearInterval(logPollInterval); // Stop polling
                            logPollInterval = null;
                            document.getElementById('logOutput').style.display = 'none'; // Hide log
                        }
                    }
                })
                .catch(error => {
                    console.error('Error getting mode:', error);
                });
        }

        function setServo(servoName, angle) {
            if (currentMode === 'manual') {
                fetch(`/setServo?name=${servoName}&angle=${angle}`)
                    .then(response => response.text())
                    .then(data => {
                        document.getElementById('alertStatus').innerText = `Servo ${servoName} set to ${angle} | Response: ${data}`;
                    })
                    .catch(error => {
                        document.getElementById('alertStatus').innerText = `Error setting servo ${servoName}: ${error}`;
                        console.error('Error setting servo:', error);
                    });
            } else {
                document.getElementById('alertStatus').innerText = 'Error: Must be in Manual Mode to set servos.';
            }
        }

        function triggerAction(actionCommand) {
            if (currentMode === 'manual') {
                fetch(actionCommand)
                    .then(response => response.text())
                    .then(data => {
                        document.getElementById('alertStatus').innerText = `Action Triggered: ${actionCommand.substring(1)} | Response: ${data}`;
                    })
                    .catch(error => {
                        document.getElementById('alertStatus').innerText = `Error triggering action: ${error}`;
                        console.error('Error triggering action:', error);
                    });
            } else {
                document.getElementById('alertStatus').innerText = 'Error: Must be in Manual Mode to trigger actions.';
            }
        }

        function fetchLogAndDisplay() {
            fetch('/getLog')
                .then(response => response.text())
                .then(data => {
                    const logOutputDiv = document.getElementById('logOutput');
                    // Only update if content has changed (simple check for efficiency)
                    if (logOutputDiv.innerText !== 'Robot Log:\n' + data) {
                        logOutputDiv.innerText = 'Robot Log:\n' + data;
                        logOutputDiv.scrollTop = logOutputDiv.scrollHeight; // Scroll to bottom
                    }
                })
                .catch(error => {
                    console.error('Error fetching log:', error);
                });
        }

        function getServoAngles() {
            fetch('/getServoAngles')
                .then(response => response.json())
                .then(data => {
                    for (const servoName in data) {
                        if (servoMap[servoName]) {
                            const slider = document.getElementById(servoMap[servoName].id);
                            const valueSpan = document.getElementById(servoMap[servoName].valueId);
                            if (slider) {
                                slider.value = data[servoName];
                                valueSpan.innerText = data[servoName];
                            }
                        }
                    }
                })
                .catch(error => {
                    console.error('Error getting servo angles:', error);
                });
        }

        // Initial updates on page load
        window.onload = function() {
            updateModeDisplay(); // Sets initial mode and manages log polling
            getServoAngles(); // Get initial servo positions
            setInterval(getServoAngles, 5000); // Poll for servo angles every 5 seconds
        };
    </script>
</body>
</html>
)rawliteral";


// --- Function Declarations ---
void connectToWiFi();
float readUltrasonic(int trigPin, int echoPin);
bool detectObstacle();
bool detectPotholeDebounced(); // Debounced version of detectPothole
void moveForward();
void moveBackward();
void stopMotors();
void turnLeft(); // New function for turning left
void turnRight(); // New function for turning right
void stopMainMotors(); // New function to specifically stop main movement motors
void runMixerMotor(unsigned long durationMs); // New function for mixer DC motor
void stopMixerMotor(); // New function to stop mixer DC motor

// No longer needed for Under Motor:
// void runUnderMotorForward(unsigned long durationMs);
// void runUnderMotorBackward(unsigned long durationMs);
// void stopUnderMotor();

void updateDistanceTravelled();
bool checkMaterialLevel();
void processGPS();
void sendHTTPPost(const String& endpoint, const String& payload); // Modified to use queue
void sendRepairData();
void sendDeviceStatus(String status);
void initializeServos();
void measureAndCenterPothole(); // New function for auto-measure and center
void executeRepairSequence(); // New function for repair

// --- Web Server Handlers ---
void handleRoot();
void handleMove(); // Consolidated movement handler
void handleStop();
void handleSetServo();
void handleTriggerMeasureCenter();
void handleTriggerRepair();
void handleGetAlert();
void handleGetServoAngles();
void handleGetLog(); // New handler for log output
void handleToggleMode();
void handleGetMode();
void handleNotFound();

// --- FreeRTOS Task Function ---
void httpSendTask(void *pvParameters) {
  (void) pvParameters; // Suppress unused parameter warning

  // Ensure client is insecure for PythonAnywhere
  client.setInsecure();

  for (;;) { // Infinite loop for the task
    // Wait indefinitely for a String pointer to be available in the queue
    String* requestData = nullptr;
    if (xQueueReceive(httpQueue, &requestData, portMAX_DELAY) == pdPASS) {
      if (requestData != nullptr) {
        // Parse the combined string "endpoint|payload"
        int separatorIndex = requestData->indexOf('|');
        if (separatorIndex != -1) {
          String endpoint = requestData->substring(0, separatorIndex);
          String payload = requestData->substring(separatorIndex + 1);

          if (WiFi.status() == WL_CONNECTED) {
            HTTPClient http;
            String serverPath = "https://" + String(serverName) + endpoint;
            http.begin(client, serverPath);
            http.addHeader("Content-Type", "application/json");

            logToWeb("HTTP Task: Sending POST to " + serverPath + " Payload: " + payload);

            int httpResponseCode = http.POST(payload);

            if (httpResponseCode > 0) {
              logToWeb("HTTP Task: HTTP Response code: " + String(httpResponseCode));
              String responsePayload = http.getString();
              logToWeb("HTTP Task: Response: " + responsePayload);
            } else {
              logToWeb("HTTP Task: Error sending POST: " + String(httpResponseCode) + " Error: " + http.errorToString(httpResponseCode));
            }
            http.end();
          } else {
            logToWeb("HTTP Task: WiFi Disconnected. Cannot send data.");
          }
        }
        delete requestData; // Free the dynamically allocated String
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinBottom, OUTPUT);
  pinMode(echoPinBottom, INPUT);

  // Main Movement Motors
  pinMode(motor1_IN1_MAIN, OUTPUT);
  pinMode(motor1_IN2_MAIN, OUTPUT);
  pinMode(motor2_IN3_MAIN, OUTPUT);
  pinMode(motor2_IN4_MAIN, OUTPUT);
  stopMainMotors(); // Ensure main motors are stopped at boot

  // Mixer DC Motor Pins
  pinMode(MIXER_MOTOR_IN1, OUTPUT);
  pinMode(MIXER_MOTOR_IN2, OUTPUT);
  stopMixerMotor(); // Ensure mixer motor is stopped at boot

  // Under (Release) Servo Pin
  releaseServo.attach(releaseServoPin); // Attach the release servo

  cementServo.attach(cementServoPin);
  waterServo.attach(waterServoPin);
  initializeServos(); // Set initial servo positions

  // !!! IMPORTANT: Pin 4 is now used for releaseServoPin.
  // The waterLevelPin definition is commented out to avoid conflict.
  // pinMode(waterLevelPin, INPUT_PULLUP);

  logToWeb("Booting up...");
  connectToWiFi();

  // Create the FreeRTOS queue
  httpQueue = xQueueCreate(5, sizeof(String*)); // Queue for 5 String pointers

  // Create the HTTP sending task
  xTaskCreate(
    httpSendTask,         // Task function
    "HTTPSender",         // Name of task
    8192,                 // Stack size (bytes)
    NULL,                 // Parameter to pass to function
    1,                    // Priority (0 is lowest)
    NULL                  // Task handle
  );

  sendDeviceStatus("Booted. Initializing system..."); // This call now uses the queue
  systemStartTime = millis(); // Record boot time

  // --- Web Server Setup ---
  server.on("/", handleRoot);
  server.on("/move", handleMove); // Handle /move?dir=forward, /move?dir=backward, etc.
  server.on("/stop", handleStop); // Handle /stop for immediate stop
  server.on("/setServo", handleSetServo);
  server.on("/triggerMeasureCenter", handleTriggerMeasureCenter);
  server.on("/triggerRepair", handleTriggerRepair);
  server.on("/getAlert", handleGetAlert);
  server.on("/getServoAngles", handleGetServoAngles);
  server.on("/getLog", handleGetLog); // New handler for log output
  server.on("/toggleMode", handleToggleMode);
  server.on("/getMode", handleGetMode);
  server.onNotFound(handleNotFound);
  server.begin();
  logToWeb("ESP32 Web Server started.");
  logToWeb("Access control page at http://" + WiFi.localIP().toString() + "/");
}

void loop() {
  processGPS(); // Always process GPS data to keep GPS data fresh
  server.handleClient(); // Handle incoming web client requests

  // --- System Initialization Delay ---
  if (!systemInitialized) {
    if (millis() - systemStartTime >= INITIAL_BOOT_DELAY_MS) {
      systemInitialized = true;
      lastDistanceUpdateTime = millis(); // Initialize for distance tracking
      logToWeb("Initialization complete. Starting operation.");
      sendDeviceStatus("System Ready: Moving forward."); // This call now uses the queue
      currentAlertMessage = "System Ready: Autonomous Mode";
    }
    return; // Do nothing else until initialized
  }

  // --- WiFi Check ---
  if (WiFi.status() != WL_CONNECTED) {
    logToWeb("WiFi disconnected. Trying to reconnect...");
    connectToWiFi();
    currentAlertMessage = "WiFi Disconnected! Reconnecting...";
    // If not connected, motors should stop to prevent uncontrolled movement
    stopMainMotors();
    currentMotorCommand = STOP; // Reset manual command
    return; // Skip main logic if not connected
  }

  // --- Main Robot Logic Flow ---
  if (manualControlMode) {
    // In manual mode, the robot only responds to web commands.
    // Motors are controlled by handlers (handleMove, handleStop)
    // Autonomous logic is paused.
    // Continuously apply the current motor command from web control
    switch (currentMotorCommand) {
      case FORWARD: moveForward(); break;
      case BACKWARD: moveBackward(); break;
      case LEFT: turnLeft(); break;
      case RIGHT: turnRight(); break;
      case STOP: stopMainMotors(); break; // Ensure main motors are stopped
    }
    // logToWeb("Robot in MANUAL CONTROL mode. Awaiting web commands."); // Too chatty for log
    currentAlertMessage = "Manual Control Active";

    // Check for triggered actions in manual mode
    if (triggerMeasureCenterFlag) {
      triggerMeasureCenterFlag = false; // Reset flag
      logToWeb("Manual Trigger: Measure & Center Pothole");
      measureAndCenterPothole(); // Execute blocking function
      currentAlertMessage = "Pothole Measured & Centered (Manual)";
    }
    if (triggerRepairFlag) {
      triggerRepairFlag = false; // Reset flag
      logToWeb("Manual Trigger: Execute Repair Sequence");
      executeRepairSequence(); // Execute blocking function
      currentAlertMessage = "Repair Sequence Completed (Manual)";
    }

    delay(50); // Small delay to prevent spamming serial and allow web server to respond
    return; // Exit loop, wait for web commands or triggered actions
  }

  // --- Autonomous Mode Logic ---
  currentAlertMessage = "Autonomous Mode Active"; // Update status for web UI
  stopMainMotors(); // Ensure main motors are stopped if switching from manual to autonomous
  currentMotorCommand = STOP; // Reset manual command

  // Check material level at the start of each major cycle
  if (!checkMaterialLevel()) {
    stopMainMotors();
    logToWeb("CRITICAL: Material level low! Stopping all operations.");
    sendDeviceStatus("Material Low - Stopped");
    currentAlertMessage = "CRITICAL: Material Low! STOPPED.";
    delay(5000); // Wait before re-checking
    return; // Exit loop, re-evaluate material level next iteration
  }

  // Check for obstacle
  float frontDistance = readUltrasonic(trigPinFront, echoPinFront);
  logToWeb("Front Sensor Distance: " + String(frontDistance, 2) + " cm");
  if (frontDistance < OBSTACLE_DETECTION_DISTANCE_CM) {
    stopMainMotors();
    logToWeb("Obstacle detected! Stopping.");
    sendDeviceStatus("Obstacle Detected");
    currentAlertMessage = "Obstacle Detected! STOPPED.";
    delay(5000); // Simple blocking wait for obstacle to clear
    logToWeb("Resuming forward movement.");
    sendDeviceStatus("Resuming Forward");
    currentAlertMessage = "Obstacle Cleared. Resuming Autonomous.";
    lastDistanceUpdateTime = millis(); // Reset for new movement
    return; // Re-evaluate conditions after obstacle handling
  }

  // Move forward if no obstacle and materials are fine
  moveForward();
  updateDistanceTravelled(); // Update distance while moving forward

  // Check for pothole (debounced)
  if (detectPotholeDebounced()) {
    stopMainMotors();
    logToWeb("Pothole detected! Starting measurement.");
    sendDeviceStatus("Pothole detected, measuring length."); // This call now uses the queue
    currentAlertMessage = "Pothole Detected! Measuring...";

    measureAndCenterPothole(); // Execute blocking function
    executeRepairSequence();   // Execute blocking function

    currentPotholeLengthCm = 0; // Reset for next pothole
    currentPotholeVolumeCm3 = 0; // Reset for next pothole
    logToWeb("Repair cycle complete. Resetting for next pothole.");
    currentAlertMessage = "Pothole Repaired. Searching...";
    delay(2000); // Wait for 2 seconds as requested before continuing
  }
  // Small delay for overall loop stability when not in a repair sequence
  delay(10);
}

// --- Utility Functions ---

void connectToWiFi() {
  logToWeb("Connecting to WiFi: " + String(ssid));
  WiFi.begin(ssid, password);
  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < 20) { // Try for 10 seconds
    delay(500);
    Serial.print("."); // Keep printing dots to serial while waiting
  }
  if (WiFi.status() == WL_CONNECTED) {
    logToWeb("\nWiFi connected");
    logToWeb("IP address: " + WiFi.localIP().toString());
  } else {
    logToWeb("\nFailed to connect to WiFi. Will retry.");
  }
}

float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000); // 25ms timeout for ~4m range
  float distance = (duration * 0.0343) / 2.0;
  return distance <= 0 ? 999.0 : distance; // Return a large number on timeout/error
}

bool detectObstacle() {
  // Read distance directly here for immediate check, logging is done in loop
  return readUltrasonic(trigPinFront, echoPinFront) < OBSTACLE_DETECTION_DISTANCE_CM;
}

bool detectPotholeDebounced() {
  unsigned long now = millis();
  if (now - lastPotholeReadTime < DEBOUNCE_DELAY_MS) {
    return isPotholeDetected;
  }
  lastPotholeReadTime = now;

  float currentDistance = readUltrasonic(trigPinBottom, echoPinBottom);
  logToWeb("Bottom Sensor Distance: " + String(currentDistance, 2) + " cm"); // Debugging
  potholeReadings[potholeReadingIndex] = currentDistance;
  potholeReadingIndex = (potholeReadingIndex + 1) % DEBOUNCE_READINGS;
  if (potholeReadingsCount < DEBOUNCE_READINGS) {
    potholeReadingsCount++;
  }

  int readingsAboveThreshold = 0;
  for (int i = 0; i < potholeReadingsCount; i++) {
    if (potholeReadings[i] > POTHOLE_THRESHOLD_DISTANCE_CM) {
      readingsAboveThreshold++;
    }
  }
  isPotholeDetected = (readingsAboveThreshold == DEBOUNCE_READINGS); //all readings must be above
  return isPotholeDetected;
}

void moveForward() {
  // logToWeb("Motors: Forward"); // Too chatty for constant movement
  digitalWrite(motor1_IN1_MAIN, LOW);  digitalWrite(motor1_IN2_MAIN, HIGH);
  digitalWrite(motor2_IN3_MAIN, LOW);  digitalWrite(motor2_IN4_MAIN, HIGH);
}

void moveBackward() {
  logToWeb("Motors: Backward");
  digitalWrite(motor1_IN1_MAIN, HIGH); digitalWrite(motor1_IN2_MAIN, LOW);
  digitalWrite(motor2_IN3_MAIN, HIGH); digitalWrite(motor2_IN4_MAIN, LOW);
}

void turnLeft() {
  logToWeb("Motors: Turn Left");
  digitalWrite(motor1_IN1_MAIN, HIGH); digitalWrite(motor1_IN2_MAIN, LOW); // Left motor backward
  digitalWrite(motor2_IN3_MAIN, LOW);  digitalWrite(motor2_IN4_MAIN, HIGH); // Right motor forward
}

void turnRight() {
  logToWeb("Motors: Turn Right");
  digitalWrite(motor1_IN1_MAIN, LOW);  digitalWrite(motor1_IN2_MAIN, HIGH); // Left motor forward
  digitalWrite(motor2_IN3_MAIN, HIGH); digitalWrite(motor2_IN4_MAIN, LOW); // Right motor backward
}

void stopMainMotors() {
  logToWeb("Main Motors: Stop");
  digitalWrite(motor1_IN1_MAIN, LOW); digitalWrite(motor1_IN2_MAIN, LOW);
  digitalWrite(motor2_IN3_MAIN, LOW); digitalWrite(motor2_IN4_MAIN, LOW);
}

// --- New DC Motor Control Functions ---
void runMixerMotor(unsigned long durationMs) {
  logToWeb("Mixer Motor: Running for " + String(durationMs) + " ms.");
  digitalWrite(MIXER_MOTOR_IN1, HIGH); // Assume HIGH for forward
  digitalWrite(MIXER_MOTOR_IN2, LOW);
  delay(durationMs);
  stopMixerMotor();
}

void stopMixerMotor() {
  logToWeb("Mixer Motor: Stop");
  digitalWrite(MIXER_MOTOR_IN1, LOW);
  digitalWrite(MIXER_MOTOR_IN2, LOW);
}

// Removed Under Motor control functions:
// void runUnderMotorForward(unsigned long durationMs)
// void runUnderMotorBackward(unsigned long durationMs)
// void stopUnderMotor()


void updateDistanceTravelled() {
  // Only update distance if main motors are actively moving forward
  // Check if IN2_MAIN and IN4_MAIN are HIGH (forward motion)
  if (digitalRead(motor1_IN2_MAIN) == HIGH && digitalRead(motor2_IN4_MAIN) == HIGH) {
    totalDistanceTravelledCm += (millis() - lastDistanceUpdateTime) / 1000.0 * VEHICLE_SPEED_CM_PER_SEC;
    lastDistanceUpdateTime = millis();
  }
}

bool checkMaterialLevel() {
  // !!! IMPORTANT: Pin 4 is now used for releaseServoPin.
  // The waterLevelPin definition is commented out in setup() to avoid conflict.
  // This function will always return true unless waterLevelPin is moved and uncommented.
  return true; // Assuming material level is always OK due to pin conflict
}

void processGPS() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isUpdated() && gps.location.isValid()) {
        lastKnownLat = gps.location.lat();
        lastKnownLon = gps.location.lng();
        // logToWeb("GPS: Lat=" + String(lastKnownLat, 6) + " Lon=" + String(lastKnownLon, 6)); // Too chatty
      }
    }
  }
}

// Modified sendHTTPPost to use the FreeRTOS queue
void sendHTTPPost(const String& endpoint, const String& payload) {
  // Combine endpoint and payload into a single string, separated by a unique delimiter
  String* requestData = new String(endpoint + "|" + payload);

  // Send the pointer to the String object to the queue
  if (xQueueSend(httpQueue, &requestData, portMAX_DELAY) != pdPASS) {
    logToWeb("Failed to send HTTP request to queue.");
    delete requestData; // If sending fails, free the memory
  } else {
    logToWeb("HTTP request added to queue.");
  }
}

void sendRepairData() {
  String jsonPayload = "{";
  jsonPayload += "\"latitude\":" + String(lastKnownLat, 6) + ",";
  jsonPayload += "\"longitude\":" + String(lastKnownLon, 6) + ",";
  jsonPayload += "\"pothole_length_cm\":" + String(currentPotholeLengthCm, 2) + ",";
  jsonPayload += "\"pothole_volume_cm3\":" + String(currentPotholeVolumeCm3, 2) + ",";
  jsonPayload += "\"cement_units_used\":" + String(currentPotholeVolumeCm3 / 8.0, 2) + ","; // Example: half volume for cement
  jsonPayload += "\"water_units_used\":" + String(currentPotholeVolumeCm3 / 5.0, 2) + ",";   // Example: one-third volume for water
  jsonPayload += "\"cumulative_distance_m\":" + String(totalDistanceTravelledCm / 100.0, 2) + ",";
  jsonPayload += "\"cumulative_potholes_fixed\":" + String(potholesFixedCount); // potholesFixedCount is already incremented before this call
  jsonPayload += "}";
  sendHTTPPost(dataEndpoint, jsonPayload);
}

void sendDeviceStatus(String status) {
  String jsonPayload = "{\"status\":\"" + status + "\"}";
  sendHTTPPost(statusEndpoint, jsonPayload);
}

void initializeServos() {
  cementServo.write(CEMENT_SERVO_CLOSED_ANGLE);
  waterServo.write(WATER_SERVO_CLOSED_ANGLE);
  releaseServo.write(RELEASE_SERVO_CLOSED_ANGLE); // Initialize new servo
  logToWeb("Dispenser Servos initialized to closed positions.");
}

void measureAndCenterPothole() {
  logToWeb("Measure & Center Pothole function triggered.");
  currentAlertMessage = "Measuring & Centering...";

  unsigned long potholeStartMeasurementTime = millis();
  logToWeb("Moving forward to measure pothole length...");
  moveForward(); // Start moving forward to measure
  while (detectPotholeDebounced()) { // Keep moving forward while still in pothole
    moveForward();
    updateDistanceTravelled();
    if (gps.location.isValid()) {
      lastKnownLat = gps.location.lat();
      lastKnownLon = gps.location.lng();
    }
    delay(50); // Small delay to allow physical movement and sensor update
  }
  stopMainMotors();
  unsigned long potholeEndMeasurementTime = millis();
  currentPotholeLengthCm = (float)(potholeEndMeasurementTime - potholeStartMeasurementTime) / 1000.0 * VEHICLE_SPEED_CM_PER_SEC;
  logToWeb("Pothole length measured: " + String(currentPotholeLengthCm, 2) + " cm");
  sendDeviceStatus("Pothole length measured: " + String(currentPotholeLengthCm, 2) + " cm");

  logToWeb("Moving backward to center over pothole.");
  unsigned long backwardMoveDurationMs = (currentPotholeLengthCm / VEHICLE_SPEED_CM_PER_SEC / 2.0) * 1000;
  moveBackward();
  delay(backwardMoveDurationMs); // Blocking delay for backward movement
  stopMainMotors();
  logToWeb("Positioned over pothole center.");
  sendDeviceStatus("Positioned over pothole center.");

  // Calculate volume (assuming hemisphere)
  float radius = currentPotholeLengthCm / 2.0;
  currentPotholeVolumeCm3 = (2.0 / 3.0) * M_PI * pow(radius, 3);
  logToWeb("Calculated Pothole Volume: " + String(currentPotholeVolumeCm3, 2) + " cm^3");
  currentAlertMessage = "Pothole Measured & Centered.";
  sendDeviceStatus("Pothole Volume: " + String(currentPotholeVolumeCm3, 2) + " cm^3");
}

void executeRepairSequence() {
  logToWeb("Execute Repair Sequence triggered.");
  currentAlertMessage = "Starting Repair Sequence...";

  // Dispense Cement
  unsigned long cementDispenseDuration = currentPotholeVolumeCm3 / 8.0; // Volume/8 milliseconds
  if (cementDispenseDuration < 500) cementDispenseDuration = 500; // Minimum duration to ensure movement
  logToWeb("Dispensing cement for " + String(cementDispenseDuration) + " ms.");
  cementServo.write(CEMENT_SERVO_OPEN_ANGLE);
  logToWeb("Cement Servo Angle: " + String(CEMENT_SERVO_OPEN_ANGLE));
  delay(cementDispenseDuration); // Blocking delay
  cementServo.write(CEMENT_SERVO_CLOSED_ANGLE);
  logToWeb("Cement Servo Angle: " + String(CEMENT_SERVO_CLOSED_ANGLE));
  logToWeb("Cement dispensed.");
  currentAlertMessage = "Cement Dispensed.";

  // Dispense Water
  unsigned long waterDispenseDuration = currentPotholeVolumeCm3 / 5.0; // Volume/5 milliseconds
  if (waterDispenseDuration < 500) waterDispenseDuration = 500; // Minimum duration
  logToWeb("Dispensing water for " + String(waterDispenseDuration) + " ms.");
  waterServo.write(WATER_SERVO_OPEN_ANGLE);
  logToWeb("Water Servo Angle: " + String(WATER_SERVO_OPEN_ANGLE));
  delay(waterDispenseDuration); // Blocking delay
  waterServo.write(WATER_SERVO_CLOSED_ANGLE);
  logToWeb("Water Servo Angle: " + String(WATER_SERVO_CLOSED_ANGLE));
  logToWeb("Water dispensed.");
  currentAlertMessage = "Water Dispensed.";

  // Mix Materials (Mixer DC Motor)
  unsigned long mixDuration = 5000; // 5 seconds as requested
  logToWeb("Mixing materials for " + String(mixDuration) + " ms using DC motor.");
  runMixerMotor(mixDuration); // Run mixer motor for 5 seconds
  logToWeb("Mixing complete.");
  currentAlertMessage = "Mixing Complete.";

  // Release Mixture (Under Servo - simulated servo behavior)
  logToWeb("Preparing to release mixture with Under Servo.");
  delay(5000); // 5 seconds delay before under servo action

  logToWeb("Under Servo: Open for 100ms");
  releaseServo.write(RELEASE_SERVO_OPEN_ANGLE); // Open for 100ms
  delay(100);
  logToWeb("Under Servo: Close for 100ms");
  releaseServo.write(RELEASE_SERVO_CLOSED_ANGLE); // Close for 100ms
  delay(100);
  logToWeb("Under Servo action complete.");
  currentAlertMessage = "Mixture Released.";


  // Send Data and Reset
  sendDeviceStatus("Repair complete, sending data."); // This call now uses the queue
  potholesFixedCount++; // Increment count for this fixed pothole
  sendRepairData(); // This call now uses the queue

  currentPotholeLengthCm = 0; // Reset for next pothole
  currentPotholeVolumeCm3 = 0; // Reset for next pothole
  logToWeb("Data sent. Resetting for next pothole.");
  currentAlertMessage = "Repair Cycle Completed.";
  delay(2000); // Wait for 2 seconds as requested before continuing
}

// --- Web Server Handler Implementations ---

void handleRoot() {
  server.send(200, "text/html", controlPageHtml);
}

void handleMove() {
  if (manualControlMode) {
    String dir = server.arg("dir");
    if (dir == "forward") {
      currentMotorCommand = FORWARD;
      server.send(200, "text/plain", "Moving Forward");
    } else if (dir == "backward") {
      currentMotorCommand = BACKWARD;
      server.send(200, "text/plain", "Moving Backward");
    } else if (dir == "left") {
      currentMotorCommand = LEFT;
      server.send(200, "text/plain", "Turning Left");
    } else if (dir == "right") {
      currentMotorCommand = RIGHT;
      server.send(200, "text/plain", "Turning Right");
    } else {
      server.send(400, "text/plain", "Invalid direction");
    }
  } else {
    server.send(403, "text/plain", "Not in Manual Control Mode. Toggle to Manual Mode first.");
  }
}

void handleStop() {
  if (manualControlMode) {
    currentMotorCommand = STOP;
    stopMainMotors(); // Ensure main motors are stopped immediately
    server.send(200, "text/plain", "Stopping Motors");
  } else {
    server.send(403, "text/plain", "Not in Manual Control Mode. Toggle to Manual Mode first.");
  }
}

void handleSetServo() {
  if (manualControlMode) {
    String servoName = server.arg("name");
    int angle = server.arg("angle").toInt();

    // Validate angle to be within the new 3-88 range
    if (angle < 3 || angle > 88) {
      server.send(400, "text/plain", "Invalid angle (3-88)");
      return;
    }

    if (servoName == "cement") {
      cementServo.write(angle);
      logToWeb("Setting Cement Servo to: " + String(angle));
    } else if (servoName == "water") {
      waterServo.write(angle);
      logToWeb("Setting Water Servo to: " + String(angle));
    } else if (servoName == "release") { // Handle the new release servo
      releaseServo.write(angle);
      logToWeb("Setting Release Servo to: " + String(angle));
    }
    else {
      // Reject attempts to control non-existent or DC motor "servos"
      server.send(400, "text/plain", "Invalid servo name or not controllable via slider: " + servoName);
      return;
    }
    server.send(200, "text/plain", "Servo " + servoName + " set to " + String(angle));
  } else {
    server.send(403, "text/plain", "Not in Manual Control Mode. Toggle to Manual Mode first.");
  }
}

void handleTriggerMeasureCenter() {
  if (manualControlMode) {
    // These flags are volatile and used by the loop to trigger the blocking functions
    // This allows the web server handler to return quickly
    triggerMeasureCenterFlag = true;
    server.send(200, "text/plain", "Measure & Center Pothole triggered. Robot will now perform sequence.");
  } else {
    server.send(403, "text/plain", "Not in Manual Control Mode. Toggle to Manual Mode first.");
  }
}

void handleTriggerRepair() {
  if (manualControlMode) {
    // These flags are volatile and used by the loop to trigger the blocking functions
    // This allows the web server handler to return quickly
    triggerRepairFlag = true;
    server.send(200, "text/plain", "Repair sequence triggered. Robot will now perform sequence.");
  } else {
    server.send(403, "text/plain", "Not in Manual Control Mode. Toggle to Manual Mode first.");
  }
}

void handleGetAlert() {
  // Explicitly cast to non-volatile String& for server.send
  server.send(200, "text/plain", currentAlertMessage);
}

void handleGetServoAngles() {
  StaticJsonDocument<200> doc; // ArduinoJson document
  doc["cement"] = cementServo.read();
  doc["water"] = waterServo.read();
  doc["release"] = releaseServo.read(); // Include the new release servo
  // Not including mixer as it's a DC motor and doesn't have an angle
  String jsonResponse;
  serializeJson(doc, jsonResponse);
  server.send(200, "application/json", jsonResponse);
}

void handleGetLog() {
  server.send(200, "text/plain", logBuffer);
}

void handleToggleMode() {
  manualControlMode = !manualControlMode; // Toggle the mode
  stopMainMotors(); // Always stop main motors when changing modes for safety
  currentMotorCommand = STOP; // Ensure internal command is stopped
  String response = manualControlMode ? "Manual Mode ON" : "Autonomous Mode ON";
  logToWeb("Mode Toggled: " + response);
  currentAlertMessage = response; // Update alert message
  server.send(200, "text/plain", response);
}

void handleGetMode() {
  server.send(200, "text/plain", String(manualControlMode ? "1" : "0"));
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}
