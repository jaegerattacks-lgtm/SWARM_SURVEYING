#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>

// ================================================================
// 1. PIN DEFINITIONS (Assign your pins here)
// ================================================================

// Motor 1 (Front Left)
#define M1_RPWM  26  // Forward Pin
#define M1_LPWM  27  // Backward Pin
#define M1_EN_A  35  // Encoder A
#define M1_EN_B  32  // Encoder B

// Motor 2 (Front Right)
#define M2_RPWM  21
#define M2_LPWM  19
#define M2_EN_A  36
#define M2_EN_B  39

// Motor 3 (Rear Left)
#define M3_RPWM  14
#define M3_LPWM  13
#define M3_EN_A  33
#define M3_EN_B  25

// Motor 4 (Rear Right)
#define M4_RPWM  18
#define M4_LPWM  4
#define M4_EN_A  34
#define M4_EN_B  23

// ================================================================
// 2. CONFIGURATION CONSTANTS
// ================================================================

// WiFi Credentials
const char* ssid = "MyESP32_AP";
const char* password = "12345678";

// Encoder Resolution
const float PPR = 100.0; // Pulses Per Revolution (Change this to your encoder's value)
const unsigned long RPM_INTERVAL = 100; // Calculate RPM every 100ms

// ================================================================
// 3. GLOBAL VARIABLES
// ================================================================

WebServer server(80);

// Encoder Ticks (Volatile because they are changed in ISR)
volatile long m1_ticks = 0;
volatile long m2_ticks = 0;
volatile long m3_ticks = 0;
volatile long m4_ticks = 0;

// RPM Values
float m1_rpm = 0;
float m2_rpm = 0;
float m3_rpm = 0;
float m4_rpm = 0;

// Timer for RPM calculation
unsigned long previousMillis = 0;

// CONTROL VARIABLES
int motorSpeed = 150; // Default speed (0-255)
int moveState = 0;    // 0=Stop, 1=Fwd, 2=Bwd, 3=Left, 4=Right

// ================================================================
// 4. INTERRUPT SERVICE ROUTINES (ISRs)
// ================================================================

// Handle Motor 1 Encoder
void IRAM_ATTR isr_m1() {
  if (digitalRead(M1_EN_B) == digitalRead(M1_EN_A)) {
    m1_ticks++;
  } else {
    m1_ticks--;
  }
}

// Handle Motor 2 Encoder
void IRAM_ATTR isr_m2() {
  if (digitalRead(M2_EN_B) == digitalRead(M2_EN_A)) {
    m2_ticks++;
  } else {
    m2_ticks--;
  }
}

// Handle Motor 3 Encoder
void IRAM_ATTR isr_m3() {
  if (digitalRead(M3_EN_B) == digitalRead(M3_EN_A)) {
    m3_ticks++;
  } else {
    m3_ticks--;
  }
}

// Handle Motor 4 Encoder
void IRAM_ATTR isr_m4() {
  if (digitalRead(M4_EN_B) == digitalRead(M4_EN_A)) {
    m4_ticks++;
  } else {
    m4_ticks--;
  }
}

// ================================================================
// 5. MOTOR CONTROL
// ================================================================

void setMotor(int rpwm, int lpwm, int speed) {
  if (speed > 0) {
    analogWrite(rpwm, speed);
    analogWrite(lpwm, 0);
  } else if (speed < 0) {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, -speed);
  } else {
    analogWrite(rpwm, 0);
    analogWrite(lpwm, 0);
  }
}

// Function to apply the current speed based on the current state
void updateMotors() {
  int spd = motorSpeed; // Use the global slider value

  switch (moveState) {
    case 0: // STOP
      setMotor(M1_RPWM, M1_LPWM, 0); setMotor(M2_RPWM, M2_LPWM, 0);
      setMotor(M3_RPWM, M3_LPWM, 0); setMotor(M4_RPWM, M4_LPWM, 0);
      break;
    case 1: // FORWARD
      setMotor(M1_RPWM, M1_LPWM, -spd); setMotor(M2_RPWM, M2_LPWM, spd);
      setMotor(M3_RPWM, M3_LPWM, -spd); setMotor(M4_RPWM, M4_LPWM, spd);
      break;
    case 2: // BACKWARD
      setMotor(M1_RPWM, M1_LPWM, spd); setMotor(M2_RPWM, M2_LPWM, -spd);
      setMotor(M3_RPWM, M3_LPWM, spd); setMotor(M4_RPWM, M4_LPWM, -spd);
      break;
    case 3: // LEFT (Turn on spot)
      setMotor(M1_RPWM, M1_LPWM, spd); setMotor(M2_RPWM, M2_LPWM, spd);
      setMotor(M3_RPWM, M3_LPWM, spd); setMotor(M4_RPWM, M4_LPWM, spd);
      break;
    case 4: // RIGHT (Turn on spot)
      setMotor(M1_RPWM, M1_LPWM, -spd); setMotor(M2_RPWM, M2_LPWM, -spd);
      setMotor(M3_RPWM, M3_LPWM, -spd); setMotor(M4_RPWM, M4_LPWM, -spd);
      break;
  }
}

// ================================================================
// 6. WEB SERVER
// ================================================================

void handleRoot() {
  String html = "<html><head><title>Robot Control</title>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body { font-family: sans-serif; text-align: center; margin-top: 50px; }";
  html += "button { width: 80px; height: 80px; font-size: 20px; margin: 5px; border-radius: 10px; }";
  html += ".slider { width: 300px; margin: 20px; }";
  html += "</style></head><body>";
  
  html += "<h1>ESP32 Robot</h1>";
  html += "<p><b>Speed: <span id='speedVal'>" + String(motorSpeed) + "</span></b></p>";
  
  // SLIDER
  html += "<input type='range' min='0' max='255' value='" + String(motorSpeed) + "' class='slider' id='spdSlider' oninput='updateSpeed(this.value)'>";
  
  html += "<br><br>";
  
  // BUTTONS
  html += "<button onclick=\"fetch('/fwd')\">&#8593;</button><br>";
  html += "<button onclick=\"fetch('/left')\">&#8592;</button>";
  html += "<button onclick=\"fetch('/stop')\" style='background-color:#ffcccc'>STOP</button>";
  html += "<button onclick=\"fetch('/right')\">&#8594;</button><br>";
  html += "<button onclick=\"fetch('/bwd')\">&#8595;</button>";

  html += "<p>RPM FL: <span id='rpm1'>0</span> | FR: <span id='rpm2'>0</span></p>";
  
  // JAVASCRIPT
  html += "<script>";
  html += "function updateSpeed(val) {";
  html += "  document.getElementById('speedVal').innerText = val;";
  html += "  fetch('/setSpeed?val=' + val);";
  html += "}";
  html += "setInterval(function(){";
  html += "  fetch('/rpm').then(r => r.text()).then(d => {";
  html += "    let parts = d.split(',');";
  html += "    if(parts.length >= 2) {";
  html += "      document.getElementById('rpm1').innerText = parts[0];";
  html += "      document.getElementById('rpm2').innerText = parts[1];";
  html += "    }";
  html += "  });";
  html += "}, 500);";
  html += "</script>";
  
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// Handlers for Buttons
void handleForward() { moveState = 1; updateMotors(); server.send(200, "text/plain", "OK"); }
void handleBackward() { moveState = 2; updateMotors(); server.send(200, "text/plain", "OK"); }
void handleLeft() { moveState = 3; updateMotors(); server.send(200, "text/plain", "OK"); }
void handleRight() { moveState = 4; updateMotors(); server.send(200, "text/plain", "OK"); }
void handleStop() { moveState = 0; updateMotors(); server.send(200, "text/plain", "OK"); }

// Handler for Slider
void handleSetSpeed() {
  if (server.hasArg("val")) {
    motorSpeed = server.arg("val").toInt();
    updateMotors(); // Apply new speed immediately if robot is moving
  }
  server.send(200, "text/plain", "OK");
}

void handleRPM() {
  // Sending simple CSV format for easier parsing in JS
  String data = String(m1_rpm, 0) + "," + String(m2_rpm, 0) + "," + String(m3_rpm, 0) + "," + String(m4_rpm, 0);
  server.send(200, "text/plain", data);
}

// ================================================================
// 7. SETUP & LOOP
// ================================================================

void setup() {
  Serial.begin(115200);
  
  // Set all Motor pins to Output
  pinMode(M1_RPWM, OUTPUT); pinMode(M1_LPWM, OUTPUT);
  pinMode(M2_RPWM, OUTPUT); pinMode(M2_LPWM, OUTPUT);
  pinMode(M3_RPWM, OUTPUT); pinMode(M3_LPWM, OUTPUT);
  pinMode(M4_RPWM, OUTPUT); pinMode(M4_LPWM, OUTPUT);

  // Set Encoder pins
  pinMode(M1_EN_B, INPUT_PULLUP);
  pinMode(M3_EN_A, INPUT_PULLUP); pinMode(M3_EN_B, INPUT_PULLUP);
  pinMode(M4_EN_B, INPUT_PULLUP);

  // Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(M1_EN_A), isr_m1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_EN_A), isr_m2, RISING);
  attachInterrupt(digitalPinToInterrupt(M3_EN_A), isr_m3, RISING);
  attachInterrupt(digitalPinToInterrupt(M4_EN_A), isr_m4, RISING);

  WiFi.mode(WIFI_AP);               // Force AP mode
  WiFi.softAP(ssid, password); // Name and Password

  // Print the IP Address (It will be 192.168.4.1)
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  server.on("/", handleRoot);
  server.on("/fwd", handleForward);
  server.on("/bwd", handleBackward);
  server.on("/left", handleLeft);
  server.on("/right", handleRight);
  server.on("/stop", handleStop);
  server.on("/setSpeed", handleSetSpeed); // New handler for slider
  server.on("/rpm", handleRPM);

  // === OTA CONFIGURATION START ===
  ArduinoOTA.setHostname("MyRobot-ESP32"); // Name of the port in Arduino IDE
  // ArduinoOTA.setPassword("admin"); // Optional: password for uploading

  ArduinoOTA
    .onStart([]() {
      String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
      // Stop motors immediately when upload starts
      setMotor(M1_RPWM, M1_LPWM, 0); setMotor(M2_RPWM, M2_LPWM, 0);
      setMotor(M3_RPWM, M3_LPWM, 0); setMotor(M4_RPWM, M4_LPWM, 0);
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  // === OTA CONFIGURATION END ===

  server.begin();
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();

  // Calculate RPM
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= RPM_INTERVAL) {
    noInterrupts();
    long t1 = m1_ticks; m1_ticks = 0;
    long t2 = m2_ticks; m2_ticks = 0;
    long t3 = m3_ticks; m3_ticks = 0;
    long t4 = m4_ticks; m4_ticks = 0;
    interrupts();

    float mult = 60000.0 / (PPR * RPM_INTERVAL);
    m1_rpm = t1 * mult; m2_rpm = t2 * mult;
    m3_rpm = t3 * mult; m4_rpm = t4 * mult;
    
    previousMillis = currentMillis;
  }
}

