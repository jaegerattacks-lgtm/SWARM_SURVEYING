#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

const char* ssid = "ESP32-Tuning-Rig";
const char* password = "12345678";

const float COUNTS_PER_REV = 360.0; 

// --- Pin Arrays ---
const int encA[4]   = {35, 36, 33, 34};
const int encB[4]   = {32, 39, 25, 23};
const int rPwm[4]   = {27, 21, 13, 18};
const int lPwm[4]   = {26, 19, 14, 4};

// --- Telemetry Data ---
volatile long pulseCount[4] = {0, 0, 0, 0};
long lastPulseCount[4] = {0, 0, 0, 0};
float rawRpm[4] = {0, 0, 0, 0};
float filteredRpm[4] = {0, 0, 0, 0}; 

// --- Control Variables ---
String currentDir = "stop";
bool maxPowerOverride = false;

// Independent Tuning Parameters
float baseRpm[4] = {50.0, 50.0, 50.0, 50.0}; 
float Kp[4] = {3.5,1.9, 1.7, 2.1};  
float Ki[4] = {1, 1, 1, 1};
float Kd[4] = {0, 0, 0, 0}; 
float alpha = 0.45; 

float targetRpm[4] = {0, 0, 0, 0};
float errorSum[4] = {0, 0, 0, 0}; 
float lastError[4] = {0, 0, 0, 0};      
int pwmOut[4] = {0, 0, 0, 0};     
int struggleCount[4] = {0, 0, 0, 0}; // Tracks how long a wheel fails to reach target

WebServer server(80);
SemaphoreHandle_t stateMutex;

// --- 4x ISRs ---
void IRAM_ATTR isr0() { static int lA=0, lB=0; int cA = digitalRead(encA[0]), cB = digitalRead(encB[0]); if(cA!=lA || cB!=lB) { if((lA==LOW&&cA==HIGH&&cB==LOW)||(lA==HIGH&&cA==LOW&&cB==HIGH)||(lB==LOW&&cB==HIGH&&cA==HIGH)||(lB==HIGH&&cB==LOW&&cA==LOW)) pulseCount[0]++; else pulseCount[0]--; } lA=cA; lB=cB; }
void IRAM_ATTR isr1() { static int lA=0, lB=0; int cA = digitalRead(encA[1]), cB = digitalRead(encB[1]); if(cA!=lA || cB!=lB) { if((lA==LOW&&cA==HIGH&&cB==LOW)||(lA==HIGH&&cA==LOW&&cB==HIGH)||(lB==LOW&&cB==HIGH&&cA==HIGH)||(lB==HIGH&&cB==LOW&&cA==LOW)) pulseCount[1]++; else pulseCount[1]--; } lA=cA; lB=cB; }
void IRAM_ATTR isr2() { static int lA=0, lB=0; int cA = digitalRead(encA[2]), cB = digitalRead(encB[2]); if(cA!=lA || cB!=lB) { if((lA==LOW&&cA==HIGH&&cB==LOW)||(lA==HIGH&&cA==LOW&&cB==HIGH)||(lB==LOW&&cB==HIGH&&cA==HIGH)||(lB==HIGH&&cB==LOW&&cA==LOW)) pulseCount[2]++; else pulseCount[2]--; } lA=cA; lB=cB; }
void IRAM_ATTR isr3() { static int lA=0, lB=0; int cA = digitalRead(encA[3]), cB = digitalRead(encB[3]); if(cA!=lA || cB!=lB) { if((lA==LOW&&cA==HIGH&&cB==LOW)||(lA==HIGH&&cA==LOW&&cB==HIGH)||(lB==LOW&&cB==HIGH&&cA==HIGH)||(lB==HIGH&&cB==LOW&&cA==LOW)) pulseCount[3]++; else pulseCount[3]--; } lA=cA; lB=cB; }

// --- Hardware  ---
void updateMotors() {
  for(int i = 0; i < 4; i++) {
    bool isLeftMotor = (i == 0 || i == 2);
    
    if (currentDir == "fwd" || maxPowerOverride) {
      analogWrite(rPwm[i], pwmOut[i]); analogWrite(lPwm[i], 0);
    } else if (currentDir == "rev") {
      analogWrite(rPwm[i], 0); analogWrite(lPwm[i], pwmOut[i]); 
    } else if (currentDir == "left") {
      if (isLeftMotor) { analogWrite(rPwm[i], 0); analogWrite(lPwm[i], pwmOut[i]); } 
      else             { analogWrite(rPwm[i], pwmOut[i]); analogWrite(lPwm[i], 0); }
    } else if (currentDir == "right") {
      if (isLeftMotor) { analogWrite(rPwm[i], pwmOut[i]); analogWrite(lPwm[i], 0); } 
      else             { analogWrite(rPwm[i], 0); analogWrite(lPwm[i], pwmOut[i]); }
    } else {
      analogWrite(rPwm[i], 0); analogWrite(lPwm[i], 0);
    }
  }
}

// --- HTML Dashboard ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta name='viewport' content='width=device-width, initial-scale=1'>
<style>
  body { font-family: sans-serif; background: #222; color: #fff; text-align: center; margin: 0; padding: 10px;}
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; max-width: 900px; margin: auto; }
  .card { background: #333; padding: 10px; border-radius: 8px; box-shadow: 0 2px 5px rgba(0,0,0,0.5); }
  .controls { margin-bottom: 20px; }
  .btn { background: #555; color: white; border: none; padding: 10px 15px; font-size: 14px; border-radius: 5px; cursor: pointer; margin: 2px; transition: background 0.3s;}
  .btn-stop { background: #dc3545; font-weight: bold;}
  .btn-danger { background: #ffc107; color: black; font-weight: bold; width: 100%; padding: 15px; margin-top: 10px;}
  input { width: 50px; padding: 5px; margin: 2px; text-align: center; background: #444; color: #fff; border: 1px solid #666; border-radius: 3px;}
  canvas { background: #111; width: 100%; height: 120px; border-radius: 4px; margin-top: 10px;}
  .metrics { font-size: 0.9em; color: #aaa; }
  .val-f { color: #00aaff; font-weight: bold;}
  .val-r { color: #888; }
</style>
</head><body>
  <h2>Robot Tuning Studio</h2>
  
  <div class="controls">
    <button class="btn" onclick="sendCmd('/dir?state=fwd')">FWD</button><br>
    <button class="btn" onclick="sendCmd('/dir?state=left')">LEFT</button>
    <button class="btn btn-stop" onclick="sendCmd('/dir?state=stop')">STOP</button>
    <button class="btn" onclick="sendCmd('/dir?state=right')">RIGHT</button><br>
    <button class="btn" onclick="sendCmd('/dir?state=rev')">REV</button><br>
    <button class="btn btn-danger" onmousedown="sendCmd('/maxPower?state=1')" onmouseup="sendCmd('/maxPower?state=0')" ontouchstart="sendCmd('/maxPower?state=1')" ontouchend="sendCmd('/maxPower?state=0')">HOLD FOR MAX PWM (Bypass PID)</button>
    
    <div style="margin-top:15px;">
      <label>Global Alpha (Filter): </label>
      <input type="number" step="0.05" id="g_alpha"> 
      <button class="btn" id="btn_alpha" onclick="updateAlpha()">Set Alpha</button>
    </div>
  </div>

  <div class="grid" id="motors"></div>

  <script>
    const labels = ["FL", "FR", "RL", "RR"];
    let history = [[], [], [], []];
    const MAX_PTS = 50;
    
    let initialLoad = true; 

    let html = "";
    for(let i=0; i<4; i++) {
      html += `<div class="card">
        <h3>${labels[i]} Motor</h3>
        Base Tgt: <input type="number" id="t${i}">
        Kp: <input type="number" step="0.1" id="p${i}">
        Ki: <input type="number" step="0.1" id="i${i}">
        Kd: <input type="number" step="0.01" id="d${i}">
        <button class="btn" id="btn_apply_${i}" onclick="updateM(${i})">Apply</button>
        <div class="metrics">Filt: <span class="val-f" id="f${i}">0</span> | Raw: <span class="val-r" id="r${i}">0</span> | PWM: <span id="pwm${i}">0</span></div>
        <canvas id="c${i}" width="400" height="120"></canvas>
      </div>`;
    }
    document.getElementById('motors').innerHTML = html;

    function sendCmd(url) { 
      let sep = url.includes('?') ? '&' : '?';
      fetch(url + sep + '_cb=' + Date.now()); 
    }

    function updateM(i) {
      let btn = document.getElementById('btn_apply_'+i);
      btn.innerText = "OK!";
      btn.style.background = "#28a745"; 
      
      let t = document.getElementById('t'+i).value;
      let p = document.getElementById('p'+i).value;
      let ki = document.getElementById('i'+i).value;
      let kd = document.getElementById('d'+i).value;
      
      fetch(`/tune?m=${i}&t=${t}&p=${p}&i=${ki}&d=${kd}&_cb=${Date.now()}`).then(() => {
        setTimeout(() => {
          btn.innerText = "Apply";
          btn.style.background = "#555";
        }, 1000);
      });
    }

    function updateAlpha() {
      let btn = document.getElementById('btn_alpha');
      btn.innerText = "OK!";
      btn.style.background = "#28a745"; 
      
      let val = document.getElementById('g_alpha').value;
      
      fetch(`/alpha?val=${val}&_cb=${Date.now()}`).then(() => {
        setTimeout(() => {
          btn.innerText = "Set Alpha";
          btn.style.background = "#555";
        }, 1000);
      });
    }

    function drawChart(i, target, filt, raw) {
      let cvs = document.getElementById('c'+i);
      let ctx = cvs.getContext('2d');
      history[i].push({t: target, f: filt, r: raw});
      if(history[i].length > MAX_PTS) history[i].shift();

      ctx.clearRect(0, 0, cvs.width, cvs.height);
      let maxH = 150; 
      let dx = cvs.width / MAX_PTS;

      function plot(key, color, width) {
        ctx.beginPath();
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        for(let j=0; j<history[i].length; j++) {
          let x = j * dx;
          let y = cvs.height - ((history[i][j][key] / maxH) * cvs.height);
          if(j===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
        }
        ctx.stroke();
      }
      plot('r', 'rgba(255,255,255,0.2)', 1); 
      plot('t', '#ff4444', 1);               
      plot('f', '#00aaff', 2);               
    }

    function pollData() {
      fetch('/data?_cb=' + Date.now())
        .then(r => r.json())
        .then(d => {
          if(initialLoad) {
            document.getElementById('g_alpha').value = d.alpha;
          }

          for(let i=0; i<4; i++) {
            let m = d.m[i];
            
            document.getElementById('f'+i).innerText = m.f.toFixed(1);
            document.getElementById('r'+i).innerText = m.r.toFixed(1);
            document.getElementById('pwm'+i).innerText = m.pwm;
            drawChart(i, m.tgt, m.f, m.r); 
            
            if(initialLoad) {
              document.getElementById('t'+i).value = m.base; 
              document.getElementById('p'+i).value = m.p;
              document.getElementById('i'+i).value = m.i;
              document.getElementById('d'+i).value = m.d;
            }
          }
          
          initialLoad = false; 
          setTimeout(pollData, 150); 
        })
        .catch(e => {
          setTimeout(pollData, 500); 
        });
    }

    pollData();
  </script>
</body></html>
)rawliteral";

// --- FreeRTOS Tasks ---

void controlLoopTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100); 
  const float timeInMinutes = 100.0 / 60000.0;      

  while(true) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    noInterrupts();
    long currentPulses[4];
    for(int i=0; i<4; i++) currentPulses[i] = pulseCount[i];
    interrupts();

    if (xSemaphoreTake(stateMutex, portMAX_DELAY)) {
      
      // 1. Calculate Actual RPM
      for(int i = 0; i < 4; i++) {
        long deltaPulses = currentPulses[i] - lastPulseCount[i];
        rawRpm[i] = abs(((float)deltaPulses / COUNTS_PER_REV) / timeInMinutes); 
        filteredRpm[i] = (alpha * rawRpm[i]) + ((1.0 - alpha) * filteredRpm[i]);
        lastPulseCount[i] = currentPulses[i];
      }

      // 2. Control Logic
      if (maxPowerOverride) {
        for(int i=0; i<4; i++) {
          pwmOut[i] = 255;
          errorSum[i] = 0; 
          lastError[i] = 0;
          targetRpm[i] = baseRpm[i]; 
        }
      } else {
        
        // --- THE PROPORTIONAL GOVERNOR (WITH TIMEOUT) ---
        float targetRatio = 1.0; 
        
        for(int i = 0; i < 4; i++) {
          // If a wheel is maxed out and missing its target by more than 5%
          if (baseRpm[i] > 5.0 && pwmOut[i] >= 250 && filteredRpm[i] < (baseRpm[i] * 0.95)) {
            struggleCount[i]++;
            if (struggleCount[i] > 20) struggleCount[i] = 20; // Cap it
          } else {
            if (struggleCount[i] > 0) struggleCount[i]--; 
          }

          // ONLY apply the penalty if it has been struggling for 5 loops (500 milliseconds)
          if (struggleCount[i] >= 5) {
            float ratio = filteredRpm[i] / baseRpm[i];
            if (ratio < targetRatio) targetRatio = ratio; 
          }
        }

        // Apply a low-pass filter to the ratio itself so the target lines glide smoothly
        static float smoothedRatio = 1.0;
        if (currentDir == "stop") smoothedRatio = 1.0; // Reset completely on stop
        else smoothedRatio = (0.2 * targetRatio) + (0.8 * smoothedRatio);

        // --- PID CALCULATION ---
        for(int i = 0; i < 4; i++) {
          if (currentDir == "stop" || baseRpm[i] == 0) {
            pwmOut[i] = 0; errorSum[i] = 0; lastError[i] = 0; struggleCount[i] = 0;
            targetRpm[i] = 0; 
          } else {
            
            // Multiply the target by our smooth, delayed governor ratio
            targetRpm[i] = baseRpm[i] * smoothedRatio; 
            
            float error = targetRpm[i] - filteredRpm[i];
            errorSum[i] += error;
            
            float iLimit = (Ki[i] > 0) ? (255.0 / Ki[i]) : 0;
            if (errorSum[i] > iLimit) errorSum[i] = iLimit;
            if (errorSum[i] < -iLimit) errorSum[i] = -iLimit;

            float dError = error - lastError[i];
            lastError[i] = error;
            
            float output = (Kp[i] * error) + (Ki[i] * errorSum[i]) + (Kd[i] * dError);
            
            if (output > 255) output = 255;
            if (output < 0) output = 0;
            pwmOut[i] = (int)output;
          }
        }
      }
      
      updateMotors();
      xSemaphoreGive(stateMutex);
    }
  }
}

void serverTask(void *pvParameters) {
  while(true) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

void setup() {
  Serial.begin(115200);
  stateMutex = xSemaphoreCreateMutex();

  for(int i = 0; i < 4; i++) {
    pinMode(encA[i], INPUT_PULLUP); pinMode(encB[i], INPUT_PULLUP);
    pinMode(rPwm[i], OUTPUT); pinMode(lPwm[i], OUTPUT);
  }
  updateMotors();

  attachInterrupt(digitalPinToInterrupt(encA[0]), isr0, CHANGE); attachInterrupt(digitalPinToInterrupt(encB[0]), isr0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[1]), isr1, CHANGE); attachInterrupt(digitalPinToInterrupt(encB[1]), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[2]), isr2, CHANGE); attachInterrupt(digitalPinToInterrupt(encB[2]), isr2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encA[3]), isr3, CHANGE); attachInterrupt(digitalPinToInterrupt(encB[3]), isr3, CHANGE);

  WiFi.softAP(ssid, password);

  server.on("/", []() { server.send_P(200, "text/html", index_html); });

  server.on("/data", []() {
    String json = "";
    if (xSemaphoreTake(stateMutex, portMAX_DELAY)) {
      json = "{\"alpha\":" + String(alpha) + ",\"m\":[";
      for(int i = 0; i < 4; i++) {
        json += "{\"f\":" + String(filteredRpm[i]) + ",\"r\":" + String(rawRpm[i]);
        json += ",\"tgt\":" + String(targetRpm[i]) + ",\"base\":" + String(baseRpm[i]) + ",\"pwm\":" + String(pwmOut[i]);
        json += ",\"p\":" + String(Kp[i]) + ",\"i\":" + String(Ki[i]) + ",\"d\":" + String(Kd[i]) + "}";
        if(i < 3) json += ",";
      }
      json += "]}";
      xSemaphoreGive(stateMutex);
    }
    server.send(200, "application/json", json);
  });

  server.on("/dir", []() {
    if (server.hasArg("state") && xSemaphoreTake(stateMutex, portMAX_DELAY)) {
      currentDir = server.arg("state"); 
      xSemaphoreGive(stateMutex);
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/maxPower", []() {
    if (server.hasArg("state") && xSemaphoreTake(stateMutex, portMAX_DELAY)) {
      maxPowerOverride = (server.arg("state") == "1");
      xSemaphoreGive(stateMutex);
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/tune", []() {
    if (server.hasArg("m") && xSemaphoreTake(stateMutex, portMAX_DELAY)) {
      int m = server.arg("m").toInt();
      if(m >= 0 && m < 4) {
        if(server.hasArg("t")) baseRpm[m] = server.arg("t").toFloat();
        if(server.hasArg("p")) Kp[m] = server.arg("p").toFloat();
        if(server.hasArg("i")) Ki[m] = server.arg("i").toFloat();
        if(server.hasArg("d")) Kd[m] = server.arg("d").toFloat();
      }
      xSemaphoreGive(stateMutex);
    }
    server.send(200, "text/plain", "OK");
  });

  server.on("/alpha", []() {
    if (server.hasArg("val") && xSemaphoreTake(stateMutex, portMAX_DELAY)) {
      alpha = server.arg("val").toFloat();
      xSemaphoreGive(stateMutex);
    }
    server.send(200, "text/plain", "OK");
  });

  server.begin();

  xTaskCreatePinnedToCore(controlLoopTask, "CtrlTask", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(serverTask, "WebTask", 4096, NULL, 1, NULL, 0);
}

void loop() { vTaskDelete(NULL); }