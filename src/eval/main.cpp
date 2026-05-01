#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <SimpleKalmanFilter.h>
#include "ICM_20948.h"

namespace {

constexpr int I2C_SCL = 33;
constexpr int I2C_SDA = 32;
constexpr int AD0_VAL = 0;
constexpr int BUTTON_PIN = 0;

constexpr char WIFI_SSID[] = "Atias";
constexpr char WIFI_PASS[] = "Good1010";
constexpr char AP_SSID[] = "UGV-EVAL";
constexpr char AP_PASS[] = "12345678";

constexpr unsigned long LOOP_PERIOD_US = 20000UL; // 50Hz
constexpr float MG_TO_MPS2 = 0.00980665f;

constexpr float STATIONARY_GYRO_DPS = 0.8f;   // was 2.0 - tighter: slow rotation no longer masks motion
constexpr float STATIONARY_ACC_MPS2 = 0.08f;  // was 0.25 - tighter: slow movement no longer triggers ZUPT
constexpr float ACC_DEADBAND_MPS2 = 0.03f;
constexpr float BIAS_TRACK_LIMIT_MPS2 = 0.35f;

struct AxisEstimator {
  float pos;
  float vel;
  float bias;
  float acc;
  uint16_t stationarySamples;

  AxisEstimator()
      : pos(0.0f),
        vel(0.0f),
        bias(0.0f),
        acc(0.0f),
        stationarySamples(0) {}

  void reset() {
    pos = 0.0f;
    vel = 0.0f;
    bias = 0.0f;
    acc = 0.0f;
    stationarySamples = 0;
  }

  void update(float acc_mps2, float dt, bool stationary) {
    if (stationary) {
      stationarySamples++;
    } else {
      stationarySamples = 0;
    }

    // Hard ZUPT: after 2 consecutive stationary samples (20 ms) freeze
    // velocity and position entirely to prevent overshoot accumulation.
    if (stationarySamples >= 2) {
      if (stationary && fabsf(acc_mps2 - bias) < BIAS_TRACK_LIMIT_MPS2) {
        bias = (0.99f * bias) + (0.01f * acc_mps2);
      }
      vel = 0.0f;
      acc = 0.0f;
      return; // position not updated while stationary
    }

    acc = acc_mps2 - bias;
    if (fabsf(acc) < ACC_DEADBAND_MPS2) {
      acc = 0.0f;
    }

    vel += acc * dt;
    vel *= 0.999f;

    if (fabsf(vel) < 0.002f && fabsf(acc) < ACC_DEADBAND_MPS2) {
      vel = 0.0f;
    }

    pos += vel * dt;
  }
};

ICM_20948_I2C myICM;
AxisEstimator axisX;
AxisEstimator axisY;
AxisEstimator axisZ;
SimpleKalmanFilter accKalmanX(0.15f, 0.05f, 0.12f);
SimpleKalmanFilter accKalmanY(0.15f, 0.05f, 0.12f);
SimpleKalmanFilter accKalmanZ(0.15f, 0.05f, 0.12f);
WebServer server(80);

float gravityX = 0.0f;
float gravityY = 0.0f;
float gravityZ = 9.80665f;

float rawAccX = 0.0f, rawAccY = 0.0f, rawAccZ = 0.0f; // mg
float rawGyrX = 0.0f, rawGyrY = 0.0f, rawGyrZ = 0.0f; // dps
float rawMagX = 0.0f, rawMagY = 0.0f, rawMagZ = 0.0f; // uT

unsigned long loopTickerUs = 0;
unsigned long hzWindowStartUs = 0;
uint32_t hzWindowCount = 0;
float evalLoopHz = 0.0f;

bool lastButtonState = HIGH;

float getPlanarDistanceCm() {
  const float dxy = sqrtf((axisX.pos * axisX.pos) + (axisY.pos * axisY.pos));
  return dxy * 100.0f;
}

float getDistanceCm() {
  const float dxy = sqrtf((axisX.pos * axisX.pos) + (axisY.pos * axisY.pos));
  const float dxyz = sqrtf((dxy * dxy) + (axisZ.pos * axisZ.pos));
  return dxyz * 100.0f;
}

void resetIntegration() {
  axisX.reset();
  axisY.reset();
  axisZ.reset();
}

String buildDistanceJson(const char *source) {
  const float dxy = sqrtf((axisX.pos * axisX.pos) + (axisY.pos * axisY.pos));
  const float dxyz = sqrtf((dxy * dxy) + (axisZ.pos * axisZ.pos));
  const float x_cm = axisX.pos * 100.0f;
  const float y_cm = axisY.pos * 100.0f;
  const float z_cm = axisZ.pos * 100.0f;
  const float dxy_cm = dxy * 100.0f;
  const float dxyz_cm = dxyz * 100.0f;

  String out;
  out.reserve(320);
  out += "{\"type\":\"distance\",\"src\":\"";
  out += source;
  out += "\",\"x\":";
  out += String(axisX.pos, 4);
  out += ",\"y\":";
  out += String(axisY.pos, 4);
  out += ",\"z\":";
  out += String(axisZ.pos, 4);
  out += ",\"dxy\":";
  out += String(dxy, 4);
  out += ",\"dxyz\":";
  out += String(dxyz, 4);
  out += ",\"x_cm\":";
  out += String(x_cm, 2);
  out += ",\"y_cm\":";
  out += String(y_cm, 2);
  out += ",\"z_cm\":";
  out += String(z_cm, 2);
  out += ",\"dxy_cm\":";
  out += String(dxy_cm, 2);
  out += ",\"dxyz_cm\":";
  out += String(dxyz_cm, 2);
  out += ",\"hz\":";
  out += String(evalLoopHz, 2);
  out += "}";
  return out;
}

void printDistance(const char *source) {
  Serial.println(buildDistanceJson(source));
}

void calibrateGravity() {
  constexpr int samples = 300;
  float sx = 0.0f;
  float sy = 0.0f;
  float sz = 0.0f;

  Serial.println("Keep robot still: calibrating gravity...");
  for (int i = 0; i < samples; i++) {
    myICM.getAGMT();
    if (myICM.status == ICM_20948_Stat_Ok) {
      sx += myICM.accX() * MG_TO_MPS2;
      sy += myICM.accY() * MG_TO_MPS2;
      sz += myICM.accZ() * MG_TO_MPS2;
    }
    delay(5);
  }

  gravityX = sx / samples;
  gravityY = sy / samples;
  gravityZ = sz / samples;

  Serial.print("Gravity baseline [m/s^2]: ");
  Serial.print(gravityX, 4);
  Serial.print(", ");
  Serial.print(gravityY, 4);
  Serial.print(", ");
  Serial.println(gravityZ, 4);
}

void handleSerialCommands() {
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == 'd' || c == 'D') {
      printDistance("serial");
    } else if (c == 'r' || c == 'R') {
      resetIntegration();
      Serial.println("{\"type\":\"reset\",\"ok\":1}");
    } else if (c == 'g' || c == 'G') {
      calibrateGravity();
      resetIntegration();
      Serial.println("{\"type\":\"gravity\",\"ok\":1}");
    } else if (c == 'h' || c == 'H') {
      Serial.println("Commands: D=distance, R=reset origin, G=recalibrate gravity");
    }
  }
}

void updateButtonRequest() {
  const bool pressed = digitalRead(BUTTON_PIN) == LOW;
  if (pressed && !lastButtonState) {
    printDistance("button");
  }
  lastButtonState = pressed;
}

bool initImu() {
  bool initialized = false;

  for (int attempt = 0; attempt < 20 && !initialized; attempt++) {
    myICM.begin(Wire, AD0_VAL);
    initialized = (myICM.status == ICM_20948_Stat_Ok);
    if (!initialized) {
      delay(100);
    }
  }

  return initialized;
}

void handleRoot() {
  const char html[] = R"HTML(
<!doctype html>
<html>
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width,initial-scale=1" />
    <title>UGV Eval Distance</title>
    <style>
      body { font-family: Helvetica, sans-serif; background: #f2f4f8; margin: 0; padding: 24px; }
      .card { max-width: 560px; margin: 0 auto; background: #fff; border-radius: 12px; padding: 20px; box-shadow: 0 8px 24px rgba(0,0,0,0.08); }
      h2 { margin-top: 0; }
      .row { display: flex; gap: 8px; flex-wrap: wrap; }
      button { border: 0; border-radius: 10px; padding: 10px 14px; color: #fff; cursor: pointer; font-weight: 600; }
      #btnDistance { background: #007a5a; }
      #btnReset { background: #1d4ed8; }
      #btnGravity { background: #d97706; }
      pre { background: #111827; color: #e5e7eb; border-radius: 10px; padding: 12px; overflow: auto; min-height: 120px; }
      small { color: #334155; }
    </style>
  </head>
  <body>
    <div class="card">
      <h2>Distance Evaluation</h2>
      <div class="row">
        <button id="btnDistance">Read Distance</button>
        <button id="btnReset">Reset Origin</button>
        <button id="btnGravity">Recalibrate Gravity</button>
      </div>
      <p><small>The core integration loop runs at 100Hz. Use buttons only for on-demand read/reset.</small></p>
      <p><small>Live normalized distance from origin:</small></p>
      <pre id="distanceCm">0.00 cm</pre>
      <p><small>Distance response:</small></p>
      <pre id="out">waiting...</pre>
      <p><small>Raw IMU data (auto-refresh):</small></p>
      <pre id="imu" style="font-family:monospace;white-space:pre;">waiting...</pre>
    </div>
    <script>
      const out = document.getElementById('out');
      const distanceCm = document.getElementById('distanceCm');
      const imu = document.getElementById('imu');
      async function hit(path) {
        const r = await fetch(path);
        const t = await r.text();
        out.textContent = t;
      }
      async function pollDistanceCm() {
        const r = await fetch('/status');
        const t = await r.text();
        try {
          const json = JSON.parse(t);
          if (typeof json.dxyz_cm === 'number') {
            distanceCm.textContent = json.dxyz_cm.toFixed(2) + ' cm';
          }
        } catch (error) {
        }
      }
      async function pollImu() {
        const r = await fetch('/imu');
        imu.textContent = await r.text();
      }
      document.getElementById('btnDistance').onclick = () => hit('/distance');
      document.getElementById('btnReset').onclick = () => hit('/reset');
      document.getElementById('btnGravity').onclick = () => hit('/gravity');
      pollDistanceCm();
      pollImu();
      setInterval(pollDistanceCm, 1000);
      setInterval(pollImu, 1000);
    </script>
  </body>
</html>
)HTML";

  server.send(200, "text/html", html);
}

void handleDistance() {
  server.send(200, "application/json", buildDistanceJson("web"));
}

void handleReset() {
  resetIntegration();
  server.send(200, "application/json", "{\"type\":\"reset\",\"ok\":1}");
}

void handleGravity() {
  calibrateGravity();
  resetIntegration();
  server.send(200, "application/json", "{\"type\":\"gravity\",\"ok\":1}");
}

void handleStatus() {
  const float dxy_cm = getPlanarDistanceCm();
  const float dxyz_cm = getDistanceCm();
  String out;
  out.reserve(180);
  out += "{\"type\":\"status\",\"hz\":";
  out += String(evalLoopHz, 2);
  out += ",\"dxy_cm\":";
  out += String(dxy_cm, 2);
  out += ",\"dxyz_cm\":";
  out += String(dxyz_cm, 2);
  out += ",\"vx\":";
  out += String(axisX.vel, 4);
  out += ",\"vy\":";
  out += String(axisY.vel, 4);
  out += ",\"vz\":";
  out += String(axisZ.vel, 4);
  out += "}";
  server.send(200, "application/json", out);
}

void handleImu() {
  char buf[220];
  snprintf(buf, sizeof(buf),
    "Accel (mg)  X=%8.2f  Y=%8.2f  Z=%8.2f\n"
    "Gyro  (dps) X=%8.3f  Y=%8.3f  Z=%8.3f\n"
    "Mag   (uT)  X=%8.2f  Y=%8.2f  Z=%8.2f",
    rawAccX, rawAccY, rawAccZ,
    rawGyrX, rawGyrY, rawGyrZ,
    rawMagX, rawMagY, rawMagZ);
  server.send(200, "text/plain", buf);
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/distance", handleDistance);
  server.on("/reset", handleReset);
  server.on("/gravity", handleGravity);
  server.on("/status", handleStatus);
  server.on("/imu", handleImu);
  server.begin();
}

void setupNetwork() {
  WiFi.mode(WIFI_STA);

  if (strlen(WIFI_SSID) > 0) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long waitStart = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - waitStart) < 10000UL) {
      delay(200);
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("AP mode. SSID: ");
    Serial.print(AP_SSID);
    Serial.print(" IP: ");
    Serial.println(WiFi.softAPIP());
  }

  setupWebServer();

  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.setHostname("ugv-eval");
    ArduinoOTA.onStart([]() {
      Serial.println("OTA start");
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("OTA done");
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.print("OTA error: ");
      Serial.println((int)error);
    });
    ArduinoOTA.begin();
    Serial.println("OTA ready");
  }
}

} // namespace

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Wire.begin(I2C_SDA, I2C_SCL, 400000);

  if (!initImu()) {
    Serial.println("IMU init failed.");
    while (true) {
      delay(1000);
    }
  }

  calibrateGravity();
  resetIntegration();

  setupNetwork();

  loopTickerUs = micros();
  hzWindowStartUs = loopTickerUs;

  Serial.println("Evaluation firmware ready. Loop target: 100Hz");
  Serial.println("Press BOOT button or send D in serial for distance.");
}

void loop() {
  server.handleClient();
  ArduinoOTA.handle();
  handleSerialCommands();
  updateButtonRequest();

  const unsigned long nowUs = micros();
  if (static_cast<unsigned long>(nowUs - loopTickerUs) < LOOP_PERIOD_US) {
    return;
  }

  const float dt = static_cast<float>(nowUs - loopTickerUs) / 1000000.0f;
  loopTickerUs += LOOP_PERIOD_US;

  // Recover quickly if execution falls behind significantly.
  if (static_cast<unsigned long>(nowUs - loopTickerUs) > (5 * LOOP_PERIOD_US)) {
    loopTickerUs = nowUs;
  }

  myICM.getAGMT();
  if (myICM.status != ICM_20948_Stat_Ok) {
    return;
  }

  rawAccX = myICM.accX();
  rawAccY = myICM.accY();
  rawAccZ = myICM.accZ();
  rawGyrX = myICM.gyrX();
  rawGyrY = myICM.gyrY();
  rawGyrZ = myICM.gyrZ();
  rawMagX = myICM.magX();
  rawMagY = myICM.magY();
  rawMagZ = myICM.magZ();

  const float ax = (myICM.accX() * MG_TO_MPS2) - gravityX;
  const float ay = (myICM.accY() * MG_TO_MPS2) - gravityY;
  const float az = (myICM.accZ() * MG_TO_MPS2) - gravityZ;

  const float gx = myICM.gyrX();
  const float gy = myICM.gyrY();
  const float gz = myICM.gyrZ();

  const float gyroMag = sqrtf((gx * gx) + (gy * gy) + (gz * gz));
  const float accMag = sqrtf((ax * ax) + (ay * ay) + (az * az));
  const bool stationary = (gyroMag < STATIONARY_GYRO_DPS) && (accMag < STATIONARY_ACC_MPS2);

  // When stationary feed 0 into Kalman so its internal state converges
  // to zero immediately rather than decaying slowly from the last motion
  // value — this eliminates the post-stop velocity tail (overshoot).
  const float axIn = stationary ? 0.0f : ax;
  const float ayIn = stationary ? 0.0f : ay;
  const float azIn = stationary ? 0.0f : az;

  const float axFiltered = accKalmanX.updateEstimate(axIn);
  const float ayFiltered = accKalmanY.updateEstimate(ayIn);
  const float azFiltered = accKalmanZ.updateEstimate(azIn);

  axisX.update(axFiltered, dt, stationary);
  axisY.update(ayFiltered, dt, stationary);
  axisZ.update(azFiltered, dt, stationary);

  hzWindowCount++;
  if (nowUs - hzWindowStartUs >= 1000000UL) {
    const float dxy_cm = getPlanarDistanceCm();
    const float dxyz_cm = getDistanceCm();
    evalLoopHz = (static_cast<float>(hzWindowCount) * 1000000.0f) /
                 static_cast<float>(nowUs - hzWindowStartUs);
    hzWindowCount = 0;
    hzWindowStartUs = nowUs;

    Serial.print("{\"type\":\"status\",\"hz\":");
    Serial.print(evalLoopHz, 2);
    Serial.print(",\"dxy_cm\":");
    Serial.print(dxy_cm, 2);
    Serial.print(",\"dxyz_cm\":");
    Serial.print(dxyz_cm, 2);
    Serial.print(",\"vx\":");
    Serial.print(axisX.vel, 4);
    Serial.print(",\"vy\":");
    Serial.print(axisY.vel, 4);
    Serial.print(",\"vz\":");
    Serial.print(axisZ.vel, 4);
    Serial.println("}");
  }
}
