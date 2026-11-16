/*
  NEXUS WEAR - Direct I2C MPU9250/6500 3D Smooth Motion Dashboard
  Board    : ESP32-C3 Dev Module (SDA: 8, SCL: 9)
  Method   : Standalone Direct I2C + Complementary Filter (Zero library errors)
*/

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Wire.h>

// ---------------- USER SETTINGS ----------------
#define I2C_SDA      8
#define I2C_SCL      9
#define MPU_ADDR     0x68     // I2C address of MPU9250/6500
#define AP_SSID      "NEXUS-WEAR"
// -----------------------------------------------

const IPAddress apIP(192, 168, 4, 1);
const byte DNS_PORT = 53;

DNSServer dns;
WebServer server(80);

bool okImu = false;
float roll = 0, pitch = 0, yaw = 0;
unsigned long lastRead = 0;
unsigned long lastTime = 0;

// ---------------- WEB PAGE (3D RENDERER) ----------------
const char PAGE[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>NEXUS 3D MOTION</title>
<style>
:root{--c:#00f0ff;--m:#ff2bd6;--bg:#04070d;--card:#0a1220}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:#cfe9ff;font-family:"Courier New",monospace;min-height:100vh;
 display:flex;flex-direction:column;align-items:center;justify-content:center;padding:12px}
h1{font-size:18px;color:var(--c);text-shadow:0 0 10px var(--c);margin-bottom:6px;letter-spacing:3px}
.sub{font-size:11px;color:#6f93b3;margin-bottom:20px}
.card{background:var(--card);border:1px solid rgba(0,240,255,.35);padding:20px;border-radius:8px;
 box-shadow:0 0 20px rgba(0,240,255,.15);width:100%;max-width:350px;text-align:center}
.stage{height:220px;perspective:600px;display:flex;align-items:center;justify-content:center;margin-bottom:15px}
#dev{position:relative;width:160px;height:100px;transform-style:preserve-3d;transition:transform 0.05s linear}
#dev div{position:absolute;display:flex;align-items:center;justify-content:center;font-size:11px;font-weight:bold;
 border:1px solid var(--c);background:rgba(0,240,255,.15);color:var(--c)}
.fr{width:160px;height:100px;transform:translateZ(15px)}
.bk{width:160px;height:100px;transform:rotateY(180deg) translateZ(15px)}
.lf{width:30px;height:100px;left:65px;transform:rotateY(-90deg) translateZ(80px)}
.rt{width:30px;height:100px;left:65px;transform:rotateY(90deg) translateZ(80px)}
.tp{width:160px;height:30px;top:35px;transform:rotateX(90deg) translateZ(50px)}
.bt{width:160px;height:30px;top:35px;transform:rotateX(-90deg) translateZ(50px)}
.fr{background:rgba(255,43,214,.2)!important;border-color:var(--m)!important;color:var(--m)!important}
.tri{display:flex;justify-content:space-around;margin-top:10px;text-align:center}
.tri div{font-size:10px;color:#6f93b3}.tri b{display:block;font-size:16px;color:#fff}
.btn{margin-top:15px;background:transparent;border:1px solid var(--c);color:var(--c);padding:8px 16px;
 font-family:inherit;font-size:12px;letter-spacing:1px;cursor:pointer;width:100%}
.btn:active{background:rgba(0,240,255,.25)}
</style></head><body>
<h1>NEXUS//3D MOTION</h1>
<div class="sub">STANDALONE MPU TELEMETRY</div>

<div class="card">
 <div class="stage"><div id="dev">
  <div class="fr">FRONT</div><div class="bk">REAR</div><div class="lf"></div><div class="rt"></div><div class="tp"></div><div class="bt"></div>
 </div></div>
 <div class="tri">
  <div>ROLL<b id="ro">0&deg;</b></div>
  <div>PITCH<b id="pi">0&deg;</b></div>
  <div>YAW<b id="ya">0&deg;</b></div>
 </div>
 <button class="btn" onclick="zero()">ZERO CALIBRATION</button>
</div>

<script>
const $=id=>document.getElementById(id);
let rOffset=0, pOffset=0, yOffset=0;

async function tick(){
 try{
  const res=await fetch('/data',{cache:'no-store'});
  const d=await res.json();
  if(d.ok){
   let r = d.roll - rOffset;
   let p = d.pitch - pOffset;
   let y = d.yaw - yOffset;
   
   $('dev').style.transform = `rotateX(${-p}deg) rotateZ(${r}deg) rotateY(${y}deg)`;
   $('ro').textContent = Math.round(r)+'\u00b0';
   $('pi').textContent = Math.round(p)+'\u00b0';
   $('ya').textContent = Math.round(y)+'\u00b0';
  }
 }catch(e){}
 setTimeout(tick, 40);
}

async function zero(){
 const res=await fetch('/zero',{cache:'no-store'});
 const d=await res.json();
 rOffset=d.roll; pOffset=d.pitch; yOffset=d.yaw;
}

tick();
</script></body></html>
)rawliteral";

// ---------------- DIRECT MPU REGISTERS ----------------
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void readSensors() {
  if (!okImu) return;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Starting register for Accel readings
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)14);

  if (Wire.available() >= 14) {
    int16_t ax_raw = (Wire.read() << 8) | Wire.read();
    int16_t ay_raw = (Wire.read() << 8) | Wire.read();
    int16_t az_raw = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Skip temperature
    int16_t gx_raw = (Wire.read() << 8) | Wire.read();
    int16_t gy_raw = (Wire.read() << 8) | Wire.read();
    int16_t gz_raw = (Wire.read() << 8) | Wire.read();

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    if (dt <= 0 || dt > 0.1) dt = 0.01;

    // Convert raw values (Scale: Accel +/- 2g, Gyro +/- 250 deg/s)
    float ax = ax_raw / 16384.0;
    float ay = ay_raw / 16384.0;
    float az = az_raw / 16384.0;

    float gx = gx_raw / 131.0;
    float gy = gy_raw / 131.0;
    float gz = gz_raw / 131.0;

    // Calculate angle from accelerometer
    float accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
    float accRoll  = atan2(ay, az) * 180.0 / PI;

    // Complementary Filter for smooth, jitter-free 3D motion
    pitch = 0.98 * (pitch + gx * dt) + 0.02 * accPitch;
    roll  = 0.98 * (roll  + gy * dt) + 0.02 * accRoll;
    yaw   = yaw + gz * dt;
  }
}

// ---------------- HANDLERS ----------------
void sendPage() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", PAGE);
}

void sendData() {
  char buf[120];
  snprintf(buf, sizeof(buf), "{\"ok\":true,\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}", roll, pitch, yaw);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

void redirectToPortal() {
  server.sendHeader("Location", String("http://") + apIP.toString() + "/", true);
  server.send(302, "text/plain", "");
}

// ---------------- SETUP & LOOP ----------------
float r_zero = 0, p_zero = 0, y_zero = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  delay(200);

  // Wake up MPU6500/9250
  writeReg(0x6B, 0x00); 
  delay(100);

  // Check connection
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() == 0) {
    okImu = true;
    Serial.println("MPU connected successfully via Direct I2C!");
  } else {
    Serial.println("MPU connection failed!");
  }

  lastTime = millis();

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID);

  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(DNS_PORT, "*", apIP);

  server.on("/", sendPage);
  server.on("/data", sendData);
  server.on("/zero", []() {
    r_zero = roll; p_zero = pitch; y_zero = yaw;
    char b[64];
    snprintf(b, sizeof(b), "{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f}", r_zero, p_zero, y_zero);
    server.send(200, "application/json", b);
  });

  server.onNotFound(redirectToPortal);
  server.begin();
}

void loop() {
  dns.processNextRequest();
  server.handleClient();
  
  if (millis() - lastRead >= 25) {
    lastRead = millis();
    readSensors();
  }
}
