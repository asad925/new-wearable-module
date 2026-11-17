/*
  NEXUS WEAR - MPU9250/6500 3D Motion + Linear Acceleration Dashboard
  Board   : ESP32-C3 Dev Module (SDA: 8, SCL: 9)  -> enable "USB CDC On Boot"
  Method  : Direct I2C (no library), complementary filter, gravity removal
  WiFi    : personal hotspot (AP mode), open network "NEXUS-WEAR" (no password) + captive portal
  Page    : auto-opens after connecting, or go to http://192.168.4.1/
*/

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Wire.h>

// ---------------- USER SETTINGS ----------------
#define I2C_SDA      6
#define I2C_SCL      7
#define MPU_ADDR     0x68
#define AP_SSID      "NEXUS-WEAR"      // hotspot name (open, no password)
#define LIN_DEADBAND 0.12f             // m/s^2, below this = 0 (kills noise)
// -----------------------------------------------

const float G0 = 9.80665f;
const float RAD2DEG = 180.0f / PI;
const float DEG2RAD = PI / 180.0f;

const IPAddress apIP(192, 168, 4, 1);
const byte DNS_PORT = 53;

DNSServer dns;
WebServer server(80);

bool okImu = false;
float roll = 0, pitch = 0, yaw = 0;           // degrees
float linX = 0, linY = 0, linZ = 0;           // linear acceleration m/s^2 (gravity removed)
float gbx = 0, gby = 0, gbz = 0;              // gyro bias deg/s
float gScale = 1.0f;                          // measured |a| at rest (in g)
unsigned long lastRead = 0, lastU = 0;

// ---------------- WEB PAGE ----------------
const char PAGE[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>NEXUS 3D MOTION</title>
<style>
:root{--c:#00f0ff;--m:#ff2bd6;--g:#39ff88;--y:#ffd400;--bg:#04070d;--card:#0a1220}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:#cfe9ff;font-family:"Courier New",monospace;min-height:100vh;padding:12px;
 background-image:linear-gradient(rgba(0,240,255,.05) 1px,transparent 1px),linear-gradient(90deg,rgba(0,240,255,.05) 1px,transparent 1px);
 background-size:28px 28px}
h1{font-size:18px;color:var(--c);text-shadow:0 0 10px var(--c);letter-spacing:3px;text-align:center}
.sub{font-size:11px;color:#6f93b3;text-align:center;margin-bottom:12px}
.grid{display:grid;gap:12px;grid-template-columns:1fr;max-width:980px;margin:0 auto}
@media(min-width:800px){.grid{grid-template-columns:1fr 1fr}}
.card{background:var(--card);border:1px solid rgba(0,240,255,.35);padding:14px;border-radius:8px;
 box-shadow:0 0 20px rgba(0,240,255,.15);text-align:center}
.card h2{font-size:12px;color:var(--c);letter-spacing:3px;margin-bottom:8px}
.stage{height:230px;perspective:650px;display:flex;align-items:center;justify-content:center}
/* ---- rotating device ---- */
#dev{position:relative;width:160px;height:100px;transform-style:preserve-3d;transition:transform .05s linear}
#dev div{position:absolute;display:flex;align-items:center;justify-content:center;font-size:11px;font-weight:bold;
 border:1px solid var(--c);background:rgba(0,240,255,.15);color:var(--c)}
.fr{width:160px;height:100px;transform:translateZ(15px);background:rgba(255,43,214,.2)!important;border-color:var(--m)!important;color:var(--m)!important}
.bk{width:160px;height:100px;transform:rotateY(180deg) translateZ(15px)}
.lf{width:30px;height:100px;left:65px;transform:rotateY(-90deg) translateZ(80px)}
.rt{width:30px;height:100px;left:65px;transform:rotateY(90deg) translateZ(80px)}
.tp{width:160px;height:30px;top:35px;transform:rotateX(90deg) translateZ(50px)}
.bt{width:160px;height:30px;top:35px;transform:rotateX(-90deg) translateZ(50px)}
/* ---- acceleration box ---- */
#scene{position:relative;width:160px;height:160px;transform-style:preserve-3d;transform:rotateX(-22deg) rotateY(32deg)}
#scene .f{position:absolute;width:160px;height:160px;border:1px solid rgba(0,240,255,.55);background:rgba(0,240,255,.05)}
.f1{transform:translateZ(80px)}.f2{transform:rotateY(180deg) translateZ(80px)}
.f3{transform:rotateY(-90deg) translateZ(80px)}.f4{transform:rotateY(90deg) translateZ(80px)}
.f5{transform:rotateX(90deg) translateZ(80px)}.f6{transform:rotateX(-90deg) translateZ(80px)}
.ax{position:absolute;left:80px;top:80px;transform-style:preserve-3d}
.ax i{position:absolute;display:block;background:rgba(255,255,255,.35)}
.axx{width:160px;height:1px;left:-80px;top:0}
.axy{width:1px;height:160px;left:0;top:-80px}
.axz{width:160px;height:1px;left:-80px;top:0;transform:rotateY(90deg)}
.lbl{position:absolute;font-size:11px;font-weight:bold;transform-style:preserve-3d}
#dot{position:absolute;left:73px;top:73px;width:14px;height:14px;border-radius:50%;
 background:radial-gradient(circle,#fff,var(--g) 60%);box-shadow:0 0 14px var(--g),0 0 30px var(--g);transform-style:preserve-3d}
.tri{display:flex;justify-content:space-around;margin-top:8px}
.tri div{font-size:10px;color:#6f93b3}.tri b{display:block;font-size:16px;color:#fff}
.bars{margin-top:8px;text-align:left}
.bw{display:flex;align-items:center;gap:6px;font-size:11px;margin:4px 0}
.bw span{width:14px}
.bt2{flex:1;height:10px;background:#07101c;border:1px solid #27506b;position:relative}
.bt2 i{position:absolute;top:0;bottom:0;left:50%;width:0;background:var(--g)}
.bt2:after{content:"";position:absolute;left:50%;top:-2px;bottom:-2px;width:1px;background:#6f93b3}
.bw b{width:70px;text-align:right;color:#fff}
.row{display:flex;gap:6px;margin-top:10px;align-items:center;flex-wrap:wrap}
.btn{flex:1;background:transparent;border:1px solid var(--c);color:var(--c);padding:8px 10px;
 font-family:inherit;font-size:11px;letter-spacing:1px;cursor:pointer}
.btn:active{background:rgba(0,240,255,.25)}
.btn.m{border-color:var(--m);color:var(--m)}
input[type=range]{flex:1;accent-color:var(--c)}
#toast{position:fixed;left:50%;bottom:16px;transform:translateX(-50%);background:#0a1220;border:1px solid var(--c);
 color:var(--c);padding:8px 14px;font-size:12px;display:none;z-index:9}
</style></head><body>
<h1>NEXUS//3D MOTION</h1>
<div class="sub">MPU TELEMETRY &bull; <span id="st">LINKING</span></div>

<div class="grid">
<div class="card"><h2>ROTATION (GYRO)</h2>
 <div class="stage"><div id="dev">
  <div class="fr">FRONT</div><div class="bk">REAR</div><div class="lf"></div><div class="rt"></div><div class="tp"></div><div class="bt"></div>
 </div></div>
 <div class="tri">
  <div>ROLL<b id="ro">0&deg;</b></div><div>PITCH<b id="pi">0&deg;</b></div><div>YAW<b id="ya">0&deg;</b></div>
 </div>
 <div class="row">
  <button class="btn" onclick="zero()">ZERO ANGLES</button>
  <button class="btn m" onclick="calib()">CALIBRATE (KEEP STILL)</button>
 </div>
</div>

<div class="card"><h2>LINEAR ACCELERATION</h2>
 <div class="stage"><div id="scene">
  <div class="f f1"></div><div class="f f2"></div><div class="f f3"></div><div class="f f4"></div><div class="f f5"></div><div class="f f6"></div>
  <div class="ax"><i class="axx"></i><i class="axy"></i><i class="axz"></i>
   <span class="lbl" style="color:#ff3355;transform:translate3d(84px,-6px,0)">X</span>
   <span class="lbl" style="color:#39ff88;transform:translate3d(-4px,-96px,0)">Y</span>
   <span class="lbl" style="color:#00f0ff;transform:translate3d(-4px,-6px,86px)">Z</span>
  </div>
  <div id="dot"></div>
 </div></div>
 <div class="bars">
  <div class="bw"><span style="color:#ff3355">X</span><div class="bt2"><i id="bx"></i></div><b id="tx">0.00</b></div>
  <div class="bw"><span style="color:#39ff88">Y</span><div class="bt2"><i id="by"></i></div><b id="ty">0.00</b></div>
  <div class="bw"><span style="color:#00f0ff">Z</span><div class="bt2"><i id="bz"></i></div><b id="tz">0.00</b></div>
 </div>
 <div class="sub" style="margin:4px 0 0">m/s&sup2; &bull; GRAVITY REMOVED &bull; |a| = <span id="tm">0.00</span></div>
 <div class="row"><span style="font-size:11px">SENS</span><input id="sens" type="range" min="2" max="20" value="8"></div>
</div>
</div>
<div id="toast"></div>

<script>
const $=id=>document.getElementById(id);
const cl=(x,a,b)=>Math.max(a,Math.min(b,x));
let rO=0,pO=0,yO=0,last={roll:0,pitch:0,yaw:0},sm={x:0,y:0,z:0},fails=0;

function toast(t){const e=$('toast');e.textContent=t;e.style.display='block';clearTimeout(toast.t);toast.t=setTimeout(()=>e.style.display='none',2500)}
function bar(id,t,v){ // v in m/s2, bar full scale = 10
 const w=cl(Math.abs(v)/10,0,1)*50;
 const e=$(id);e.style.width=w+'%';e.style.left=v>=0?'50%':(50-w)+'%';
 $(t).textContent=v.toFixed(2);
}
function wrapA(a){return ((a+540)%360)-180}

async function tick(){
 try{
  const res=await fetch('/data',{cache:'no-store'});
  const d=await res.json();
  fails=0;$('st').textContent='ONLINE';
  if(d.ok){
   last=d;
   const r=wrapA(d.roll-rO),p=wrapA(d.pitch-pO),y=wrapA(d.yaw-yO);
   $('dev').style.transform=`rotateX(${-p}deg) rotateZ(${r}deg) rotateY(${y}deg)`;
   $('ro').textContent=Math.round(r)+'\u00b0';$('pi').textContent=Math.round(p)+'\u00b0';$('ya').textContent=Math.round(y)+'\u00b0';
   // smooth linear acceleration for a calm dot
   sm.x+=(d.ax-sm.x)*.4;sm.y+=(d.ay-sm.y)*.4;sm.z+=(d.az-sm.z)*.4;
   const k=+$('sens').value;            // px per m/s2
   const px=cl(sm.x*k,-68,68),py=cl(sm.y*k,-68,68),pz=cl(sm.z*k,-68,68);
   // sensor X -> right, Y -> up, Z -> toward viewer
   $('dot').style.transform=`translate3d(${px}px,${-py}px,${pz}px)`;
   bar('bx','tx',sm.x);bar('by','ty',sm.y);bar('bz','tz',sm.z);
   $('tm').textContent=Math.sqrt(sm.x*sm.x+sm.y*sm.y+sm.z*sm.z).toFixed(2);
  }
 }catch(e){if(++fails>3)$('st').textContent='LINK LOST'}
 setTimeout(tick,50);
}
function zero(){rO=last.roll;pO=last.pitch;yO=last.yaw;toast('ANGLES ZEROED')}
async function calib(){
 toast('CALIBRATING - KEEP DEVICE STILL');
 try{await fetch('/calib',{cache:'no-store'});rO=pO=yO=0;toast('CALIBRATION DONE')}catch(e){toast('FAILED')}
}
tick();
</script></body></html>
)rawliteral";

// ---------------- MPU DIRECT I2C ----------------
void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// a[] in g, g[] in deg/s (bias NOT removed here)
bool readRaw(float *a, float *g) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) return false;
  int16_t v[7];
  for (int i = 0; i < 7; i++) {
    uint8_t hi = Wire.read();            // read order made explicit
    uint8_t lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }
  a[0] = v[0] / 16384.0f; a[1] = v[1] / 16384.0f; a[2] = v[2] / 16384.0f;   // +-2g
  g[0] = v[4] / 131.0f;   g[1] = v[5] / 131.0f;   g[2] = v[6] / 131.0f;     // +-250 dps
  return true;
}

float wrap180(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

// keep device still: measures gyro bias + real 1g magnitude, resets angles
void calibrate() {
  const int N = 300;
  float sa[3] = {0, 0, 0}, sg[3] = {0, 0, 0};
  int n = 0;
  for (int i = 0; i < N; i++) {
    float a[3], g[3];
    if (readRaw(a, g)) {
      for (int k = 0; k < 3; k++) { sa[k] += a[k]; sg[k] += g[k]; }
      n++;
    }
    delay(3);
  }
  if (n < 50) return;
  for (int k = 0; k < 3; k++) { sa[k] /= n; sg[k] /= n; }
  gbx = sg[0]; gby = sg[1]; gbz = sg[2];
  gScale = sqrtf(sa[0] * sa[0] + sa[1] * sa[1] + sa[2] * sa[2]);
  if (gScale < 0.8f || gScale > 1.2f) gScale = 1.0f;
  roll  = atan2f(sa[1], sa[2]) * RAD2DEG;
  pitch = atan2f(-sa[0], sqrtf(sa[1] * sa[1] + sa[2] * sa[2])) * RAD2DEG;
  yaw = 0; linX = linY = linZ = 0;
  lastU = micros();
}

void updateMotion() {
  float a[3], g[3];
  if (!readRaw(a, g)) return;

  unsigned long nowU = micros();
  float dt = (nowU - lastU) / 1000000.0f;
  lastU = nowU;
  if (dt <= 0 || dt > 0.1f) dt = 0.01f;

  float gx = g[0] - gbx, gy = g[1] - gby, gz = g[2] - gbz;
  if (fabsf(gz) < 0.15f) gz = 0;                 // tiny yaw-drift suppressor

  float accRoll  = atan2f(a[1], a[2]) * RAD2DEG;
  float accPitch = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2])) * RAD2DEG;

  // trust accelerometer only when |a| ~ 1g (not during hard shaking)
  float mag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) / gScale;
  float k = (mag > 0.85f && mag < 1.15f) ? 0.02f : 0.0f;

  // gx -> roll (about X), gy -> pitch (about Y), gz -> yaw (about Z)
  roll  += gx * dt;  roll  += k * wrap180(accRoll  - roll);   roll  = wrap180(roll);
  pitch += gy * dt;  pitch += k * wrap180(accPitch - pitch);
  yaw   += gz * dt;  yaw = wrap180(yaw);

  // remove gravity using the estimated orientation
  float r = roll * DEG2RAD, p = pitch * DEG2RAD;
  float gvx = -sinf(p);
  float gvy =  sinf(r) * cosf(p);
  float gvz =  cosf(r) * cosf(p);
  float lx = (a[0] / gScale - gvx) * G0;
  float ly = (a[1] / gScale - gvy) * G0;
  float lz = (a[2] / gScale - gvz) * G0;
  linX += 0.3f * (lx - linX);
  linY += 0.3f * (ly - linY);
  linZ += 0.3f * (lz - linZ);
}

// ---------------- HANDLERS ----------------
void sendPage() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", PAGE);
}

static inline float db(float v) { return fabsf(v) < LIN_DEADBAND ? 0.0f : v; }

void sendData() {
  char buf[200];
  snprintf(buf, sizeof(buf),
    "{\"ok\":%s,\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f}",
    okImu ? "true" : "false", roll, pitch, yaw, db(linX), db(linY), db(linZ));
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

void redirectToPortal() {
  server.sendHeader("Location", String("http://") + apIP.toString() + "/", true);
  server.send(302, "text/plain", "");
}

// ---------------- SETUP & LOOP ----------------
void startWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID);                      // open network, no password
  WiFi.setSleep(false);                      // lower latency for live 3D
  // WiFi.setTxPower(WIFI_POWER_8_5dBm);     // uncomment if your C3 board has unstable WiFi

  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(DNS_PORT, "*", apIP);            // captive portal: any domain -> this page
  Serial.print("Hotspot '" AP_SSID "' ready. Open: http://");
  Serial.println(WiFi.softAPIP());
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  delay(100);

  writeReg(0x6B, 0x01);   // wake, PLL clock
  delay(100);
  uint8_t who = readReg(0x75);
  Serial.printf("WHO_AM_I = 0x%02X (0x71 MPU9250, 0x70 MPU6500, 0x73 MPU9255)\n", who);
  if (who != 0xFF && who != 0x00) {
    okImu = true;
    writeReg(0x19, 4);      // sample rate 200 Hz
    writeReg(0x1A, 0x03);   // gyro low-pass ~41 Hz
    writeReg(0x1B, 0x00);   // gyro  +-250 dps
    writeReg(0x1C, 0x00);   // accel +-2 g
    writeReg(0x1D, 0x03);   // accel low-pass ~44 Hz
    Serial.println("MPU OK. Keep it still - calibrating...");
    calibrate();
    Serial.printf("Gyro bias: %.2f %.2f %.2f dps | 1g = %.3f\n", gbx, gby, gbz, gScale);
  } else {
    Serial.println("MPU connection failed! Check SDA=6 SCL=7, power, AD0 address.");
  }
  lastU = micros();

  startWiFi();

  server.on("/", sendPage);
  server.on("/data", sendData);
  server.on("/calib", []() {
    if (okImu) calibrate();
    server.send(200, "text/plain", "OK");
  });
  server.onNotFound(redirectToPortal);
  server.begin();
}

void loop() {
  dns.processNextRequest();
  server.handleClient();

  if (okImu && millis() - lastRead >= 10) {   // ~100 Hz
    lastRead = millis();
    updateMotion();
  }
}
