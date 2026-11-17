/*
  NEXUS WEAR v2 - MPU9250/6500 flip-free 3D Motion + Linear Acceleration Dashboard
  Board   : ESP32-C3 Dev Module (SDA: 6, SCL: 7)  -> enable "USB CDC On Boot"
  Method  : Direct I2C, Mahony quaternion filter (no gimbal lock, no 180 deg flips),
            gravity removal, stationary detection (gyro bias auto-learn + zero-rate lock)
  WiFi    : personal open hotspot "NEXUS-WEAR" + captive portal, or http://192.168.4.1/
  Page    : quaternion -> CSS matrix3d, client-side smoothing, customizable 3D object
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
#define KP_MOVING    1.0f              // tilt correction gain while moving
#define KP_STILL     2.5f              // tilt correction gain while still
#define STILL_DPS    1.5f              // gyro below this (after bias) counts as still
#define IMU_PERIOD_US 5000             // 200 Hz filter update
// -----------------------------------------------

const float G0 = 9.80665f;
const float RAD2DEG = 180.0f / PI;
const float DEG2RAD = PI / 180.0f;

const IPAddress apIP(192, 168, 4, 1);
const byte DNS_PORT = 53;

DNSServer dns;
WebServer server(80);

bool okImu = false;
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;         // orientation quaternion (body -> world)
float linX = 0, linY = 0, linZ = 0;           // linear acceleration m/s^2 (gravity removed)
float gbx = 0, gby = 0, gbz = 0;              // gyro bias deg/s
float gScale = 1.0f;                          // measured |a| at rest (in g)
float aPrev[3] = {0, 0, 1};
uint16_t stillCnt = 0;
bool still = false;
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
.stage{height:270px;perspective:750px;display:flex;align-items:center;justify-content:center}
/* ---- rotating object (built by JS) ---- */
#world{position:relative;width:0;height:0;transform-style:preserve-3d}
#dev{position:absolute;left:0;top:0;width:0;height:0;transform-style:preserve-3d;will-change:transform}
#dev .fc{position:absolute;display:flex;align-items:center;justify-content:center;text-align:center;
 font-weight:bold;line-height:1.15;backface-visibility:hidden;overflow:hidden}
#dev .fc small{display:block;opacity:.75;font-weight:normal}
#dev .ln{position:absolute;transform-origin:0 50%}
#floor{position:absolute;width:320px;height:320px;left:-160px;top:-160px;
 border:1px solid rgba(0,240,255,.4);background-color:rgba(0,240,255,.04);
 background-image:linear-gradient(rgba(0,240,255,.25) 1px,transparent 1px),linear-gradient(90deg,rgba(0,240,255,.25) 1px,transparent 1px);
 background-size:40px 40px;font-size:10px;color:#6f93b3;text-align:center}
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
details{margin-top:10px;text-align:left;border:1px solid #27506b;padding:6px 8px}
summary{font-size:11px;color:var(--c);letter-spacing:2px;cursor:pointer}
.set{display:grid;grid-template-columns:70px 1fr 34px;gap:6px 6px;align-items:center;font-size:11px;margin-top:8px}
.set b{color:#fff;text-align:right}
select{grid-column:2/4;background:#07101c;color:#fff;border:1px solid #27506b;padding:4px;font-family:inherit}
.chk{grid-column:1/4;display:flex;gap:14px}
#toast{position:fixed;left:50%;bottom:16px;transform:translateX(-50%);background:#0a1220;border:1px solid var(--c);
 color:var(--c);padding:8px 14px;font-size:12px;display:none;z-index:9}
</style></head><body>
<h1>NEXUS//3D MOTION</h1>
<div class="sub">MPU TELEMETRY &bull; <span id="st">LINKING</span></div>

<div class="grid">
<div class="card"><h2>ROTATION (GYRO + ACCEL)</h2>
 <div class="stage"><div id="world"><div id="floor"></div><div id="dev"></div></div></div>
 <div class="tri">
  <div>ROLL<b id="ro">0&deg;</b></div><div>PITCH<b id="pi">0&deg;</b></div><div>YAW<b id="ya">0&deg;</b></div>
 </div>
 <div class="row">
  <button class="btn" onclick="zeroYaw()">ZERO YAW</button>
  <button class="btn" onclick="zeroAll()">ZERO ALL</button>
  <button class="btn m" onclick="resetZero()">RESET</button>
 </div>
 <div class="row"><button class="btn m" onclick="calib()">CALIBRATE (KEEP STILL)</button></div>
 <details><summary>3D OBJECT SETTINGS</summary>
  <div class="set">
   <span>PRESET</span>
   <select id="pre"><option value="">custom...</option>
    <option value="130,80,14">BOARD</option><option value="70,115,10">PHONE</option>
    <option value="90,90,16">WATCH</option><option value="60,60,60">CUBE</option>
    <option value="170,40,40">BAR</option></select>
   <span>LENGTH X</span><input id="oL" type="range" min="30" max="200"><b id="vL"></b>
   <span>WIDTH Y</span><input id="oW" type="range" min="20" max="200"><b id="vW"></b>
   <span>THICK Z</span><input id="oT" type="range" min="4" max="120"><b id="vT"></b>
   <span>VIEW TILT</span><input id="oV" type="range" min="0" max="85"><b id="vV"></b>
   <span>SMOOTH</span><input id="oS" type="range" min="0" max="100"><b id="vS"></b>
   <div class="chk"><label><input id="oF" type="checkbox"> FLOOR</label><label><input id="oA" type="checkbox"> AXES</label></div>
  </div>
 </details>
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
function toast(t){const e=$('toast');e.textContent=t;e.style.display='block';clearTimeout(toast.t);toast.t=setTimeout(()=>e.style.display='none',2500)}

// ---------- object settings ----------
let O={L:130,W:80,T:14,view:55,smooth:40,floor:1,axes:1};
try{Object.assign(O,JSON.parse(localStorage.getItem('nx3d')||'{}'))}catch(e){}
function save(){try{localStorage.setItem('nx3d',JSON.stringify(O))}catch(e){}}
const SL=[['L','oL','vL'],['W','oW','vW'],['T','oT','vT'],['view','oV','vV'],['smooth','oS','vS']];
function sync(){
 SL.forEach(([k,i,v])=>{$(i).value=O[k];$(v).textContent=O[k]});
 $('oF').checked=!!O.floor;$('oA').checked=!!O.axes;
}
SL.forEach(([k,i,v])=>$(i).oninput=()=>{O[k]=+$(i).value;$(v).textContent=O[k];save();build()});
$('oF').onchange=()=>{O.floor=$('oF').checked?1:0;save();build()};
$('oA').onchange=()=>{O.axes=$('oA').checked?1:0;save();build()};
$('pre').onchange=()=>{const s=$('pre').value;if(!s)return;const a=s.split(',').map(Number);O.L=a[0];O.W=a[1];O.T=a[2];sync();save();build()};

// body frame: sensor +X = right, +Y = up, +Z = toward viewer (floor = world XY plane)
function build(){
 const L=O.L,W=O.W,T=O.T,d=$('dev');
 const defs=[
  ['TOP','+Z','255,43,214',L,W,`translateZ(${T/2}px)`],
  ['BOTTOM','-Z','100,130,175',L,W,`rotateY(180deg) translateZ(${T/2}px)`],
  ['RIGHT','+X','255,70,90',T,W,`rotateY(90deg) translateZ(${L/2}px)`],
  ['LEFT','-X','170,50,70',T,W,`rotateY(-90deg) translateZ(${L/2}px)`],
  ['FRONT','+Y','57,255,136',L,T,`rotateX(90deg) translateZ(${W/2}px)`],
  ['BACK','-Y','35,150,90',L,T,`rotateX(-90deg) translateZ(${W/2}px)`]
 ];
 let h='';
 defs.forEach(f=>{
  const fs=cl(Math.min(f[3],f[4])/4.2,6,13);
  h+=`<div class="fc" style="left:${-f[3]/2}px;top:${-f[4]/2}px;width:${f[3]}px;height:${f[4]}px;transform:${f[5]};`+
     `background:rgba(${f[2]},.30);border:1px solid rgb(${f[2]});color:rgb(${f[2]});font-size:${fs}px"><div>${f[0]}<small>${f[1]}</small></div></div>`;
 });
 if(O.axes){
  const e=28,B='opacity:.35;',R='#ff3355',G='#39ff88',C='#00f0ff';
  h+=`<div class="ln" style="left:0;top:-1px;width:${L/2+e}px;height:2px;background:${R}"></div>`+
     `<div class="ln" style="left:0;top:-1px;width:${L/2+e}px;height:2px;background:${R};${B}transform:rotateZ(180deg)"></div>`+
     `<div class="ln" style="left:0;top:-1px;width:${W/2+e}px;height:2px;background:${G};transform:rotateZ(-90deg)"></div>`+
     `<div class="ln" style="left:0;top:-1px;width:${W/2+e}px;height:2px;background:${G};${B}transform:rotateZ(90deg)"></div>`+
     `<div class="ln" style="left:0;top:-1px;width:${T/2+e}px;height:2px;background:${C};transform:rotateY(-90deg)"></div>`+
     `<div class="ln" style="left:0;top:-1px;width:${T/2+e}px;height:2px;background:${C};${B}transform:rotateY(90deg)"></div>`;
 }
 d.innerHTML=h;
 const f=$('floor');f.style.display=O.floor?'block':'none';
 f.style.transform=`translateZ(${-(T/2+12)}px)`;
 f.innerHTML='<div style="margin-top:2px">&#9650; 0&deg; (FWD)</div>';
 $('world').style.transform=`rotateX(${O.view}deg)`;
}

// ---------- quaternion helpers ----------
const mul=(a,b)=>[a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
 a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
 a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
 a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]];
const conj=a=>[a[0],-a[1],-a[2],-a[3]];
const norm=a=>{const n=Math.hypot(a[0],a[1],a[2],a[3])||1;return a.map(v=>v/n)};
const dotq=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[3]*b[3];
const yawOf=q=>Math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));

function setMat(q){
 const w=q[0],x=q[1],y=q[2],z=q[3];
 const R=[[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
          [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
          [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]];
 const c=[1,-1,1];                         // sensor frame (y up) -> CSS frame (y down)
 const m=(i,j)=>(c[i]*c[j]*R[i][j]).toFixed(5);
 $('dev').style.transform=`matrix3d(${m(0,0)},${m(1,0)},${m(2,0)},0,${m(0,1)},${m(1,1)},${m(2,1)},0,${m(0,2)},${m(1,2)},${m(2,2)},0,0,0,0,1)`;
}

let raw=[1,0,0,0],qr=[1,0,0,0],qt=[1,0,0,0],qs=[1,0,0,0];
let lin={x:0,y:0,z:0},sm={x:0,y:0,z:0},fails=0;

function zeroYaw(){const p=yawOf(qt);qr=norm(mul(qr,[Math.cos(p/2),0,0,Math.sin(p/2)]));qt=norm(mul(conj(qr),raw));toast('YAW ZEROED')}
function zeroAll(){qr=raw.slice();qt=[1,0,0,0];toast('ALL ANGLES ZEROED')}
function resetZero(){qr=[1,0,0,0];qt=norm(mul(conj(qr),raw));toast('ZERO CLEARED')}
async function calib(){
 toast('CALIBRATING - KEEP DEVICE STILL');
 try{await fetch('/calib',{cache:'no-store'});qr=[1,0,0,0];toast('CALIBRATION DONE')}catch(e){toast('FAILED')}
}

function showEuler(q){
 const w=q[0],x=q[1],y=q[2],z=q[3],D=180/Math.PI;
 const r=Math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))*D;
 const p=Math.asin(cl(2*(w*y-z*x),-1,1))*D;
 const ya=Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))*D;
 $('ro').textContent=Math.round(r)+'°';$('pi').textContent=Math.round(p)+'°';$('ya').textContent=Math.round(ya)+'°';
}
function bar(id,t,v){
 const w=cl(Math.abs(v)/10,0,1)*50;
 const e=$(id);e.style.width=w+'%';e.style.left=v>=0?'50%':(50-w)+'%';
 $(t).textContent=v.toFixed(2);
}

async function poll(){
 try{
  const res=await fetch('/data',{cache:'no-store'});
  const d=await res.json();
  fails=0;$('st').textContent='ONLINE • '+(d.st?'STILL':'MOVING');
  if(d.ok){
   raw=[d.qw,d.qx,d.qy,d.qz];
   qt=norm(mul(conj(qr),raw));
   lin={x:d.ax,y:d.ay,z:d.az};
   showEuler(qt);
  }
 }catch(e){if(++fails>3)$('st').textContent='LINK LOST'}
 setTimeout(poll,15);
}

let tp=performance.now();
function frame(t){
 const dt=Math.min(.1,(t-tp)/1000);tp=t;
 // quaternion smoothing (shortest path, no flips)
 const tau=.005+O.smooth/100*.16,k=1-Math.exp(-dt/tau);
 let tq=qt;if(dotq(qs,tq)<0)tq=tq.map(v=>-v);
 qs=norm(qs.map((v,i)=>v+(tq[i]-v)*k));
 setMat(qs);
 // linear acceleration dot
 const ka=1-Math.exp(-dt/.06);
 sm.x+=(lin.x-sm.x)*ka;sm.y+=(lin.y-sm.y)*ka;sm.z+=(lin.z-sm.z)*ka;
 const s=+$('sens').value;
 const px=cl(sm.x*s,-68,68),py=cl(sm.y*s,-68,68),pz=cl(sm.z*s,-68,68);
 $('dot').style.transform=`translate3d(${px}px,${-py}px,${pz}px)`;
 bar('bx','tx',sm.x);bar('by','ty',sm.y);bar('bz','tz',sm.z);
 $('tm').textContent=Math.hypot(sm.x,sm.y,sm.z).toFixed(2);
 requestAnimationFrame(frame);
}
sync();build();poll();requestAnimationFrame(frame);
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
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }
  a[0] = v[0] / 16384.0f; a[1] = v[1] / 16384.0f; a[2] = v[2] / 16384.0f;   // +-2g
  g[0] = v[4] / 131.0f;   g[1] = v[5] / 131.0f;   g[2] = v[6] / 131.0f;     // +-250 dps
  return true;
}

// orientation from gravity only (yaw = 0)
void quatFromAccel(float ax, float ay, float az) {
  float r = atan2f(ay, az);
  float p = atan2f(-ax, sqrtf(ay * ay + az * az));
  float cr = cosf(r * 0.5f), sr = sinf(r * 0.5f), cp = cosf(p * 0.5f), sp = sinf(p * 0.5f);
  q0 = cr * cp; q1 = sr * cp; q2 = cr * sp; q3 = -sr * sp;
}

// keep device still: measures gyro bias + real 1g magnitude, resets orientation
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
  quatFromAccel(sa[0], sa[1], sa[2]);
  for (int k = 0; k < 3; k++) aPrev[k] = sa[k];
  linX = linY = linZ = 0;
  stillCnt = 0; still = false;
  lastU = micros();
}

void updateMotion() {
  float a[3], g[3];
  if (!readRaw(a, g)) return;

  unsigned long nowU = micros();
  float dt = (nowU - lastU) / 1000000.0f;
  lastU = nowU;
  if (dt <= 0 || dt > 0.1f) dt = 0.005f;

  float gx = g[0] - gbx, gy = g[1] - gby, gz = g[2] - gbz;       // deg/s
  float amag = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  float aN = amag / gScale;                                      // ~1.0 at rest

  // ---- stationary detection: learn gyro bias, freeze drift ----
  float da = fabsf(a[0] - aPrev[0]) + fabsf(a[1] - aPrev[1]) + fabsf(a[2] - aPrev[2]);
  aPrev[0] = a[0]; aPrev[1] = a[1]; aPrev[2] = a[2];
  bool quiet = fabsf(gx) < STILL_DPS && fabsf(gy) < STILL_DPS && fabsf(gz) < STILL_DPS &&
               fabsf(aN - 1.0f) < 0.04f && da < 0.03f;
  if (quiet) { if (stillCnt < 60000) stillCnt++; } else stillCnt = 0;
  still = stillCnt > 100;                                        // ~0.5 s of calm
  if (still) {
    gbx += 0.002f * (g[0] - gbx);
    gby += 0.002f * (g[1] - gby);
    gbz += 0.002f * (g[2] - gbz);
    gx = gy = gz = 0;                                            // zero-rate lock: no yaw creep
  }

  // ---- Mahony quaternion filter (gyro + accel tilt correction) ----
  float gxr = gx * DEG2RAD, gyr = gy * DEG2RAD, gzr = gz * DEG2RAD;
  if (aN > 0.8f && aN < 1.2f) {
    float ax = a[0] / amag, ay = a[1] / amag, az = a[2] / amag;
    float vx = 2.0f * (q1 * q3 - q0 * q2);                       // estimated "up" in body frame
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    float kp = still ? KP_STILL : KP_MOVING;
    gxr += kp * (ay * vz - az * vy);
    gyr += kp * (az * vx - ax * vz);
    gzr += kp * (ax * vy - ay * vx);
  }
  float h = 0.5f * dt;
  gxr *= h; gyr *= h; gzr *= h;
  float qa = q0, qb = q1, qc = q2;
  q0 += (-qb * gxr - qc * gyr - q3 * gzr);
  q1 += ( qa * gxr + qc * gzr - q3 * gyr);
  q2 += ( qa * gyr - qb * gzr + q3 * gxr);
  q3 += ( qa * gzr + qb * gyr - qc * gxr);
  float n = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= n; q1 *= n; q2 *= n; q3 *= n;

  // ---- remove gravity using the orientation ----
  float ux = 2.0f * (q1 * q3 - q0 * q2);
  float uy = 2.0f * (q0 * q1 + q2 * q3);
  float uz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
  float lx = (a[0] / gScale - ux) * G0;
  float ly = (a[1] / gScale - uy) * G0;
  float lz = (a[2] / gScale - uz) * G0;
  linX += 0.2f * (lx - linX);
  linY += 0.2f * (ly - linY);
  linZ += 0.2f * (lz - linZ);
}

// ---------------- HANDLERS ----------------
void sendPage() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", PAGE);
}

static inline float db(float v) { return fabsf(v) < LIN_DEADBAND ? 0.0f : v; }

void sendData() {
  char buf[220];
  snprintf(buf, sizeof(buf),
    "{\"ok\":%s,\"st\":%d,\"qw\":%.5f,\"qx\":%.5f,\"qy\":%.5f,\"qz\":%.5f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f}",
    okImu ? "true" : "false", still ? 1 : 0, q0, q1, q2, q3, db(linX), db(linY), db(linZ));
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
  lastRead = micros();

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

  if (okImu && (unsigned long)(micros() - lastRead) >= IMU_PERIOD_US) {
    lastRead = micros();
    updateMotion();
  }
}
