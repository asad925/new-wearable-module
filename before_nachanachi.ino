/*
  NEXUS WEAR v2 - ESP32-C3 Mini wearable dashboard
  Sensors : INA226 (power), MLX90614 (IR temp), LSM303 (accel + mag)
  WiFi    : open hotspot "NEXUS-WEAR" (no password) + captive portal redirect
  New     : stable phone-style SoC (coulomb counting), mAh counters, stable backup time,
            3D zero/compass calibration (saved in flash), C/F toggle.

  Libraries: INA226 (Rob Tillaart), Adafruit LSM303DLHC (Adafruit_LSM303_U),
             Adafruit Unified Sensor, Adafruit MLX90614
  Board    : ESP32C3 Dev Module (enable "USB CDC On Boot" for Serial)
*/

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Preferences.h>
#include "INA226.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_MLX90614.h>

// ---------------- USER SETTINGS ----------------
#define I2C_SDA      6
#define I2C_SCL      7
#define INA_ADDR     0x40
#define SHUNT_OHMS   0.1      // your INA226 module's shunt resistor
#define MAX_AMPS     0.8      // max expected current for that shunt
#define AP_SSID      "NEXUS-WEAR"

#define BATT_MAH_DEFAULT 1000 // battery capacity in mAh (can also be changed on the web page)
#define CURRENT_SIGN     1    // set to -1 if CHARGING shows as DISCHARGING (and vice versa)
#define FULL_VOLT        4.15 // charge-complete detection: voltage above this ...
#define FULL_TAPER_MA    50   // ... and charge current below this for 30 s => 100%
#define REST_MA          15   // |current| below this counts as "resting battery"
// -----------------------------------------------

const IPAddress apIP(192, 168, 4, 1);
const byte DNS_PORT = 53;

DNSServer dns;
WebServer server(80);
Preferences prefs;

INA226 ina(INA_ADDR);
Adafruit_LSM303_Accel_Unified accel(54321);
Adafruit_LSM303_Mag_Unified   mag(12345);
Adafruit_MLX90614 mlx;

bool okIna = false, okMlx = false, okImu = false;

// live values
float V = 0, I = 0, P = 0, Ta = 0, To = 0;      // I: + = discharging, - = charging (mA)
float rollR = 0, pitchR = 0, hdgR = 0, G = 1;   // raw filtered angles
float ax = 0, ay = 0, az = 0;
unsigned long lastRead = 0;

// 3D calibration (saved in flash)
float r0 = 0, p0 = 0, h0 = 0;                   // zero offsets
float mxo = 0, myo = 0, mzo = 0;                // magnetometer hard-iron offsets
bool  magCal = false;
unsigned long magCalEnd = 0;
float mnx, mny, mnz, mxx, mxy, mxz;

// battery state of charge (coulomb counting)
float capMah = BATT_MAH_DEFAULT;
float soc = 50;            // %
float dischMah = 0;        // mAh discharged since last full charge
float avgI = 0;            // long-term averaged current (mA), used for backup time
bool  avgInit = false;
unsigned long lastInteg = 0, lastSave = 0, fullTimer = 0;
float lastSavedSoc = -100, lastSavedDis = -100;
float restTimer = 0;
bool  restSynced = false;

const float OCV_V[] = {3.27,3.61,3.69,3.71,3.73,3.75,3.77,3.79,3.80,3.82,3.84,3.85,3.87,3.91,3.95,3.98,4.02,4.08,4.11,4.15,4.20};
const float OCV_P[] = {0,5,10,15,20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100};

float ocvSoc(float v) {
  if (v <= OCV_V[0]) return 0;
  if (v >= OCV_V[20]) return 100;
  for (int k = 1; k < 21; k++) {
    if (v <= OCV_V[k]) {
      float f = (v - OCV_V[k - 1]) / (OCV_V[k] - OCV_V[k - 1]);
      return OCV_P[k - 1] + f * (OCV_P[k] - OCV_P[k - 1]);
    }
  }
  return 100;
}

// ---------------- WEB PAGE ----------------
const char PAGE[] PROGMEM = R"rawliteral(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>NEXUS WEAR</title>
<style>
:root{--c:#00f0ff;--m:#ff2bd6;--g:#39ff88;--y:#ffd400;--r:#ff3355;--bg:#04070d;--card:#0a1220}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:#cfe9ff;font-family:"Courier New",monospace;min-height:100vh;
 background-image:linear-gradient(rgba(0,240,255,.05) 1px,transparent 1px),linear-gradient(90deg,rgba(0,240,255,.05) 1px,transparent 1px);
 background-size:28px 28px;padding:12px;letter-spacing:.5px}
h1{font-size:20px;color:var(--c);text-shadow:0 0 12px var(--c);letter-spacing:4px}
header{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px}
.sub{font-size:11px;color:#6f93b3}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;background:var(--g);box-shadow:0 0 10px var(--g);margin-right:6px;animation:p 1.2s infinite}
.dot.off{background:var(--r);box-shadow:0 0 10px var(--r)}
@keyframes p{50%{opacity:.3}}
#banner{border:1px solid var(--g);color:var(--g);padding:10px;margin-bottom:12px;font-size:13px;
 background:rgba(57,255,136,.07);clip-path:polygon(0 0,calc(100% - 12px) 0,100% 12px,100% 100%,12px 100%,0 calc(100% - 12px))}
#banner.w{border-color:var(--y);color:var(--y);background:rgba(255,212,0,.08)}
#banner.c{border-color:var(--r);color:var(--r);background:rgba(255,51,85,.1);animation:blink .8s infinite}
@keyframes blink{50%{background:rgba(255,51,85,.3)}}
#banner div{margin:2px 0}
.grid{display:grid;gap:12px;grid-template-columns:1fr}
@media(min-width:900px){.grid{grid-template-columns:repeat(3,1fr)}}
.card{position:relative;background:var(--card);border:1px solid rgba(0,240,255,.35);padding:14px;
 box-shadow:0 0 18px rgba(0,240,255,.12) inset;
 clip-path:polygon(0 0,calc(100% - 16px) 0,100% 16px,100% 100%,16px 100%,0 calc(100% - 16px))}
.card h2{font-size:13px;color:var(--c);letter-spacing:3px;margin-bottom:10px}
.card.warn{border-color:var(--y);box-shadow:0 0 18px rgba(255,212,0,.25) inset}
.card.crit{border-color:var(--r);box-shadow:0 0 22px rgba(255,51,85,.35) inset;animation:blink .8s infinite}
.badge{position:absolute;top:8px;right:22px;font-size:22px;display:none}
.warn .badge{display:block;color:var(--y);text-shadow:0 0 10px var(--y);animation:p 1s infinite}
.crit .badge{display:block;color:var(--r);text-shadow:0 0 10px var(--r);animation:p .5s infinite}
.row{display:flex;justify-content:space-around;text-align:center}
.gauge{width:32%}
.gauge svg{width:100%}
.gauge .bg{fill:none;stroke:#16263a;stroke-width:10}
.gauge .fg{fill:none;stroke:var(--c);stroke-width:10;stroke-linecap:round;stroke-dasharray:188.5;stroke-dashoffset:188.5;
 filter:drop-shadow(0 0 4px var(--c));transition:stroke-dashoffset .3s,stroke .3s}
.gauge .val{font-size:17px;color:#fff;margin-top:-18px}
.gauge .lab{font-size:10px;color:#6f93b3}
.soc{display:flex;align-items:center;gap:10px;margin-bottom:10px}
.ring{width:44%;flex:none}
.rb{fill:none;stroke:#16263a;stroke-width:12}
.rf{fill:none;stroke:var(--g);stroke-width:12;stroke-linecap:round;stroke-dasharray:351.86;stroke-dashoffset:351.86;
 filter:drop-shadow(0 0 6px var(--g));transition:stroke-dashoffset .6s,stroke .6s}
.ring text{fill:#fff;text-anchor:middle}
.stats{flex:1}
.kv{display:flex;justify-content:space-between;font-size:12px;margin:5px 0;color:#8fb3d1}
.kv b{color:#fff}
canvas{width:100%;height:60px;background:#07101c;border:1px solid #27506b}
.tw{display:flex;align-items:center;justify-content:space-around;margin:6px 0}
#orb{width:130px;height:130px;border-radius:50%;transition:all .5s;animation:pulse 2s infinite}
@keyframes pulse{50%{transform:scale(1.07)}}
.thermo{width:26px;height:140px;border:1px solid #27506b;border-radius:13px;position:relative;overflow:hidden;background:#07101c}
.thermo i{position:absolute;bottom:0;left:0;right:0;height:0;transition:height .5s,background .5s}
.big{font-size:28px;color:#fff;text-align:center}
.bar{height:16px;border:1px solid #27506b;margin:8px 0;position:relative;background:#07101c}
.bar i{display:block;height:100%;width:0;background:linear-gradient(90deg,var(--g),var(--y),var(--r));transition:width .3s}
.bar span{position:absolute;inset:0;text-align:center;font-size:11px;line-height:16px;color:#fff;mix-blend-mode:difference}
.stage{height:210px;perspective:600px;display:flex;align-items:center;justify-content:center}
#dev{position:relative;width:150px;height:90px;transform-style:preserve-3d}
#dev div{position:absolute;display:flex;align-items:center;justify-content:center;font-size:10px;
 border:1px solid var(--c);background:rgba(0,240,255,.12);color:var(--c)}
.fr{width:150px;height:90px;transform:translateZ(12px)}
.bk{width:150px;height:90px;transform:rotateY(180deg) translateZ(12px)}
.lf{width:24px;height:90px;left:63px;transform:rotateY(-90deg) translateZ(75px)}
.rt{width:24px;height:90px;left:63px;transform:rotateY(90deg) translateZ(75px)}
.tp{width:150px;height:24px;top:33px;transform:rotateX(90deg) translateZ(45px)}
.bt{width:150px;height:24px;top:33px;transform:rotateX(-90deg) translateZ(45px)}
.fr{background:rgba(255,43,214,.18)!important;border-color:var(--m)!important;color:var(--m)!important}
.tri{display:flex;justify-content:space-around;margin-top:6px;text-align:center}
.tri div{font-size:10px;color:#6f93b3}.tri b{display:block;font-size:18px;color:#fff}
.comp{width:80px;height:80px}
.btns{display:flex;gap:6px;flex-wrap:wrap;margin-top:8px;align-items:center}
.btn{background:transparent;border:1px solid var(--c);color:var(--c);padding:7px 10px;font-family:inherit;font-size:11px;
 letter-spacing:1px;cursor:pointer}
.btn:active{background:rgba(0,240,255,.25)}
.btn.m{border-color:var(--m);color:var(--m)}
.unit{position:absolute;top:8px;right:40px}
.unit.b2{right:48px}
input{background:#07101c;border:1px solid #27506b;color:#fff;padding:6px;width:80px;font-family:inherit;font-size:12px}
#toast{position:fixed;left:50%;bottom:16px;transform:translateX(-50%);background:#0a1220;border:1px solid var(--c);
 color:var(--c);padding:8px 14px;font-size:12px;display:none;z-index:9}
#foot{text-align:center;font-size:10px;color:#46637d;margin-top:12px}
</style></head><body>
<header><div><h1>NEXUS//WEAR</h1><div class="sub">WEARABLE TELEMETRY CORE</div></div>
<div class="sub"><span class="dot" id="dot"></span><span id="st">LINKING</span><br>UP <span id="up">0</span>s</div></header>

<div id="banner">SCANNING SYSTEMS...</div>

<div class="grid">

<div class="card" id="cP"><span class="badge">&#9888;</span><h2>&#9889; POWER MANAGEMENT</h2>
 <div class="soc">
  <svg class="ring" viewBox="0 0 140 140"><circle class="rb" cx="70" cy="70" r="56"/>
   <circle class="rf" id="rf" cx="70" cy="70" r="56" transform="rotate(-90 70 70)"/>
   <text id="socT" x="70" y="80" font-size="30">--%</text>
   <text id="chg" x="70" y="104" font-size="14">&nbsp;</text></svg>
  <div class="stats">
   <div class="kv"><span>MODE</span><b id="mode">--</b></div>
   <div class="kv"><span>REMAINING</span><b><span id="rem">--</span> mAh</b></div>
   <div class="kv"><span>DISCHARGED</span><b><span id="dis">--</span> mAh</b></div>
   <div class="kv"><span>CAPACITY</span><b><span id="capv">--</span> mAh</b></div>
   <div class="kv"><span id="rtl">BACKUP</span><b id="rt">--</b></div>
  </div>
 </div>
 <div class="row">
  <div class="gauge"><svg viewBox="0 0 140 80"><path class="bg" d="M10 70 A60 60 0 0 1 130 70"/><path class="fg" id="gv" d="M10 70 A60 60 0 0 1 130 70"/></svg><div class="val"><span id="v">--</span>V</div><div class="lab">VOLTAGE</div></div>
  <div class="gauge"><svg viewBox="0 0 140 80"><path class="bg" d="M10 70 A60 60 0 0 1 130 70"/><path class="fg" id="gi" d="M10 70 A60 60 0 0 1 130 70"/></svg><div class="val"><span id="i">--</span>mA</div><div class="lab">CURRENT</div></div>
  <div class="gauge"><svg viewBox="0 0 140 80"><path class="bg" d="M10 70 A60 60 0 0 1 130 70"/><path class="fg" id="gp" d="M10 70 A60 60 0 0 1 130 70"/></svg><div class="val"><span id="p">--</span>mW</div><div class="lab">POWER</div></div>
 </div>
 <canvas id="spark" width="300" height="60" style="margin-top:8px"></canvas>
 <div class="sub" style="margin-top:3px">POWER HISTORY (LAST 60 SAMPLES)</div>
 <div class="btns">
  <input id="capin" type="number" min="50" max="20000" placeholder="mAh">
  <button class="btn" onclick="setCap()">SET CAP</button>
  <button class="btn" onclick="act('/soc?full=1','BATTERY MARKED FULL')">FULL 100%</button>
  <button class="btn m" onclick="act('/resetdis','COUNTER RESET')">RESET mAh</button>
 </div>
</div>

<div class="card" id="cT"><span class="badge">&#9888;</span><h2>&#127777; THERMAL SCAN</h2>
 <button class="btn unit" id="ubtn" onclick="togU()">&deg;C</button>
 <div class="tw">
  <div id="orb"></div>
  <div class="thermo"><i id="th"></i></div>
 </div>
 <div class="big"><span id="to">--</span><span class="u">&deg;C</span></div>
 <div class="sub" style="text-align:center">OBJECT / SKIN TEMPERATURE</div>
 <div class="kv"><span>AMBIENT</span><b><span id="ta">--</span><span class="u">&deg;C</span></b></div>
 <div class="kv"><span>DELTA</span><b><span id="td">--</span><span class="u">&deg;C</span></b></div>
 <div class="kv"><span>STATUS</span><b id="ts">--</b></div>
</div>

<div class="card" id="cM"><span class="badge">&#9888;</span><h2>&#129517; MOTION (LSM303)</h2>
 <div class="stage"><div id="dev">
  <div class="fr">NEXUS</div><div class="bk">REAR</div><div class="lf"></div><div class="rt"></div><div class="tp"></div><div class="bt"></div>
 </div></div>
 <div class="tri">
  <div>ROLL<b id="ro">0</b></div><div>PITCH<b id="pi">0</b></div><div>HEADING<b id="he">0</b></div>
  <svg class="comp" viewBox="0 0 100 100"><circle cx="50" cy="50" r="44" fill="none" stroke="#27506b" stroke-width="2"/>
   <text x="50" y="16" fill="#ff3355" font-size="12" text-anchor="middle">N</text>
   <g id="needle"><polygon points="50,18 56,50 44,50" fill="#ff3355"/><polygon points="50,82 56,50 44,50" fill="#00f0ff"/></g></svg>
 </div>
 <div class="bar"><i id="gb"></i><span id="gt">1.00 G</span></div>
 <div class="kv"><span>ORIENTATION</span><b id="ori">--</b></div>
 <div class="btns">
  <button class="btn" onclick="act('/zero','POSITION SET AS ZERO')">SET ZERO</button>
  <button class="btn m" onclick="act('/zeroreset','ZERO CLEARED')">RESET</button>
  <button class="btn" onclick="act('/magcal','ROTATE DEVICE IN ALL DIRECTIONS - 20s')">CAL COMPASS</button>
 </div>
 <div class="sub" id="mcs" style="margin-top:6px"></div>
</div>

</div>
<div id="foot">OPEN HOTSPOT &bull; 192.168.4.1 &bull; LIVE LINK</div>
<div id="toast"></div>

<script>
const $=id=>document.getElementById(id);
const L=188.5, RL=351.86, cl=(x,a,b)=>Math.max(a,Math.min(b,x));
// ---- thresholds (temperatures in C) ----
const T={vcrit:3.0,vhigh:4.25,imax:2000,pmax:8000,socLow:15,socCrit:5,
         skinWarn:38,skinCrit:40,ambWarn:40,impact:2.5,tilt:70};
// ---- 3D axis signs: flip to 1/-1 if the model moves the wrong way ----
const S={p:1,r:-1,h:-1};
let cur={r:0,p:0,h:0},hist=[],fails=0,lastD=null,U='C';
try{U=localStorage.getItem('u')||'C'}catch(e){}

const cv=c=>U=='F'?c*1.8+32:c;
const dv=c=>U=='F'?c*1.8:c;
function applyUnit(){
 $('ubtn').innerHTML='&deg;'+U;
 document.querySelectorAll('.u').forEach(e=>e.innerHTML='&deg;'+U);
}
function togU(){U=U=='C'?'F':'C';try{localStorage.setItem('u',U)}catch(e){};applyUnit();if(lastD)render(lastD)}
applyUnit();

function toast(t){const e=$('toast');e.textContent=t;e.style.display='block';clearTimeout(toast.t);toast.t=setTimeout(()=>e.style.display='none',2500)}
async function act(url,msg){try{await fetch(url,{cache:'no-store'});toast(msg)}catch(e){toast('COMMAND FAILED')}}
function setCap(){const v=parseInt($('capin').value);if(v>=50)act('/cap?mah='+v,'CAPACITY SET '+v+' mAh');else toast('ENTER mAh')}

function fmtH(h){
 if(h<0||h>99)return '--';
 const m=Math.round(h*60);return Math.floor(m/60)+'h '+(m%60)+'m';
}
function arc(id,f,col){const e=$(id);e.style.strokeDashoffset=L*(1-cl(f,0,1));e.style.stroke=col}
function lvlCol(l){return l==2?'#ff3355':l==1?'#ffd400':'#00f0ff'}
function setCard(id,l){const c=$(id);c.classList.remove('warn','crit');if(l==1)c.classList.add('warn');if(l==2)c.classList.add('crit')}

function spark(){
 const c=$('spark'),x=c.getContext('2d');x.clearRect(0,0,300,60);
 if(hist.length<2)return;
 const mx=Math.max(100,...hist);x.strokeStyle='#00f0ff';x.shadowColor='#00f0ff';x.shadowBlur=6;x.lineWidth=2;x.beginPath();
 hist.forEach((v,k)=>{const px=k*300/59,py=58-(v/mx)*54;k?x.lineTo(px,py):x.moveTo(px,py)});x.stroke();
}

function render(d){
 lastD=d;
 const W=[];  // [level,text,card]
 // ---------- POWER ----------
 if(d.ok_ina){
  $('v').textContent=d.v.toFixed(2);$('i').textContent=Math.round(d.i);$('p').textContent=Math.round(d.p);
  const chg=d.i<-5, dsc=d.i>5;
  // state of charge ring
  const sc=d.soc;
  $('rf').style.strokeDashoffset=RL*(1-cl(sc/100,0,1));
  const scol=sc>30?'#39ff88':(sc>T.socLow?'#ffd400':'#ff3355');
  $('rf').style.stroke=scol;
  $('socT').textContent=Math.round(sc)+'%';
  $('chg').textContent=chg?'\u26A1 CHARGING':(dsc?'DISCHARGING':'IDLE');
  $('chg').style.fill=chg?'#00f0ff':'#6f93b3';
  $('mode').textContent=chg?'CHARGING':(dsc?'DISCHARGING':'IDLE');
  $('rem').textContent=Math.round(d.rem);$('dis').textContent=d.dis.toFixed(1);$('capv').textContent=Math.round(d.cap);
  if(chg){$('rtl').textContent='FULL IN ~';$('rt').textContent=fmtH(d.tf)}
  else{$('rtl').textContent='BACKUP';$('rt').textContent=fmtH(d.rt)}
  if(document.activeElement!==$('capin')&&!$('capin').value)$('capin').placeholder=Math.round(d.cap)+' mAh';
  // warnings (SoC based, so voltage sag does not trigger them)
  if(sc<=T.socCrit&&!chg)W.push([2,'BATTERY CRITICAL '+Math.round(sc)+'%','cP']);
  else if(sc<=T.socLow&&!chg)W.push([1,'LOW BATTERY '+Math.round(sc)+'%','cP']);
  let vl=0;
  if(d.v<T.vcrit){vl=2;W.push([2,'UNDER-VOLTAGE '+d.v.toFixed(2)+'V','cP'])}
  else if(d.v>T.vhigh){vl=2;W.push([2,'OVER-VOLTAGE '+d.v.toFixed(2)+'V','cP'])}
  let il=0;if(Math.abs(d.i)>T.imax){il=2;W.push([2,'OVER-CURRENT '+Math.round(d.i)+'mA','cP'])}
  else if(Math.abs(d.i)>T.imax*.8){il=1;W.push([1,'HIGH CURRENT','cP'])}
  let pl=0;if(d.p>T.pmax){pl=2;W.push([2,'POWER OVERLOAD','cP'])}
  arc('gv',(d.v-2.5)/2,lvlCol(vl));arc('gi',Math.abs(d.i)/T.imax,lvlCol(il));arc('gp',d.p/T.pmax,lvlCol(pl));
  hist.push(d.p);if(hist.length>60)hist.shift();spark();
 }else{W.push([2,'INA226 POWER SENSOR OFFLINE','cP']);
  ['v','i','p','rem','dis','capv'].forEach(k=>$(k).textContent='--');$('socT').textContent='--%'}
 // ---------- THERMAL ----------
 if(d.ok_mlx){
  $('to').textContent=cv(d.to).toFixed(1);$('ta').textContent=cv(d.ta).toFixed(1);$('td').textContent=dv(d.to-d.ta).toFixed(1);
  const h=220-cl((d.to-20)/22,0,1)*220;
  $('orb').style.background='radial-gradient(circle,hsl('+h+',100%,65%),hsl('+h+',100%,35%) 55%,transparent 70%)';
  $('orb').style.boxShadow='0 0 40px hsl('+h+',100%,50%)';
  $('th').style.height=cl((d.to-10)/40,0,1)*100+'%';$('th').style.background='hsl('+h+',100%,50%)';
  let s='NORMAL';
  const tu='\u00b0'+U;
  if(d.to>=T.skinCrit){s='DANGER HEAT';W.push([2,'HIGH TEMPERATURE '+cv(d.to).toFixed(1)+tu,'cT'])}
  else if(d.to>=T.skinWarn){s='FEVER / HOT';W.push([1,'ELEVATED TEMPERATURE '+cv(d.to).toFixed(1)+tu,'cT'])}
  if(d.ta>=T.ambWarn){W.push([1,'HOT AMBIENT '+cv(d.ta).toFixed(1)+tu,'cT'])}
  $('ts').textContent=s;
 }else{W.push([2,'MLX90614 TEMP SENSOR OFFLINE','cT']);
  ['to','ta','td'].forEach(k=>$(k).textContent='--')}
 // ---------- MOTION ----------
 if(d.ok_imu){
  const dr=((d.roll-cur.r+540)%360)-180, dp=((d.pitch-cur.p+540)%360)-180, dh=((d.hdg-cur.h+540)%360)-180;
  cur.r+=dr*.5;cur.p+=dp*.5;cur.h+=dh*.5;
  $('dev').style.transform='rotateX('+(S.p*cur.p)+'deg) rotateZ('+(S.r*cur.r)+'deg) rotateY('+(S.h*cur.h)+'deg)';
  $('ro').textContent=Math.round(d.roll)+'\u00b0';$('pi').textContent=Math.round(d.pitch)+'\u00b0';$('he').textContent=Math.round(d.hdg)+'\u00b0';
  $('needle').setAttribute('transform','rotate('+(-d.hdg)+' 50 50)');
  $('gb').style.width=cl(d.g/4,0,1)*100+'%';$('gt').textContent=d.g.toFixed(2)+' G';
  let ori='STABLE';
  if(d.g>T.impact){ori='IMPACT!';W.push([2,'IMPACT / FALL DETECTED '+d.g.toFixed(1)+'G','cM'])}
  else if(Math.abs(d.roll)>T.tilt||Math.abs(d.pitch)>T.tilt){ori='TILTED';W.push([1,'EXTREME TILT ANGLE','cM'])}
  $('ori').textContent=ori;
  $('mcs').textContent=d.mc>0?'\u26A0 CALIBRATING COMPASS: ROTATE DEVICE ALL DIRECTIONS ('+d.mc+'s)':'';
  if(d.mc>0)W.push([1,'COMPASS CALIBRATION RUNNING '+d.mc+'s','cM']);
 }else{W.push([2,'LSM303 MOTION SENSOR OFFLINE','cM'])}
 // ---------- cards + banner ----------
 ['cP','cT','cM'].forEach(id=>{let l=0;W.forEach(w=>{if(w[2]==id)l=Math.max(l,w[0])});setCard(id,l)});
 const b=$('banner');let top=0;W.forEach(w=>top=Math.max(top,w[0]));
 if(!W.length){b.className='';b.innerHTML='&#10004; ALL SYSTEMS NOMINAL'}
 else{b.className=top==2?'c':'w';b.innerHTML=W.map(w=>'<div>&#9888; '+w[1]+'</div>').join('')}
 $('up').textContent=d.up;
}

async function tick(){
 try{
  const r=await fetch('/data',{cache:'no-store'});const d=await r.json();
  fails=0;$('dot').className='dot';$('st').textContent='ONLINE';render(d);
 }catch(e){
  if(++fails>3){$('dot').className='dot off';$('st').textContent='LINK LOST';
   $('banner').className='c';$('banner').innerHTML='&#9888; CONNECTION TO DEVICE LOST'}
 }
 setTimeout(tick,120);
}
tick();
</script></body></html>
)rawliteral";

// ---------------- HELPERS ----------------
float wrap180(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

void saveCal() {
  prefs.putFloat("r0", r0); prefs.putFloat("p0", p0); prefs.putFloat("h0", h0);
  prefs.putFloat("mxo", mxo); prefs.putFloat("myo", myo); prefs.putFloat("mzo", mzo);
}

void saveSoc(bool force) {
  if (!force && millis() - lastSave < 60000) return;
  if (force || fabs(soc - lastSavedSoc) >= 0.5 || fabs(dischMah - lastSavedDis) >= 5) {
    prefs.putFloat("soc", soc);
    prefs.putFloat("dis", dischMah);
    lastSavedSoc = soc; lastSavedDis = dischMah;
  }
  lastSave = millis();
}

// ---------------- BATTERY (coulomb counting) ----------------
void updateSoc() {
  unsigned long now = millis();
  float dt = (now - lastInteg) / 1000.0f;
  lastInteg = now;
  if (dt <= 0 || dt > 5) return;

  float dmAh = I * dt / 3600.0f;               // + = discharge, - = charge
  soc -= dmAh / capMah * 100.0f;
  if (I > 0) dischMah += dmAh;
  soc = constrain(soc, 0.0f, 100.0f);

  // long-term average current (tau ~90 s) => stable backup estimate
  if (!avgInit) { avgI = I; avgInit = true; }
  else avgI += (I - avgI) * (dt / 90.0f);

  // charge complete: high voltage + current tapered off for 30 s
  if (V >= FULL_VOLT && I < 0 && -I < FULL_TAPER_MA) {
    if (fullTimer == 0) fullTimer = now;
    if (now - fullTimer > 30000) { soc = 100; dischMah = 0; }
  } else fullTimer = 0;

  // empty protection
  if (V < 3.0 && I > 0) soc = 0;

  // long rest: gently re-sync to open-circuit voltage once
  if (fabs(I) < REST_MA) {
    restTimer += dt;
    if (restTimer > 300 && !restSynced) { soc = ocvSoc(V); restSynced = true; }
  } else { restTimer = 0; restSynced = false; }

  saveSoc(false);
}

float runtimeH() {            // hours of backup left (-1 = n/a)
  if (avgI < 8 || millis() < 20000) return -1;
  float h = (soc / 100.0f * capMah) / avgI;
  return h > 99 ? -1 : h;
}
float timeToFullH() {         // hours to full (-1 = n/a)
  if (avgI > -8 || millis() < 20000) return -1;
  float h = ((100.0f - soc) / 100.0f * capMah) / (-avgI);
  return h > 99 ? -1 : h;
}

// ---------------- SENSOR READ ----------------
void readSensors() {
  if (okIna) {
    V = ina.getBusVoltage();                    // volts
    I = ina.getCurrent_mA() * CURRENT_SIGN;     // mA, + = discharging
    P = ina.getPower_mW();
    if (P < 0) P = -P;
  }
  if (okMlx) {
    float a = mlx.readAmbientTempC(), o = mlx.readObjectTempC();
    if (!isnan(a)) Ta = a;
    if (!isnan(o)) To = o;
  }
  if (okImu) {
    sensors_event_t ea, em;
    accel.getEvent(&ea);
    mag.getEvent(&em);
    ax = ea.acceleration.x; ay = ea.acceleration.y; az = ea.acceleration.z;

    float mx = em.magnetic.x, my = em.magnetic.y, mz = em.magnetic.z;
    if (magCal) {                               // collect min/max while user rotates device
      mnx = min(mnx, mx); mxx = max(mxx, mx);
      mny = min(mny, my); mxy = max(mxy, my);
      mnz = min(mnz, mz); mxz = max(mxz, mz);
      if ((long)(millis() - magCalEnd) >= 0) {
        mxo = (mxx + mnx) / 2; myo = (mxy + mny) / 2; mzo = (mxz + mnz) / 2;
        magCal = false; saveCal();
      }
    }
    mx -= mxo; my -= myo; mz -= mzo;

    float r = atan2(ay, az);
    float p = atan2(-ax, sqrt(ay * ay + az * az));
    float xh = mx * cos(p) + mz * sin(p);
    float yh = mx * sin(r) * sin(p) + my * cos(r) - mz * sin(r) * cos(p);
    float h = atan2(yh, xh) * 180.0 / PI;
    if (h < 0) h += 360;

    float rd = r * 180.0 / PI, pd = p * 180.0 / PI;
    rollR  += 0.35 * (rd - rollR);
    pitchR += 0.35 * (pd - pitchR);
    float dh = fmod(h - hdgR + 540.0, 360.0) - 180.0;
    hdgR = fmod(hdgR + 0.25 * dh + 360.0, 360.0);

    G = sqrt(ax * ax + ay * ay + az * az) / 9.80665;
  }
}

// ---------------- HANDLERS ----------------
void sendPage() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", PAGE);
}

void sendData() {
  char buf[560];
  int mc = magCal ? (int)max(0L, (long)(magCalEnd - millis()) / 1000 + 1) : 0;
  snprintf(buf, sizeof(buf),
    "{\"ok_ina\":%s,\"ok_mlx\":%s,\"ok_imu\":%s,"
    "\"v\":%.3f,\"i\":%.1f,\"p\":%.1f,\"ta\":%.2f,\"to\":%.2f,"
    "\"roll\":%.1f,\"pitch\":%.1f,\"hdg\":%.1f,\"g\":%.2f,"
    "\"soc\":%.1f,\"rem\":%.0f,\"dis\":%.1f,\"cap\":%.0f,\"rt\":%.2f,\"tf\":%.2f,"
    "\"mc\":%d,\"up\":%lu}",
    okIna ? "true" : "false", okMlx ? "true" : "false", okImu ? "true" : "false",
    V, I, P, Ta, To,
    wrap180(rollR - r0), wrap180(pitchR - p0), fmod(hdgR - h0 + 360.0, 360.0), G,
    soc, soc / 100.0f * capMah, dischMah, capMah, runtimeH(), timeToFullH(),
    mc, millis() / 1000UL);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buf);
}

void ok() { server.send(200, "text/plain", "OK"); }

void redirectToPortal() {
  server.sendHeader("Location", String("http://") + apIP.toString() + "/", true);
  server.send(302, "text/plain", "");
}

// ---------------- SETUP / LOOP ----------------
void setup() {
  Serial.begin(115200);
  prefs.begin("wear", false);
  r0 = prefs.getFloat("r0", 0); p0 = prefs.getFloat("p0", 0); h0 = prefs.getFloat("h0", 0);
  mxo = prefs.getFloat("mxo", 0); myo = prefs.getFloat("myo", 0); mzo = prefs.getFloat("mzo", 0);
  capMah = prefs.getFloat("cap", BATT_MAH_DEFAULT);
  dischMah = prefs.getFloat("dis", 0);
  float savedSoc = prefs.getFloat("soc", -1);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);   // MLX90614 (SMBus) only works reliably up to 100 kHz
  delay(200);

  Serial.println("I2C scan:");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf("  found 0x%02X\n", a);
  }

  okIna = ina.begin();
  if (okIna) ina.setMaxCurrentShunt(MAX_AMPS, SHUNT_OHMS);
  for (int t = 0; t < 3 && !okMlx; t++) { okMlx = mlx.begin(0x5A, &Wire); delay(100); }
  okImu = accel.begin() && mag.begin();
  if (okImu) mag.enableAutoRange(true);
  Serial.printf("INA226:%d MLX90614:%d LSM303:%d\n", okIna, okMlx, okImu);

  // initial state of charge: saved value, or estimate from voltage
  readSensors();
  if (okIna) {
    float ocv = ocvSoc(V);
    if (savedSoc < 0) soc = ocv;
    else if (fabs(I) < 30 && fabs(ocv - savedSoc) > 20) soc = ocv;   // battery changed / charged while off
    else soc = savedSoc;
  } else soc = savedSoc < 0 ? 50 : savedSoc;
  lastSavedSoc = soc; lastSavedDis = dischMah;
  lastInteg = millis();

  // open hotspot, no password
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID);
  // WiFi.setTxPower(WIFI_POWER_8_5dBm); // uncomment if your C3 board has unstable WiFi

  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(DNS_PORT, "*", apIP);

  server.on("/", sendPage);
  server.on("/data", sendData);

  // 3D calibration
  server.on("/zero", []() { r0 = rollR; p0 = pitchR; h0 = hdgR; saveCal(); ok(); });
  server.on("/zeroreset", []() { r0 = p0 = h0 = 0; saveCal(); ok(); });
  server.on("/magcal", []() {
    mnx = mny = mnz = 1e9; mxx = mxy = mxz = -1e9;
    magCal = true; magCalEnd = millis() + 20000; ok();
  });

  // battery
  server.on("/soc", []() {
    if (server.hasArg("full")) { soc = 100; dischMah = 0; }
    else if (server.hasArg("v")) soc = constrain(server.arg("v").toFloat(), 0.0f, 100.0f);
    saveSoc(true); ok();
  });
  server.on("/cap", []() {
    float c = server.arg("mah").toFloat();
    if (c >= 50 && c <= 20000) { capMah = c; prefs.putFloat("cap", capMah); }
    ok();
  });
  server.on("/resetdis", []() { dischMah = 0; saveSoc(true); ok(); });

  // captive-portal probes -> redirect so the "sign in" popup opens the page
  server.on("/generate_204", redirectToPortal);
  server.on("/gen_204", redirectToPortal);
  server.on("/hotspot-detect.html", redirectToPortal);
  server.on("/library/test/success.html", redirectToPortal);
  server.on("/connecttest.txt", redirectToPortal);
  server.on("/ncsi.txt", redirectToPortal);
  server.on("/fwlink", redirectToPortal);
  server.onNotFound(redirectToPortal);
  server.begin();

  Serial.print("AP ready: "); Serial.println(WiFi.softAPIP());
}

void loop() {
  dns.processNextRequest();
  server.handleClient();
  if (millis() - lastRead >= 80) {
    lastRead = millis();
    readSensors();
    if (okIna) updateSoc();
  }
}
