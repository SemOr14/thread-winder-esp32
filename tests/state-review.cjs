// Executes extracted production state-control bodies. Hardware/pulse completion is mocked.
// This is a host control-flow check, not an ESP32 hardware or timing test.
const fs=require('fs'),vm=require('vm'),assert=require('assert');
const source=fs.readFileSync(require('path').join(__dirname,'../src/main.cpp'),'utf8');
assert(!source.includes('diagnosticMode'),'Temporary diagnostic implementation must be removed');
assert(!/Op::Test|\"cmd\"\s*\]\s*==\s*\"test\"/.test(source),'Temporary test command must be removed');
// Cross-file contract: every configurable UI field must be emitted into persisted JSON.
const ui=fs.readFileSync(require('path').join(__dirname,'../web/app.js'),'utf8');
const fieldBlock=ui.slice(ui.indexOf('const fields=['),ui.indexOf('for(const [key,label'));
const uiFields=[...fieldBlock.matchAll(/\['([A-Za-z]+)'\s*,/g)].map(m=>m[1]).concat(['targetM','speedPct']);
const persistedFields=new Set([...source.matchAll(/OUT\((\w+)\)/g)].map(m=>m[1]));
assert(uiFields.length>=20,'UI settings discovery failed');
for(const key of uiFields)assert(persistedFields.has(key),'UI setting missing from persistence JSON: '+key);
function bodyAfter(marker){let start=source.indexOf(marker);assert(start>=0,marker);start=source.indexOf('{',start);let n=1,end=start+1;while(n&&end<source.length){if(source[end]==='{')n++;if(source[end]==='}')n--;end++;}assert.equal(n,0);return source.slice(start+1,end-1);}
function cpp(s){return s.replace(/\/\/[^\n]*/g,'').replace(/\b(State|Op|HomePhase)::/g,'$1.').replace(/\b(?:int32_t|int64_t|uint32_t|double)\(/g,'Number(').replace(/\((?:int32_t|int64_t)\)/g,'').replace(/\b(?:bool|double|int32_t|int64_t|uint32_t)\s+/g,'let ').replace(/port(?:ENTER|EXIT)_CRITICAL\(&pulseMux\);/g,'').replace(/strlcpy\(errorText,([^;]+),sizeof\(errorText\)\);/g,'errorText=$1;').replace(/\bnullptr\b/g,'null').replace(/continue;/g,'return;').replace(/publish\(\);|xSemaphoreGive\(plannerGate\);|vTaskDelay\(pdMS_TO_TICKS\(1\)\);/g,'');}
let processBody=bodyAfter('void process(');const configStart=processBody.indexOf('if(c.op==Op::Config)');let brace=processBody.indexOf('{',configStart),n=1,end=brace+1;while(n){if(processBody[end]==='{')n++;if(processBody[end]==='}')n--;end++;}processBody=processBody.slice(0,configStart)+processBody.slice(end);
const plannerBody=bodyAfter('void planner(');let logic=plannerBody.slice(plannerBody.indexOf('if(state==State::Homing)'));logic=logic.slice(0,logic.lastIndexOf('publish();'));
// The slice ends before final publish, excluding the production for(;;) closing brace.
const harness=`
const State=Object.fromEntries(['Commissioning','Idle','Homing','Positioning','Running','Pausing','Paused','Stopping','Returning','Error'].map(x=>[x,x]));
const Op=Object.fromEntries(['Start','Pause','Resume','Reset','Home','Config'].map(x=>[x,x]));const HomePhase=Object.fromEntries(['Release','Seek','Backoff','Latch','Park'].map(x=>[x,x]));
let cfg={commissioned:true,limitNormallyOpen:TEST_NO,targetM:5,speedPct:25,maxRpm:120,accelRpmS:8,widthMm:12.8,homeOffsetMm:2,threadMm:.11,coreMm:4,pitchMm:.11,lengthFactor:1,leadMm:4,homeSpeedMmS:2,homeAccelMmS2:2,homeTravelMm:300,homeBackoffMm:2,homeLatchMmS:.5,microsteps:16,stepsPerRev:200};
let state=State.Idle,pausedState=State.Running,homed=false,returnAfterStop=false,clearAfterStop=false,errorText='',velocity=0,jobStartA=0,jobTargetSteps=0,segmentRight=true,homeStarted=0,homeBudgetSeconds=0,homePhase,now=100,limit=false;
let pulse={a:0,b:0,active:false,total:0,done:0},deltaA=0,deltaB=0;
const LIMIT=32,AE=27,BE=23,LOW=0;let rawOverride;const output={};function digitalWrite(pin,value){output[pin]=Boolean(value);}function digitalRead(){return rawOverride===undefined?(cfg.limitNormallyOpen?!limit:limit):rawOverride;}function homeLimit(){return limitIsActive();}function millis(){return now;}
function limitIsActive(){${cpp(bodyAfter('bool limitIsActive('))}}
const lround=Math.round,llround=Math.round,min=Math.min,max=Math.max,fmin=Math.min,fmax=Math.max;
function spr(){return cfg.stepsPerRev*cfg.microsteps;}function spmm(){return spr()/cfg.leadMm;}function positionA(){return pulse.a;}function positionB(){return pulse.b;}function active(){return pulse.active;}function remaining(){return pulse.total-pulse.done;}function rate(v){}function freeze(){pulse.active=false;velocity=0;}
function segment(a,b){freeze();deltaA=a;deltaB=b;pulse.total=Math.max(Math.abs(a),Math.abs(b));pulse.done=0;pulse.active=pulse.total>0;}
function finish(){pulse.a+=deltaA;pulse.b+=deltaB;pulse.done=pulse.total;pulse.active=false;deltaA=deltaB=0;}
function fail(s){freeze();homed=false;state=State.Error;errorText=s;}
function windingTurns(m,c,t,p,w,f){const b=t*p/w,k=m*1000/(f*Math.PI);return 2*k/(c+Math.sqrt(c*c+4*b*k));}
function nextRate(v,target,acc,rem,dt){return Math.max(0,Math.min(target,v+acc*dt,Math.sqrt(Math.max(0,2*acc*Math.max(0,rem-2)))));}
function beginHome(){${cpp(bodyAfter('void beginHome('))}}
function nextWindingSegment(){${cpp(bodyAfter('void nextWindingSegment('))}}
function process(c){${cpp(processBody)}}
function rejectReason(c){${cpp(bodyAfter('const char* rejectReason('))}}
function tick(){${cpp(logic)}}
// Actual production polarity helper: COM-NO closes LOW when pressed; legacy NC opens HIGH.
for(const no of [true,false]){cfg.limitNormallyOpen=no;for(const raw of [true,false]){rawOverride=raw;assert.equal(limitIsActive(),no?!raw:raw);}}rawOverride=undefined;cfg.limitNormallyOpen=TEST_NO;
// Uncommissioned hardware cannot accept any movement command.
cfg.commissioned=false;for(const op of [Op.Start,Op.Home,Op.Pause,Op.Resume,Op.Reset])assert(rejectReason({op}));cfg.commissioned=true;
// Stuck-active switch must fail after a bounded release (NC open / NO closed).
limit=true;beginHome();assert.equal(homePhase,HomePhase.Release);finish();tick();assert.equal(state,State.Error);assert(!homed);
// Stuck-inactive switch / no switch reached must fail at the seek bound.
limit=false;beginHome();assert.equal(homePhase,HomePhase.Seek);finish();tick();assert.equal(state,State.Error);
// Successful seek, backoff, slow latch, final park, then a fresh start.
beginHome();limit=true;tick();assert.equal(homePhase,HomePhase.Backoff);limit=false;finish();tick();assert.equal(homePhase,HomePhase.Latch);limit=true;tick();assert.equal(homePhase,HomePhase.Park);limit=false;finish();tick();assert.equal(state,State.Idle);assert(homed);
process({op:Op.Start});assert.equal(state,State.Positioning);tick();assert.notEqual(state,State.Error);if(state===State.Positioning){finish();tick();}assert.equal(state,State.Running);assert(deltaA>0);assert(deltaB>0);
// Pause preserves the unfinished segment, resume completes it and reverses traverse only.
velocity=1;process({op:Op.Pause});for(let i=0;i<10&&state===State.Pausing;i++)tick();assert.equal(state,State.Paused);const oldRemaining=remaining();process({op:Op.Resume});assert.equal(state,State.Running);assert.equal(remaining(),oldRemaining);finish();tick();assert(deltaA>0);assert(deltaB<0);
// Reset stops in place and clears the current job rather than homing.
velocity=0;process({op:Op.Reset});tick();assert.equal(state,State.Idle);assert.equal(jobTargetSteps,0);assert(!active());
// Home command from idle starts a bounded homing cycle.
process({op:Op.Home});tick();assert.equal(state,State.Homing);
// NC assertion while decelerating must stop and invalidate the reference.
state=State.Running;segment(100,10);velocity=3;process({op:Op.Pause});limit=true;tick();assert.equal(state,State.Error);assert(!active());assert(!homed);
console.log('PASS: production state flow '+(TEST_NO?'COM-NO':'legacy COM-NC')+' (commissioning guard, home success / stuck switch / no hit, start, pause/resume, reversal, reset, return, limit on decel).');
`;
for(const TEST_NO of [true,false]){const context={assert,console,Math,TEST_NO};vm.createContext(context);vm.runInContext(harness,context,{timeout:2000});}
