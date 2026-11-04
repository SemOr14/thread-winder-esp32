const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const root=require('node:path').join(__dirname,'..');
// Execute the actual portable C++ arithmetic bodies as JavaScript; both use IEEE doubles.
let math=fs.readFileSync(root+'/include/motion_math.h','utf8').replace(/^#.*$/mg,'').replace(/inline double (\w+)\(([^)]*)\)/g,(_,n,p)=>`function ${n}(${p.replace(/double /g,'')})`).replace(/const double /g,'const ');
const ctx={sqrt:Math.sqrt,fmax:Math.max,fmin:Math.min};vm.createContext(ctx);vm.runInContext(math,ctx);
for(const core of [1,4,30])for(const thread of [.02,.11,1])for(const length of [1,5,40]){
 let n=ctx.windingTurns(length,core,thread,.11,12.8,1);assert(n>0);assert(Math.abs(ctx.windingLength(n,core,thread,.11,12.8,1)-length)<1e-10);
}
const source=fs.readFileSync(root+'/src/main.cpp','utf8');let start=source.indexOf('void IRAM_ATTR onTimer(){')+'void IRAM_ATTR onTimer(){'.length,depth=1,end=start;
while(depth){if(source[end]==='{')depth++;if(source[end]==='}')depth--;end++;}
let body=source.slice(start,end-1).replace(/portENTER_CRITICAL_ISR\(&pulseMux\);|portEXIT_CRITICAL_ISR\(&pulseMux\);/g,'').replace(/GPIO.out_w1tc=\(1UL<<AS\)\|\(1UL<<BS\);/g,'digitalWrite(AS,LOW);digitalWrite(BS,LOW);').replace(/GPIO.out_w1ts=1UL<<(AS|BS);/g,'digitalWrite($1,HIGH);').replace(/(\d+)U\b/g,'$1');
ctx.AS=25;ctx.BS=18;ctx.LOW=0;ctx.HIGH=1;let tick=0,edges={25:[],18:[]};ctx.digitalWrite=(pin,value)=>{if(value)edges[pin].push(tick)};
vm.runInContext('function onTimer(){'+body+'}',ctx);
function segment(a,b){ctx.pulse={a:0,b:0,na:Math.abs(a),nb:Math.abs(b),total:Math.max(Math.abs(a),Math.abs(b)),done:0,ea:0,eb:0,phase:0,rateMilli:0,sa:Math.sign(a),sb:Math.sign(b),active:true};tick=0;edges={25:[],18:[]};}
function runTicks(n){for(let i=0;i<n;i++){ctx.onTimer();tick++;}}
for(const [a,b] of [[32000,1000],[32000,-1000],[0,-1600],[7,3],[3,-7]]){
 segment(a,b);ctx.pulse.rateMilli=8000000;runTicks(ctx.pulse.total*4+10);assert.equal(ctx.pulse.active,false);assert.equal(ctx.pulse.a,a);assert.equal(ctx.pulse.b,b);for(const pin of [25,18])for(let i=1;i<edges[pin].length;i++)assert(edges[pin][i]-edges[pin][i-1]>=3,'pulse high/low width at max rate');
}
segment(32000,-10240);let v=0;let paused=false,finished=false;
for(let ms=0;ms<200000;ms++){
 let rem=ctx.pulse.total-ctx.pulse.done;
 if(ms>=5000&&ms<11000){v=Math.max(0,v-400*.001);if(v===0)paused=true;}
 else {v=ctx.nextRate(v,2000,400,rem,.001);if(rem<=2)v=Math.min(2000,Math.max(v,5));}
 ctx.pulse.rateMilli=Math.round(v*1000);runTicks(25);
 assert(ctx.pulse.b<=0 && ctx.pulse.b>=-10240);assert(ctx.pulse.a>=0 && ctx.pulse.a<=32000);
 if(!ctx.pulse.active){finished=true;break;}
}
assert(paused,'pause reaches full rest');assert(finished,'resume reaches exact segment endpoint');assert.equal(ctx.pulse.a,32000);assert.equal(ctx.pulse.b,-10240);
for(let rem=0;rem<1000;rem+=3)for(let v=0;v<1000;v+=37){let n=ctx.nextRate(v,500,100,rem,.001);assert(n>=0);assert(Math.abs(n-v)<=.10000001);}
console.log('PASS: length inversion, actual ISR DDA endpoints, pulse widths, paused/resumed synchronized bounds, acceleration bounds');
// Execute the actual homing branches with controlled pulse-completion and switch inputs.
function blockAfter(marker){let p=source.indexOf(marker);assert(p>=0,marker);p=source.indexOf('{',p)+1;let level=1,e=p;while(level){if(source[e]==='{')level++;if(source[e]==='}')level--;e++;}return source.slice(p,e-1);}
function js(cpp){return cpp.replace(/portENTER_CRITICAL\(&pulseMux\);|portEXIT_CRITICAL\(&pulseMux\);/g,'').replace(/\b(bool|double|int32_t) /g,'let ').replace(/(State|HomePhase|Op)::/g,'$1.').replace(/nullptr/g,'null');}
let home=source.slice(source.indexOf('    bool hit=homeLimit();'),source.indexOf('\n   }\n  }else if(state==State::Pausing'));
ctx.State=Object.fromEntries(['Homing','Idle','Running','Positioning','Error','Paused','Commissioning','Stopping','Pausing'].map(x=>[x,x]));ctx.HomePhase=Object.fromEntries(['Release','Seek','Backoff','Latch','Park'].map(x=>[x,x]));ctx.Op=Object.fromEntries(['Start','Config','Pause','Resume','Reset','Home'].map(x=>[x,x]));
ctx.cfg={homeBackoffMm:2,homeTravelMm:300,homeSpeedMmS:2,homeLatchMmS:.5,homeAccelMmS2:2,targetM:5,coreMm:4,threadMm:.11,pitchMm:.11,widthMm:12.8,lengthFactor:1,homeOffsetMm:2,maxRpm:120,speedPct:25,accelRpmS:8};
let moving=false,hit=false,failed=false;
Object.assign(ctx,{state:'Homing',homePhase:'Release',velocity:0,homed:false,errorText:[],pulse:{b:0,a:0},homeLimit:()=>hit,active:()=>moving,remaining:()=>100,spmm:()=>800,spr:()=>3200,lround:Math.round,llround:Math.round,fmin:Math.min,fmax:Math.max,positionA:()=>ctx.pulse.a,positionB:()=>ctx.pulse.b,freeze:()=>{moving=false;ctx.velocity=0;},segment:(a,b)=>{moving=true;ctx.lastSegment=[a,b]},rate:()=>{},fail:()=>{failed=true;ctx.state='Error';}});
vm.runInContext('function homeTick(){'+js(home)+'}',ctx);
ctx.homeTick();assert.equal(ctx.homePhase,'Seek');hit=true;ctx.homeTick();assert.equal(ctx.homePhase,'Backoff');hit=false;moving=false;ctx.homeTick();assert.equal(ctx.homePhase,'Latch');hit=true;ctx.homeTick();assert.equal(ctx.homePhase,'Park');assert.equal(ctx.pulse.b,0);assert.equal(ctx.lastSegment[1],1600);
hit=false;moving=false;ctx.pulse.b=1600;ctx.homeTick();assert.equal(ctx.state,'Idle');assert.equal(ctx.homed,true);assert.equal(failed,false);
ctx.strlcpy=()=>{};ctx.digitalRead=()=>hit;ctx.limitIsActive=()=>hit;ctx.LIMIT=32;
vm.runInContext('function startJob(){'+js(blockAfter('if(c.op==Op::Start && state==State::Idle)'))+'}',ctx);
ctx.startJob();assert.equal(ctx.state,'Positioning');assert.equal(ctx.lastSegment[1],0);
ctx.nextWindingSegment=()=>{ctx.state='Running'};moving=false;
vm.runInContext('function positioningTick(){'+js(blockAfter('}else if(state==State::Running || state==State::Positioning)'))+'}',ctx);
ctx.positioningTick();assert.equal(ctx.state,'Running');assert.equal(failed,false,'homing must release switch before next start');
console.log('PASS: actual Home release/seek/backoff/latch/park → Idle → Start → Running branches');
