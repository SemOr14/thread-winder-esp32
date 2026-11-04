const fs=require('node:fs'),assert=require('node:assert/strict');
const src=fs.readFileSync(require('node:path').join(__dirname,'../src/main.cpp'),'utf8');
const config=src.match(/struct Config \{([\s\S]*?)\n\};/)[1];
const defaults={};for(const m of config.matchAll(/\b(\w+)=([\d.]+|true|false)/g))defaults[m[1]]=JSON.parse(m[2].startsWith('.')?'0'+m[2]:m[2]);
const encoder=src.match(/void configJson\(JsonObject o,const Config& c\)\{([\s\S]*?)\n\}/)[1];
const fields=[...encoder.matchAll(/OUT\((\w+)\);/g)].map(m=>m[1]);
assert.equal(fields.length,22);assert.deepEqual([...fields].sort(),Object.keys(defaults).sort(),'every Config member must persist');
const changed={...defaults,targetM:40,speedPct:67,maxRpm:80,accelRpmS:5,widthMm:13,homeOffsetMm:3,threadMm:.12,coreMm:4.2,pitchMm:.12,lengthFactor:1.04,leadMm:4,homeSpeedMmS:1.5,homeAccelMmS2:1,homeTravelMm:200,homeBackoffMm:1.5,homeLatchMmS:.3,microsteps:8,stepsPerRev:400,invertSpindle:true,invertTraverse:true,commissioned:true};
for(const settings of [defaults,changed]){let encoded=JSON.stringify({schema:2,config:Object.fromEntries(fields.map(k=>[k,settings[k]]))});assert.deepEqual(JSON.parse(encoded).config,settings);assert(Buffer.byteLength(encoded)<2048);}
assert(src.includes('p.putString("config",data)==data.length()'),'check durable write byte count');
assert(src.includes('p.getString("config","")==data'),'readback verification');
assert(src.includes('doc["config"].size()==22'),'reject incomplete boot record');
assert(src.includes('doc["schema"]==2'),'versioned current boot record');assert(src.includes('if(legacy)loaded.limitNormallyOpen=false'),'legacy migration preserves NC polarity');
assert(src.includes('acceptedCommand.store(!why&&commandSucceeded)'),'failed persistence cannot ACK success');
console.log('PASS: all22 Config members/default and changed roundtrips, schema2/legacyNC migration coverage and durable ACK guards (host contract; not NVS hardware test)');
