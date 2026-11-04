import {UUID,JsonLines,encodeCommand,writeChunks,validateRecipe,isIdle} from './transport.mjs';
const $=id=>document.getElementById(id);
let mode=null,device=null,rx=null,poller=null,snapshot=null,lastSeen=0,busy=false,pending=Promise.resolve(),bleListener=null,txCharacteristic=null,sequence=0,generation=0;
const acknowledgements=new Map();
const labels={commissioning:'Первичная настройка',idle:'Готов к работе',homing:'Поиск концевика',positioning:'Выход к началу намотки',running:'Намотка',pausing:'Плавная остановка',paused:'На паузе',stopping:'Остановка и сброс',returning:'Возврат к концевику',error:'Ошибка станка'};
const fields=[
 ['maxRpm','Максимальная скорость шпинделя, об/мин',1,150,1],['accelRpmS','Ускорение шпинделя, об/мин за секунду',0.1,30,0.1],
 ['widthMm','Ширина намотки, мм',1,50,0.1],['homeOffsetMm','Отступ начала от концевика, мм',0.5,100,0.1],
 ['threadMm','Диаметр нити, мм',0.02,1,0.001],['coreMm','Начальный диаметр катушки, мм',1,30,0.1],
 ['pitchMm','Шаг укладки за оборот, мм',0.02,1,0.001],['lengthFactor','Коэффициент калибровки длины',0.5,2,0.001],
 ['microsteps','Микрошаги A4988',1,16,1],['stepsPerRev','Полных шагов двигателя на оборот',200,400,200],
 ['leadMm','Ход ШВП за оборот, мм',1,20,0.1],['homeSpeedMmS','Скорость поиска дома, мм/с',0.1,5,0.1],
 ['homeAccelMmS2','Ускорение поиска дома, мм/с²',0.1,10,0.1],['homeTravelMm','Предельный путь поиска дома, мм',5,500,1],
 ['homeBackoffMm','Отъезд от концевика, мм',0.5,10,0.1],['homeLatchMmS','Скорость точного подхода, мм/с',0.1,1,0.1],
 ['limitNormallyOpen','Концевик подключён к COM–NO (при нажатии замыкается)'],['invertSpindle','Изменить направление шпинделя'],['invertTraverse','Изменить направление каретки'],['commissioned','Подключение и направления проверены: разрешить работу и автопоиск дома']
];
for(const [key,label,min,max,step] of fields){const el=document.createElement('label');el.textContent=label;const input=document.createElement('input');input.id='cfg_'+key;input.name=key;if(min===undefined){input.type='checkbox';el.className='check';el.prepend(input);}else{input.type='number';input.min=min;input.max=max;input.step=step;input.required=true;el.append(input);} $('fields').append(el);}
function notice(text,error=false){$('notice').textContent=text;$('notice').classList.toggle('error',error);$('notice').hidden=!text;}
function connected(){return mode&&lastSeen&&Date.now()-lastSeen<6000;}
function render(){const online=connected(),s=snapshot;const idle=s&&isIdle(s.state),paused=s?.state==='paused',running=['running','positioning'].includes(s?.state);$('connection').textContent=online?(mode==='ble'?'Bluetooth подключён':'Wi-Fi подключён'):'Не подключён';$('connection').classList.toggle('online',Boolean(online));$('disconnect').hidden=!mode;$('ble').disabled=busy||Boolean(mode);$('wifi').disabled=busy||Boolean(mode);$('start').disabled=!online||busy||!s||!['idle','running','positioning','paused'].includes(s.state)||!s.config?.commissioned;$('start').textContent=running?'Пауза':paused?'Продолжить':'Начать намотку';$('reset').disabled=!online||busy||!s||!s.config?.commissioned||s.state==='stopping';$('home').disabled=!online||busy||!s||!s.config?.commissioned||['homing','returning','stopping','pausing'].includes(s.state);$('saveRecipe').disabled=!online||busy||!idle;$('saveConfig').disabled=!online||busy||!idle;for(const el of document.querySelectorAll('#recipe input,#configForm input'))el.disabled=!online||busy||!idle;$('state').textContent=online?(labels[s?.state]||s?.state||'Получение данных'):'Ожидание подключения';if(!s)return;$('homeBadge').textContent=s.homed?'Позиция найдена':'Позиция неизвестна';$('homeBadge').classList.toggle('online',Boolean(s.homed));$('length').textContent=Number(s.lengthM||0).toLocaleString('ru-RU',{minimumFractionDigits:2,maximumFractionDigits:2});const pct=Math.min(100,Math.max(0,100*(s.lengthM||0)/(s.targetM||1)));$('progress').value=pct;$('percent').textContent=Math.floor(pct)+'%';$('goal').textContent='Цель '+Number(s.targetM||0).toLocaleString('ru-RU')+' м';$('position').textContent=Number(s.positionMm||0).toLocaleString('ru-RU',{maximumFractionDigits:2})+' мм';$('limit').textContent=typeof s.limitActive==='boolean'?(s.limitActive?'Нажат':'Свободен'):'Нет показания';$('raw').textContent=JSON.stringify(s,null,2);}
function applyConfig(c){if(!c)return;$('target').value=c.targetM;$('speed').value=c.speedPct;$('speedNumber').value=c.speedPct;for(const [key]of fields){const input=$('cfg_'+key);if(key in c){if(input.type==='checkbox')input.checked=c[key];else input.value=c[key];}}}
function receive(data){if(!data||typeof data!=='object')return;const ack=acknowledgements.get(data.id);if(ack){clearTimeout(ack.timer);acknowledgements.delete(data.id);if(data.ok===false)ack.reject(Error(data.error||'Команда отклонена контроллером'));else ack.resolve(data);}if(data.ok===false){notice(data.error||'Команда отклонена контроллером',true);return;}if(!data.state)return;const first=!snapshot;snapshot=data;lastSeen=Date.now();if(first)applyConfig(data.config);if(data.error)notice(data.error,true);else if(data.state==='commissioning')notice('Станок заблокирован до первичной проверки. Для настроек нажмите пять раз на название «Намотчик».');render();}
async function request(path,options={}){const controller=new AbortController();const timer=setTimeout(()=>controller.abort(),4000);try{const response=await fetch(path,{...options,signal:controller.signal,cache:'no-store'});const data=await response.json();if(!response.ok||data.ok===false)throw Error(data.error||'Ошибка контроллера '+response.status);return data;}finally{clearTimeout(timer);}}
async function poll(){if(mode!=='http')return;const session=generation;try{const data=await request('./api/status');if(session===generation&&mode==='http')receive(data);}catch(e){if(session===generation&&mode==='http'){notice('Нет ответа от станка: '+e.message,true);render();}}}
async function send(command){if(!connected())throw Error('Сначала подключитесь к станку');if(mode==='http'){const data=await request('./api/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(command)});receive(data);await poll();}else{const id=++sequence,session=generation,characteristic=rx;const bytes=encodeCommand({...command,id});let acknowledgement;const task=pending.then(async()=>{if(session!==generation||characteristic!==rx)throw Error('Соединение изменилось. Подключитесь снова.');acknowledgement=new Promise((resolve,reject)=>{const timer=setTimeout(()=>{acknowledgements.delete(id);reject(Error('Нет подтверждения команды. Проверьте показания станка перед повтором.'));},5000);acknowledgements.set(id,{resolve,reject,timer});});acknowledgement.catch(()=>{});try{await writeChunks(characteristic,bytes);return await acknowledgement;}catch(e){const ack=acknowledgements.get(id);if(ack){clearTimeout(ack.timer);acknowledgements.delete(id);ack.reject(e);}throw e;}});pending=task.catch(()=>{});await task;}}
function cancelAcknowledgements(){generation++;for(const ack of acknowledgements.values()){clearTimeout(ack.timer);ack.reject(Error('Соединение прервано.'));}acknowledgements.clear();}
async function action(fn){if(busy)return;busy=true;render();try{notice('');await fn();}catch(e){notice(e.name==='NotFoundError'?'Подключение отменено.':e.message,true);}finally{busy=false;render();}}
const connectHintDefault='Выберите Bluetooth или откройте эту страницу в сети станка.';
function detachBle(){if(device)device.removeEventListener('gattserverdisconnected',lost);if(txCharacteristic&&bleListener)txCharacteristic.removeEventListener('characteristicvaluechanged',bleListener);txCharacteristic=null;bleListener=null;}
async function disconnect(){cancelAcknowledgements();mode=null;lastSeen=0;snapshot=null;clearInterval(poller);poller=null;const previous=device;detachBle();device=null;rx=null;pending=Promise.resolve();previous?.gatt?.disconnect();$('connectHint').textContent=connectHintDefault;render();}
function lost(){cancelAcknowledgements();mode=null;lastSeen=0;snapshot=null;rx=null;detachBle();device=null;pending=Promise.resolve();$('connectHint').textContent=connectHintDefault;notice('Bluetooth отключён. Используйте кнопку на станке для остановки; затем подключитесь снова.',true);render();}
function validStatus(data){return data&&typeof data==='object'&&Object.hasOwn(labels,data.state)&&typeof data.homed==='boolean'&&Number.isFinite(data.lengthM)&&data.lengthM>=0&&Number.isFinite(data.positionMm)&&validateRecipe(data.targetM,data.speedPct)&&data.config&&typeof data.config==='object'&&validateRecipe(data.config.targetM,data.config.speedPct)&&fields.every(([key,,min,max])=>min===undefined?typeof data.config[key]==='boolean':Number.isFinite(data.config[key])&&data.config[key]>=min&&data.config[key]<=max)&&[1,2,4,8,16].includes(data.config.microsteps)&&[200,400].includes(data.config.stepsPerRev);}
function timeout(promise,ms,text){let timer;return Promise.race([promise,new Promise((_,reject)=>{timer=setTimeout(()=>reject(Error(text)),ms);})]).finally(()=>clearTimeout(timer));}
async function connectBle(){
 if(!navigator.bluetooth)throw Error('Web Bluetooth недоступен. Используйте Chrome на Windows или Android и HTTPS-страницу, либо Wi-Fi станка.');
 await disconnect();
 const selected=await navigator.bluetooth.requestDevice({filters:[{services:[UUID.service]}]});
 const session=generation;let failure=null;
 for(let attempt=1;attempt<=3;attempt++){
  let stage='подключение к устройству',tx=null,listener=null,active=true,ready=false,rejectFirst,resolveFirst,rejectInterrupted;
  const firstStatus=new Promise((resolve,reject)=>{rejectFirst=reject;resolveFirst=resolve;});
  const interrupted=new Promise((_,reject)=>{rejectInterrupted=reject;});
  const onInitialLost=()=>{const error=Error('Связь прервалась до получения показаний');rejectFirst(error);rejectInterrupted(error);};
  firstStatus.catch(()=>{});interrupted.catch(()=>{});
  const step=async(promise)=>{const value=await timeout(Promise.race([promise,interrupted]),8000,'Устройство не ответило');if(!active||session!==generation)throw Error('Подключение отменено');return value;};
  if(selected.gatt.connected)selected.gatt.disconnect();
  selected.addEventListener('gattserverdisconnected',onInitialLost);
  try{
   $('connectHint').textContent='Bluetooth: попытка '+attempt+' из 3 · подключение';
   const server=await step(selected.gatt.connect());
   stage='поиск службы';$('connectHint').textContent='Bluetooth: попытка '+attempt+' из 3 · поиск службы';
   if(!server.connected)throw Error('GATT-соединение разорвано');
   const service=await step(server.getPrimaryService(UUID.service));
   stage='подготовка каналов';const writeCharacteristic=await step(service.getCharacteristic(UUID.rx));tx=await step(service.getCharacteristic(UUID.tx));
   const parser=new JsonLines(data=>{if(!active||session!==generation)return;if(ready)receive(data);else if(validStatus(data))resolveFirst(data);});
   listener=event=>{if(!active||session!==generation)return;try{const v=event.target.value;parser.push(new Uint8Array(v.buffer,v.byteOffset,v.byteLength));}catch(e){if(ready)notice('Ошибка данных Bluetooth: '+e.message,true);/* Initial subscribe may start midway through an already-sent status line. Ignore its tail and await the next complete validated status within the same timeout. */}};
   tx.addEventListener('characteristicvaluechanged',listener);
   stage='включение уведомлений';await step(tx.startNotifications());
   stage='получение первых показаний';$('connectHint').textContent='Bluetooth: попытка '+attempt+' из 3 · ожидание показаний';
   await step(writeChunks(writeCharacteristic,encodeCommand({cmd:'get'})));
   const initial=await step(timeout(firstStatus,4000,'Нет первых показаний контроллера'));
   if(!selected.gatt.connected)throw Error('Связь прервалась при получении показаний');
   selected.removeEventListener('gattserverdisconnected',onInitialLost);selected.addEventListener('gattserverdisconnected',lost);
   device=selected;rx=writeCharacteristic;txCharacteristic=tx;bleListener=listener;ready=true;mode='ble';$('connectHint').textContent=connectHintDefault;receive(initial);return;
  }catch(e){failure=Error('Bluetooth: '+stage+', попытка '+attempt+' из 3. '+(e.message||'Ошибка соединения'));}
  finally{if(!ready){active=false;selected.removeEventListener('gattserverdisconnected',onInitialLost);if(tx&&listener)tx.removeEventListener('characteristicvaluechanged',listener);try{selected.gatt.disconnect();}catch{}}}
  if(session!==generation)throw Error('Подключение отменено');
  if(attempt<3)await new Promise(resolve=>setTimeout(resolve,400));
 }
 $('connectHint').textContent=connectHintDefault;render();
 throw Error(failure.message+' Проверьте питание ESP32 и расстояние до неё, закройте другие подключения к станку и повторите. Команды движения не отправлялись.');
}
$('ble').onclick=()=>action(connectBle);
$('wifi').onclick=()=>action(async()=>{if(location.protocol!=='http:'||!['192.168.4.1','threadwinder.local','localhost','127.0.0.1'].includes(location.hostname))throw Error('Подключитесь к сети ThreadWinder (пароль winding12345) и откройте http://192.168.4.1. На GitHub Pages доступен Bluetooth.');await disconnect();receive(await request('./api/status'));mode='http';lastSeen=Date.now();poller=setInterval(poll,1000);render();});
$('disconnect').onclick=()=>action(disconnect);
$('start').onclick=()=>action(async()=>{if(['running','positioning'].includes(snapshot.state))return send({cmd:'pause'});if(snapshot.state==='paused')return send({cmd:'resume'});const targetM=Number($('target').value),speedPct=Number($('speedNumber').value);if(!validateRecipe(targetM,speedPct))throw Error('Длина: 1–40 м, скорость: целое число от 1 до 100%.');if(targetM!==snapshot.targetM||speedPct!==snapshot.speedPct)await send({cmd:'config',config:{targetM,speedPct}});await send({cmd:'start'});});
$('reset').onclick=()=>action(()=>send({cmd:'reset'}));$('home').onclick=()=>action(()=>send({cmd:'home'}));
$('speed').oninput=()=>{$('speedNumber').value=$('speed').value;};$('speedNumber').oninput=()=>{if($('speedNumber').validity.valid)$('speed').value=$('speedNumber').value;};
$('recipe').onsubmit=event=>{event.preventDefault();action(async()=>{const targetM=Number($('target').value),speedPct=Number($('speedNumber').value);if(!validateRecipe(targetM,speedPct))throw Error('Длина: 1–40 м, скорость: целое число от 1 до 100%.');await send({cmd:'config',config:{targetM,speedPct}});notice('Параметры сохранены в памяти ESP32.');});};
$('configForm').onsubmit=event=>{event.preventDefault();action(async()=>{const config={targetM:Number($('target').value),speedPct:Number($('speedNumber').value)};for(const [key]of fields){const input=$('cfg_'+key);config[key]=input.type==='checkbox'?input.checked:Number(input.value);}if(![1,2,4,8,16].includes(config.microsteps))throw Error('Микрошаги A4988: 1, 2, 4, 8 или 16.');await send({cmd:'config',config});notice('Настройки сохранены в памяти ESP32 и будут восстановлены после включения.');});};
function openAdvanced(){$('advanced').hidden=false;applyConfig(snapshot?.config);$('advanced').scrollIntoView({behavior:'smooth',block:'start'});}$('advancedOpen').onclick=openAdvanced;$('advancedClose').onclick=()=>{$('advanced').hidden=true;};let taps=[];$('title').onclick=()=>{taps=taps.filter(t=>Date.now()-t<2500);taps.push(Date.now());if(taps.length>=5){taps=[];openAdvanced();}};$('title').onkeydown=e=>{if(e.key==='Enter')$('title').click();};
$('refresh').onclick=()=>action(()=>send({cmd:'get'}));setInterval(()=>{if(mode==='ble'&&rx&&Date.now()-lastSeen>2000&&!busy){const session=generation,characteristic=rx;pending=pending.then(()=>{if(session===generation&&characteristic===rx)return writeChunks(characteristic,encodeCommand({cmd:'get'}));}).catch(e=>{if(session===generation)notice(e.message,true);});}render();},1500);render();
if(location.protocol==='http:'&&location.hostname==='192.168.4.1')$('wifi').click();
