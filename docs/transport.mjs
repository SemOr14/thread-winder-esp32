export const UUID = Object.freeze({service:'7b4e0001-6b2d-4e53-9c71-9a03f30c8a10',rx:'7b4e0002-6b2d-4e53-9c71-9a03f30c8a10',tx:'7b4e0003-6b2d-4e53-9c71-9a03f30c8a10'});
export class JsonLines {
 constructor(onMessage){this.onMessage=onMessage;this.decoder=new TextDecoder();this.buffer='';}
 push(bytes){this.buffer+=this.decoder.decode(bytes,{stream:true});if(this.buffer.length>16384){this.buffer='';throw Error('Слишком длинный ответ контроллера');}let n;while((n=this.buffer.indexOf('\n'))>=0){const line=this.buffer.slice(0,n).trim();this.buffer=this.buffer.slice(n+1);if(line)this.onMessage(JSON.parse(line));}}
}
export function encodeCommand(command){const bytes=new TextEncoder().encode(JSON.stringify(command)+'\n');if(bytes.length>1024)throw Error('Команда превышает допустимый размер');return bytes;}
export async function writeChunks(characteristic,bytes){for(let i=0;i<bytes.length;i+=20)await characteristic.writeValueWithResponse(bytes.slice(i,i+20));}
export function validateRecipe(target,speed){return Number.isFinite(target)&&target>=1&&target<=40&&Number.isInteger(speed)&&speed>=1&&speed<=100;}
export function isIdle(state){return ['idle','commissioning','error'].includes(state);}
