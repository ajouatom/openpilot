import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {createHash,webcrypto} from 'node:crypto';
import vm from 'node:vm';
import {validateManifest,verifyBytes,binaryString} from '../manifest.mjs';

const code=(await readFile(new URL('../installer.js',import.meta.url),'utf8')).replace(/^import .*;\n/gm,'');
async function run({mode='update',fault=''}={}) {
  const nodes=new Map();
  const events=[];
  for(const id of ['status','log','progress','flash','board','mode']) nodes.set(id,{textContent:'',value:0,checked:true,addEventListener(event,handler){this[event]=handler;}});
  nodes.get('mode').value=mode;
  const bytes=new Uint8Array([0,1,2,3]);
  const manifest={chip:'ESP32-S3',board:'N16R8',parts:[['bootloader.bin',0],['partition-table.bin',0x8000],['carrot-s3.bin',0x10000]].map(([path,offset])=>({path,offset,size:4,sha256:createHash('sha256').update(bytes).digest('hex'),md5:createHash('md5').update(bytes).digest('hex')}))};
  let nvsReads=0;
  class Transport {async disconnect(){events.push('disconnect');}}
  class ESPLoader {
    constructor(){this.chip={CHIP_NAME:fault==='chip'?'ESP32':'ESP32-S3'};}
    async main(){events.push('connect');}
    async getFlashSize(){return 16384;}
    async readFlash(address,size){assert.equal(address,0x8000);assert.equal(size,4);return fault==='partition'?new Uint8Array(4):bytes;}
    async flashMd5sum(address,size){assert.equal(address,0x9000);assert.equal(size,0x6000);return fault==='nvs'&&nvsReads++?'changed':'same';}
    async writeFlash(options){
      assert.equal(options.eraseAll,false);assert.equal(options.flashMode,'keep');
      const part=options.fileArray[0];assert.notEqual(part.address,0x9000);
      assert.equal(options.calculateMD5Hash(part.data),createHash('md5').update(Buffer.from(part.data,'binary')).digest('hex'));
      events.push(part.address);options.reportProgress(0,4,4);
    }
    async after(mode){assert.equal(mode,'hard_reset');events.push('reset');}
  }
  const context={document:{getElementById:id=>nodes.get(id)},isSecureContext:true,navigator:{serial:{async requestPort(){return {};}}},
    Transport,ESPLoader,validateManifest,verifyBytes,binaryString,crypto:webcrypto,Uint8Array,
    fetch:async path=>({ok:true,json:async()=>manifest,arrayBuffer:async()=>fault==='download'?new Uint8Array(4).buffer:bytes.buffer})};
  vm.runInNewContext(code,context);
  await nodes.get('flash').click();
  return {events,status:nodes.get('status').textContent,enabled:!nodes.get('flash').disabled};
}
test('update writes app only; initial install writes disjoint partitions',async()=>{
  for(const mode of ['update','install']){
    const result=await run({mode});
    assert.deepEqual(result.events,['connect',...(mode==='update'?[0x10000]:[0,0x8000,0x10000]),'reset','disconnect']);
    assert.match(result.status,/설치 완료/);assert.equal(result.enabled,true);
  }
});
test('wrong chip, bad partition and corrupt downloads cannot reach flash',async()=>{
  for(const fault of ['chip','partition','download']){
    const result=await run({fault});assert.equal(result.events.some(x=>typeof x==='number'),false);
    assert.match(result.status,/설치 중단/);assert.equal(result.enabled,true);
  }
});
test('NVS mismatch reports failure and releases the port',async()=>{
  const result=await run({fault:'nvs'});assert.match(result.status,/저장 영역 검증/);
  assert.equal(result.events.at(-1),'disconnect');assert.equal(result.events.includes('reset'),false);
});
