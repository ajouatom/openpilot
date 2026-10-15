import {test} from 'node:test';
import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {validateManifest, verifyBytes, binaryString} from '../manifest.mjs';
const bytes=new Uint8Array([0,128,255,42]);
const file=path=>({path,size:4,sha256:createHash('sha256').update(bytes).digest('hex'),md5:'a'.repeat(32)});
function manifest(){return {chip:'ESP32-S3',board:'N16R8',parts:[
  {...file('bootloader.bin'),offset:0},{...file('partition-table.bin'),offset:0x8000},{...file('carrot-s3.bin'),offset:0x10000}]};}
test('release partitions cannot erase NVS or overflow app',()=>{
  assert.equal(validateManifest(manifest()).parts.length,3);
  for(const [index,changes] of [[0,{offset:0x9000}],[1,{size:0x2000}],[2,{size:0x300004}],[2,{path:'../private.bin'}]]){
    const value=manifest();Object.assign(value.parts[index],changes);assert.throws(()=>validateManifest(value));
  }
  const value=manifest();value.parts[1]=value.parts[0];assert.throws(()=>validateManifest(value));
});
test('download corruption stops before flashing',async()=>{
  await verifyBytes(bytes,file('app'));
  await assert.rejects(()=>verifyBytes(new Uint8Array([0,128,255,43]),file('app')));
  await assert.rejects(()=>verifyBytes(bytes.subarray(1),file('app')));
  assert.deepEqual([...binaryString(bytes)].map(x=>x.charCodeAt(0)),[0,128,255,42]);
});
