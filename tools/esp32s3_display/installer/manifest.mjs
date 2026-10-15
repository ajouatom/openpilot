export function validateManifest(value) {
  const slots = new Map([['bootloader.bin',[0,0x8000]],['partition-table.bin',[0x8000,0x1000]],['carrot-s3.bin',[0x10000,0x300000]]]);
  if (value?.chip !== 'ESP32-S3' || value?.board !== 'N16R8' || value?.parts?.length !== 3) throw new Error('지원하지 않는 펌웨어 정보');
  for (const part of value.parts) {
    const slot=slots.get(part.path);
    if (!slot || part.offset!==slot[0] || !Number.isInteger(part.size) || part.size<=0 || part.size>slot[1] || part.size%4 ||
        !/^[0-9a-f]{64}$/.test(part.sha256) || !/^[0-9a-f]{32}$/.test(part.md5)) throw new Error('잘못된 펌웨어 파일 또는 주소');
    slots.delete(part.path);
  }
  return value;
}
export async function verifyBytes(bytes, part) {
  if (bytes.length !== part.size) throw new Error('펌웨어 크기 불일치');
  const hash=Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256', bytes)), x=>x.toString(16).padStart(2,'0')).join('');
  if (hash !== part.sha256) throw new Error('펌웨어 SHA-256 불일치');
}
export function binaryString(bytes) {
  let value='';
  for (let offset=0; offset<bytes.length; offset+=8192) value+=String.fromCharCode(...bytes.subarray(offset,offset+8192));
  return value;
}
