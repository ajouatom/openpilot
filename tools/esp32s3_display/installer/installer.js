import {ESPLoader, Transport} from './vendor/esptool-js-bundle.js';
import {validateManifest, verifyBytes, binaryString} from './manifest.mjs';

const $ = id => document.getElementById(id);
const status = text => { $('status').textContent = text; };
const log = text => { $('log').textContent = ($('log').textContent + text + '\n').slice(-16000); };
let busy = false;
const supported = isSecureContext && 'serial' in navigator;
function enabled() {
  $('flash').disabled = busy || !supported || !$('board').checked;
  $('mode').disabled = $('board').disabled = busy;
}
$('board').addEventListener('change', enabled);
status(supported ? 'UART 포트를 연결하고 보드 확인란을 선택하세요.' : 'PC의 Chrome 또는 Edge에서 HTTPS 설치 페이지를 열어 주세요.');
enabled();

$('flash').addEventListener('click', async () => {
  let transport, loader;
  const mode = $('mode').value;
  busy = true; enabled(); $('log').textContent = ''; $('progress').value = 0;
  try {
    const port = await navigator.serial.requestPort();
    status('파일을 검증하고 있습니다.');
    const response = await fetch('./release.json', {cache:'no-store'});
    if (!response.ok) throw new Error('펌웨어 정보를 받지 못했습니다.');
    const manifest = validateManifest(await response.json());
    const parts = await Promise.all(manifest.parts.map(async part => {
      const response = await fetch(part.path);
      if (!response.ok) throw new Error(`${part.path} 다운로드 실패`);
      const bytes = new Uint8Array(await response.arrayBuffer());
      await verifyBytes(bytes, part);
      return {...part, bytes};
    }));
    transport = new Transport(port, false);
    loader = new ESPLoader({transport, baudrate:460800,
      terminal:{clean(){}, write:log, writeLine:log}});
    status('S3에 연결 중입니다…');
    await loader.main();
    if (loader.chip.CHIP_NAME !== 'ESP32-S3' || await loader.getFlashSize() !== 16384) {
      throw new Error('ESP32-S3 / Flash 16MB 보드가 아닙니다. 설치를 중단했습니다.');
    }
    const partition = parts.find(part => part.offset === 0x8000);
    if (mode === 'update') {
      const bytes = await loader.readFlash(0x8000, partition.size);
      try { await verifyBytes(bytes, partition); }
      catch { throw new Error('기존 CARROT 파티션이 아닙니다. 새 보드라면 처음 설치를 선택하세요.'); }
    }
    const nvsBefore = await loader.flashMd5sum(0x9000, 0x6000);
    const selected = mode === 'update' ? parts.filter(part => part.offset === 0x10000) : parts;
    for (let index=0; index<selected.length; index++) {
      const part=selected[index];
      status(`설치 중 (${index+1}/${selected.length}) — 케이블을 빼지 마세요.`);
      await loader.writeFlash({fileArray:[{address:part.offset, data:binaryString(part.bytes)}],
        flashSize:'keep', flashMode:'keep', flashFreq:'keep', eraseAll:false, compress:true,
        calculateMD5Hash:() => part.md5,
        reportProgress:(_file, written, total) => { $('progress').value=(index+written/total)/selected.length*100; }});
    }
    if (nvsBefore !== await loader.flashMd5sum(0x9000,0x6000)) {
      throw new Error('설치 후 등록 저장 영역 검증이 일치하지 않습니다. 진행 기록을 확인하세요.');
    }
    await loader.after('hard_reset');
    status('설치 완료! 등록 정보 보존을 확인했습니다. 아래 4번 Wi-Fi/C4 설정을 진행하세요.');
    $('progress').value=100;
  } catch(error) {
    status(error.name==='NotFoundError' ? '보드 선택을 취소했습니다.' : `설치 중단: ${error.message}`);
    log(error.message);
  } finally {
    if (transport) { try { await transport.disconnect(); } catch(error) { log(error.message); } }
    busy=false; enabled();
  }
});
