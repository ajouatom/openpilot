"""Carrot CANable 2.0 installer: Windows GUI or Linux CLI; never writes option bytes."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import queue
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import webbrowser

ROOT = Path(sys.executable if getattr(sys, 'frozen', False) else __file__).resolve().parent
VERSION = 'carrot-b1-slcan-2'
IMAGE = 'canable2-'+VERSION+'.bin'
IMAGE_SHA256 = 'aa59e9f7733d3398537d1a7d7c07cdd5e17edc91ac689fd7e0bd738dad03cd26'
WIKI = 'https://github.com/ajouatom/openpilot/wiki/KO-CANable-2.0-설치'


def run(arguments, log=print, cwd=None, timeout=45):
  flags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0
  result = subprocess.run([str(a) for a in arguments], cwd=cwd, capture_output=True,
                          text=True, errors='replace', timeout=timeout, creationflags=flags)
  output = result.stdout+'\n'+result.stderr
  if result.returncode:
    raise RuntimeError(output.strip()[-4000:])
  return output


def utility(log=print):
  bundled = ROOT/'dfu-util'/'dfu-util.exe'
  if os.name == 'nt' and bundled.is_file():
    return bundled
  system = shutil.which('dfu-util')
  if system:
    return Path(system)
  if sys.platform != 'linux':
    raise RuntimeError('압축을 모두 푼 뒤 실행하세요. dfu-util 폴더가 필요합니다.')
  # Protected Jetson root filesystems cannot use apt install. Extract the
  # architecture-matched package into a private cache without changing the OS.
  cache = Path('/opt/carrot-jetlink/canable-installer') if Path('/opt/carrot-jetlink').is_dir() else Path.home()/'.cache/carrot-canable'
  executable = cache/'usr/bin/dfu-util'
  if not executable.is_file():
    cache.mkdir(parents=True, exist_ok=True)
    log('dfu-util 준비 중 / Preparing dfu-util…')
    run(['apt-get', 'download', 'dfu-util'], log, cwd=cache, timeout=120)
    packages = list(cache.glob('dfu-util_*.deb'))
    if len(packages) != 1:
      raise RuntimeError('dfu-util 패키지를 하나로 확인할 수 없습니다: '+str(cache))
    run(['dpkg-deb', '-x', packages[0], cache], log)
  run([executable, '--version'], log)
  return executable


def parse_devices(output):
  devices = {}
  for line in output.splitlines():
    if 'Found DFU: [0483:df11]' not in line:
      continue
    serial = re.search(r'serial="([0-9A-Fa-f]{12,24})"', line)
    alternate = re.search(r'alt=(\d+), name="([^"]+)"', line)
    if serial and alternate:
      devices.setdefault(serial[1], {})[int(alternate[1])] = re.sub(r'\s+', '', alternate[2])
  supported = []
  for serial, areas in devices.items():
    flash = re.fullmatch(r'@InternalFlash/0x08000000/(32|64)\*02Kg', areas.get(0, ''))
    # STM32G431 memory geometry, not merely the shared STM32 DFU VID/PID.
    if flash and areas.get(1) == '@OptionBytes/0x1FFF7800/01*40Be' and areas.get(2) == '@OTPMemory/0x1FFF7000/01*01Ke':
      supported.append({'serial':serial, 'size':int(flash[1])*2048})
  return supported


def detect(executable, log=print, serial=None):
  output = run([executable, '-l'], log)
  devices = parse_devices(output)
  if serial:
    devices = [d for d in devices if d['serial'] == serial]
  if len(devices) != 1:
    raise RuntimeError('지원되는 BOOT 장치를 하나로 확인하지 못했습니다. CANable 하나만 연결하고 BOOT 점퍼를 확인하세요.\n'
                       'Windows에서 장치는 보이지만 열리지 않으면 안내서의 WinUSB 드라이버 단계를 진행하세요.\n'+output[-1600:])
  log('CANable G431 확인 / Device checked: '+devices[0]['serial'])
  return devices[0]


def image_bytes():
  data = (ROOT/IMAGE).read_bytes()
  if hashlib.sha256(data).hexdigest() != IMAGE_SHA256:
    raise ValueError('펌웨어 파일 검증 실패. 공식 설치 ZIP을 다시 내려받으세요.')
  return data


def install(log=print, serial=None, board=None):
  if board != 'mks-canable-v2':
    raise ValueError('사진 위쪽의 MKS CANable V2.0 기본형인지 확인하세요. CLI: --board mks-canable-v2')
  if sys.platform == 'linux' and os.geteuid() != 0:
    raise PermissionError('젯슨에서는 sudo python3 install_canable.py --install --board mks-canable-v2 로 실행하세요.')
  payload = image_bytes()
  executable = utility(log)
  board = detect(executable, log, serial)
  if len(payload) > board['size']:
    raise ValueError('펌웨어가 보드의 플래시보다 큽니다.')
  base = Path('/opt/carrot-jetlink') if sys.platform == 'linux' and Path('/opt/carrot-jetlink').is_dir() else Path.home()/'Carrot-CANable'
  base.mkdir(parents=True, exist_ok=True)
  backup = Path(tempfile.mkdtemp(prefix='canable-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'-', dir=base))
  original = backup/'original.bin'
  args = [executable, '-d', '0483:df11', '-S', board['serial'], '-a', '0']
  log('1/3 기존 펌웨어 백업 / Backing up: '+str(original))
  run([*args, '-s', '0x08000000:'+str(board['size']), '-U', original], log)
  if original.stat().st_size != board['size']:
    raise RuntimeError('백업 크기가 다릅니다. 기록을 시작하지 않았습니다.')
  receipt = {**board, 'version':VERSION, 'original_sha256':hashlib.sha256(original.read_bytes()).hexdigest(), 'new_sha256':IMAGE_SHA256}
  (backup/'receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
  log('2/3 펌웨어 기록 / Programming '+VERSION)
  run([*args, '-s', '0x08000000', '-D', ROOT/IMAGE], log)
  log('3/3 기록 내용 다시 읽어 비교 / Reading back and comparing')
  readback = backup/'readback.bin'
  run([*args, '-s', '0x08000000:'+str(len(payload)), '-U', readback], log)
  if readback.read_bytes() != payload:
    raise RuntimeError('기록 검증 실패. BOOT 상태를 유지하고 다시 설치하세요. 백업: '+str(backup))
  receipt['verified'] = True
  (backup/'receipt.json').write_text(json.dumps(receipt, indent=2)+'\n')
  log('설치·검증 완료 / Verified. 백업: '+str(backup))
  log('USB와 CAN을 분리 → BOOT 점퍼 제거 → USB부터 재연결 → 정상 버전 확인 후 CAN 연결.')
  return receipt


def verify_runtime(log=print):
  sys.path.insert(0, str(ROOT/'vendor'))
  import serial
  from serial.tools import list_ports
  ports = [p for p in list_ports.comports() if (p.vid, p.pid) in ((0xad50, 0x60c4), (0x16d0, 0x117e))]
  if len(ports) != 1:
    raise RuntimeError('정상 모드 CANable 하나만 연결하세요. BOOT 점퍼는 제거해야 합니다.')
  with serial.Serial(ports[0].device, 115200, timeout=.5, write_timeout=.5) as port:
    if sys.platform == 'linux':
      import fcntl
      try:
        fcntl.flock(port.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
      except BlockingIOError:
        raise RuntimeError('제어 프로그램이 이미 사용 중입니다. 장치를 중단시키지 않았습니다.') from None
    port.reset_input_buffer()
    port.write(b'C\rV\r')  # Keep CAN closed; version check cannot send a CAN frame.
    response = port.read(512)
  if (VERSION+' github.com/normaldotcom/canable2.git\r').encode() not in response:
    raise RuntimeError('예상 펌웨어 버전이 아닙니다: '+repr(response[:120]))
  log('정상 실행 확인 / Runtime verified: '+VERSION)
  return VERSION


def gui():
  import tkinter as tk
  from tkinter import ttk, scrolledtext
  window = tk.Tk()
  window.title('Carrot CANable 2.0 설치')
  window.geometry('820x590')
  frame = ttk.Frame(window, padding=20);frame.pack(fill='both', expand=True)
  ttk.Label(frame, text='CANable 2.0 펌웨어 설치', font=('Malgun Gothic', 20, 'bold')).pack(anchor='w')
  ttk.Label(frame, text='MKS CANable V2.0 / V2.0 S 기본형 · STM32G431\nPRO·V1.0·다른 MCU 보드에는 설치하지 마세요.', font=('Malgun Gothic', 11)).pack(anchor='w', pady=10)
  ttk.Label(frame, text='① CAN선을 분리하고 USB를 뺍니다.\n② 사진의 BOOT 두 핀에 점퍼를 끼우고 USB를 PC에 꽂습니다.\n③ 보드 확인 → 설치 시작. 완료되면 BOOT 점퍼를 빼고 다시 연결합니다.', font=('Malgun Gothic', 11)).pack(anchor='w')
  bar = ttk.Frame(frame);bar.pack(fill='x', pady=15)
  confirmed = tk.BooleanVar(value=False)
  ttk.Checkbutton(frame, text='사진 위쪽 MKS CANable V2.0 기본형입니다 (Pro / V1.0 제외).', variable=confirmed).pack(anchor='w')
  output = scrolledtext.ScrolledText(frame, height=17, font=('Malgun Gothic', 10));output.pack(fill='both', expand=True)
  messages = queue.Queue()
  buttons = []
  busy = False
  def log(message):messages.put(str(message))
  def execute(action):
    nonlocal busy
    busy = True
    for button in buttons:button.configure(state='disabled')
    def work():
      try:action()
      except Exception as exc:log('오류 / Error: '+str(exc))
      finally:messages.put(None)
    threading.Thread(target=work, daemon=True).start()
  def installation():
    board = 'mks-canable-v2' if confirmed.get() else None
    execute(lambda:install(log, board=board))
  for label, action in [('보드 확인', lambda:detect(utility(log), log)), ('정상 버전 확인', lambda:verify_runtime(log))]:
    button = ttk.Button(bar, text=label, command=lambda a=action:execute(a));button.pack(side='left', padx=(0,8));buttons.append(button)
  install_button = ttk.Button(bar, text='설치 시작', command=installation)
  install_button.pack(side='left', padx=(0,8));buttons.append(install_button)
  guide = ROOT/'START.html'
  ttk.Button(bar, text='사진 설명서', command=lambda:webbrowser.open(guide.as_uri() if guide.exists() else WIKI)).pack(side='left')
  ttk.Button(bar, text='WinUSB 안내', command=lambda:webbrowser.open('https://zadig.akeo.ie/')).pack(side='left', padx=8)
  def poll():
    nonlocal busy
    while not messages.empty():
      message = messages.get()
      if message is None:
        busy = False
        for button in buttons:button.configure(state='normal')
      else:
        output.insert('end', message+'\n');output.see('end')
    window.after(100,poll)
  window.protocol('WM_DELETE_WINDOW', lambda:log('작업 완료 후 창을 닫아 주세요.') if busy else window.destroy())
  poll();window.mainloop()


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--install', action='store_true')
  parser.add_argument('--check', action='store_true')
  parser.add_argument('--verify-runtime', action='store_true')
  parser.add_argument('--serial')
  parser.add_argument('--board', choices=['mks-canable-v2'], help='Confirm the pictured standard MKS V2.0 board, not Pro/V1.0')
  args = parser.parse_args()
  if args.install:install(serial=args.serial, board=args.board)
  elif args.check:detect(utility(), serial=args.serial)
  elif args.verify_runtime:verify_runtime()
  elif os.name == 'nt':gui()
  else:parser.print_help()


if __name__ == '__main__':
  main()
