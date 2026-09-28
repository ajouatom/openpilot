"""CPU-rendered USB diagnosis, independent of comma packets, model and Xorg."""
from io import BytesIO
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'openpilot/selfdrive/carrot/cluster'))
from boot_status import collect
from display_owner import hud_requested, panel_owner


def lines(status):
  units = status.get('units', {})
  failed = [name for name, value in units.items() if value.get('ActiveState') == 'failed']
  stage = status.get('stage', {})
  storage = status.get('storage', {}).get('state', 'waiting')
  error = bool(failed or stage.get('error') or storage == 'base-recovery')
  title = ('부팅 상태를 확인해 주세요', 'BOOT NEEDS ATTENTION') if error else ('젯슨 연결 준비 중', 'JETSON STARTING / WAITING')
  rows = [
    ('시스템 보호', 'SYSTEM STORAGE', storage),
    ('부팅 단계', 'BOOT STAGE', stage.get('stage', 'starting')),
    ('무선 네트워크', 'WI-FI', str(status.get('wifi', 'waiting'))),
    ('접속 주소', 'IP ADDRESS', ' / '.join(status.get('addresses', [])) or 'Waiting for network'),
    ('원격 접속', 'SSH', units.get('ssh', {}).get('ActiveState', 'waiting')),
    ('콤마 USB 서비스', 'COMMA USB SERVICE', units.get('carrot-jetlink', {}).get('ActiveState', 'waiting')),
  ]
  detail = ', '.join(failed[:3]) or stage.get('error', '')
  return title, rows, detail, error


def render(status, root=ROOT, size=(1920, 462)):
  from PIL import Image, ImageDraw, ImageFont
  image = Image.new('RGB', size, '#101821')
  draw = ImageDraw.Draw(image)
  font_path = root / 'openpilot/selfdrive/assets/fonts/KaiGenGothicKR-Bold.ttf'
  font = lambda n: ImageFont.truetype(str(font_path), n)
  title, rows, detail, error = lines(status)
  accent = '#ff8876' if error else '#65d7bb'
  draw.rectangle((0, 0, 12, size[1]), fill=accent)
  draw.text((38, 20), title[0], font=font(36), fill='white')
  draw.text((40, 65), title[1], font=font(19), fill=accent)
  draw.text((1550, 34), f"BOOT +{status.get('uptime', 0)}s", font=font(22), fill='#97a9bc')
  if status.get('temp_c') is not None:
    draw.text((1750, 34), f"{status['temp_c']:.1f} C", font=font(22), fill=accent)
  for index, (ko, en, value) in enumerate(rows):
    x, y = 40 + (index % 3) * 625, 116 + (index // 3) * 122
    draw.text((x, y), ko, font=font(26), fill='#dce7f1')
    draw.text((x, y+34), en, font=font(15), fill='#8198ae')
    value_font = font(23)
    value = str(value)
    while value and draw.textlength(value, font=value_font) > 585:
      value = value[:-4] + '...' if len(value) > 4 else ''
    draw.text((x, y+61), value, font=value_font, fill=accent)
  footer = detail or '콤마 연결과 계기판 화면 준비를 기다립니다'
  draw.text((40, 373), footer[:105], font=font(22), fill=accent)
  draw.text((40, 410), 'Waiting for comma / dashboard. This is a status screen, not driving guidance.',
            font=font(17), fill='#97a9bc')
  return image


def main():
  import cluster_usb_display as usb
  import openpilot.common.usbgpu_bus_lock as bus
  # RAM path exists even if the storage setup failed before mounting /tmp.
  bus.USBGPU_BUS_LOCK_PATH = '/run/carrot-jetlink-display/usb.lock'
  usb._set_cluster_hud_connected = lambda value: None
  usb._usbgpu_reset_protected = lambda: True  # Diagnostics never reset the shared USB path.
  class Panel(usb.TuringUsbDisplay):
    def close(self):
      # Preserve the last diagnostic image until the HUD sends its first frame.
      self._disconnected = True
      super().close()
  while True:
    with panel_owner() as acquired:
      if acquired and usb.find_supported_usb_product(0x0092) is not None:
        panel = Panel(brightness=65, display_fps=1, expected_product_id=0x0092)
        try:
          panel.open()
          while not hud_requested():
            frame = render(collect(), size=(panel.landscape_width, panel.landscape_height))
            stream = BytesIO()
            frame.save(stream, format='JPEG', quality=82)
            panel.send_jpeg(stream.getvalue())
            time.sleep(1)
        except Exception as error:
          print('Boot display retry:', type(error).__name__, flush=True)
        finally:
          panel.close()
    time.sleep(1)


if __name__ == '__main__':
  main()
