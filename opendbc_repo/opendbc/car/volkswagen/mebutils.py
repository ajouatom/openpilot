from opendbc.car.common.conversions import Conversions as CV

# MEB 계기판 ACC_Tempolimit 인코딩 테이블 (DBC 매핑, 오름차순)
_ACC_TEMPOLIMIT_KPH = (
  (1, 5), (2, 7), (3, 10), (4, 15), (5, 20), (6, 25), (7, 30), (8, 35),
  (9, 40), (10, 45), (11, 50), (12, 55), (13, 60), (14, 65), (15, 70),
  (16, 75), (17, 80), (18, 85), (19, 90), (20, 95), (21, 100), (22, 110),
  (23, 120), (24, 130), (25, 140), (26, 150), (27, 160), (28, 200), (30, 250),
)


def map_speed_to_acc_tempolimit(v_ms):
  v_kph = int(round(v_ms * CV.MS_TO_KPH))
  acc_value = 0
  for val, limit in _ACC_TEMPOLIMIT_KPH:
    if v_kph >= limit:
      acc_value = val
    else:
      break
  return acc_value
