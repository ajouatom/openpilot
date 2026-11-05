# PEAK PCAN-USB 연결

[지원 장치 목록으로 돌아가기](indicator-adapters.md)

기존 **클래식 PCAN-USB, USB 식별값 0c72:000c**를 젯슨 USB에 연결합니다. PCAN-USB FD나 Pro를 같은 제품으로 취급하지 않습니다. **PEAK에 CANable용 펌웨어를 넣지 않습니다.**

1. 지원 코드가 포함된 carrot-wip을 Git Pull하고 콤마와 젯슨을 재시작합니다. 대응 젯슨 이미지에서는 기존 PEAK 드라이버를 준비합니다.
2. 전원을 끈 상태에서 해당 차량에서 확인된 CAN H/L과 기준접지 배선을 연결합니다. 이미 사용하던 PCAN 케이블이면 같은 커넥터와 같은 버스를 유지합니다. 제품별 핀 배치는 [PEAK 공식 PCAN-USB 자료](https://www.peak-system.com/PCAN-USB.199.0.html?&L=1)를 확인하세요.
3. PCAN-USB를 젯슨 USB에 연결합니다. 다른 USB CAN이나 S3가 이미 제어 중이면 대기하고, 먼저 준비된 장치가 선택됩니다.
4. P단 정차 상태에서 좌·우 리모컨 조작과 실제 차량 램프를 확인합니다.

현재 USB 제어 프로필은 아이오닉 5 PE에서 확인한 클래식 CAN 500 kbit/s의 0x3C1·0x413을 사용합니다. PCAN-USB가 다른 차량의 CAN을 수신할 수 있다는 사실만으로 이 제어 프로필이 그 차량에 호환된다고 판단하지 않습니다.

PEAK는 제조사 펌웨어를 그대로 사용합니다. CANable 설치 도구나 S3 설치 도구를 실행할 필요가 없습니다.

[English](../en/peak-can.md)
