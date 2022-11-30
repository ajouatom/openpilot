APILOT
------
* PREVIEW

  ![image](https://user-images.githubusercontent.com/43668841/234758895-29ce21fa-521a-444f-8318-e00ce991a03c.png)

  ![image](https://user-images.githubusercontent.com/43668841/234758774-851624ee-5f15-4492-bbaf-9239bf5ddb94.png)

  ![image](https://user-images.githubusercontent.com/43668841/231068110-1bd61f5b-8c1f-406b-bacd-419e47289a08.png)
  
  ![image](https://user-images.githubusercontent.com/43668841/231068378-ebbcaafe-55d1-4e18-a08c-8dd01a337a7f.png)
  
  ![image](https://user-images.githubusercontent.com/43668841/231068460-11c7977a-feaa-4f1c-b4be-263d4132d213.png)

* 읽어보기

  * 설정값을 함부로 건들면 사고의 위험이 있으며, 사고/고장 발생시 본인의 책임입니다.
  * Panda코드를 수정하였으므로, comma connect에 연결되면 BAN당함(연결되어도 업로드안됨)!
  * 배선개조: SCC모듈(레이더)의 CCAN연결을 절단 -> 판다의 CAN2에 연결
  * NDA지원 (Neokii)
  * 설치브랜치
    * 콤마3: c3-master
    * 콤마2, 이온/원플러스: c2-master
  * 신호정지모드
    * 신호정지감속정지상태에서 GAS페달: 신호무시
    * 소프트 브레이크홀드기능: 정지상태에서 Brake페달 0.7초이상: 신호무시하고 정지유지 
  * 테스트된차량: SCC모듈(레이더) 배선개조 CAN BUS2로 연결된차량
    * Hyundai SANTAFE HYBRID Hybrid 2022 (배선개조 안된것도 가능,단,AEB off됨) (RadarTracks)
    * Kia STINGER
    * Hyundai GENESIS (White PANDA를 이용하여 MDPS개조된차량)
    * KONA EV
    * SANTAFE TM (RadarTracks)
    * 펠리세이드 (RadarTracks)
  * 인게이지
    * 롱컨모드
      * 배선개조된경우: 크루즈셋버튼, (+)(-) 버튼
      * 순정배선(SCC모듈기능정지,AEB OFF): (+)(-) 버튼
      * 자동인게이지 기능
      * 기어선택유무에 관계없이, 안전벨트만 착용하면 인게이지 가능
    * SCC모드
      * 크루즈셋버튼: 차량의 크루즈셋 조건에 따름.
  * 크루즈 차량간격: 속도에 따른 자동차간거리
    * 1단계: 연비절감모드,
    * 2단계: 안전주행모드,
    * 3단계: 일반주행,
    * 4단계: 고속모드 (신호감지안함)
  * 레인체인지시작조건(오토턴기능켠경우)    
    * 정지출발
      * 깜박이: 오토턴시작
    * 저속(레인체인지속도 이하)
      * 경계검출: 오토턴
      * 깜박이(우) + 핸들토크: 우회전
      * 깜박이(우): 레인체인지
      * 깜박이(좌) + 핸들토크: 좌회전
    * 중속 (80키로미만)
      * 깜박이(좌/우) + 브레이크 + 경계검출(X) + 핸들토크: 오토턴
      * 깜박이(좌/우) + 브레이크 + 경계검출(X): 레인체인지
      * 깜박이(우) + 브레이크 + 경계검출(우) + 턴속도미만: 오토턴
      * 깜박이(우) + 브레이크 + 경계검출(우) + 핸들토크: 오토턴
      * 깜박이(우) + 브레이크 + 경계검출(우) : 레인체인지
      * 깜박이(좌) + 브레이크 + 경계검출(좌) + 턴속도미만: 오토턴
      * 깜박이(좌) + 브레이크 + 경계검출(좌) + 핸들토크: 오토턴
      * 깜박이(좌) + 브레이크 + 경계검출(좌) : 오토턴
      * 깜박이(좌/우) + 브레이크(X) : 레인체인지
    * 고속
      * 깜박이(좌/우): 레인체인지
  * 레인체인진진행중
    * 깜박이(X) : 중지    
    * BSD검출: 핸들토크 기다림.
    * 경계검출: 검출이 끝나고 차선변경시작
    * 깜박이(우)+경계검출+턴속도미만: 오토턴
    * 핸들토크발생: 같은방향이면 턴으로 변경, 반대방향 차선변경 취소
    * 연속차선변경: 핸들토크필요, 레인체인지 완료 후 깜박이 켜져있으면 
  * 오토턴진행중
    * 핸들토크(반대): 취소

* 설정: 토글
  * Experimental openpilot Longitudinal Control(롱컨트롤)
    * 순정 SCC의 기능을 사용하지 않고 오픈파일럿의 크루즈제어 기능 선택
    * 배선 개조시 선택함.
    * 배선 비개조시, Enable RadarTracks(ON)한경우: 테스트차량만 가능
      * 지원: SANTAFE HEV, NEXO
      * 미지원: GENESIS(DH), KONA
  * SCC Module connected BUS2
    * 배선을 개조하여 판다의 BUS2에 연결한 경우 ON
  * EnableRadarTracks
    * SCC모듈에서 제공하는 레이더정보를 무시하고, SCC모듈내의 레이더 정보를 이용함.

* 설정: 기타
  * 차량선택: 현대/기아 차량만 선택가능
  * 업데이트: 현재선택되어 있는 브랜치에 대해 새로운 업데이트가 있으면 적용 후 Rebooting
    * (주의) 반드시 시동을 끄고 할것!
  * 과속카메라작동방법
    * 0: 사용안함
    * 1: NDA 사용, Neokii의 NDA지원 단말기 설치시 자동으로 연결: 같은 네트워크에 있어야함.
    * 과속카메라 속도제한은 무조건 작동함.
  * 크루즈소리 
    * 0: 크루즈 작동소리가 안남
    * 1: 일부소리가 남.
    * 2: 모든 크루즈 작동소리가 안남.
  * CustomMapBox입력
    * http://IP주소:8082 접속하여 mapboxtoken을 입력하면 자동으로 켜짐.
    * OFF하면 입력된 Token값을 제거함.
  * Show Debug UI
    * 화면에 디버그 정보를 표시함.
  * 시간정보표시
    * 화면에 시간을 표시합니다.
  * 인게이지 유지모드
    * 롱컨선택시 유효하며, Drive 기어, 안전벨트착용하면 최초 1회에 한하여 자동인게이지 됨.
  * 레인모드속도설정
    * 차선이 보일때 자동으로 차선의 중앙으로 따라감.
    * 해당속도이상이 되면 자동으로 자동레인모드로 변경됨
  * 차선치우침 좌우보정
  * 핸들햅팅기능
    * 핸들진동기능이 있는 차량은 과속카메라 감속시 진동을 2번 울려줌.

* 설정: 크루즈
  * 가속시 크루즈속도를 맞춤
    * 가속페달을 밟아 설정된 속도보다 높아지면 자동으로 설정속도를 올려줌
  * 자동속도증가모드(100%)
    * 선행차량의 속도가 빨라지면, RoadSpeedLimit * 비율의 속도까지 자동으로 설정속도를 올려줌.
    * 0: 사용안함
  * 선행차속도에 크루즈속도맞추기(+40km/h)
    * 선행차를 만나면 선행차의 속도에 설정속도로 미리 낮추어줌.
  * 운전모드 초기값(3)
    * 1: 연비모드 : 가속을 제한하여 연비위주의 운전모드
    * 2: 안전모드 : 일반주행보다 더 천천히 가속, 천천히 감속, 차량을 멀리 떨어져서 주행하도록 함.
    * 3: 일반주행모드 : 일반적인 주행모드
    * 4: 고속모드 : 신호정지 기능을 사용안함.
    * 화면의 거리조정(GAP) 표시를 누르면 바뀜.
    * 차간거리조정(GAP)스위치를 길게 누르면 바뀜
  * 드라이브모드: 연비가속비율
    * 연비모드 선택시 일반모드가속대비 비율
  * 드라이브모드: 안전비율
    * 가속도, 차간거리, 감속율등을 안전하게 비율대로 줄이거나 늘려줌.
  * 크루즈버튼작동모드 (Cruise Button Mode)
    * 0: 일반속도제어  (Normal Mode)
      * (+)(-) 1씩증감, 길게누르면 10씩증감 (short: 1,long: +-10)
      * (-)버튼: 현재속도로 설정 (current speed set)
    * 1: 사용자속도제어1 (User1)
      * (+)버튼: 도로속도제한까지 한번에 올라감, 이후 +10km/h씩올라감. (upto road speed limit, after +10)
      * (-)버튼: 현재속도로 설정 (current speed set)
    * 2: 사용자속도제어2 (User2)
      * 사용자속도제어1의 기능외에 (include User1)
      * (-)버튼: 또 누르면, 관성제어모드(크루즈꺼짐)  ( twrice (-) button: cruise off, but leadcar or traffic signal detected cruise on automatically)
    * 3: 사용자속도제어3 (User3)
      * (+)버튼: 1씩증가, 길게 10씩증가  (short:1, long: +10)
      * (-)버튼: 현재속도셋, 10씩 감소, 길게 10씩감소 (-: current speed, after -10)
    * 4: 사용자속도제어4 (User4)
      * (+)버튼: 1씩증가, 길게 10씩증가 (shoft:+1, long: +10)
      * (-)버튼: 10씩 감소, 길게 10씩감소 (shoft: -10, long: -10))
  * 엑셀크루즈ON: 60%이상 엑셀을 밟으면 크루즈가 켜짐.
  * 엑셀크루즈ON: 속도(30): 지정된 속도가 올라가면 크루즈가 켜짐
  * 엑셀크루즈ON: 속도설정방법
    * 0: 현재속도로 세팅
    * 1: 기존 속도로 세팅
    * 2: 선행차 있을때만 기존속도로 세팅
  * 브레이크해제 크루즈ON  사용: 브레이크를 떼면 크루즈를 켜는 기능을 사용
  * 브레이크해제 크루즈ON:주행중,선행차(20): 선행차거리: 브레이크를 떼고 선행차가 일정거리 이상이면 크루즈 ON
  * 브레이크해제 크루즈ON:정지상태, 선행차: 정지상태에서 선행차가 10M이내에 있으면 크루즈 ON
  * 브레이크해제 크루즈ON:주행중, 속도: 0:사용안함, 브레이크를 떼고 일정속도이상이면 크루즈를 현재속도로 ON
  * 브레이크해제 크루즈ON:주행중,신호: 60km/h이하에서 브레이크를 떼고 신호가 감지되면 현재속도로 크루즈 ON

* 설정: 튜닝
  * StopDistance(600cm): 선행차에 대한 정지위치를 조정
  * 신호정지 위치 조정(200cm): 신호등에 대한 정지위치 조정
  * 신호정지 모델속도(OFF): 신호등에 대한 정지시 모델이 제공하는 속도에 따름
    * 사용안함.
  * 롱컨: JERK값(8): 값을 크게하면 가속/감속에 대한 차량의 반응이 강해지나, 브레이크작동이 잘 안되면 값을 조금씩 올림.
  * 롱컨: FF게인(106%): 속도플래너의 가속도에 대한 피드포워드, 이값은 조정하시 말것!
  * 롱컨: P게인(100): 차량 속도제어에 대한 P게인, 50~100정도로 조정.
  * X_EGO_COST(6): 높으면 신호정지위치에 정밀하게 서려고함.
  * J_EGO_COST(5): 고정
  * A_CHANGE_COST(150): 끼워들기 차량에 대해 반응이 너무 강하면 올릴것. (50~200)
  * DANGER_ZONE_COST(100): 고정
  * 차량간격유지 동적제어(OFF): 전방차량이 멀어지는 것에 대한 반응을 빠르게 COST를 동적으로 적용함.
  * 차량간격 동적제어:상대속도(110%): 선행차와의 상대속도에 따라 차량간격을 동적으로 조정함.
  * 차량간격 동적제어:감속(110%): 차량의 감속도에 따라 차량간격을 동적으로 조정함. 급감속을 하면 더 멀리.
  * 차량간격 비율(100%)
    * 위험: 추돌의 위험이 있음.
    * 속도에 따라 차량간격을 자동조정하고 있으나, 좀더 가깝게 하고 싶을때
  * 신호감지 감속율(80%): 신호등감지시 감속도를 조절하여 서서히 속도를 줄이고 싶을때
  * 모델의 자동속도조절의 적용속도(0): 사용안함.
  * 초기가속제한속도(3km/h): 설정속도까 연비운전모드의 가속률을 사용함.
  * 모델혼잡시 조향가속비율적용(ON): 일반적으로 도로상황이 안좋을때, 연비운전모드의 가속률을 사용함.
  * 가속도 제어(100): 가속도를 좀더 올리고 싶을때 사용.
  * 모델커브속도조절(ON): 곡선도로를 만나면 자동으로 속도를 줄여줌.

* 엑셀에서 발을 떼었을때
  * 크루즈ON 조건 
    * 엑셀크루즈ON 설정상태
    * 핸들20도이내이고 엑셀크루즈ON속도이상 또는 가속페달을 60%이상 밟았을때
    * 적색 신호감지는 안되어있는 상태 
    * 속도세트
      * 60%이상밟았을때 기존속도
      * 엑셀크루즈ON: 속도복원설정에 따라 속도복원
  * 크루즈OFF조건
    * 소프트홀드
    * 신호감지 감속중
    * 저속주행
    * 설정속도 보다 느리고, 앞에 차가 있고, 턴중
  * 크루즈 속도변경
    * 페달을 0.6초이내에 뗀경우
    * 크루즈속도보다 높을때

* 브레이크 크루즈ON/OFF/속도조절
  * 크루즈ON조건
    * 소프트홀드
    * 직진, 선행차가 일정거리 이상, 깜박이 꺼짐
    * 직진, 속도 70키로이내, 적색신호등감지, 깜박이 꺼짐
    * 선행차없음, 일정속도이상인경우
    * 정지중, 선행차 10M(설정값)이내인경우

* 설정값참조: SANTAFE_TM_HEV (SCC배선개조)
  * 조향
    * 레인모드작동속도: 0
    * 자동모델턴제어: 1
    * 자동모델턴제어: 속도 : 45
    * 자동모델턴제어:시간제한 : 240
    * 자동차선변경속도: 30
    * 차선치우침좌우보정: 0
  * 정지
    * 가속초기속도 : 0
    * 신호정지 위치조정: 200
    * 정지거리: 550
    * 정지유지브레이크: 30
  * 조향튜닝
    * LiveSteerRatioApply: 100
    * SteerActuatorDelay:30
    * SteerDeltaDown: 7
    * SteerDeltaUp: 4
    * SteeringRateCost: 10
    * _LateralAccelCost: 100
    * _LateralJerkCost: 5
    * _LateralMotionCost: 9
    * _PathCostApply: 100
  * 주행설정
    * 안전비율: 80
    * 연비가속비율: 80
  * 가속설정
    * 가속설정1: 200
    * 가속설정2: 140
    * 가속설정3: 80
    * 가속설정4: 70
    * 가속설정5: 60
    * 가속설정6: 30
  * 주행튜닝
    * A_CHANGE_COST: 200
    * DANGER_ZONE_COST: 100
    * FF게인: 100
    * JERK값: 8
    * KiV게인: 500
    * KpV게인: 100
    * LEAD_DANGER_FACTOR: 85
    * _LADLowerBound: 10
    * _LADUpperBound: 70
    * 신호정지 감속모델선택: 30
    * 신호정지 감속율: 80
  * 감속제어
    * 과속카메라작동방법: 1
    * 과속카메라감속시작시간: 15
    * 과속카메라감속완료시간: 8
    * 커브속도: 1
    * 커브속도제어INDEX: 15
    * 커브속도조절비율: 130
  * 버튼설정
    * 크루즈갭: 초기값: 4
    * 크루즈갭버튼 작동모드: 2
    * 크루즈버튼작동모드: 2
  * 자동크루즈
    * 브레이크해제 크루즈ON:사용: 1
    * 브레이크해제 크루즈ON:정지: 1
    * 브레이크해제 크루즈ON:주행,선행차: 10
    * 브레이크해제 크루즈ON:주행,속도: 40
    * 브레이크해제 크루즈ON,주행,신호: 1
    * 엑셀크루즈OFF:모드: 1
    * 엑셀크루즈ON: 사용: 1
    * 엑셀크루즈ON: 복원속도: 0
    * 엑셀크루즈ON: 속도: 35
    * 자동속도업데이트속도: 130
    * 자동속도증가모드: 130
    * 크루즈최저속도: 15
  * 화면
    * 화면TPMS: 1
    * 화면가속도표시: 0
    * 화면디바이스정보: 0
    * 화면레이더감지정보: 2
    * 화면부가정보: 0
    * 화면차선정보: 0
    * 화면패쓰끝표시: 0
    * 화면표시모드: 2
    * 화면핸들표시모드: 2
    * 화면핸들회전: 1
  * 시작
    * SCC배선개조 BUS2연결: 1
    * 드라이브모드: 초기값: 3
    * 레이더트랙사용: 1
    * 롱컨사용: 1
    * 소프트오토홀드기능: 2
    * 인게이지유지모드: 1
    * 자동인게이지사용: 1
    * 핸들햅팅기능사용: 0
  * 차량간격
    * 감속: 105
    * 차량간격: 92
    * 상대속도: 102
    * 차량간격비율: 95
    * 차량간격유지 동적제어사용: 1
  

![](https://i.imgur.com/b0ZyIx5.jpg)

Table of Contents
=======================

* [What is openpilot?](#what-is-openpilot)
* [Running in a car](#running-on-a-dedicated-device-in-a-car)
* [Running on PC](#running-on-pc)
* [Community and Contributing](#community-and-contributing)
* [User Data and comma Account](#user-data-and-comma-account)
* [Safety and Testing](#safety-and-testing)
* [Directory Structure](#directory-structure)
* [Licensing](#licensing)

---

What is openpilot?
------

[openpilot](http://github.com/commaai/openpilot) is an open source driver assistance system. Currently, openpilot performs the functions of Adaptive Cruise Control (ACC), Automated Lane Centering (ALC), Forward Collision Warning (FCW), and Lane Departure Warning (LDW) for a growing variety of [supported car makes, models, and model years](docs/CARS.md). In addition, while openpilot is engaged, a camera-based Driver Monitoring (DM) feature alerts distracted and asleep drivers. See more about [the vehicle integration](docs/INTEGRATION.md) and [limitations](docs/LIMITATIONS.md).

<table>
  <tr>
    <td><a href="https://youtu.be/NmBfgOanCyk" title="Video By Greer Viau"><img src="https://i.imgur.com/1w8c6d2.jpg"></a></td>
    <td><a href="https://youtu.be/VHKyqZ7t8Gw" title="Video By Logan LeGrand"><img src="https://i.imgur.com/LnBucik.jpg"></a></td>
    <td><a href="https://youtu.be/VxiR4iyBruo" title="Video By Charlie Kim"><img src="https://i.imgur.com/4Qoy48c.jpg"></a></td>
    <td><a href="https://youtu.be/-IkImTe1NYE" title="Video By Aragon"><img src="https://i.imgur.com/04VNzPf.jpg"></a></td>
  </tr>
  <tr>
    <td><a href="https://youtu.be/iIUICQkdwFQ" title="Video By Logan LeGrand"><img src="https://i.imgur.com/b1LHQTy.jpg"></a></td>
    <td><a href="https://youtu.be/XOsa0FsVIsg" title="Video By PinoyDrives"><img src="https://i.imgur.com/6FG0Bd8.jpg"></a></td>
    <td><a href="https://youtu.be/bCwcJ98R_Xw" title="Video By JS"><img src="https://i.imgur.com/zO18CbW.jpg"></a></td>
    <td><a href="https://youtu.be/BQ0tF3MTyyc" title="Video By Tsai-Fi"><img src="https://i.imgur.com/eZzelq3.jpg"></a></td>
  </tr>
</table>


Running on a dedicated device in a car
------

To use openpilot in a car, you need four things
* A supported device to run this software: a [comma three](https://comma.ai/shop/products/three).
* This software. The setup procedure of the comma three allows the user to enter a URL for custom software.
The URL, openpilot.comma.ai will install the release version of openpilot. To install openpilot master, you can use installer.comma.ai/commaai/master, and replacing commaai with another GitHub username can install a fork.
* One of [the 150+ supported cars](docs/CARS.md). We support Honda, Toyota, Hyundai, Nissan, Kia, Chrysler, Lexus, Acura, Audi, VW, and more. If your car is not supported but has adaptive cruise control and lane-keeping assist, it's likely able to run openpilot.
* A [car harness](https://comma.ai/shop/products/car-harness) to connect to your car.

We have detailed instructions for [how to mount the device in a car](https://comma.ai/setup).

Running on PC
------

All of openpilot's services can run as normal on a PC, even without special hardware or a car. To develop or experiment with openpilot you can run openpilot on recorded or simulated data.

With openpilot's tools, you can plot logs, replay drives, and watch the full-res camera streams. See [the tools README](tools/README.md) for more information.

You can also run openpilot in simulation [with the CARLA simulator](tools/sim/README.md). This allows openpilot to drive around a virtual car on your Ubuntu machine. The whole setup should only take a few minutes but does require a decent GPU.

A PC running openpilot can also control your vehicle if it is connected to a [webcam](https://github.com/commaai/openpilot/tree/master/tools/webcam), a [black panda](https://comma.ai/shop/products/panda), and a [harness](https://comma.ai/shop/products/car-harness).

Community and Contributing
------

openpilot is developed by [comma](https://comma.ai/) and by users like you. We welcome both pull requests and issues on [GitHub](http://github.com/commaai/openpilot). Bug fixes and new car ports are encouraged. Check out [the contributing docs](docs/CONTRIBUTING.md).

Documentation related to openpilot development can be found on [docs.comma.ai](https://docs.comma.ai). Information about running openpilot (e.g. FAQ, fingerprinting, troubleshooting, custom forks, community hardware) should go on the [wiki](https://github.com/commaai/openpilot/wiki).

You can add support for your car by following guides we have written for [Brand](https://blog.comma.ai/how-to-write-a-car-port-for-openpilot/) and [Model](https://blog.comma.ai/openpilot-port-guide-for-toyota-models/) ports. Generally, a car with adaptive cruise control and lane keep assist is a good candidate. [Join our Discord](https://discord.comma.ai) to discuss car ports: most car makes have a dedicated channel.

Want to get paid to work on openpilot? [comma is hiring](https://comma.ai/jobs/).

And [follow us on Twitter](https://twitter.com/comma_ai).

User Data and comma Account
------

By default, openpilot uploads the driving data to our servers. You can also access your data through [comma connect](https://connect.comma.ai/). We use your data to train better models and improve openpilot for everyone.

openpilot is open source software: the user is free to disable data collection if they wish to do so.

openpilot logs the road-facing cameras, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver-facing camera is only logged if you explicitly opt-in in settings. The microphone is not recorded.

By using openpilot, you agree to [our Privacy Policy](https://comma.ai/privacy). You understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of comma. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to comma for the use of this data.

Safety and Testing
----

* openpilot observes ISO26262 guidelines, see [SAFETY.md](docs/SAFETY.md) for more details.
* openpilot has software-in-the-loop [tests](.github/workflows/selfdrive_tests.yaml) that run on every commit.
* The code enforcing the safety model lives in panda and is written in C, see [code rigor](https://github.com/commaai/panda#code-rigor) for more details.
* panda has software-in-the-loop [safety tests](https://github.com/commaai/panda/tree/master/tests/safety).
* Internally, we have a hardware-in-the-loop Jenkins test suite that builds and unit tests the various processes.
* panda has additional hardware-in-the-loop [tests](https://github.com/commaai/panda/blob/master/Jenkinsfile).
* We run the latest openpilot in a testing closet containing 10 comma devices continuously replaying routes.

Directory Structure
------
    .
    ├── cereal              # The messaging spec and libs used for all logs
    ├── common              # Library like functionality we've developed here
    ├── docs                # Documentation
    ├── opendbc             # Files showing how to interpret data from cars
    ├── panda               # Code used to communicate on CAN
    ├── third_party         # External libraries
    ├── pyextra             # Extra python packages
    └── system              # Generic services
        ├── camerad         # Driver to capture images from the camera sensors
        ├── clocksd         # Broadcasts current time
        ├── hardware        # Hardware abstraction classes
        ├── logcatd         # systemd journal as a service
        └── proclogd        # Logs information from /proc
    └── selfdrive           # Code needed to drive the car
        ├── assets          # Fonts, images, and sounds for UI
        ├── athena          # Allows communication with the app
        ├── boardd          # Daemon to talk to the board
        ├── car             # Car specific code to read states and control actuators
        ├── controls        # Planning and controls
        ├── debug           # Tools to help you debug and do car ports
        ├── locationd       # Precise localization and vehicle parameter estimation
        ├── loggerd         # Logger and uploader of car data
        ├── manager         # Deamon that starts/stops all other daemons as needed
        ├── modeld          # Driving and monitoring model runners
        ├── monitoring      # Daemon to determine driver attention
        ├── navd            # Turn-by-turn navigation
        ├── sensord         # IMU interface code
        ├── test            # Unit tests, system tests, and a car simulator
        └── ui              # The UI

Licensing
------

openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

---

<img src="https://d1qb2nb5cznatu.cloudfront.net/startups/i/1061157-bc7e9bf3b246ece7322e6ffe653f6af8-medium_jpg.jpg?buster=1458363130" width="75"></img> <img src="https://cdn-images-1.medium.com/max/1600/1*C87EjxGeMPrkTuVRVWVg4w.png" width="225"></img>

[![openpilot tests](https://github.com/commaai/openpilot/workflows/openpilot%20tests/badge.svg?event=push)](https://github.com/commaai/openpilot/actions)
[![Total alerts](https://img.shields.io/lgtm/alerts/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/alerts/)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/context:python)
[![Language grade: C/C++](https://img.shields.io/lgtm/grade/cpp/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/context:cpp)
[![codecov](https://codecov.io/gh/commaai/openpilot/branch/master/graph/badge.svg)](https://codecov.io/gh/commaai/openpilot)
