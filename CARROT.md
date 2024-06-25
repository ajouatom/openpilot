당근
------

**당근설명서**
 * 오픈파일럿은 차량마다 설정이 많이 다를수 있다.
 * 현대/기아차량은 CAN방식과 CANFD방식으로 나눌수 있다.
  * CAN방식은 거의 대부분의 차량에서 원할하게 작동될것이다.
  * CANFD방식은 롱컨이 지원되는 차량(레이더사용불가)과 롱컨이 지원되지 않고 차량의 SCC를 지원하는 차량으로 나뉜다.
 * 롱컨과 순정(비롱컨) 방식이 있다.
  * 롱컨(Longigudinal Control)이란 오픈파일럿이 직접 차량의 주행을 제어하는 것을 말한다.
   * 순정 SCC모듈에서 지원하는 모든기능이 정지된다. 다만, 상황에 따라서 AEB(긴급제동기능)기능도 정지되는 경우도 있다.
  * 순정방식은 차량의 SCC(스마트크루트컨트롤)이 직접 제어하게된다.
   * 오픈파일럿에서는 자동재출발과 속도조절(일부차량)을 지원한다.
    * 자동재출발과 속도조절은 RES/ACCEL 버튼을 만들어서 차량에 전송하는 방식으로 작동이 원할하지 않는경우도 있다.
 * [당근설정](CARROT_SETTINGS.md)
 * [당근주행설정](CARROT_SETTINGS_LONG.md)
 * [당근조향설정](CARROT_SETTINGS_LAT.md)
 * [당근맨설명](CARROTMAN.md)

**당근의 특별한기능**
 * SCC배선개조, 레이더트랙지원
 * 신호정지/출발기능
 * 다양한 주행모드(일반/연비/안전/고속)
 * 소프트오토홀드기능
 * 멋진 당근UI
 * 자동크루즈ON/OFF제어
 * 연비속도제어
 * 당근맨(스마트폰앱)지원: 네비연결,설정, 유지보수,음성지원
 * NOO Helper기능 : 자동차선변경 및 속도제어 지원
 * 차선추적기능(LaneMode): 차선이 1개만 있어도 차선을 따라감.

**지원차량**
 * 오파가 지원하는 차량들은 당근 모두 지원해야하지만 혼자만의 한계로 테스트가 힘들어서 내차 SantaFe HEV 2022위주로만 시험한다.
 * 현기차는 배선종류만 많을뿐 통신구조는 거의 같아 거의 대부분 적용이 된다.
 * 현기차의 최근 CANFD차량은 시험해보지 않았다.
 * GM의 경우  종류가 너무많아서 모른다.
 * 토요타외에 몇몇차량은 지원되는듯하다.

**현기차 지원대상(CANFD제외)**
 * 순정차량
   * 순정차량은 순정의 크루즈제어(롱컨)를 이용하고, 조향만 오파가 제어한다.
   * 물론, 오파가 속도제어는 어느정도 해준다.
 * SCC배선개조차량 (Bus2만지원됨)
   * 거의 대부분의 차량은 SCC모듈(레이더모듈)에서 차량주행제어(크루즈, 롱컨, longcontrol)를 한다.
   * SCC모듈은 기본적으로 자동차를 제어하기 위한 C-CAN에 연결되어 있다. 오파가 차량의 롱컨을 제어하기 위해서는 SCC모듈에서 나오는 차량제어코드(SCCxx등)를 변조하면된다.
   * SCC코드를 변조하기 위해, SCC모듈의 캔선을 잘라내어 오파 판다의 BUS2에 물려주면 된다.
   * 일부차량은 LKAS모듈에서 SCC제어를 하기 때문에 개조가 필요없다.
 * MDPS 배선 개조차량(지원안됨)
   * SMDPS개조된것만 지원됨
     * SMDPS란 Smart MDPS라고 sunnypilot에서 만든 용어이다.
     * 저속에서 조향이 안되는 구형차량을 위해 만든개조방식이다.
     * MDPS배선중간에 WhitePanda를 연결하여 캔데이터를 조작하여 마치 빨리달리는 차량인것 처럼 속여주는것이다.

직접설치
------

 * 브랜치
   * carrot_vxxxxxx
     * 직접설치용 주소
     * Build가 되어 있어 빠른실행가능
     * 업데이트가 안될수 있음
   * carrot-master
     * 최신버젼
     * 안정화되면 빌드되어 직접설치로 만들어짐
   * carrot-devXXXXXX
     * 시험용
     * 에러에 의해 실행되지 못하는 경우가 있음.
 * 주소입력
   * https://smiskol.com/fork/ajouatom/carrot
 * 직접설치후 git연결하기
   * ssh연결 후
   * cd /data/openpilot
   * git init
   * nano .git/config
   * 내용을 다음과 같이 수정한다.
     * [remote "origin"]
     *  url = https://github.com/ajouatom/openpilot.git
     *  fetch = +refs/heads/carrot*:refs/remotes/origin/carrot*


수동설치
------

**ssh의 연결**
 * ssh key만들기 (PEM옵션넣어야함)
   * ssh-keygen -m PEM -t rsa -f ~/.ssh/id_rsa
 * ssh key의 등록 (github에 등록함)
 * 오파의 설정
   * 네트워크 - 고급설정 - SSH 사용 : ON
   * 네트워크 - 고급설정 - SSH키 : 추가  (자신의 id를 입력)
 * 연결하기
   * MobaXterm의 설치 및 ssh설정

**소스받아오기(clone하기)**
 * ssh로그인
 * 기존것 삭제
   * cd /data
   * mv openpilot openpilot_bak
 * 데이터받기
   * git clone -b carrot-master https://github.com/ajouatom/carrotpilot openpilot

**빌드**
 * cd /data/openpilot
 * ./restart.py

**빌드확인**
 * tmux a

초기세팅
------
 * 언어설정
 * 차량선택
 * 당근기본값으로 설정: 장치(Device) -> Set to default(for HKG)
 * 차량에 맞게 설정
   * 순정차량
   * 롱컨지원차량

수정이력
------
 * 250120
   * 당근시작(개구리 삭제)


