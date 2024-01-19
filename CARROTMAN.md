당근맨
------
 * 설치
   * PC와 설치하려는 스마트폰을 USB로 연결(디버그 활성화)
   * PC에 adb설치
   * adb install -r CarrotMan16.apk 
   * 스마트폰에서 당근맨의 권한은 모두 허용
   * 네비게이션을 사용한다면?
     * adb shell pm grant com.ajouatom.carrotman android.permission.READ_LOGS
     * [당근맨은 설치했고 잘돌아가는데 네비가 없어요!, 주행중인 당근파일럿 사진첨부합니다.](http://pf.kakao.com/_xbviqG/chat)
           
 * SSH입력
   * SSH키 전송: 위에서 만든 id_rsa파일을 스마트폰에 복사
   * 당근맨에서 SSH 버튼 -> 개인키 버튼을 누르고, id_rsa 선택  (파일제일위에 BEGIN RSA PRIVATE KEY라고 되어 있어야함)
 * 연결
   * 오픈파일럿과 스마트폰은 동일 네트웍에 연결되어 있어야함.
   * 스마트폰에 핫스팟을 켜고, 오파를 연결시키는 방법이 제일 쉬움.
   * 연결이 성공되면 오파화면좌측상단에 APM 또는 APN이라고 아이콘이 생김.
 * 네비게이션 연결
   * 네비게이션 안내모드 또는 주행모드로 변경되면, 오파화면에는 APN아이콘이 생긴다.
   * 네비를 연결하면 과속카메라, 과속방지턱등 다양한 속도제어를 할 수 있다.
 * 오파에서 에러가나면?
   * TMUX버튼을 얼른 누른다.
   * 하단의 복사 버튼을 누르고... 여기에 남겨준다.  [이슈남기기](https://github.com/ajouatom/carrotpilot/issues)
 * 소프트웨어 버젼관리
   * GIT BRANCH
     * 원격 또는 오파내부에 있는 브랜치로 변경한다.
   * GIT PULL
     * 원격에서 소스를 가져온다.
     * 만일 가져오기에 오류가 있다면, GIT RESET을 눌러주고 다시 한다.
   * GIT RESET
     * GIT을 리셋한다.
   * GIT SYNC
     * 불필요한 깃을 없앨때 사용한다.
   * GIT RESET HEAD
     * 가장 최근의 추가내용을 삭제한다.
   * REBOOT
     * 오파를 리부트한다.
   * RESTART
     * 오파만 리스타트한다. 이것을 실행하면 간혹 메모리 부족이나 에러가 날수 있다. (시동꺼진상태에서만 사용)
   * REBUILD
     * 다시 모두 빌드하고 싶을때사용한다. 이건 할필요없다. 
