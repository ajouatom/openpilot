* [당근설명](CARROT.md)

Why carrot?
------
In Korea, there's a humorous and casual way of saying 'of course' using the word for 'carrot,' which is 'dang-geun' in Korean. This is because 'dang-geun' (carrot) sounds very similar to 'dang-yeon-ha-da', which means 'of course.' It's a playful pun commonly used in everyday conversation among Koreans. So, if you hear someone responding with 'dang-geun,' they are affirmatively saying 'of course' in a light-hearted way. (한국말로 "당근"은 "당연하다"라는 표현으로 많이 사용한다. 당근파일럿에서는 "필요한 기능은 당연히 있다!" 라는 의미로 당근파일럿을 개발하였다.)

What is the useful features of CarrotPilot? 
------
**Radar Tracks: HKG cars only**
  - If you only turn on the "EnableRadarTracks" in the Toggle menu, it will be activated when the ignition is turned on.(레이더트랙을 지원하는 차량의 경우, 시동시 자체 활성화하는 기능이 있다.)

**SCC Wiring Modification Support: HKG cars only**
  - If the C-CAN wiring of the SCC modules is bypassed and connected to BUS2, long control using radar is supported.(레이더(SCC)모듈의 배선을 개조하여 BUS2에 연결한 경우, 롱컨을 지원한다.)
    
**APM: CarrotMan for Android**
  - When CarrotMan connected, APM is activated. (CarrotMan이 연결되면 APM이 활성화된다.)
  - CarrotMan is for Android Phone/Tablet.  (APM은 안드로이드 폰이나 태블릿에서 작동한다)
  - The features of CarrotPilot can be set remotely via APM (당근의 기능은 APM으로 원격설정이 가능하다.)
    
**APN: Korea navigation only**
  - When navigatgion is connected, you can use APN. (APM이 설치된 폰에 특정 네비게이션을 설치하면 APN모드를 사용할 수 있다)
  - It's a feature that automatically recuces speed in response to speed cameras and speed bumps for accident prevention. (오파의 NOO활성화 및 사고방지턱, 과속카메라에 대해 속도조절이 된다)
    
**Carrot UI**
  - You can experience a variety of APILOT UIs using APM. (APILOT의 다이내믹한 UI를 경험할 수 있다)
  - Path UI: When long control is turned off or on, and depending on whether it's in lane mode or laneless mode, it's possible to specify different shapes and colors for the path. Above all, the moving path shape looks cool. (레인모드, 레인리스모드, 크루즈OFF에 대한 패쓰경로표시 방식 및 색상을 변경할 수 있다)
  - Debug plotting: It's a plotting tool for tuning long control and steering control. (튜닝을 위하여 차량의 주행상황, 레이더등의 그래프를 지원한다)
    
**APILOT Traffic Stop/Go Mode: APILOT feature**  

**Custom Acceleration Based on Speed: APM**
  - In APM, you can fine-tune the accleration based on driving speed.  

**Fuel Efficiency Control Feature: APM, APILOT feature**
  - It controls to slightly exceed the set speed, then naturally reduces acceleration at the set speed to lessen engine strain (Hybrid vehicles naturally switch to EV mode).
  - CruiseEcoControl(APM)
    
**Use Lane line mode:**
  - If the speed exceeds the set speed, it automatically switches to lane mode. This method uses lat_mpc
    
**Automatic Engage: HKG car only**
  * Setting - APM/Carrot - Cruise (설정/당근/크루즈)
    * (HKG) Auto Cruise Control (자동 크루즈 제어)
      * 1: Enable Automatic cruise control (자동크루즈 제어 켜기)
      * 2: Enable Automatic cruies control, and auto cruise activate when the lead car approaches and you do not gas/brake press. (자동크루즈 제어켜기, 선행차가 가까와질때)
  * Setting - APM - Cruise
    * BrakeCruiseON: Stopping, Traffic (브레이크해제 크루즈ON)
      * At brake release, Automatic cruise activate when RED traffic light is detect or speed is faster than setting speed[GasCruise ON: Speed] 
    * GasCruise ON: Speed (엑셀 크루즈ON:속도)
      * If the gas pedal is pressed and the speed exceeds the set speed, Enage automatically activates
    * GasCruise OFF: Mode (엑셀 크루즈OFF:모드)
      * 1: When the speed is lower than the Gas Cruise ON speed, turning off the cruise control when the gas pedal is pressed. (엑셀크루즈ON속도보다 낮은속도로 주행중에 엑셀을 밟으면 크루즈를 끈다)
      * 2: Including item 1, and also turning off the cruise control when the gas pedal is pressed during deceleration due to traffic light detection. (1번항목포함하고, 신호감속중 엑셀을 밟으면 크루즈를 끈다)    
    * CruiseOnDist(0cm) (크루즈ON거리)
      * If driving without pressing either the brake or gas pedal, and the distance detected by the radar falls within the set value, then the cruise control is activated. But, if the value is negative, only a warning is issued." (브레이크/엑셀을 모두 밟지 않고 주행하다가 레이더거리가 설정값 이내에 들어오면 크루즈를 켠다. 단, 음수의 경우에는 경고만 한다)
  * SOFTHOLD: It's a feature that corresponds to the Autohold function. (오토홀드의 기능을 대신하는 기능)  

**Voice Recognition: APM, Korean only, APILOT feature, under construction!**
  - The connected APM supports voice recognition. It allows for lane changes and speed control using voice commands.

---
[![openpilot on the comma 3X](https://github.com/commaai/openpilot/assets/8762862/f09e6d29-db2d-4179-80c2-51e8d92bdb5c)](https://comma.ai/shop/comma-3x)

What is openpilot?
------

[openpilot](http://github.com/commaai/openpilot) is an open source driver assistance system. Currently, openpilot performs the functions of Adaptive Cruise Control (ACC), Automated Lane Centering (ALC), Forward Collision Warning (FCW), and Lane Departure Warning (LDW) for a growing variety of [supported car makes, models, and model years](docs/CARS.md). In addition, while openpilot is engaged, a camera-based Driver Monitoring (DM) feature alerts distracted and asleep drivers. See more about [the vehicle integration](docs/INTEGRATION.md) and [limitations](docs/LIMITATIONS.md).

<table>
  <tr>
    <td><a href="https://youtu.be/NmBfgOanCyk" title="Video By Greer Viau"><img src="https://github.com/commaai/openpilot/assets/8762862/2f7112ae-f748-4f39-b617-fabd689c3772"></a></td>
    <td><a href="https://youtu.be/VHKyqZ7t8Gw" title="Video By Logan LeGrand"><img src="https://github.com/commaai/openpilot/assets/8762862/92351544-2833-40d7-9e0b-7ef7ae37ec4c"></a></td>
    <td><a href="https://youtu.be/SUIZYzxtMQs" title="A drive to Taco Bell"><img src="https://github.com/commaai/openpilot/assets/8762862/05ceefc5-2628-439c-a9b2-89ce77dc6f63"></a></td>
  </tr>
</table>

To start using openpilot in a car
------

To use openpilot in a car, you need four things:
1. **Supported Device:** a comma 3/3X, available at [comma.ai/shop](https://comma.ai/shop/comma-3x).
2. **Software:** The setup procedure for the comma 3/3X allows users to enter a URL for custom software. Use the URL `openpilot.comma.ai` to install the release version.
3. **Supported Car:** Ensure that you have one of [the 250+ supported cars](docs/CARS.md).
4. **Car Harness:** You will also need a [car harness](https://comma.ai/shop/car-harness) to connect your comma 3/3X to your car.

We have detailed instructions for [how to install the harness and device in a car](https://comma.ai/setup). Note that it's possible to run openpilot on [other hardware](https://blog.comma.ai/self-driving-car-for-free/), although it's not plug-and-play.

To start developing openpilot
------

openpilot is developed by [comma](https://comma.ai/) and by users like you. We welcome both pull requests and issues on [GitHub](http://github.com/commaai/openpilot).

* Join the [community Discord](https://discord.comma.ai)
* Check out [the contributing docs](docs/CONTRIBUTING.md)
* Check out the [openpilot tools](tools/)
* Read about the [development workflow](docs/WORKFLOW.md)
* Code documentation lives at https://docs.comma.ai
* Information about running openpilot lives on the [community wiki](https://github.com/commaai/openpilot/wiki)

Want to get paid to work on openpilot? [comma is hiring](https://comma.ai/jobs#open-positions) and offers lots of [bounties](docs/BOUNTIES.md) for external contributors.

Safety and Testing
----

* openpilot observes [ISO26262](https://en.wikipedia.org/wiki/ISO_26262) guidelines, see [SAFETY.md](docs/SAFETY.md) for more details.
* openpilot has software-in-the-loop [tests](.github/workflows/selfdrive_tests.yaml) that run on every commit.
* The code enforcing the safety model lives in panda and is written in C, see [code rigor](https://github.com/commaai/panda#code-rigor) for more details.
* panda has software-in-the-loop [safety tests](https://github.com/commaai/panda/tree/master/tests/safety).
* Internally, we have a hardware-in-the-loop Jenkins test suite that builds and unit tests the various processes.
* panda has additional hardware-in-the-loop [tests](https://github.com/commaai/panda/blob/master/Jenkinsfile).
* We run the latest openpilot in a testing closet containing 10 comma devices continuously replaying routes.

User Data and comma Account
------

By default, openpilot uploads the driving data to our servers. You can also access your data through [comma connect](https://connect.comma.ai/). We use your data to train better models and improve openpilot for everyone.

openpilot is open source software: the user is free to disable data collection if they wish to do so.

openpilot logs the road-facing cameras, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver-facing camera is only logged if you explicitly opt-in in settings. The microphone is not recorded.

By using openpilot, you agree to [our Privacy Policy](https://comma.ai/privacy). You understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of comma. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to comma for the use of this data.

Licensing
------

openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

---

<img src="https://d1qb2nb5cznatu.cloudfront.net/startups/i/1061157-bc7e9bf3b246ece7322e6ffe653f6af8-medium_jpg.jpg?buster=1458363130" width="75"></img> <img src="https://cdn-images-1.medium.com/max/1600/1*C87EjxGeMPrkTuVRVWVg4w.png" width="225"></img>

![openpilot tests](https://github.com/commaai/openpilot/actions/workflows/selfdrive_tests.yaml/badge.svg)
[![codecov](https://codecov.io/gh/commaai/openpilot/branch/master/graph/badge.svg)](https://codecov.io/gh/commaai/openpilot)
