# carrotpilot

[한국어](#한국어) · [English](#english) · [简体中文](#简体中文)

[사용 설명서](https://g4iwnl.gitbook.io/carrotpilot) · [CarrotPilot Wiki](https://github.com/ajouatom/openpilot/wiki)

![carrotpilot](https://github.com/user-attachments/assets/4d80d256-7e66-4473-a289-04a50733b7e0)

## 한국어

carrotpilot은 [openpilot](https://github.com/commaai/openpilot)을 기반으로 현대·기아·제네시스 차량의 호환성, 종방향 제어 품질과 실제 주행 경험을 개선하기 위해 개발되는 독립적인 운전자 보조 소프트웨어입니다.

carrotpilot은 comma.ai가 개발·검토·승인하거나 공식적으로 지원하는 openpilot 배포판이 아닙니다. upstream openpilot과 다른 차량 제어, safety 및 Driver Monitoring 정책을 포함하므로 두 프로젝트의 동작과 지원 기준이 같다고 가정하면 안 됩니다.

### 설치 및 배포 정책

초기화한 장치의 설치 주소 입력 화면에서 다음 주소를 사용합니다.

```text
i.carrotpilot.app/carrot-wip
```

`carrot-wip`은 상시 개발과 배포가 동시에 이루어지는 rolling branch입니다. 차량별 문제, 위험 요소 및 버그를 수정하기 위해 수시로 업데이트되며, 일반적인 stable release처럼 장기간 고정된 동작을 보장하지 않습니다. 업데이트 전에 변경 내용을 확인하고, 업데이트 후에는 차량을 즉시 제어할 수 있는 환경에서 동작을 확인하세요.

prebuilt `carrot` branch도 공개되어 있지만 적극적으로 관리되지 않으므로 현재 권장 배포판은 `carrot-wip`입니다.

### 지원 장치

- comma three(C3)
- comma 3X(C3X)
- comma four(C4)

일부 비정품·클론 장치에도 제한적인 호환성을 제공하지만 하드웨어 구성과 품질이 일정하지 않습니다. 특히 운전자용 카메라가 없거나 정상적으로 지원되지 않는 장치가 있으므로 이러한 구성은 권장하지 않으며, 장치별 호환성과 Driver Monitoring 동작을 보장하지 않습니다.

“실험적” 또는 “제한적 호환”이라는 표시는 기술적 성숙도와 검증 범위를 설명하는 것이며 법률, 안전기준 또는 사용자의 책임을 면제한다는 의미가 아닙니다.

### 차량 지원 범위

carrotpilot의 개발과 실제 피드백은 현대·기아·제네시스(HKG) 차량을 우선으로 합니다.

저장소에는 upstream openpilot에서 이어받은 다른 제조사 차량 코드와 지원 목록도 포함되어 있지만, carrotpilot에서 해당 차량들을 모두 별도로 검증하거나 적극적으로 지원한다는 의미는 아닙니다. 실제 사용자 피드백이 없는 제조사와 차량은 동작 여부 및 품질을 보장할 수 없습니다.

차량별 호환성, 필요한 하네스와 알려진 제한은 설치 전에 [사용 설명서](https://g4iwnl.gitbook.io/carrotpilot)와 [Wiki](https://github.com/ajouatom/openpilot/wiki)에서 확인하세요.

### 현대·기아·제네시스 하네스와 롱컨

| 차량 구성 | 일반적인 연결 및 제어 방식 |
|---|---|
| CAN 차량 | comma 정품 하네스, 카메라 연결 |
| CAN-FD 일반 차량 | comma 정품 하네스, 카메라 연결 |
| CAN-FD HDA2(ADAS 모듈 장착) 차량 | 별도 하네스, ADAS 모듈 연결 |

지원되는 CAN-FD HDA2 구성에서는 장치를 ADAS 모듈에 연결하고, carrotpilot의 openpilot longitudinal control이 가속·감속 명령을 직접 생성합니다. 이는 순정 SCC의 종방향 제어를 그대로 사용하는 방식과 다릅니다.

모든 차량과 연식이 지원되는 것은 아닙니다. 같은 차명이라도 플랫폼, ECU, 카메라 및 ADAS 구성에 따라 하네스, 연결 위치와 롱컨 지원 여부가 달라질 수 있으므로 반드시 차량별 지원 정보를 확인하세요.

### upstream openpilot과 다른 점

#### Vehicle safety 및 CAN 정책

carrotpilot은 지원 차량에서 확인한 CAN 메시지 주기, 카운터, 제어 방식과 차량 오류 사례를 반영하기 위해 일부 safety 정책과 CAN 허용 조건을 upstream openpilot과 다르게 운용합니다. 특정 메시지나 타이밍 조건이 차량의 정상적인 제어 흐름을 중단하지 않도록 조정된 코드가 포함되어 있습니다.

이 차이는 carrotpilot이 comma.ai의 공식 safety model과 동일하지 않다는 뜻입니다. 또한 carrotpilot의 변경이 모든 차량과 상황에서 더 안전하다거나 특정한 기능 안전 표준을 충족한다는 보증도 아닙니다. safety 변경은 지원 차량의 로그, 재현 사례 및 저장소의 테스트를 근거로 검토해야 합니다.

#### Driver Monitoring

carrotpilot은 운전자가 전방을 주시하고 있는데도 반복적으로 경고가 발생하는 사례를 줄이기 위해 Driver Monitoring 동작을 조정합니다.

Driver Monitoring은 기본적으로 활성화됩니다. 다만 일부 비정품 장치처럼 운전자용 카메라를 사용할 수 없는 하드웨어를 위해 비활성화 설정도 존재합니다. 이 설정을 사용하면 카메라 기반 운전자 상태 감지와 그에 따른 경고·감속 기능이 제공되지 않으므로 권장하지 않습니다. Driver Monitoring의 활성 여부와 관계없이 운전자는 항상 전방을 주시하고 즉시 차량을 제어할 준비를 해야 합니다.

### 개발 및 검증 범위

이 저장소에는 openpilot에서 이어받은 테스트와 carrotpilot의 차량 제어·설정·safety 관련 테스트가 포함되어 있습니다. 그러나 이는 comma.ai의 공식 릴리스 검증이나 내부 HIL(Hardware-in-the-Loop) 체계와 동일하지 않으며, 모든 차량·하네스·장치 조합이 검증되었다는 의미도 아닙니다.

`carrot-wip`은 실제 사용 피드백에 따라 계속 변경됩니다. 문서에 명시되지 않은 구성이나 피드백이 없는 차량은 미검증 상태로 간주하세요.

### 법적 안내

차량에 설치되는 소프트웨어의 변경 또는 사용은 국가와 지역, 차량 구성에 따라 법률, 자동차 안전기준, 검사 및 보험 조건의 적용을 받을 수 있습니다.

대한민국 자동차관리법은 자동차의 안전한 운행에 지장을 줄 수 있는 소프트웨어의 임의 변경·설치·추가 또는 삭제를 제한합니다. 자세한 내용은 [국가법령정보센터의 자동차관리법](https://www.law.go.kr/lsInfoP.do?chrClsCd=010102&lsId=001747&lsiSeq=260419&viewCls=lsRvsDocInfoR)을 확인하세요.

carrotpilot은 개발 중인 운전자 보조 소프트웨어이며 인증된 완성차 부품이나 자율주행 제품이 아닙니다. 이 저장소는 특정 차량에 대한 적합성, 법적 적합성 또는 안전성을 보증하지 않습니다. 사용자는 적용되는 법률과 규정을 직접 확인하고 자신의 장치, 설치, 설정 및 사용에 대한 책임을 부담합니다. 이 안내는 법률 자문이 아닙니다.

### 문서

- [CarrotPilot 사용 설명서](https://g4iwnl.gitbook.io/carrotpilot)
- [CarrotPilot Wiki](https://github.com/ajouatom/openpilot/wiki)

carrotpilot의 공식 공개 안내는 위 사용 설명서와 Wiki를 중심으로 제공합니다. 먼저 해당 문서에서 차량별 요구 사항과 설정 설명을 확인하세요.

### 출처와 라이선스

carrotpilot은 comma.ai와 openpilot 기여자들이 개발한 [openpilot](https://github.com/commaai/openpilot)을 기반으로 합니다. openpilot은 MIT License로 배포되며 일부 구성 요소에는 별도의 라이선스가 적용될 수 있습니다. 자세한 내용은 [LICENSE](LICENSE)와 각 구성 요소의 라이선스 표시를 확인하세요.

“openpilot”과 comma 하드웨어 명칭은 해당 권리자의 명칭입니다. carrotpilot은 comma.ai와 제휴하거나 comma.ai의 보증을 받는 프로젝트가 아닙니다.

---

## English

carrotpilot is an independent driver-assistance software project based on [openpilot](https://github.com/commaai/openpilot). Its primary development focus is compatibility, longitudinal-control quality, and real-world driving behavior on Hyundai, Kia, and Genesis vehicles.

carrotpilot is not an openpilot distribution developed, reviewed, approved, or officially supported by comma.ai. It contains vehicle-control, safety, and Driver Monitoring policies that differ from upstream openpilot. Do not assume that the two projects have identical behavior or support requirements.

### Installation and release policy

After resetting the device, enter the following address in the installer URL field:

```text
i.carrotpilot.app/carrot-wip
```

`carrot-wip` is a rolling branch used for both continuous development and distribution. It is updated frequently to address vehicle-specific problems, potential hazards, and bugs. Its behavior is not frozen like a conventional stable release. Review changes before updating and verify operation in an environment where you can immediately take control of the vehicle.

A prebuilt `carrot` branch is also public, but it is not actively maintained. The currently recommended branch is `carrot-wip`.

### Supported devices

- comma three (C3)
- comma 3X (C3X)
- comma four (C4)

Limited compatibility is provided for some non-genuine or clone devices, but their hardware configuration and quality are not consistent. Some do not have a usable driver-facing camera. These configurations are not recommended, and device compatibility or Driver Monitoring operation is not guaranteed.

Labels such as “experimental” or “limited compatibility” describe technical maturity and validation scope. They do not waive legal requirements, safety standards, or user responsibility.

### Vehicle support scope

Development and real-world feedback primarily cover Hyundai, Kia, and Genesis (HKG) vehicles.

The repository also contains vehicle code and support lists inherited from upstream openpilot for other manufacturers. Their presence does not mean that every listed vehicle has been separately validated or is actively supported by carrotpilot. Vehicles and manufacturers without user feedback have no guarantee of operation or quality.

Before installation, check the [manual](https://g4iwnl.gitbook.io/carrotpilot) and [Wiki](https://github.com/ajouatom/openpilot/wiki) for vehicle compatibility, required harnesses, and known limitations.

### Hyundai, Kia, and Genesis harnesses and longitudinal control

| Vehicle configuration | Typical connection and control path |
|---|---|
| CAN vehicle | Genuine comma harness connected to the camera |
| Standard CAN-FD vehicle | Genuine comma harness connected to the camera |
| CAN-FD HDA2 vehicle with an ADAS module | Separate harness connected to the ADAS module |

On supported CAN-FD HDA2 configurations, the device connects to the ADAS module and carrotpilot's openpilot longitudinal control directly generates acceleration and deceleration commands. This differs from leaving longitudinal control entirely to the stock SCC system.

Not every vehicle or model year is supported. Harness, connection point, and longitudinal-control compatibility can vary with platform, ECU, camera, and ADAS configuration even when the vehicle name is the same. Always verify the vehicle-specific support information.

### Differences from upstream openpilot

#### Vehicle safety and CAN policy

carrotpilot uses some safety policies and CAN acceptance conditions that differ from upstream openpilot. These changes account for CAN message timing, counters, control behavior, and vehicle faults observed on supported vehicles. The repository includes adjustments intended to avoid interrupting valid vehicle-control message flows under identified timing or message conditions.

This means carrotpilot does not use an identical safety model to the official comma.ai software. It is also not a guarantee that carrotpilot is safer in every vehicle or situation, or that it complies with any particular functional-safety standard. Safety changes should be evaluated against logs, reproducible cases, and the tests maintained in this repository.

#### Driver Monitoring

carrotpilot adjusts Driver Monitoring behavior to reduce repeated warnings observed when a driver is still looking forward.

Driver Monitoring is enabled by default. A disable option exists for hardware on which the driver-facing camera cannot be used, including some non-genuine devices. Disabling it removes camera-based driver-state detection and the related warning and deceleration behavior, so this configuration is not recommended. Regardless of Driver Monitoring status, the driver must always watch the road and be ready to take immediate control.

### Development and validation scope

This repository contains tests inherited from openpilot as well as carrotpilot-specific tests for vehicle control, settings, and safety behavior. This is not equivalent to comma.ai's official release validation or internal Hardware-in-the-Loop infrastructure, and it does not mean that every vehicle, harness, and device combination has been validated.

`carrot-wip` changes continuously in response to real-world feedback. Treat undocumented configurations and vehicles without feedback as unvalidated.

### Legal notice

Installing or modifying software in a vehicle may be subject to laws, vehicle-safety requirements, inspection rules, and insurance conditions depending on the jurisdiction and vehicle configuration.

South Korea's Motor Vehicle Management Act restricts arbitrary modification, installation, addition, or deletion of software that may interfere with the safe operation of a vehicle. Refer to the [official Korean law database](https://www.law.go.kr/lsInfoP.do?chrClsCd=010102&lsId=001747&lsiSeq=260419&viewCls=lsRvsDocInfoR) for details.

carrotpilot is driver-assistance software under active development, not a certified vehicle component or an autonomous-driving product. This repository makes no warranty of vehicle compatibility, legal compliance, or safety for a particular use. Users are responsible for the device, installation, settings, use, and compliance with applicable laws and regulations. This notice is not legal advice.

### Documentation

- [CarrotPilot Manual](https://g4iwnl.gitbook.io/carrotpilot)
- [CarrotPilot Wiki](https://github.com/ajouatom/openpilot/wiki)

Official public guidance for carrotpilot is provided primarily through the manual and Wiki above. Check those documents first for vehicle requirements and setting descriptions.

### Attribution and license

carrotpilot is based on [openpilot](https://github.com/commaai/openpilot), developed by comma.ai and openpilot contributors. openpilot is distributed under the MIT License, while some components may use other licenses. See [LICENSE](LICENSE) and the license notices in individual components for details.

“openpilot” and comma hardware names belong to their respective rights holders. carrotpilot is not affiliated with or endorsed by comma.ai.

---

## 简体中文

carrotpilot 是一个基于 [openpilot](https://github.com/commaai/openpilot) 的独立驾驶辅助软件项目，主要面向现代·起亚·Genesis 车辆的兼容性、纵向控制质量和真实驾驶体验进行开发与改进。

carrotpilot 不是由 comma.ai 开发、审查、批准或正式支持的 openpilot 发行版。它包含与上游 openpilot 不同的车辆控制、安全策略和 Driver Monitoring 逻辑，因此不能假设这两个项目在行为和支持要求上完全相同。

### 安装与发布策略

在重置设备后，在安装地址输入框中使用以下地址：

```text
i.carrotpilot.app/carrot-wip
```

`carrot-wip` 是持续开发和发布同步进行的 rolling branch。为了解决车辆特定问题、风险点和错误，代码会经常更新，它并不像传统稳定版那样长期冻结不变。更新前请确认改动内容，更新后请在能够立即接管车辆控制的环境中进行验证。

公开的预构建 `carrot` 分支也存在，但没有积极维护，因此当前推荐的发布分支是 `carrot-wip`。

### 支持设备

- comma three (C3)
- comma 3X (C3X)
- comma four (C4)

部分非原厂或克隆设备也提供有限兼容性，但其硬件配置和质量并不一致。特别是部分设备没有可用的驾驶员正视摄像头，或者其摄像头支持不正常。这些配置不推荐使用，也不保证设备兼容性或 Driver Monitoring 行为。

“实验性”或“有限兼容”标签仅用于描述技术成熟度和验证范围，并不免除法律要求、安全标准或使用者责任。

### 车辆支持范围

carrotpilot 的开发和真实反馈主要聚焦现代·起亚·Genesis（HKG）车辆。

仓库中也包含从 upstream openpilot 继承来的其他制造商车辆代码和支持列表，但这并不意味着这些车辆都已被 carrotpilot 单独验证或被积极支持。没有用户反馈的制造商和车型无法保证其可用性和质量。

在安装前，请先查阅 [使用说明书](https://g4iwnl.gitbook.io/carrotpilot) 和 [Wiki](https://github.com/ajouatom/openpilot/wiki)，确认车辆兼容性、所需线束和已知限制。

### 现代·起亚·Genesis 线束与纵向控制

| 车辆配置 | 常见连接与控制方式 |
|---|---|
| CAN 车辆 | 使用 comma 原厂线束，连接摄像头 |
| CAN-FD 普通车辆 | 使用 comma 原厂线束，连接摄像头 |
| 配备 ADAS 模块的 CAN-FD HDA2 车辆 | 使用单独线束，连接 ADAS 模块 |

在受支持的 CAN-FD HDA2 配置中，设备连接到 ADAS 模块，carrotpilot 的 openpilot 纵向控制会直接生成加速和减速指令。这不同于完全依赖原厂 SCC 进行纵向控制的方式。

并非所有车辆和车型年都受支持。即使车名相同，线束、连接位置和纵向控制兼容性也可能因平台、ECU、摄像头和 ADAS 配置而不同，因此请务必确认车辆具体支持信息。

### 与 upstream openpilot 的差异

#### Vehicle safety 与 CAN 策略

carrotpilot 会在部分安全策略和 CAN 接受条件上与 upstream openpilot 不同。这些调整考虑了受支持车辆上已确认的 CAN 消息周期、计数器、控制方式以及车辆故障案例。仓库中包含针对特定消息或时序条件进行调优的代码，旨在避免干扰车辆正常控制流程。

这意味着 carrotpilot 与 comma.ai 官方 safety model 并不完全一致，也不能保证它在所有车辆和场景下都更安全，或满足特定的功能安全标准。安全相关修改应基于日志、可复现案例和本仓库中的测试来评估。

#### Driver Monitoring

carrotpilot 会调整 Driver Monitoring 行为，以减少当驾驶员确实在看前方时仍反复触发告警的问题。

Driver Monitoring 默认开启。但在一些无法使用驾驶员正视摄像头的硬件上（包括部分非原厂设备），也提供关闭选项。关闭后，将无法使用基于摄像头的驾驶员状态检测，以及相关的告警和减速功能，因此不推荐使用此配置。无论 Driver Monitoring 是否启用，驾驶员都应始终注视道路，并准备好随时接管车辆控制。

### 开发与验证范围

本仓库包含从 openpilot 继承来的测试，以及 carrotpilot 自有的车辆控制、设置和安全相关测试。但这并不等同于 comma.ai 的官方发布验证或内部 HIL（硬件在环）体系，也不意味着所有车辆、线束和设备组合都已完成验证。

`carrot-wip` 会根据真实使用反馈持续更新。对于文档中未说明的配置或没有反馈的车辆，应视为未验证状态。

### 法律声明

在车辆中安装或修改软件，可能会受到司法管辖区和车辆配置相关法律、汽车安全要求、检查规定及保险条款的约束。

韩国《汽车管理法》限制对可能影响车辆安全运行的软件进行任意修改、安装、增设或删除。详细内容请参考 [国家法令信息센터的《汽车管理法》](https://www.law.go.kr/lsInfoP.do?chrClsCd=010102&lsId=001747&lsiSeq=260419&viewCls=lsRvsDocInfoR)。

carrotpilot 是一个正在开发中的驾驶辅助软件，而不是经认证的车辆零件或自动驾驶产品。本仓库不对特定场景下的车辆适配性、法律合规性或安全性做出担保。用户应自行确认适用法律法规，并对自身设备、安装、设置和使用承担责任。本声明不构成法律意见。

### 文档

- [CarrotPilot 使用说明书](https://g4iwnl.gitbook.io/carrotpilot)
- [CarrotPilot Wiki](https://github.com/ajouatom/openpilot/wiki)

carrotpilot 的官方公开说明主要以上述使用说明书和 Wiki 为主。请先查阅这些文档，确认车辆要求和设置说明。

### 署名与许可证

carrotpilot 基于 comma.ai 和 openpilot 贡献者开发的 [openpilot](https://github.com/commaai/openpilot)。openpilot 以 MIT License 发布，部分组件可能适用其他许可证。详细信息请查看 [LICENSE](LICENSE) 和各组件的许可证声明。

“openpilot”与 comma 硬件名称均属于其相应权利人。carrotpilot 不属于 comma.ai 的关联项目，也未获得 comma.ai 的背书。
