# 特斯拉车辆集成与使用指南

[English](../en/tesla.md) | [한국어](../ko/tesla.md) | [用户文档目录](../README.md)

本文结合英文、韩文 Tesla 指南和当前仓库实现，说明 Tesla 专用功能，以及手机导航、变道等共享功能在 Tesla 上的接入条件。这里的“支持”表示代码存在相应实现和门禁，不代表所有年款、固件、线束都已实车认证，也不是自动驾驶能力承诺。通用设置的全部取值仍以 [English 设置指南](../en/settings.md)和设备上的本地化说明为准。

**自动转向灯特别限制：已有实车反馈确认左右灯能被触发，但只闪三次。当前代码已实现持续请求、完成取消和人工接管处理；这些逻辑能否让目标车辆全程持续闪烁、再可靠熄灭，仍须采集原始 CAN 日志并在封闭场地验收。不能把软件测试通过或状态 `active` 当成实车通过。**

## 目录

- [车型、硬件与适配边界](#supported-vehicles)
- [功能总览](#features)
- [设置与控制模式](#settings)
- [车辆输入、接入与静止转向](#engagement-and-standstill)
- [协同转向与驾驶员接管](#cooperative-steering)
- [手机导航、App 变道与 ATC](#navigation)
- [自动巡航设置速度](#automatic-cruise-speed)
- [自动转向灯](#automatic-turn-signals)
- [胎压、盲点、原厂预警与雷达](#vehicle-feedback)
- [唤醒、P 挡与设备电源](#wake-and-park)
- [排查与封闭场地验收](#validation)
- [实现依据与相关文档](#implementation-reference)

<a id="supported-vehicles"></a>
## 车型、硬件与适配边界

当前 `tesla/values.py` 登记的主要车型如下。年份是代码中的车型描述，不是逐车认证名单；部分 2023 年车辆已使用 HW4，应在车机“软件 → 其他车辆信息”中确认 Autopilot 计算机版本，不要仅凭交付年份选线束。

| 车型 | 硬件 | 当前代码登记年份 | 线束类别 |
|---|---|---|---|
| Model 3 | HW3 | 2019–2023 | `tesla_a` |
| Model 3 | HW4 | 2024–2026 | `tesla_b` |
| Model Y | HW3 | 2020–2023 | `tesla_a` |
| Model Y | HW4 | 2024–2025 | `tesla_b` |

线束必须适合具体车型、硬件和车辆 CAN 布局。年份登记不代表所有附加功能都适用，尤其不能据此认定无拨杆车型也支持本文的自动转向灯。

Model X 虽有平台定义，但接口明确设置为 `dashcamOnly`，不属于本文可驾驶控制的支持范围；Model S 也没有相应平台定义。不要通过选择相近车型或打开开关绕过这些限制。

原厂 Autosteer/FSD 与 carrotpilot 不应同时控制转向。在能读到原厂 Autosteer 设置的车辆上，该设置必须关闭；普通 TACC 巡航本身不被当成 Autosteer 冲突。代码针对部分已识别 EPS 固件提供 FSD 14 控制类型兼容，这不等于所有 FSD 14 版本均已适配。出现未知固件、`invalidLkasSetting` 或 `FSD 14 detected, but FW not in FSD_14_FW set` 时，应检查车辆设置和固件匹配，不要关闭冲突检查。

被识别为原厂 Summon/Autopark 会话时，Panda 停止本方控制发送并保留原厂通信。这不是 carrotpilot 实现了召唤或自动泊车，也不能用它与原厂自动驾驶同时操作。

<a id="features"></a>
## 功能总览

| 功能 | 当前行为 | 必要条件或边界 |
|---|---|---|
| 横向控制 | 按规划角度发送转向请求 | 支持车型、有效车辆输入、横向许可、无 EPS/原厂转向冲突 |
| 原厂纵向模式 | 由原厂 ACC/TACC 负责加减速 | 不启用 alpha longitudinal；不启用 DAS 限速自动滚轮 |
| carrotpilot 纵向模式 | 发送加减速和取消请求 | `AlphaLongitudinalEnabled`、可用的开发配置及匹配 Panda 固件 |
| 协同转向 | 在规划转向上叠加驾驶员扭矩意图 | 仅车速严格大于 25 km/h 时使用扭矩叠加；低速仍可正常横向控制 |
| 手机导航与 App 变道 | 使用现有 Carrot 导航、ATC 和变道状态机 | App 通信正常；保留原有使能、盲点和确认规则 |
| DAS 限速自动滚轮 | 按车辆限速逐格调整巡航设置速度 | 纵向模式、额外车辆 CAN、有效目标和原厂滚轮模板 |
| 自动转向灯 | 对现有转弯、变道请求管理灯光会话 | Model 3/Y 的已适配拨杆布局、额外 CAN、开关及完整反馈；持续闪灯待实车验收 |
| 三指触屏 | 切换 `ExperimentalMode` | 额外 CAN 提供触点数据，且已确认实验模式说明 |
| 四轮胎压 | 读取并提供给共享显示 | 额外 CAN 提供 TPMS 数据 |
| 原厂盲点、FCW/AEB | 读取原厂状态，保留安全层让行规则 | 依赖原厂信号，不是新增碰撞避免能力 |
| DAS 道路信息 | 相关数据服务可提供停止线距离和信号灯颜色 | 只提供新鲜原厂数据，不授予导航或起步权限 |
| 可选前向雷达 | 解析匹配的 Continental 雷达目标 | 特定早期车辆、对应雷达及单独正确接入 |
| 实际静止时转向 | ESP 确认静止时可允许横向控制 | 仍需横向已开启、输入就绪和其他控制许可 |
| CAN 唤醒与 P 挡处理 | 分离设备唤醒和驾驶画面状态 | 有效电源、挡位、车门和安全带信号；唤醒还需对应固件 |

代码中的 `CANBUS.party = 0`、`vehicle = 1`、`autopilot_party = 2` 是 Panda 逻辑总线编号，不是连接器针脚图。额外车辆 CAN 的检测条件是 CAN1 收到 `0x3DF`。检测成功只建立基础条件，不能替代各功能自己的消息、模板和反馈检查。

<a id="settings"></a>
## 设置与控制模式

| 参数或设置 | 默认/来源 | 作用与限制 |
|---|---|---|
| `TeslaAutoTurnSignals` | 默认关闭；启动设置中的“特斯拉自动转向灯” | 修改后需重启。只增加灯光请求，不开启导航、变道、转向或起步权限 |
| `AlwaysLateral` | 默认关闭；“全时横向控制” | 允许巡航未激活时使用已开启的横向控制，仍受前进挡、输入就绪和故障检查约束 |
| `AlphaLongitudinalEnabled` | 开发设置中的 alpha longitudinal | 决定是否申请 carrotpilot 纵向控制；受平台和构建限制，不等同实验驾驶模式 |
| `ExperimentalMode` | 共享实验模式设置 | 三指触屏切换的是此参数，不是纵向控制权或自动灯开关 |
| `ExperimentalModeConfirmed` | 用户确认实验模式说明后的状态 | 未确认时，Tesla 三指触屏不切换实验模式 |
| `TeslaWakeOnCAN` | 参数默认开启 | 只对已识别/选定的受支持 Model 3/Y 选择 Tesla CAN 唤醒固件；不是自动灯或驾驶使能开关 |

自动限速滚轮没有独立 Carrot Web 开关，协同转向的 Tesla 实现也没有独立用户开关。不要把内部功能标志当成额外可调设置。

原厂纵向与 carrotpilot 纵向是不同配置。开启 alpha longitudinal 后，Tesla 接口申请纵向控制，并在检测到额外车辆 CAN 时申请自动限速滚轮权限；Panda 只在 `ALLOW_DEBUG` 构建中接受该纵向权限，不能仅凭参数已开启就认定纵向已生效。纵向控制请求的加速度限制为 `-3.48～+2.0 m/s²`，仍受规划、驾驶员输入及安全层约束，不是车辆必然能实现的制动力保证。

**关闭巡航不一定关闭横向控制**，因为 `AlwaysLateral` 可以让横向继续工作。应查看实际横向状态，不能仅凭巡航图标判断是否仍有转向或自动灯请求。

<a id="engagement-and-standstill"></a>
## 车辆输入、接入与静止转向

车辆状态读取车速、四轮轮速、仪表速度、巡航设置速度、方向盘角度/角速度/扭矩、油门、刹车、挡位、车门、安全带、驻车和保持状态等。显示速度和巡航速度会处理 km/h 与 mph 的转换；主机油门检测使用滞回，减少接近零位的抖动，Panda 仍有独立的踏板安全判断。

### 刹车兼容

较早车辆可能提供 `0x39D IBST_status`，部分新 Model Y 不提供。当前实现改用受支持平台共有的 `0x145 ESP_status` 中 `ESP_driverBrakeApply` 判断驾驶员踩刹车，CarState 和 Panda 使用同一信号来源。Panda 在移动车辆中检测踩刹车时清除控制许可。

这解决的是缺少旧消息造成 `IBST_status not valid`、`canValid = false` 的输入兼容问题，不是修改刹车执行策略。若必需消息缺失或无效，应查线束、总线和消息完整性，不能伪造消息或取消校验。

### 接入、退出与静止

ESP 的 `ESP_vehicleStandstillSts` 用于真实静止判断，不再把原厂巡航是否处于保持状态当成唯一依据。支持车辆在驾驶控制有效时，或在前进挡启用 `AlwaysLateral` 且横向已开启时，可以在真实静止状态转向。

首次启动仍须通过输入就绪检查。EPS 故障、横向关闭、接管抑制等仍可阻止转向；“实际静止可转向”也没有取消仍在低速移动时的最低转向速度门槛。静止时允许转向不等于允许车辆自行起步。

EPS 的正常 `IDLE`、`MIN_SPEED` 抑制状态不再一概当成临时故障；永久故障及其他异常仍受处理。代码也没有仅因部分新 Model Y 的 SCCM 角度有效位为零就判定全部车辆传感器失效。

原厂 ACC 连续处于空闲状态时，不再持续取消新的巡航接入；从原厂 ACC 活动/保持状态真正转入取消状态时，仍短暂转发取消事件。实际使用仍遵守该车辆和当前 Carrot 配置的接入操作，不因本项兼容而自动接入。

<a id="cooperative-steering"></a>
## 协同转向与驾驶员接管

车速严格大于 25 km/h 且横向有效时，协同转向把驾驶员施加的方向盘扭矩转换为角度修正，叠加到规划角度，并限制变化速率。接入时有平滑处理，达到角度限制时会回退多余的累计修正，减少持续顶住限幅的情况。

在 25 km/h 及以下，控制器清除协同修正，回到正常的角度控制和限制；这不是关闭低速或静止转向。停用转向时，请求角度跟随 EPS 测量值，而不是继续沿用规划修正。

执行器反馈报告最终限幅后的请求角度，在两次发送之间保持该请求值；它不是实测执行运动，CAN 角度还存在量化。查看日志时应把请求与 EPS 测量值分别比较。

EPS 强人工干预或高角速度安全抑制仍会停止相应转向请求。原厂转向活动时，Panda 会让原厂转向消息通过并阻止本方转向消息，不能将协同转向理解为可以与 Autosteer/FSD 同时开启。

<a id="navigation"></a>
## 手机导航、App 变道与 ATC

Tesla 沿用 carrotpilot 现有的手机导航链路、`carrotMan` 数据和 `DesireHelper` 决策，不要求安装 xpilot 的导航、模型或其他控制状态机。现有 App 通信入口包括 UDP `7706`，但“已连上 App”不等于已获得导航指令或控制许可。

App 左右变道按钮使用 `LANECHANGE` 请求；短命令只发出约 0.2 秒的变道意图，是否开始和完成仍由原变道状态机决定。导航可提供转弯、准备转弯、分流和 ATC 意图，具体行为仍取决于现有设置及状态。

常用变道条件包括 `LaneChangeNeedTorque`、`LaneChangeBsd`、`LaneLineCheck`、`LaneChangeDelay`、`LaneChangeLeverWait` 等。自动灯不会绕过这些规则，也不会把自己点亮的灯误作驾驶员拨杆确认。关于共享流程和取值，见 [English 变道指南](../en/lane-change.md)。

导航可用、正在变道、请求点灯是三个不同状态。只有符合灯光门禁的自动意图才进入 Tesla 灯光执行器；人工打灯、App 请求或导航图标本身都不保证自动灯会被发送，也不保证车辆会变道或转弯。原有红绿灯和起步限制不变。

<a id="automatic-cruise-speed"></a>
## 自动巡航设置速度

本功能模拟右侧速度滚轮，依据车辆 DAS 的 `DAS_fusedSpeedLimit` 调整巡航**设置速度**，不是直接把某个 CAN 字段当作油门或刹车命令。

它要求支持车型、检测到额外车辆 CAN、启用 alpha longitudinal、巡航和 carrotpilot 均已接入，并具有有效限速和新鲜原厂空闲 `0x3C2` 滚轮模板。原厂纵向模式不启用此功能。

### 自动调整

限速目标稳定至少 0.5 秒后，按车辆巡航显示单位每次调整 1 km/h 或 1 mph；两次发送至少间隔 0.5 秒，并先等待车辆设置速度反馈。限速反馈需在 0.5 秒内更新，滚轮模板有效期为 1.5 秒；设置速度反馈等待最长 1.2 秒。

如果车辆没有响应，不会对不变的目标和设置速度无限重试。限速缺失、数据过期、踩刹车、取消或退出控制都会停止自动输出。车辆 DAS 限速也可能识别错误，驾驶员仍须核对道路限速和实际设置速度。

导航/Carrot 速度控制处于有效状态时优先，DAS 自动滚轮暂停；其结束后重新等待稳定限速。代码通过 `hudControl.activeCarrot >= 2` 判断这类优先状态，单纯连接导航并不一定暂停。

### 人工暂停与恢复

直接转动右侧速度滚轮会暂停自动调整，包括限速暂时缺失期间的手动操作。新的限速不会擅自清除人工暂停。

恢复方式为：一秒内先向一个方向滚动、再向相反方向滚动；或退出后重新接入控制。导航优先状态结束也不会自行解除仍存在的人工暂停。应在安全、允许操作的状态下使用，不要为了触发恢复而在行驶中反复试滚轮。

<a id="automatic-turn-signals"></a>
## 自动转向灯

### 开启前提与独立门禁

`TeslaAutoTurnSignals` 默认关闭，修改后需重启。当前只针对 Model 3/Y 已适配的真实拨杆和灯光布局；无拨杆或其他信号布局不会因为打开开关就获得支持。

除额外 CAN1 的 `0x3DF` 检测外，还需要下列独立输入：

| 输入 | 用途 |
|---|---|
| `0x249 SCCM_leftStalk` | 判断真实驾驶员拨杆方向和快速按下/释放 |
| CAN1 `0x3E9 DAS_bodyControls` | 提供校验正确的原厂空闲模板 |
| CAN0 `0x311 UI_warning` | 确认仪表灯的关闭、闪烁暗相、亮相和无效状态 |
| CAN1 `0x3F5` 前灯反馈 | 区分物理反馈的 OFF、ON、故障和不可用 |
| Panda TX 回显 | 确认本次发送结果，拒绝结果也需处理 |

真实拨杆输入一旦在 CAN1 被观察到，本次行车不再用 CAN0 的中立值掩盖它的丢失；从未收到 CAN1 拨杆时，保留 CAN0 兼容。输入缺失、过期或两个新鲜来源冲突都会阻止自动输出。**能自动调巡航速度不代表灯光条件已满足**，滚轮 `0x3C2` 与灯控 `0x3E9` 不是同一功能。

执行器还检查非被动模式、匹配的 CarParams/Panda 灯光权限、新鲜控制上下文、`canValid`、D 挡、实际横向有效以及无人工拨杆/双闪。意图层另保留拖车等限制。自动灯不依赖开启 alpha longitudinal，也不增加横向、变道或起步权限。

### 变道和转弯生命周期

软件目标是从有效变道/转弯意图开始请求相应方向的灯，在动作期间继续请求，在完成、意图撤销或权限丢失后取消自己拥有的灯光会话。

App 变道的灯光意图会锁存到原有 `preLaneChange → laneChangeStarting → laneChangeFinishing` 生命周期，不因约 0.2 秒命令脉冲结束而立即撤灯。导航准备转弯也可在现有距离和状态条件满足时请求灯光，不是收到任何远距离左/右转提示就立刻点灯。

转弯灯使用独立于转向状态机的完成判据，不直接跟随 `turn_disable_count` 提前结束：当前启发式结合目标方向的转向角、模型预测角速度和航向，先积累转弯开始证据，再等待移动中的稳定回正证据约一秒。已经开始的转弯即使导航暂时变成无指令或直行，也可保留原灯光会话；新的反方向指令会重置几何证据，并先清理旧方向。

这些是模型和车辆输入上的软件判据，不是“车辆已回到正确道路”的真值。停车、数据缺失或 NaN 不算完成；阈值和方向约定仍须在目标车验证，不能据此承诺每个路口都准确结束闪灯。

### 持续请求与故障退出

每次灯控发送都消耗一个新鲜原厂空闲模板，保留其他字段、递增计数器并更新校验；确认对应 TX 回显后，才允许下一次发送。不重放已消耗模板，也不通过放宽 Panda 限制维持闪灯。

主机会话上限为 60 秒；Panda 保留既有的连续请求计数与 60 秒会话边界。模板上限 1.5 秒、控制上下文上限 0.2 秒、灯光反馈上限 0.5 秒、TX 回显等待上限 1.2 秒。首次灯亮和后续灯光确认还有 2.5 秒超时。

仪表的闪烁暗相不是 OFF，不能每到暗相就当作动作结束。已亮灯后双路真实 OFF 持续 0.5 秒、反馈失效、缺少新模板等情况会触发退出，而不是只凭历史上曾亮过就一直报告正常。

普通完成/退出需要发送取消，并确认回显与双侧 UI、物理灯稳定 OFF。取消最多尝试三次、窗口五秒；未确认取消会锁存故障，本次行车不再发自动灯请求，等待下一次 onroad 实例。**取消超时、CAN 断线或设备掉电都不能保证物理灯已熄灭，应由驾驶员观察并处理。**

### 人工灯与自动灯的区分

真实拨杆方向和按下计数用于识别驾驶员操作；自动会话保存自己的方向和请求 ID。自动点灯、取消尾部或来源不明的仪表闪烁，不算人工确认变道。

检测到同向/反向人工拨杆、快速轻按、双闪或意外的对侧灯时，软件停止自动方向请求并交还灯光所有权，不立即发送可能熄灭人工灯的通用 CANCEL。随后等待真实拨杆新鲜中立、双侧仪表和物理灯 OFF 稳定 0.5 秒，再有界取消旧 Panda 会话，清理后才允许新会话。

这不能原子撤回人工动作发生前已经发出的 CAN 帧，延迟清理命令在目标车辆上的实际效果也仍须验证；“驾驶员优先”是处理策略，不是绝对不干扰的物理保证。

### 状态与诊断

状态记录在 `carState.autoTurnSignalStatus` 和 `tesla_auto_turn_signal` 日志事件中，不是新增屏幕告警。常见值如下：

| 状态 | 含义与检查方向 |
|---|---|
| `disabled:setting_off` | 自动灯开关未开启 |
| `disabled:unsupported_platform` / `disabled:passive` | 平台不支持，或处于被动模式 |
| `blocked:vehicle_bus_unavailable` | 未检测到额外车辆 CAN；检查 CAN1 `0x3DF` |
| `blocked:restart_required` | 开关与本次 CarParams 不一致，需重启应用配置 |
| `blocked:safety_configuration` | CarParams 中缺少匹配的 Tesla 灯光安全权限 |
| `blocked:driver_input_unavailable` | 真实拨杆输入缺失、过期或有歧义 |
| `blocked:context_stale` / `blocked:can_invalid` | 控制上下文或车辆 CAN 无效 |
| `blocked:lateral_inactive` / `blocked:not_in_drive` | 横向未激活，或不在 D 挡 |
| `waiting:body_template` | 缺少有效、新鲜的灯控空闲模板 |
| `waiting:ui_feedback` / `waiting:front_feedback` | 缺少对应灯光反馈 |
| `waiting:invalid_lamp_feedback` / `waiting:lamps_off` | 反馈无效，或启动前灯并未全部关闭 |
| `active` | 软件会话活动；不等于已确认物理灯全程正常 |
| `driver_takeover:waiting_off` | 人工接管，等待拨杆中立和双侧真 OFF |
| `cancelling:driver_cleanup` | 人工灯结束后清理旧会话 |
| `cancelling:body_template_starved` / `cancelling:tx_echo_timeout` | 持续请求缺少新模板或发送回显 |
| `cancelling:lamp_not_confirmed` / `cancelling:lamps_lost` / `cancelling:lamp_feedback_lost` | 未确认点亮，或后续灯光确认丢失 |
| `fault:cancel_not_confirmed` / `fault:panda_rejected_cancel` | 取消未确认或被 Panda 拒绝；不能视作已熄灯 |

<a id="vehicle-feedback"></a>
## 胎压、盲点、原厂预警与雷达

额外车辆 CAN 提供 `VCSEC_TPMSDisplay` 时，四轮胎压可进入共享显示。Tesla 原始压力转换为 psi，传感器不可用值转换为不可用结果，而不是显示成异常高胎压。没有扩展线或 TPMS 消息时，不能承诺能显示四轮数值。

原厂 `DAS_status` 提供左右后方盲点和前碰撞预警，`DAS_control` 提供 AEB 状态。是否阻止变道仍依赖现有 Carrot 盲点设置和状态机；没有告警不等于周围无人或可以安全变道。

原厂 AEB 激活时，纵向安全逻辑会阻止本方纵向/取消消息并保留原厂 AEB 消息的转发；本方不合成 AEB 事件。原厂紧急转向也有独立的转发优先处理。这是避免控制冲突的规则，不是 carrotpilot 新增了 AEB、FCW 或自动避障能力。

相关数据共享服务运行时，还可从 CAN2 的 `DAS_road` 提供停止线距离和原厂信号灯颜色；超过一秒没有更新时不继续提供旧数据。这不等于导航已接管，也不构成红灯停车或绿灯起步许可。自动转向灯不以开启数据共享服务为前提。

前向雷达是可选的，适用范围为部分约 2017 年中至 2021 年初 Model 3、约 2020 年中至 2021 年初 Model Y 使用的 Continental ARS4-B 雷达。上述范围描述的是雷达硬件，并不把未列入支持范围的旧车型变成可控车型。

雷达需按相应方案接入 CAN1，车型具有雷达 DBC 且检测到 `0x410` 后才启用，解析最多 40 个目标及距离、横向位置、相对速度等，并报告雷达不可用、遮挡或故障。无该硬件/接入时 `radarUnavailable` 为真，不能凭开启导航或设置获得原厂雷达目标。车身扩展 CAN 与雷达接线不是同一适配承诺，不要因为逻辑编号相同就自行并线。

额外 CAN 的 `UI_status2.UI_activeTouchPoints` 变化到三指时会产生按键事件。用户已确认实验模式说明后，它可切换 `ExperimentalMode`；它不切换 `AlphaLongitudinalEnabled`，也不是巡航接入、取消或自动灯开关。当前 Tesla 驱动没有实现完整的原厂按钮映射，跟车距离按键仍标为待实现；控制器的 Tesla HUD 输出也仍有 TODO，不应把数据可读取误认为可向原厂仪表显示全部 Carrot 信息。

<a id="wake-and-park"></a>
## 唤醒、P 挡与设备电源

`TeslaWakeOnCAN` 默认开启。`pandad` 只对已识别的 Tesla Model 3/Y，或在尚无 CarParams 时明确选定的受支持车型，选择与 Panda MCU 匹配的 `_tesla_wake` 固件。关闭参数会选择普通固件；改变选择应在停车状态下通过正常重启/固件管理流程应用，不要手动跳过固件签名检查。缺少所选固件是错误，不会静默当成已具备唤醒支持。

有效 `0x221 VCFRONT_LVPowerState` 电源消息可让唤醒固件在 accessory/conditioning 等车辆尚未进入驾驶状态时保持设备唤醒，但不因电源非 OFF 就自动进入驾驶画面。Panda 对电源、挡位和座舱信息检查消息长度、校验和及连续计数器。

| 车辆状态 | 驾驶画面/ignition 处理 |
|---|---|
| R、N、D，且电源/挡位消息有效 | 打开 onroad ignition 状态；不等于允许倒车、空挡驾驶控制 |
| P，安全带未系或车门打开 | 关闭 onroad ignition 状态 |
| P，安全带已系且车门关闭 | 保留此前 ignition 状态；挂 P 不一定立即退出驾驶画面 |
| 对应 CAN 消息停止更新 | 唤醒和 ignition 各自按其超时逻辑失效 |

唤醒、驾驶画面状态与实际控制许可是三件不同的事。设备亮着、P 挡仍显示驾驶画面，都不代表允许转向、加减速或自动起步；电源信号也不能替代灯光取消确认。

<a id="validation"></a>
## 排查与封闭场地验收

### 先检查数据，再判断执行问题

| 现象 | 优先检查 |
|---|---|
| 新 Model Y 仍报 `IBST_status not valid` | 是否运行了当前兼容代码；`0x145` 和其他必需 CAN 是否有效 |
| 可导航、可变道，但自动灯不亮 | 开关是否重启生效；CAN1 检测、真实拨杆、BODY 模板、UI/前灯反馈、Panda 权限分别检查 |
| 滚轮可用但灯不可用 | 两者输出地址和反馈不同；不能以滚轮成功替代灯控输入检查 |
| 转向灯只闪三次 | 同步检查请求 ID/方向、连续新模板、TX 回显和实灯反馈；未排除协议/车辆差异，不能只归因于 App 脉冲 |
| 灯还在闪但软件已取消/故障 | 软件状态不是物理熄灯证明；驾驶员处理车辆灯光，保存取消与灯光反馈证据 |
| 自动设置速度停止 | 检查人工暂停、导航优先、限速新鲜度、模板和车辆响应；不一定是故障 |
| 静止不转向 | ESP 静止、首次输入就绪、实际横向状态、挡位、EPS 和原厂冲突条件 |
| 三指触屏无反应 | 额外 CAN 触点数据、实验模式确认状态；不要把它当作安全接管按钮 |
| P 挡后驾驶屏未退出 | 先看安全带和车门，可能是保持此前 ignition 的预期行为 |

诊断应同步保留车辆车型、硬件/固件、软件版本、CarParams 功能标志、设置和原始 CAN 时间戳。灯控重点关联 `modelV2.meta.autoTurnSignalDirection/autoTurnSignalRequestId`、真实拨杆有效性/方向/按下计数、`autoTurnSignalStatus`、CAN1 模板与前灯反馈、CAN0 UI 反馈及 Panda 返回结果。日志对外分享前应去除个人和位置等敏感信息。

### 自动灯的最低验收范围

应由有资质的人员在封闭场地、驾驶员始终可立即接管的条件下进行，不应把公共道路当作首次试验环境。

| 场景 | 需要确认的结果 |
|---|---|
| 无有效意图、开关关闭或输入不满足 | 不新建自动灯请求，能定位阻止原因 |
| App 左/右变道 | 两个方向分别验证，命令脉冲结束不提前丢失会话，动作期间实灯持续，完成后实灯关闭 |
| 导航左/右转及准备阶段 | 保守触发；弯中不提前撤灯，停车不误判完成，回正后取消 |
| 导航提前转为空/直行、新反方向指令 | 已开始动作的生命周期与新会话切换正确，无旧方向残留 |
| 同向/反向拨杆、快速轻按、双闪 | 能区分人工来源，不继续压制人工灯，清理前等待真 OFF |
| 模板、反馈、回显缺失或取消失败 | 用离线回放/台架核验超时、取消和故障锁存，不伪报成功 |
| 接管后再发新请求 | 旧会话已清理后才能启动，反向切换不复用旧模板或几何证据 |

对变道、转弯、人工灯和故障场景，须同时观察物理灯与记录的软件状态。不要在车辆运动中拔线、断电或故意制造故障；断线和掉电后的物理状态应通过安全的台架或静态方法验证。软件测试不能替代上述实车验收，也不能证明车辆会正确响应持续请求。

<a id="implementation-reference"></a>
## 实现依据与相关文档

下列链接指向本仓库实际实现，便于后续核对功能条件。本文是中文补充指南，不替代原有英文、韩文成对维护规则；更新 Tesla 功能说明时也应同步核对这里的支持范围和限制。

| 主题 | 实现或说明 |
|---|---|
| 平台、硬件描述、标志和角度/加速度限制 | [`values.py`](../../../opendbc_repo/opendbc/car/tesla/values.py) |
| 车型、额外 CAN、纵向/灯光/雷达能力判定 | [`interface.py`](../../../opendbc_repo/opendbc/car/tesla/interface.py) |
| 踏板、静止、拨杆来源、胎压、触屏和原厂状态 | [`carstate.py`](../../../opendbc_repo/opendbc/car/tesla/carstate.py) |
| 控制器、协同转向和 DAS 自动滚轮 | [`carcontroller.py`](../../../opendbc_repo/opendbc/car/tesla/carcontroller.py)、[`coop_steering.py`](../../../opendbc_repo/opendbc/car/tesla/coop_steering.py)、[`speed_limit_controller.py`](../../../opendbc_repo/opendbc/car/tesla/speed_limit_controller.py) |
| CAN 封装、FSD 14 类型兼容和前向雷达 | [`teslacan.py`](../../../opendbc_repo/opendbc/car/tesla/teslacan.py)、[`radar_interface.py`](../../../opendbc_repo/opendbc/car/tesla/radar_interface.py) |
| 三指实验模式切换、静止横向和实际状态 | [`car_specific.py`](../../../openpilot/selfdrive/car/car_specific.py)、[`controlsd.py`](../../../openpilot/selfdrive/controls/controlsd.py)、[`card.py`](../../../openpilot/selfdrive/car/card.py) |
| 手机链路、变道/转弯与独立灯光意图 | [`carrot_man.py`](../../../openpilot/selfdrive/carrot/carrot_man.py)、[`desire_helper.py`](../../../openpilot/selfdrive/controls/lib/desire_helper.py) |
| 原厂 DAS 道路信息的新鲜度处理 | [`xiaoge_data.py`](../../../openpilot/selfdrive/carrot/xiaoge_data.py) |
| 自动灯执行、人工交接、取消和故障 | [`tesla_turn_signals.py`](../../../openpilot/selfdrive/car/tesla_turn_signals.py) |
| Panda 转向/纵向、原厂优先及灯光独立门禁 | [`safety_tesla.h`](../../../opendbc_repo/opendbc/safety/safety/safety_tesla.h)、[`safety_tesla_turn_signals.h`](../../../opendbc_repo/opendbc/safety/safety/safety_tesla_turn_signals.h) |
| 默认参数和本地化设置说明 | [`params_keys.h`](../../../openpilot/common/params_keys.h)、[`carrot_settings.json`](../../../openpilot/selfdrive/carrot_settings.json) |
| 唤醒固件选择和电源/挡位逻辑 | [`pandad.py`](../../../openpilot/selfdrive/pandad/pandad.py)、[`can_common.h`](../../../panda/board/drivers/can_common.h) |
| 自动灯移植、测试范围与实车限制 | [移植与测试报告](../../tesla_auto_turn_signals_migration_20261023.md) |
| 共享功能补充 | [English 变道](../en/lane-change.md)、[速度与减速](../en/speed-deceleration.md)、[巡航与跟车距离](../en/cruise-gap.md)、[雷达](../en/radar.md)、[Carrot Web](../en/carrot-web.md) |
