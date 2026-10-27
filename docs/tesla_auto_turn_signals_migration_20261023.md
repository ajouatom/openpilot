# Tesla 自动转向灯移植与测试报告

日期：2026-10-23

版本说明：第 1–7 节记录初始移植时的本机验证及边界，随后该版本已发布为 `73fb3a6b5`。第 8 节记录实车不亮灯反馈后的复现；第 9、10 节修复及测试适配已发布为 `c75eeaea0b`。第 11 节是随后“三闪”反馈的持续生命周期、人工接管与完成判断修订，取代前文对应的旧行为说明；各版本均未完成目标车辆验收。

**2026-10-24 补充：第 12 节已复现第 11 节修订中的真实 Cap'n Proto 数组切片异常。它可在自动转弯灯请求后使模型调用路径抛异常；此前普通 Python 列表测试未覆盖这一差异，原通过数量不能证明已发布版本没有此崩溃。第 12 节记录本机最小修复及验证，实际交付状态以交付消息为准。**

**后续方向反馈：第 13 节使用参考项目的四字节拨杆布局，复现了右重拨被读成左、左拨无响应，以及不可用状态被误作人工接管的问题。第 14 节已落实按实际帧长度兼容新旧协议、来源消歧和灯控诊断，并与第 12 节崩溃修复一起完成本机回归；没有修改 Panda 策略，物理持续闪灯仍未通过目标车辆验收。**

发布说明（2026-10-24）：按用户要求，将第 9、10 节修复一起提交并发布至 `navipilot/openpilot:try`，以前版 `73fb3a6b5` 为基线；各节“未提交/未推送”为对应测试时点的状态。实际发布 SHA 及是否成功以远端核验和交付消息为准。本次不包含原有 Hyundai DBC、字体改动，不新增固件保护，也不代表已部署、刷写或完成实车验收。

第 11 节发布：以 `c75eeaea0b724362e0404d6a6618038e7bd291ca` 为基线，仅向 `navipilot/openpilot:try` 非强制发布第 11 节修订。本地仍为 `Carrot`，`origin` 仍为 GitCode；实际新 SHA 以远端核验和交付消息为准。

本轮修复基线为 `0fa35123a339eaae3901751c9442cdade1fe95d8`，交付范围为第 12、14 节的软件修复、相关测试及本报告，不包含既有字体、Hyundai DBC 或先前未提交的全仓导入修复。发布目标仍为 `navipilot/openpilot:try`，发布是否完成及 SHA 以远端核验和交付消息为准。

## 1. 结论与交付边界

已在 carrot 实现：使用既有导航/ATC 转弯、变道意图，向受支持的 Tesla 请求左右转向灯，保持至既有动作结束，再取消自身请求。包含自动灯来源隔离、真实拨杆优先、反馈确认、独立失联看门狗、有界取消重试和 Panda 安全约束。

**本次完成的是代码移植、本机合成/回归测试及固件构建，不是实车认证。功能默认关闭，不能据此宣称所有 Tesla 年款均可正常使用。目标车辆必须先完成封闭场地验收。**

| 项目 | 结果 |
|---|---|
| 主机端灯控、意图、拨杆、车型、启动门禁 | 221 passed，另有 23 subtests passed |
| Tesla 安全层完整回归 | 107 passed，3 failed，38 skipped；另有 105 subtests passed |
| 上述 3 项失败的原版对照 | 使用未修改的 HEAD Tesla 安全代码，复现同样 3 项失败 |
| 排除已有失败后的安全回归 | 107 passed，37 skipped，4 deselected |
| 新代码及聚焦修改的 Ruff | 通过；card/modeld 原有 25 条诊断无新增 |
| 原生 Params、Cap'n Proto C++ | 构建成功；新参数默认关闭及隔离目录读写通过 |
| F4/H7 普通及 Tesla-wake 固件 | 四种均成功构建、开发签名 |
| 原有固件选择测试及实际文件签名 | 4 passed；四个本地构建的选择与 128 字节签名读取均通过 |
| 本地 Wiki | 三语说明补丁应用、重新生成；595 个 Markdown 文件验证通过 |
| 用户文档映射、diff 空白检查 | 通过 |
| 实车、部署、刷写、Git 提交/推送、远端 Wiki 发布 | 均未进行 |

完整安全回归**不是全绿**。已有失败未被改成跳过、删除断言或通过修改转向安全策略来掩盖。

## 2. 基线、范围与支持条件

- 参考仓库：`/home/jeff/xpilot`，分支 `dev20261018-slgame-jetlink`，HEAD `67936d9e0555b8187a6df405f5a50de69924626a`。
- 目标仓库：`/home/jeff/carrot`，分支 `Carrot`，基线 `214e7c24f7635201e19e970b465413a2a6ea10b2`。
- 保留 carrot 原有手机导航入口、CarrotMan、ATC、变道状态机、转弯判断、BSD/扭矩/车道线门禁、启动准备及红绿灯起步限制。
- 未移植 xpilot 的 NavAssist 私有导航协议、Jetlink、模型、SP 参数体系、氛围灯或诊断 UI。手机仍通过 carrot 已有方式提供导航；不是让 xpilot 专用 App 协议直接兼容 carrot。
- 未修改 Hyundai 行为。任务开始前已有的两份 Hyundai DBC 和四份字体文件修改保持原样，未清理其他已有生成文件。

启用必须同时满足：

1. Model 3/Y；Model X 仍不支持本功能，未解除原有 dashcam-only 限制。
2. 检测到额外 vehicle CAN 总线：bus 1 的 `0x3DF`。
3. 用户启用 `TeslaAutoTurnSignals`，并重启以重新生成 CarParams。
4. 存在有效、足够新鲜的 `SCCM_leftStalk` 真实拨杆信号。没有拨杆或使用其他信号布局的年款不能仅靠开关获得支持。
5. 有真实、校验正确的 idle `DAS_bodyControls` 模板，UI 和物理灯反馈有效、初始双侧关闭。
6. 最终横向控制处于有效状态，车辆 CAN 有效、D 挡，modelV2/carControl 新鲜且有效，驾驶员未操作拨杆、未开双闪。

缺少这些条件时不会通过伪造反馈、猜测驾驶员状态或放宽起步/转向权限来强行开启。

## 3. 实现清单

| 文件 | 改动 |
|---|---|
| `openpilot/selfdrive/car/tesla_turn_signals.py` | 独立灯控执行器：会话、原始 CAN、模板、发送回显、取消、超时和诊断 |
| `openpilot/selfdrive/controls/lib/desire_helper.py` | 输出灯光方向/请求编号；剥离自动灯的驾驶员归因；完成消费与防重触发 |
| `openpilot/selfdrive/modeld/modeld.py` | 仅按 Tesla 配置启用 producer，发布既有 modelV2 的追加字段 |
| `openpilot/selfdrive/car/card.py` | 接收真实反馈；独立取消检查；在最终启动/横向门禁后追加灯光 CAN |
| `opendbc_repo/opendbc/car/tesla/carstate.py` | 可选真实拨杆接收、新鲜度、左右按压计数；同批次按下/释放不丢失 |
| `opendbc_repo/opendbc/car/tesla/interface.py`、`values.py` | Model 3/Y、vehicle bus、参数开关门禁；独立 CP flag 32 / safety flag 8 |
| `opendbc_repo/opendbc/car/car.capnp` | 追加 autoTurnSignalDirection/Status、turnSignalDriverValid/Direction |
| `openpilot/cereal/log.capnp` | 追加 model meta 的方向及 UInt64 请求编号；不更改旧字段编号 |
| `opendbc_repo/opendbc/safety/safety/safety_tesla_turn_signals.h` | 真实模板约束、可修改位、计数/校验、会话和取消重试限制 |
| `opendbc_repo/opendbc/safety/safety/safety_tesla.h` | 独立权限接入；保留 Summon、转向、纵向、AEB 等原规则 |
| `openpilot/common/params_keys.h`、`openpilot/selfdrive/carrot_settings.json` | 注册默认关闭的持久参数；接入启动设置菜单和中英韩说明 |
| `docs/user/{en,ko}/{settings,tesla}.md` | 默认值、生效方式、能力要求、诊断和安全边界 |
| `tools/docs/wiki_settings/patches/tesla-auto-turn-signals.patch`、`README.md` | 三语 Wiki MANUAL 区域补丁及本地应用方式；不发布远端 |

新增测试文件：

- `openpilot/selfdrive/car/tests/test_tesla_turn_signals.py`
- `openpilot/selfdrive/car/tests/test_tesla_signal_card.py`
- `openpilot/selfdrive/controls/tests/test_tesla_auto_signal_intent.py`
- `opendbc_repo/opendbc/car/tesla/tests/test_auto_turn_signals.py`
- `opendbc_repo/opendbc/safety/tests/test_tesla_turn_signals.py`

为运行实际 C 安全回归，修正了 `tests/libsafety/safety.c` 的主机夹具兼容性：补足生产驱动已有的 `safety_tx_buffered_for_fwd` 符号，并将 Python 测试的旧 bus/address 转发 ABI 转接到现有 packet hook。没有修改生产转发规则。原夹具直接构建会出现缺失符号，随后旧转发 ABI 会导致段错误；修正后转发测试正常执行。

## 4. 行为与保护

### 意图与来源隔离

转弯使用 carrot 已有 turn desire；变道使用既有 pre/starting/finishing 状态，finishing 期间不提前取消。准备转弯可在既有导航方向和距离窗口内预亮灯，但不因此授予转向权限。

每段新方向请求有独立编号，同一持续请求保持编号不变；完成后的等待状态不重复开灯，导航意图变化或真正开始下一次既有变道后才重新考虑。模型进程新实例使用单调时间初始化编号。

`leftBlinker/rightBlinker` 保留真实车辆灯反馈。新增所有权字段仅用于 DesireHelper 的驾驶员输入归因：自动灯及取消尾巴不能把左侧 ATC 的等待状态错误推进为变道，也不能在转弯结束后触发第二次变道。真实拨杆方向和按压计数仍可确认驾驶员操作。

### CAN 执行与确认

- bus 1，`0x3E9`，8 字节；左请求 1、右请求 2、取消 3。
- 仅从校验正确的原厂 idle 帧生成命令，保留非灯控字段；仅改变 request、reason、counter、checksum。
- 每个接收模板仅消费一次；计数器取原始值加一并按 4 bit 回绕。
- 匹配 bus 1 的发送回显 `0x81`；`0xC1` 拒绝反馈不是成功。
- 真实 UI：bus 0，`0x311`，校验 checksum；物理灯：bus 1，`0x3F5`。
- 关闭确认需要 UI 与物理灯双侧均为 OFF，持续 0.5 秒且反馈保持新鲜。闪烁的灭相位、FAULT、SNA、UI 无效值均不能假装关闭成功。
- 真实拨杆、同批次快速按压或双闪出现后取消自动请求并释放自动所有权；手动接管状态不是“已确认物理熄灯”。

### 超时与安全层

| 限制 | 当前值 |
|---|---|
| 原始模板最长年龄 | 1.5 秒 |
| 控制/意图上下文最长年龄 | 0.2 秒 |
| UI/物理灯反馈最长年龄 | 0.5 秒 |
| 单次发送等待回显 | 1.2 秒 |
| 亮灯确认等待 | 2.5 秒 |
| 单次自动会话 | 60 秒 |
| 取消确认等待 | 5 秒 |
| 取消尝试 | 最多 3 次 |
| Panda 活跃帧数量 | 每会话最多 1200 帧 |

上下文看门狗在 CAN 接收侧独立推进，controlsd/modeld 停止更新后也可请求取消，不依赖新的有效控制消息到来。Panda 取消重试允许在已接收首次取消、但主机丢失回显时进行，仍受 3 次/5 秒边界约束。

Panda 必须拥有独立 feature bit、正确 bus/长度/标准帧、真实新鲜 idle 模板、匹配非所属位、counter 和 checksum 才允许发送。反方向必须先取消；取消在 `controls_allowed` 为 false 时仍可执行，但不能绕过 Summon 或继电器故障限制。旧 feature flag 组合仍拒绝新增灯控输出。

取消无法确认或被拒绝时记录 fault，停止后续自动输出至下一次创建 onroad 控制实例；所有权保留至真实 OFF 稳定，避免尾巴误触发。**这些逻辑不能在设备断电、card 停止运行或 CAN 断线后凭空发送关闭命令，也不能保证车辆物理灯必然关闭。**

## 5. 测试环境与复现命令

所有本机验证均先进入 carrot，通过用户指定的 `op venv`：

```bash
cd /home/jeff/carrot
op venv
python -c 'import sys; print(sys.executable)'
# /home/jeff/carrot/.venv/bin/python
```

自动化 shell 使用交互 bash 加载 `op` alias，再在该 venv 的子 shell 执行以下命令；没有绕过此要求直接使用系统 Python。缺失的测试依赖 `parameterized==0.9.0` 已安装至此 venv；局域网代理当时不可达，改用直接连接安装成功。未更改依赖清单。

### 主机端与原有门禁

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python -m pytest -q -o addopts= -p no:cacheprovider \
  --confcutdir=openpilot/selfdrive \
  openpilot/selfdrive/car/tests/test_tesla_turn_signals.py \
  openpilot/selfdrive/car/tests/test_tesla_signal_card.py \
  openpilot/selfdrive/controls/tests/test_tesla_auto_signal_intent.py \
  openpilot/selfdrive/controls/tests/test_desire_helper_stalk.py \
  opendbc_repo/opendbc/car/tesla/tests/test_auto_turn_signals.py \
  opendbc_repo/opendbc/car/tesla/tests/test_tesla.py \
  openpilot/selfdrive/controls/tests/test_lateral_readiness.py \
  openpilot/selfdrive/controls/tests/test_lateral_startup_control.py
```

结果：**221 passed，23 subtests passed**。覆盖左右完整会话、稳定熄灯、超时/无回显/拒绝、驾驶员/双闪接管、失联独立取消、故障锁存、所有权尾巴、schema 实际 reader 与 sendcan 字节序列化、真实 DBC 拨杆解析、Model 3/Y/X 能力组合、finishing 保持及完成消费。

card 测试执行生产 `controls_update` 方法的完整 AST，替换外部 I/O，不启动设备进程；验证启动未就绪、ImpactDashcamReboot、过期 model/control 均不能发送开灯命令。它不是实车或完整多进程回放。

### C 安全回归

必须先重建库，不能使用旧 `libsafety.so`。本次使用现有 `tests/libsafety/SConscript`，通过会话目录中的最小 SConstruct 调用；等价的可复现编译为：

```bash
gcc -c -fPIC -Wall -Werror -nostdlib -fno-builtin -std=gnu11 \
  -Wfatal-errors -Wno-pointer-to-int-cast \
  -Iopendbc_repo/opendbc/safety/tests/libsafety \
  -Iopendbc_repo/opendbc/safety/board \
  -Iopendbc_repo/opendbc/safety \
  opendbc_repo/opendbc/safety/tests/libsafety/safety.c \
  -o opendbc_repo/opendbc/safety/tests/libsafety/safety.os
gcc -shared opendbc_repo/opendbc/safety/tests/libsafety/safety.os \
  -o opendbc_repo/opendbc/safety/tests/libsafety/libsafety.so

PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python -m pytest -q -o addopts= -p no:cacheprovider \
  --confcutdir=opendbc_repo/opendbc/safety/tests \
  opendbc_repo/opendbc/safety/tests/test_tesla_turn_signals.py \
  opendbc_repo/opendbc/safety/tests/test_tesla.py
```

新增 10 项安全测试全部通过，包括默认拒绝、所有 flag 组合、wrong bus/长度/修改位/计数/校验、模板过期和消费、换向、时间/数量上限、取消重试、extended/returned/rejected 模板、Summon 和继电器故障。

全量结果：**107 passed，3 failed，38 skipped，105 subtests passed**。失败项均为既有 `test_angle_cmd_when_disabled`，分别位于 `TestTeslaStockSafety`、`TestTeslaLongitudinalSafety`、`TestTeslaAutoSpeedLimitSafety`。

为区分回归，另建隔离 C 库，将 Tesla header 替换为 `git show HEAD:opendbc_repo/opendbc/safety/safety/safety_tesla.h` 的原文，其余生产规则及同一主机夹具不变，重新编译并运行这三项：**同样 3 failed**。因此未修改本任务无关的转向安全行为。使用 `-k 'not angle_cmd_when_disabled'` 得到 **107 passed，37 skipped，4 deselected**；第四个 deselected 是基类收集项，不是额外失败。

pytest 的三条 unknown-config warning 来自禁用非必要插件后仍存在的 asyncio/cpp 配置选项，不计为逻辑通过或失败。

### 原生构建、参数与静态检查

```bash
scons -Q -j2 openpilot/common/params_pyx.so openpilot/cereal/libcereal.a \
  panda/board/obj/panda.bin.signed \
  panda/board/obj/panda_h7.bin.signed \
  panda/board/obj/panda_tesla_wake.bin.signed \
  panda/board/obj/panda_h7_tesla_wake.bin.signed
git diff --check
```

四种固件均按现有 `-Wall -Wextra -Werror` 构建成功。`pandad.get_firmware_path()` 的原有选择逻辑不变：TeslaWakeOnCAN 决定普通或 wake 变体，MCU 决定 F4/H7。新增安全规则包含在四种构建中，不能只部署 Python 而继续使用不匹配固件。

另运行 `openpilot/selfdrive/pandad/tests/test_tesla_wake_firmware.py`：4 passed。使用仅提供 MCU 信息的假对象、隔离 FakeParams 和四个实际本地固件文件验证路径及签名读取，四种组合均通过，没有实例化 Panda 或访问车辆。临时脚本重复序列化同一 CarParams builder 时产生一条 pycapnp write-flag 提醒，不影响断言结果。

新 Params 已用隔离临时目录验证：key 注册、默认 False、True/False 写入及读回均成功；未修改本机实际驾驶参数。

新增 Python 和聚焦 Tesla/Desire 文件 Ruff 通过。对 card.py/modeld.py 另与 HEAD 原文逐项比较诊断：分别存在 22/3 条旧诊断，本次无新增；不宣称整个旧文件 lint 全绿。未运行全仓测试或设备 GPU 模型构建。

本地最终固件 SHA-256：

| 文件（`panda/board/obj/`） | 字节数 | SHA-256 |
|---|---:|---|
| `panda.bin.signed` | 58984 | `64292266a38fa3294251da0158b5d89faefc03a23b942fa6fff99a51a1955803` |
| `panda_h7.bin.signed` | 88372 | `b5affaec91d40f29ba5afc217ecfea4bf4c3a68be503fd769d9c09b7916db100` |
| `panda_tesla_wake.bin.signed` | 58996 | `003f2f82cf37bd265db3ab9b877cd21dc73d4ad33f199d9d9c864acb714a5e47` |
| `panda_h7_tesla_wake.bin.signed` | 88384 | `0089f4aff3a891523b171be15ab7bd0d49863ed1515b97590b9dd2742e06e326` |

## 6. 设置、文档和诊断

开关路径：Driving / Start & Auto / Engage 中的 Tesla Automatic Turn Signals，对应 `TeslaAutoTurnSignals`，默认 `0`。开关不立即改变已经运行的 CarParams，修改后需重启。

诊断字段：

- `carState.autoTurnSignalDirection`：自动会话/取消尾巴仍拥有的方向。
- `carState.autoTurnSignalStatus`：idle、active、waiting、cancelling、cancelled、driver_takeover 或 fault 原因。
- `modelV2.meta.autoTurnSignalDirection/autoTurnSignalRequestId`：上游请求。
- `tesla_auto_turn_signal` cloudlog 事件：状态变化记录。

KO/EN 用户指南及三语设置说明已同步，参数总数更新为 190。Wiki 仅在会话目录生成：先生成三语页面，应用本仓库三语 MANUAL 补丁，再以该目录作为 `--wiki-dir` 重新生成；对 Markdown 页面验证为 **595 file(s) passed**。`Settings-Catalog.json` 是索引而不是单条内容文档，未错误地交给单条内容 schema 验证。Wiki review 标记仍未认证，未推送远端。

原始本机测试输出保存在本会话 `files/` 下的 `tesla-host-result.log`、`tesla-safety-result.log`、`tesla-safety-full-result.log`、`tesla-baseline-result.log`、`tesla-build-result.log`。这是当前工作树的验证记录，不是已发布版本的证明。

## 7. 实车验收前置与剩余风险

默认保持关闭。在目标车的封闭测试场地、具备随时人工接管条件时，至少确认：

1. 实际 bus 布局、idle 模板、拨杆枚举、UI 与物理灯编码和刷新率与支持条件一致；不一致则停止启用，而非放宽门禁。
2. 导航左/右转、左/右变道、准备阶段、finishing、完成、取消的真实灯光和回读一致；左侧原有人工确认规则未被自动灯绕过。
3. 同侧/反侧拨杆、快速按压、双闪、连续路口及下一次变道能按驾驶员意图接管，没有反复重亮或尾巴触发。
4. 未就绪、停车挡、退出横向、CAN/模型/控制消息中断、无回显、灯故障时不发新的自动开启请求；取消失败显示诊断并停止重试。
5. 实际安装的普通/wake、F4/H7 固件与该代码一致；操作前按既有维护流程处理，不把本机开发构建当作已刷写。

尚未验证具体车辆响应、固件与 Tesla 版本兼容、无拨杆年款、真实手机 App 端到端路线、离线日志的不同单调时钟及断电行为。拨杆新鲜度采用运行设备单调时钟，离线回放若不对齐 CAN 时间会保守禁用，不能把本机夹具测试称作实车回放通过。

**必须由实车验收补齐“目标车可正常使用”的证据；本报告没有保证断线/断电时车辆一定熄灯，也没有保证安全回归的原有失败已经解决。**

## 8. 实车不亮灯反馈后的排查

用户反馈：开关开启，7706 的 `carrotCmd=LANECHANGE` 左右变道与导航左右转弯均能执行，但转向灯不亮，Comma 3 和车辆无报错。本节针对已发布源码 `73fb3a6b533bde2ebb198c1f83973f3795fd782a` 做只读分析；没有取得该次实车 rlog、运行时 CarParams 或 Panda 状态，不能据此认定目标车根因已经确认或修复。

### 本机已复现的阻断点与实现缺陷

所有执行均先进入 `carrot`，使用 `op venv`。复现使用真实 DBC、当前参数生成逻辑和已有 DesireHelper/执行器夹具，没有连接车辆或发送真实 CAN。

| 检查项 | 实际复现结果 | 结论边界 |
|---|---|---|
| 参数开启，但 fingerprint 的 bus1 缺少 `0x3DF` | `CP.flags=4`，自动灯功能位关闭，`safetyParam=0`；相同夹具加入该消息后为 `flags=44`、`safetyParam=8` | 开关开启不代表运行时准入成功。这些数值来自合成 fingerprint，不是用户车辆读数 |
| 实体拨杆消息来源 | 同一 DBC 的 idle `0x249` 在 bus0 时 `turnSignalDriverValid=True`；只在 bus1 时为 False，party 时间戳为 0 | 当前解析固定读取 bus0。两个 DBC 都有该消息，不证明用户车辆实际上在哪条总线上发送 |
| 拨杆无效时的灯请求 | 即使模拟有效的导航方向、控制和灯反馈，执行器发送 0 帧，状态仍为 `idle` | 有静默阻断缺陷；无界面报错不能证明灯控链路正常 |
| 真实 App 短命令的完整起始阶段 | LEFT/RIGHT 均在第 0–3 帧有对应灯方向；第 4 帧起方向为 0，但原状态机仍为 `laneChangeStarting` | App 请求约 0.2 秒到期会错误地结束灯意图。不同于原状态机仍继续执行的变道生命周期 |
| CAN 时间域偏移 | 将反馈时间设置为执行器时间之后 30 秒时，发送 0 帧、状态为 `waiting_for_vehicle_feedback` | 新鲜度检查依赖时间域一致。此为合成风险检查，不代表用户设备存在该偏移 |

主要代码位置：

- `opendbc_repo/opendbc/car/tesla/interface.py`：`has_vehicle_bus` 与 CP/safety 功能位。
- `opendbc_repo/opendbc/car/tesla/carstate.py`：仅从 `cp_party` 读取 `SCCM_leftStalk`，以及拨杆有效性和计数。
- `openpilot/selfdrive/controls/lib/desire_helper.py`：App 的 0.2 秒计数，以及 `_update_auto_signal` 对当前 `atc_enabled` 的持续依赖。
- `openpilot/selfdrive/car/tesla_turn_signals.py`：`update` 准入；尚未创建会话时 `_cancel` 直接返回，未记录具体阻断原因。
- `openpilot/selfdrive/car/card.py`：灯状态进入 carState/cloudlog，但没有相应的用户界面告警。

重新运行现有意图、Tesla 参数/拨杆、执行器和 card 四个测试模块，结果为 **76 passed，3 条既有 pytest 配置警告**。这些原测试仍能通过，同时上述缺陷能够复现，说明之前覆盖不足：意图测试以持续导航意图为主，没有覆盖真实 App 短命令的完整生命周期；拨杆测试只注入 bus0，执行器夹具默认拨杆有效。不能用这些通过结果宣称目标车可正常亮灯。

### 应先用现有实车日志区分的公共阻断点

变道/转弯由 carrot 原有链路驱动，自动灯另走独立准入和 CAN 链路，所以“车辆完成了动作”不能证明自动灯已发出请求。两个场景都完全不亮，优先检查公共链路；App 的短命令问题不能单独解释导航也不亮。

1. 核对实际安装版本、开关修改后是否重启，以及本次运行的 `CarParams.flags & 32`、`safetyConfigs[].safetyParam & 8`。再比对实际 `pandaStates` 的安全模式和参数；本机构建成功不等于车端已刷入匹配固件。
2. 核对原始接收 CAN 中 bus1 的 `0x3DF`、`0x3E9`、`0x3F5`，以及 bus0 的 `0x311`。原有转向控制可用不代表 vehicle CAN 已接入或映射正确；缺少必需总线时不能靠删门禁解决。
3. 同时核对 `0x249` 的实际来源、刷新率和枚举，以及 `carState.turnSignalDriverValid/Direction`。没有实体拨杆的年款不能假定该信号存在，需要明确对应的驾驶员输入协议。
4. 对齐 `modelV2.meta.autoTurnSignalDirection/autoTurnSignalRequestId`、`carControl.latActive`、`carState.canValid/gearShifter` 和 `autoTurnSignalStatus`；检查 `sendcan` 是否真正含有 bus1 `0x3E9`。
5. 已有发送时，区分同报文的 bus1 TX echo（src `0x81`）、拒绝回报（`0xC1`），以及真实 UI/物理灯变化。TX echo 只证明报文发出，不证明车辆接受或灯已亮。

取证优先使用本次已有日志；不要为了补日志再次在公共道路试验，不要绕过安全门禁或用原始 CAN 命令试点灯。

### 最小修复方案与当前状态

1. 将 App 灯请求绑定到原状态机已接受的变道会话，锁存来源、方向和请求 ID，按既有状态机的完成/取消条件结束，而不是随 0.2 秒命令消失。保留原有盲区、驾驶员确认、横向退出等条件，不能用自动灯反过来批准变道。
2. 依据实际车辆拓扑修正真实拨杆解析。若证实信号在 vehicle bus，则显式注册和解析该来源，保留校验、新鲜度、按压计数及驾驶员优先；不能直接令 `turnSignalDriverValid=True` 或拿自动灯回读代替驾驶员输入。
3. 将未启用、拨杆缺失/过期、模板缺失、UI/前灯反馈缺失、控制/模型过期、Panda 拒绝等原因分别诊断。避免在没有会话时一律留在 `idle`，也不能把“开关已开”显示成“功能已就绪”。
4. 增加左右 App 短命令贯穿 starting/finishing/完成/取消的回归，以及实际来源拨杆、消息缺失/陈旧、固件不匹配和逐层阻断的回归；复核时间域。之后才进行匹配固件的台架和封闭场地验收。

**本轮仅更新排查报告，没有修改灯控实现、刷写设备或推送分支。软件缺陷已复现，但该车的公共阻断点仍待实际日志确认，不能宣称已修复。**

## 9. CAN1 拓展线链路修复（2026-10-24）

### 依据与检测原则

用户补充车辆已连接 CAN1 拓展线，现有自动模拟巡航滚轮功能可以使用。代码核对确认，两项功能**原本就共用** `interface.py` 中 `has_vehicle_bus = 0x3DF in fingerprint[CANBUS.vehicle]` 的检测，生成同一个 `TeslaFlags.HAS_VEHICLE_BUS`。不是自动灯遗漏了拓展线判断，也不能仅凭旧的 bus0 拨杆解析就推断没有接线。

自动滚轮发的是 CAN1 `0x3C2`，自动灯发的是 CAN1 `0x3E9`；两者只共享总线可用性判断，不共享报文内容或执行权限。保持现有识别标准，不将“任何 bus 上有消息”视为已接入拓展线。灯控还需独立的真实 idle 模板、UI/前灯反馈和 feature 权限；滚轮工作并不等于这些灯控条件全部满足。

### 已实施的最小修复

| 修改 | 结果与保护 |
|---|---|
| CAN1 真实拨杆来源 | 仅在灯功能启用时，为 vehicle parser 预注册可选 `SCCM_leftStalk`，不改变原有 UI/TPMS 必需消息。CAN1 一旦收到该信号即优先使用，未曾收到时兼容原 CAN0 来源；CAN1 过期后不偷偷切回另一总线的 idle 值 |
| 来源冲突与按压计数 | 新鲜来源方向冲突、无效枚举或另一来源出现无法对应的快速按压时，拒绝自动输出；半按到全按不重复计数，镜像输入及短暂无效恢复也不将同一次按压当作新请求。不伪造有效拨杆，不用灯反馈替代驾驶员输入 |
| App 变道生命周期 | 灯方向锁存到原状态机已经接受的 pre/starting/finishing 会话，而不是 0.2 秒命令脉冲。完成、取消、驾驶员操作或原有准入失效时释放；同一已取消会话不恢复重亮，下一条真正接受的 App 命令可获得新 ID |
| 导航准备到转弯 | 左/右 `turn … prepare` 切到同方向 `turn …` 时，原 ATC 会抑制动作切换一帧，旧灯意图因此错误归零。现在仅桥接已存在的同方向灯请求，保持 ID；原有转弯动作的一帧抑制仍然保留，不增加转向权限 |
| CAN 时间域 | C++ CAN 事件使用 `CLOCK_BOOTTIME`；仅灯控接收/发送时钟和真实拨杆新鲜度改用相同时钟（不支持的平台使用 `CLOCK_MONOTONIC`）。模型/控制的接收新鲜度仍使用 SubMaster 原有 monotonic 时间，不修改巡航滚轮或普通控制器时钟。REPLAY 的灯执行器仍用日志时间，未宣称完整车辆离线回放已通过 |
| 明确阻断原因 | 配置时区分开关关闭、车型不支持、passive、未识别拓展线、需要重启、CP 安全配置缺失。运行时区分拨杆不可用、控制/CAN/D 挡不满足、模板与 UI/前灯反馈缺失等。首次 Tesla 状态也记录 cloudlog，不再因没有会话而始终留在 `idle` |

CP 灯功能位仍为 32，Panda 独立权限仍为 8。自动灯不依赖 alpha longitudinal 或自动滚轮是否开启，避免把灯控错误绑定到纵向功能。现有滚轮发送逻辑、Panda 生产规则、报文格式、模板消费、取消与熄灯确认均未放宽；此次没有新增 schema、参数或固件修改。

### 当前诊断含义

状态从 `carState.autoTurnSignalStatus` 和 `tesla_auto_turn_signal` 事件读取，**不是新增屏幕告警**。

| 状态 | 应检查的环节 |
|---|---|
| `blocked:vehicle_bus_unavailable` | 本次启动是否在 CAN1 检测到 `0x3DF`，而非只看设置开关 |
| `blocked:restart_required` | 设置已开，但本次 CarParams 没有灯功能位，需按原流程重启重新识别 |
| `blocked:safety_configuration` | CP 请求的 Tesla 安全配置中缺少灯控权限；不代表已经验证设备固件 |
| `blocked:driver_input_unavailable` | `0x249` 来源、刷新率、枚举、双来源冲突；无拨杆或不同协议布局仍未自动支持 |
| `blocked:context_stale` / `blocked:lateral_inactive` / `blocked:can_invalid` / `blocked:not_in_drive` | 原有控制和新鲜度准入 |
| `waiting:ui_feedback` / `waiting:front_feedback` | 缺少新鲜 bus0 `0x311` / bus1 `0x3F5` |
| `waiting:body_template` | 缺少新鲜、正确校验的 bus1 idle `0x3E9` |
| `waiting:lamps_off` / `waiting:invalid_lamp_feedback` | 当前不具备双侧关闭且有效的起始灯状态 |
| `cancelling:panda_rejected` / `fault:panda_rejected_cancel` | 收到拒绝回报；需核对实际安全模式、固件及原始收发报文，不能视为车辆已执行 |

### 验证结果

均先进入 `carrot` 并执行 `op venv`。主机回归：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q \
  -o addopts= -p no:cacheprovider --confcutdir=/home/jeff/carrot \
  openpilot/selfdrive/controls/tests/test_tesla_auto_signal_intent.py \
  openpilot/selfdrive/controls/tests/test_desire_helper.py \
  openpilot/selfdrive/controls/tests/test_desire_helper_stalk.py \
  openpilot/selfdrive/controls/tests/test_lever_forced_turn.py \
  openpilot/selfdrive/controls/tests/test_lane_change_side_obstacles.py \
  openpilot/selfdrive/controls/tests/test_lateral_readiness.py \
  openpilot/selfdrive/controls/tests/test_lateral_startup_control.py \
  opendbc_repo/opendbc/car/tesla/tests \
  openpilot/selfdrive/car/tests/test_tesla_turn_signals.py \
  openpilot/selfdrive/car/tests/test_tesla_signal_card.py
```

结果：**351 passed，23 subtests passed**。覆盖左右 App 短命令贯穿整个状态机、350ms 后才有灯模板/反馈仍可请求、完成与驾驶员/控制/CAN/拖车取消后不重亮、下一条命令、导航准备到左右转弯、CAN1/兼容 CAN0 拨杆、同批次按压释放、来源冲突与镜像计数、无拓展线/错误 bus 的标记、拨杆缺失、配置/反馈阻断诊断，以及现有 Tesla 自动滚轮、转向与启动门禁。模拟暂停导致的两个时钟相差 5 秒时，card 仍使用 CAN 对应时钟；未把时间域差当作接收陈旧。

独立运行 `opendbc_repo/opendbc/safety/tests/test_tesla_turn_signals.py`：**10 passed**，沿用现有 C 测试库，包含 feature 位组合、真实模板、可修改位、总线/长度/校验、反向取消和有界重试。没有重新运行全量安全套件或重建固件；第 5 节原有 3 项转向安全失败没有在此次处理，不能将这 10 项通过说成全量安全全绿。

新增/修改的聚焦 Python Ruff 通过；`card.py` 对照当前 HEAD 的 22 条旧诊断无新增。pytest 仍有 3 条已有插件配置警告。EN/KO 用户指南及三语 Wiki 补丁同步；本地重新生成、应用补丁、再次生成后 **595 个 Markdown 文件验证通过**，用户文档映射检查通过。Wiki 未发布。

**当前交付是本地软件修复及上述本机证据，未提交、未推送、未部署或刷写。仍未取得目标车原始日志，不能证明它此前一定是 CAN1 拨杆问题，也不能保证此次修改后该车已能正常亮灯。设备仍须具备匹配的 Panda 灯控规则，并先完成台架/封闭场地验证，不应直接继续公共道路试验。**

## 10. 历史三项转向安全测试失败的核对与适配

### 同一个预期冲突，不是三个独立故障

三项均为继承自 `safety/tests/common.py` 的 `test_angle_cmd_when_disabled`：

- `TestTeslaStockSafety`：原厂纵向。
- `TestTeslaLongitudinalSafety`：openpilot 纵向。
- `TestTeslaAutoSpeedLimitSafety`：openpilot 纵向加自动巡航滚轮。

先在 `carrot` 中执行 `op venv`，运行既有 SCons C 库目标确认源码状态，再运行原三项测试，复现 **3 failed**。进一步完整枚举每种模式的 `controls_allowed`、转向请求位、19 个测量角度和 19 个请求角度，即每种模式 1444 个组合：三种模式均只有 **361 个 `controls_allowed=False` 且转向请求位为开**的组合与通用测试预期不同。请求位为 NONE 时的角度约束没有出现此类差异。

原因在本分支既有 `opendbc_repo/opendbc/safety/safety.h` 的 `steer_angle_cmd_checks`：局部 `aol_allowed` 固定为 true，允许横向独立于原厂巡航，仍执行角速率检查。通用测试却沿用“`controls_allowed=False` 时所有主动转向都应拒绝”的预期。这个测试把原厂巡航授权关闭与转向报文的 NONE 位混在了一起，与当前分支的独立横向策略不一致。主机 `controlsd.py` 也明确允许 `(selfdrive_active or always_lateral)` 后再检查横向开关、故障、车速、启动准备等条件。

**不能为了通过该通用测试就直接将 `aol_allowed` 改为 false，或在 Tesla 中无条件添加 `!controls_allowed` 禁止转向；这会改变本分支独立横向/全时横向的行为。**

### 本次修改及验证边界

此次只修改 `opendbc_repo/opendbc/safety/tests/test_tesla.py`：

1. 在 Tesla 测试基类中提供相同测试名的分支专属预期。保留完整的授权/请求位/测量角度/请求角度组合；主动请求在预置相同上一请求、满足角速率等条件时可以独立于巡航，NONE 请求仍必须贴合测量角度。逐次确认 TX 不改变巡航授权。
2. 扩展原有 inactive 检查，在巡航授权开/关两种状态下均验证测量角度限幅，以及正、负两个方向的错误关闭角度必须被拒绝。
3. 新增未开巡航时的角速率正反例，覆盖 0、1、5、10、25、50m/s，以及负角、零角、正角；每次单独设置上一请求，确保超速变化仍被拒绝，不能通过连续请求的副作用蒙混过关。
4. 新增横向/纵向隔离检查：独立横向报文被接受后，不得打开 `controls_allowed`，也不得因此允许非零纵向加速。

没有修改共享通用测试、生产安全代码、任何限值或权限位，也没有增加 skip/xfail 标记。**这是测试与既有分支策略的适配，不是新增 Panda 独立横向授权机制或固件安全加固。** 固件中的 `aol_allowed=true` 仍然存在，Panda 并没有因此开始独立读取主机的 `AlwaysLateral` 开关；主机横向门禁仍是现有架构的一部分。测试全绿不能被解释为这方面的架构风险已经消失。

### 最新验证结果

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q \
  -o addopts= -p no:cacheprovider --confcutdir=/home/jeff/carrot \
  opendbc_repo/opendbc/safety/tests/test_tesla.py \
  opendbc_repo/opendbc/safety/tests/test_tesla_turn_signals.py
```

结果：**116 passed，40 skipped，339 subtests passed，0 failed**。没有用 `-k` 排除历史失败。40 个跳过包含既有抽象基类及不适用测试；相较原 38 个增加的 2 个来自基类新增的两种测试方法被收集，不是将失败改成跳过。仍有 3 条既有 pytest 插件配置警告。

为验证适配后不会放过真正的约束破坏，另在会话目录生成并编译两份隔离 C 库，未覆盖正常库或生产源码：

| 隔离故障注入 | 测试确实拒绝的例子 |
|---|---|
| 去掉 NONE 请求的测量角度检查 | 修正后的同名测试在测量 -90°、请求 -80°、转向请求位为关时断言失败 |
| 未开巡航时跳过主动转向角速率检查 | 新增角速率测试在 0m/s、上一请求 -90°、单次变化 +6.1° 时断言失败 |

这两个预期失败证明测试并非无条件接受当前返回值；它们不是正常代码的回归失败。隔离产物及结果保存在本会话 `files/tesla-disabled-mutants/`，正常 C 库未替换。

同时运行实际控制器反馈、协同转向、主机 `AlwaysLateral`/转向门禁及启动准备测试：**103 passed**。修改文件 Ruff 与 diff 空白检查通过。生产安全规则无变化，未为此次测试适配重新构建或刷写四种固件；第 9 节灯控仍要求设备具备匹配固件。

**截至本节，原三项测试失败已通过明确的分支策略适配消除；没有据此宣称车辆问题已全部解决、固件增加了新保护，或完成实车安全认证。本次仍未提交或推送。**

## 11. 三闪反馈后的持续生命周期修订（2026-10-24）

### 已证实的软件缺陷与未知的车辆协议行为

目标车已报告 CAN1 可以触发左右灯，但只出现三闪。**没有取得该次原始 CAN 收发、TX 回报或 UI/前灯日志，不能由三闪反推“只发了一帧”，也不能确认车辆是否将重复的相同请求解释为持续保持。** 参考端同样使用新 idle 模板与 TX 回报的循环，没有足够证据新增协议位或复用已消费模板。

对基线源码使用 `git show HEAD:<path>` 加载到内存，再运行本轮新增回归，以下六项均按预期触发断言失败；没有覆盖工作树或使用实车：

| 原版确定性复现 | 修订 |
|---|---|
| 首帧被接受后没有新 BODY，其他输入仍新鲜，状态一直是 active | 最后一次提交后 1.5 秒仍不能重复且已无待确认 echo，进入 `cancelling:body_template_starved`；不重复旧模板 |
| 曾确认亮灯，之后 UI/前灯持续全 OFF，原版仍不断发 active | 真 OFF 持续 0.5 秒后进入 `cancelling:lamps_lost`；亮相确认不再是永久有效的历史锁存 |
| 同向人工操作立即产生 CANCEL | 立即停止自动输出、撤下自动所有权，等待真实关闭后才清理会话 |
| 未见拨杆事件的意外对侧灯出现，原版仍维持自动方向 | 未知对侧来源也保守让行，不能自动取消它或反向覆盖 |
| owner=0、拨杆有效且中立，仅物理灯为 ON 就被当成驾驶员确认 | 自动功能开启时必须有真实方向/物理事件计数来源；未知来源不能触发驾驶员确认 |
| 100° 转角、当前模型 yaw rate 0.4、未来 0.2，使 turn-disable=200，灯意图同时归零 | 原转向判断仍产生相同的 200 tick 禁用；仅灯光改为独立完成判断，ID 不变 |

这些回归证明软件缺陷的修订，不证明目标车辆三闪的唯一根因已找到。

### 输出存活与故障处理

- 原有 BODY `0x3E9` bus1、idle 模板校验/新鲜度/逐次消费、模板计数器加一、checksum、精确 TX echo、拒绝回报、60 秒/1200 帧 Panda 上限均保持不变。
- UI=1 是闪烁暗相，不是关闭；短暂前灯暗相不触发 `lamps_lost`。即使 UI 一直报告闪烁，2.5 秒没有再次确认 UI 闪烁且物理灯为 ON，也进入 `cancelling:lamp_feedback_lost`。首次未亮、反馈陈旧、丢失/拒绝 echo 和控制失联仍有原有边界。
- 不进行未经协议验证的“自动重试新会话来再闪三次”。故障取消仍最多三次/五秒，未确认时锁存故障、禁止本次运行继续自动输出；收到人工操作也不能把已有 `fault:*` 隐藏成接管成功。

### 人工优先与 Panda 会话同步

`card` 现在先观察本次解码后的真实拨杆，再处理 CAN 回报和独立取消出口，避免在同一轮已经收到人工输入后，紧急出口仍先发 CANCEL。

同向/反向按压、快速按放、双闪和意外对侧灯使状态变为 `driver_takeover:waiting_off`。这个阶段没有 active 或 CANCEL 输出，也不丢弃尚存在的 Panda 旧方向会话。人工灯持续多久就等待多久，不能套用自动会话超时去熄灭人的灯，也不会把等待标为取消成功。没有新鲜中立拨杆或双侧 UI/物理灯真关闭时，不允许清理。

双侧关闭稳定至少 0.5 秒且拨杆新鲜中立后，进入 `cancelling:driver_cleanup`，使用新的 idle 模板发有界 CANCEL；只有其回报和真实关闭确认完成后，才释放主机记录。这样下一条反向自动请求不会被固件遗留方向永久阻挡。清理阶段仍有三次/五秒限制；缺失模板/回报导致清理失败时保持故障，不假装已经同步。已经发出的报文与恰好之后的人工动作无法原子撤回，物理 CANCEL 的具体效果仍须车辆日志/封闭场地确认。

驾驶员来源在 DesireHelper 中独立保存：快速按放即使早于灯反馈到达，最多保留一秒等待其灯；已确认的同向人工灯维持来源，稳定 OFF 后清除。等待期间没有灯时不伪造持续的驾驶员灯状态。自动灯尾部或未知来源灯不能重新触发变道。功能关闭时仍保留原灯状态处理。

### 灯光专属转弯完成

保留原 `turn_disable_count`、变道 FSM、BSD/扭矩门禁、完成阶段、AlwaysLateral 与启动逻辑。灯光不再直接消费 turn-disable：

1. 持续至少 0.3 秒同时满足移动（>1m/s）、目标方向的转角 >20° 与同方向模型 yaw rate >0.1rad/s，作为开始转弯的合成证据。反方向转角或 yaw rate 不计入开始证据。
2. 开始后，移动中转角回到 `min(20°, 本轮峰值的25%)` 以内，模型前 16 个 yaw rate 的绝对值均 <0.05rad/s，模型 heading 的第 5–15 点差 <0.06rad，连续稳定一秒才消费该灯请求。
3. 数据缺失/非有限值不完成，停车/极低速不完成，仅横向加速度小不完成。已开始的同向转弯在导航先撤下类型时仍保持原 ID，直到稳定完成或原有控制/安全边界结束；覆盖空字符串及生产者实际输出的 `none`、`none prepare`、`straight`、`straight prepare`。中止后不能用同一意图立即重亮。
4. 下一条反向转弯是新请求，不借用上一条的开始/峰值/稳定证据；原有类型切换一帧抑制保持不变，之后可以建立新 ID，不会被旧请求的 consumed 标志永久阻挡。执行器仍先取消旧方向并确认关闭，再允许新方向。

这些是保守、待实车校准的灯光判据，不是车道/路口识别真值，也不是转向控制新权限。未能取得完成证据时仍由执行器的会话与故障边界收束，不能无限保持自动输出。

### 最终本机验证

均使用用户要求的 `bash -ic 'cd /home/jeff/carrot && op venv'`，随后执行 `set -e`、项目目录与 `.venv/bin/python` 对应的 `python -m pytest`；保留 `PYTHONDONTWRITEBYTECODE=1`、`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`、`-o addopts= -p no:cacheprovider --confcutdir=/home/jeff/carrot`。最终主机选择器与第 9 节相同，安全选择器与第 10 节相同。

| 检查 | 结果 |
|---|---|
| 主机意图、执行器、card 边界、原 FSM、Tesla 解析/控制器、横向/启动回归 | **402 passed，23 subtests passed** |
| 完整 Tesla C 安全套件，包括原有转向测试 | **116 passed，40 skipped，339 subtests passed** |
| Wiki 生成器/只读检查/验证器测试 | **25 passed** |
| 本地三语补丁应用后再生成并验证 Markdown | **595 文件通过** |
| EN/KO 用户文档映射与聚焦代码 Ruff | 通过 |
| card、两份旧 Wiki 测试 Ruff 基线比较 | 各保留原有 22、21、1 条诊断，无新增 |

总计 **543 passed，40 skipped，另有 362 subtests passed**，不是物理认证。pytest 仍有三条既有插件配置警告。Wiki 测试初跑发现两处旧的 197 项断言未计入已有 Tesla 设置；同步为 198 项并显式断言包含 `TeslaAutoTurnSignals` 后，25 项全部通过。未更改生成器生产逻辑或现有设置。

复核时还确定性重现了两项本轮代码边界：仅空字符串被当作导航退出，真实 `none` 等类型会提前丢失灯意图；已开始转弯后直接切换为反向导航，旧 consumed 标志会使新方向始终无请求。补齐真实类型、反向新会话与有符号开始证据后，新增 20 项回归，以上三个选择器组全部重新通过。

另一次未指定选择器的全仓 pytest 尝试在收集阶段出现 19 项导入错误，包括缺少 `tools.signal_analysis`、旧接口缺失以及 `HARDWARE` 导入冲突；该全仓套件未执行，不在上述通过范围内，也没有为本次 Tesla 修改去更改这些无关模块或安装替代依赖。

新增覆盖包括左右各十秒/101 次新模板请求、计数器回绕、精确回报，以及实际 C safety 逐次接受、拒绝复用模板；结束时接受取消且不重亮。左右 App 命令保持六秒后走完原 finishing。人工同向/反向保持 65 秒不发送取消，再以新鲜中立/真实关闭清理，实际 C safety 接受清理及下一条反向请求。另覆盖模板饥饿、初亮后灭灯、暗相、无后续亮相、延迟/丢失/拒绝回报、未知来源、快速按放后延迟灯反馈、双闪、人工结束、真实 turn-disable 触发、低速/停车/弯中不完成、缺失/NaN/非直路模型、不具备开始证据、完成不重亮及中止。

**没有修改 Panda 生产源码，因此本轮没有重建或刷写固件；设备仍必须具备此前已定义的匹配灯控规则。未部署车辆、未发送任何真实 CAN、未改变 xpilot、原有六份修改或 origin。Wiki 仅保存并验证源补丁，没有向外部 Wiki 发布。**

### 尚需目标车辆的封闭场地验收

不能将十秒合成请求或 C safety 接受解释为物理灯已经连续闪烁十秒。取得私有原始记录，逐项对齐 bus1 idle `0x3E9`、实际 TX、`0x81`/`0xC1`、bus0 `0x311`、bus1 `0x3F5`、真实拨杆计数、请求 ID 与状态，才能定位三闪属于模板停止、回报失败、车辆忽略重复请求还是其他机制。

必须分别验证左右长变道/慢速转弯超过三闪、弯中停车/继续、动作结束可靠熄灯、导航提前结束、同向/反向人工与快速按放、双闪、人工自行结束后反向新请求、失联/反馈/取消失败。若 OEM 不把当前重复协议视为持续请求，仍需先解明原始协议，不能复用旧模板、模拟真实拨杆或放松安全限制来“修好”。本轮交付是这些已验证的保守生命周期修复和更明确的诊断，不承诺未经日志与场地验证的物理持续功能。

## 12. 转弯后进程通信异常：真实模型消息类型修复（2026-10-24）

### 反馈与确定性复现

用户反馈转弯时先点灯一次，随即出现未看清的报错，随后尝试巡航时提示进程通信异常，无法接入。本轮未取得该次车辆 traceback、managerState 或原始行车日志，因此不能将具体车辆报错最终归因，也不能声称已排除所有其他故障。

但已在发布代码中复现一个与时序高度吻合、必须修复的确定性缺陷：

```text
DesireHelper.update
  -> _update_auto_signal
  -> _update_auto_turn_geometry
  -> rates[:16]
TypeError: an integer is required
```

`modeld.py` 向 `DesireHelper.update` 传入真实 `modelV2` builder，随后才发布模型消息。真实 `orientationRate.z`、`orientation.z` 是 Cap'n Proto 动态列表，不是普通 Python `list`；builder 和 reader 都不支持切片。第 11 节新增的有限值检查和角速度前缀检查却使用了切片。

灯方向可先在一个循环末尾建立，后续循环才开始几何检查，因此先出现灯请求、随后触发异常是可能的。异常沿该调用链传播会影响模型进程与消息发布；`selfdrived` 对进程退出和消息缺失保留 `processNotRunning`、`commIssue` 等阻止接入事件。这里不能以屏蔽通信告警或继续允许巡航代替修复。

### 最小修复与此前测试缺口

仅将这两路几何数组通过迭代转换为普通 Python 列表，再保留原有的长度、前 16 点有限值、方向、停车和稳定回正判断。没有添加宽泛异常捕获或伪造正常模型消息，也没有改变转向状态机、完成阈值、起步权限、CAN 编码或 Panda 安全策略。

此前几何用例以 `SimpleNamespace` 内的普通 Python 列表构造模型，所以切片合法；使用真实 meta 的灯控测试，也没有覆盖真实模型几何进入此函数的组合。这是测试替身与生产运行时类型不一致造成的漏检，不是实车 CAN 线束问题的证据。

新回归包含左右转弯各自的真实 builder/reader 生命周期，真实消息中的短数组、NaN 和非有限车辆输入，以及真实模型序列化/解析后将灯意图交给执行器的左右连续请求和结束取消。执行器反馈为合成 CAN，不代表车端接受或物理闪灯。

### 本机验证结果与边界

均先进入 carrot 执行 `op venv`，使用前文相同的 pytest 隔离选项。

| 检查 | 结果 |
|---|---|
| 修复前新增的真实模型左右 builder/reader 生命周期 | **4 failed**，均复现上述 TypeError |
| 修复后意图、原 DesireHelper/FSM、灯执行器、card 边界五个文件 | **180 passed**，其中包含本轮新增的 18 个参数化用例 |
| 两个修改 Python 文件的 Ruff | 通过 |
| 修改文件的 diff 空白检查 | 通过 |

pytest 仍有三条既有插件配置警告。本轮没有执行全仓测试、完整 C safety 套件、GPU 模型推理或真实进程间通信试验；生产调用点与故障传播链通过代码核对，模型数组和消息边界通过真实 cereal 类型验证。没有刷写设备、发送真实 CAN 或完成车辆复测。

此修复消除的是已复现的运行时类型异常，不证明连续闪灯协议已被目标车辆接受。实车需核对首次异常 traceback、`modeld` 退出记录和通信异常涉及的服务。未应用修复前，应安全停车后关闭 `TeslaAutoTurnSignals` 并重启，避免再次触发；修复的正常交付仍通过 Git Pull 和重启，不通过关闭安全检查继续驾驶。

## 13. 手动左右不对称、反向变道与三闪的补充协议分析（2026-10-24）

### 参考项目与当前拨杆布局不同

用户补充：App 左右请求都能执行，手动左灯不能向左变道，手动右灯有时能向右、有时却触发向左。出现反向请求时应停止道路测试，不能继续依赖自动变道。

当前 carrot 的 party/vehicle DBC 均将 `0x249 SCCM_leftStalk` 声明为三字节，`SCCM_turnIndicatorStalkStatus` 为三位：`1/2` 是 UP，`3/4` 是 DOWN，`5` 不可用。`get_tesla_stalk_direction` 因而把 `3/4` 映射成左。

参考 xpilot 的 `tesla_model3_vehicle.dbc` 则声明四字节、四位状态：`1–4` 为不同深度的 UP，`5–8` 为不同深度的 DOWN，`9` 不可用。其 `teslacan.py` 也明确以四字节构造 `2=RIGHT`、`6=LEFT` 的验证帧。这不是相同协议的简单左右枚举重命名。

本机在 `op venv` 中构造符合参考布局及其 CRC 算法的四字节帧，送入当前真实 CANParser、CarState 和 DesireHelper；仅隔离完整车辆 CAN 就绪及侧向道路几何，以便单独核对方向传播。没有向车辆发送报文。

| 参考四字节输入 | 当前三位解析 | 当前驾驶员方向 | 当前 DesireHelper 结果 |
|---|---|---|---|
| 右轻拨，原始值 `2` | `2` | 右 | 向右变道 |
| 右重拨，原始值 `4` | `4` | 左 | **向左变道** |
| 左轻拨，原始值 `6` | `6`，超出当前有效范围 | 无效 | 不发变道意图 |
| 左重拨，原始值 `8` | 截去第四位成为 `0` | 中立 | 不发变道意图 |

当前 parser 并未因为实际 DLC=4 与 DBC 声明=3 不同就阻止上述错误解析。这个组合与用户描述高度吻合，但尚未读取该车的原始 `0x249`，所以车辆实际采用的布局仍需确认。App 命令直接提供 LEFT/RIGHT，绕过了物理拨杆解码，能左右正常执行并不否定这个缺陷。

### 不可用输入还会伪装成人工接管

另一个只读复现：参考四位状态 `9=SNA` 经当前三位解析后变成 `1`，被当成有效右拨杆。将该 CarState 交给已经活动的自动灯执行器后，状态转为 `driver_takeover:waiting_off`，后续输出为零。

这证明协议误读不仅会产生反向意图，也可能提前终止自动灯会话。它是三闪现象的一个待核对的软件路径，不能在没有实车报文时认定目标车辆已经发生了这个状态。

### 正确修复应保留哪些边界

不能全局交换左右、把所有 DBC 直接换成新布局，或用仪表亮灯替代真实驾驶员输入。旧三字节协议中的 `3/4` 本来就是左，与新四字节协议存在直接冲突。

后续适配应从原始接收帧识别 DLC，按已确认的三字节/四字节布局分别解码；保留每个来源的布局、接收时间、有效性和批内按下/释放顺序，再统一为中立/左/右的驾驶员语义。双总线比对必须比较解码后的方向，不能直接比较不同布局的原始数值。未知长度、SNA、校验失败或无法消歧的来源必须阻止输出，不得默认左侧；也不得把新协议的左重拨截成中立后丢失人工事件。

回归范围应覆盖两个总线、两种布局、所有深度与不可用值、轻拨到重拨只计一次、同批按放、双向连续输入、来源冲突、陈旧输入，以及真实消息经 CarState 到变道意图和灯光所有权的完整链路。参考项目的 SCCM 发送函数只用于理解协议，不应移植成模拟驾驶员拨杆的发送路径。

### App 一次命令不等于只发送一次灯控 CAN

当前 App 变道意图已经独立锁存到既有变道生命周期。App 请求一次是正常设计，**不能通过重复发送 `LANECHANGE` 来续灯**，否则可能变成新的变道请求。

只读对比 xpilot 的 BODY 编码，在计数器 `0/4/15`、左/右/取消共九个组合中，当前 `body_frame` 与参考 `create_body_control_frame` 的结果逐字节相同：相同方向枚举、活动 reason=8、取消 reason=4，以及原字段保留、计数器与校验处理。参考导航适配器使用 `hold_until_cancel=True`，实时执行器同样要求新原厂模板与 TX 回显，不能从这个参数推导出额外的物理“持续闪灯位”。

因此，三闪还需按实际日志区分：拨杆误读触发人工接管、模型进程/控制上下文中断、新 BODY 模板停止、回显/灯反馈失败，还是车辆不接受当前重复请求。应将同一请求 ID 下的实际发送次数、模板时间、精确回显、拨杆原始帧和 UI/前灯反馈对齐；不能只看 App 命令次数或软件 `active` 状态。

本节记录分析阶段的结论：当时只完成可复现的协议分析和修复边界设计，未改变拨杆 DBC、CarState、灯控 CAN 或 Panda 策略；当时的本地代码修复仍是第 12 节的模型数组类型修复。后续实施与验证见第 14 节，不能把协议分析本身当成实车修复验收。

## 14. 新旧拨杆兼容、来源消歧及灯控诊断修复（2026-10-24）

### 本轮落实的代码

`CarInterface.update` 在真实 CAN 接收入口向 `CarState` 提供 `0x249` 原始数据、来源和时间，而不是再让固定三位 DBC 提前截掉第四位。仅接收 bus 0/1；发送成功、发送拒绝回显及其他总线不能成为人工拨杆输入。共享 DBC 保持不变，也没有发送或模拟 SCCM 拨杆。

| 实际长度 | 有效方向解码 | 无效输入 |
|---|---|---|
| 旧三字节 | `0` 中立，`1/2` 右，`3/4` 左 | `5–7` |
| 新四字节 | `0` 中立，`1–4` 右，`5–8` 左 | `9–15` 或 CRC 错误 |
| 其他长度 | 不推测布局 | 全部无效 |

四字节按参考项目的 CRC8 OpenSafety 与计数器 MAGIC 校验，复用仓库已有 CRC 表。右重拨 `4` 因而不再变成左；左轻拨 `6`、左重拨 `8` 不再被丢弃或截成中立。此兼容是对已确认参考布局的条件支持，不证明目标车辆实际报文一定采用该布局。

每个来源保留独立接收时间和批内方向序列，统一成左右语义后才进行双总线比对。保留 CAN1 一旦出现即优先、陈旧后不回退掩盖的约束；来源冲突、SNA、CRC 失败继续阻止自动输出。每条批内事件也单独检查时间，旧按压或未来时间按压不能借后续的新鲜中立帧重新变成有效请求。

左右按压计数沿用同一物理方向历史及 UInt8 回绕：半拨到全拨不重复计数，同批按下/释放不丢失，镜像来源及无效间隙不重复计算仍按住的一次拨杆。模型端增加 Tesla 专属消歧：两个计数同时变化且没有明确当前方向时，不再默认左侧；无效来源也不能靠计数变化或灯光尾迹建立人工变道请求。其他品牌保留原逻辑。

`9=SNA` 现在不会再冒充右侧人工接管。它仍是不可用输入，因此已有自动灯会话会按 `driver_input_unavailable` 安全取消，而不是假定驾驶员中立并继续发灯控。这里修复的是错误来源归属，**不是绕过人工输入有效性门禁来强行保持闪灯**。

同时交付第 12 节的真实 Cap'n Proto 数组转换修复，保留原有转弯完成、几何、BSD、扭矩、起步和安全门禁。

### 用实际发送证据区分“三闪”的原因

`tesla_auto_turn_signal` 事件保留即时状态变化日志；存在会话且状态不变时，最多每秒补一份快照，空闲不周期刷日志。内容包括请求 ID/方向、活动请求提交数、取消提交数、精确匹配的成功 TX 回显数、Panda 拒绝数、待回显标志、人工接管、灯亮确认、模板/反馈/控制上下文年龄，以及驾驶员来源有效性和左右计数。

统计仅观察现有流程，不额外发 CAN、不推进会话、不重放 App 指令，也不修改 BODY 编码、模板一次性消费、计数器或安全规则。重复接收同一回显不能增加计数。提交数不等于已上总线，成功 TX 回显也不等于车辆接受持续灯请求；物理灯亮仍须以实际 UI/前灯反馈及车辆观察确认。

若复测仍只三闪，需对齐同一请求 ID 的这些日志及原始 CAN：有无持续提交、是否精确回显、是否被 Panda 拒绝、是否因模型/输入/模板/反馈中断退出，或在提交/回显都持续时车辆仍结束闪灯。没有实车证据时，不猜新的“保持位”，也不通过反复 `LANECHANGE`、重用模板或放宽安全门禁续灯。

### 本轮本机验证

先进入 `/home/jeff/carrot` 执行 `op venv`，采用前文相同的隔离 pytest 选项，运行完整 Tesla 车辆测试目录、Tesla 原生 safety 两文件，以及意图、原 DesireHelper/FSM、灯执行器和 card 边界五文件。

| 检查 | 结果 |
|---|---|
| 上述联合回归 | **601 passed，40 skipped，362 subtests passed，无失败** |
| 新旧布局、两个总线、轻/重拨及同批释放的真实 `CarInterface → CarState → DesireHelper` 方向矩阵 | 48 个组合通过；最终变道方向与人工方向一致 |
| 新协议全状态、16 个计数器的 CRC/SNA、未知长度、镜像/冲突、批内陈旧/未来事件、非接收来源 | 通过 |
| 第 12 节真实 builder/reader、非法几何、序列化模型到灯执行器 | 通过 |
| 灯控统计、重复回显不重复计数、状态立即记录与会话周期节流 | 通过 |
| 除 card 外八个修改 Python 文件 Ruff | 通过 |
| card 与 HEAD 的 Ruff 逐诊断基线比较 | 原有 22 条仍为 22 条，新增 0 |

40 项跳过保持原有基类/平台条件，未新增跳过或删除断言以掩盖失败。仍有三条原有 pytest 插件配置警告。这不是全仓运行结果，也未运行 GPU 模型推理、真实设备 IPC、部署、刷写或车辆 CAN 发送。

**已修复并回归的是确定性运行时崩溃与已复现的拨杆误读/来源错误；尚未取得该车原始拨杆报文、异常 traceback 或持续灯光验收结果。正常更新后须重启并先在台架/封闭场地核对左右方向、人工优先、持续闪灯与动作结束取消；出现过反向变道的旧版本不应继续公共道路自动变道试验。**

## 15. 按要求原样替换 xpilot DBC，取消旧拨杆协议（2026-10-24）

本轮按用户要求直接复制 xpilot 的六份 Tesla DBC 源文件，不再进行字段级拼接或保留旧拨杆协议。参考主仓提交为 `67936d9e0555b8187a6df405f5a50de69924626a`，其 opendbc 提交为 `6d864ee2140338e0eb41b3487ed79b97a7a80f2e`。

`tesla_can.dbc`、`tesla_powertrain.dbc`、`tesla_model3_party.dbc`、`tesla_model3_vehicle.dbc`、`tesla_modely_hw4_perception.dbc` 和 `tesla_model3_party_supplement.dbc` 均与本地 xpilot 对应文件逐字节一致；前两份原本已经相同。现有生成雷达 DBC 未删除，也没有用补充 DBC 覆盖主 party DBC。

### carrot 消费端调整

- `0x249` 只接受四字节拨杆：`0` 中立，`1–4` 右，`5–8` 左，`9–15` 不可用。三字节帧明确无效，不再按旧枚举解释；CRC、来源、时效、同批按放、按压计数和人工优先保护仍保留。
- 仪表单位直接读取 `DI_uiSpeedUnits` 的 bit 33，删除根据车速比值猜单位的逻辑；数字油门 bit 34 参与踏板按下判断，但不污染原有模拟油门迟滞，释放后不会额外锁存。
- 原样 vehicle DBC 不含胎压和交通控制消息。胎压改由 bus 1 的独立 HW4 解析器接收；交通灯广播改由 bus 2 的 HW4 `APP_trafficControl` 解析，停止线距离按每位一米读取，保持外部 JSON 字段名不变。
- 胎压属于低频显示数据，独立解析器不再建立额外的 bus-off 门禁，避免正常一秒周期被通用半秒总线看门狗误判。超过十秒、未来时间或非八字节数据不再显示旧压力，失效/恢复变化有日志。
- 已知 SNA、非有限值、越界值和错误长度不能作为正常车速、轮速或油门测量继续使用；保留上次有效数值并设置 `vehicleSensorsInvalid`、记录状态变化。无效 CAN/传感器禁止自动调速并使自动灯会话进入取消；陈旧或未来的限速目标、滚轮模板不能发送滚轮指令。
- HW4 融合限速按固定 km/h 处理，不再随巡航显示单位转换；不更改 carrot 的导航决策、变道状态机、BODY 编码或 Panda 安全策略。

补充文件 `tesla_model3_party_supplement.dbc` 按要求原样保存，但不接入运行时收发。其 `BatteryInfo_1.Batt_Current` 位于 bit 24、长 16 位，超出该消息声明的四字节长度；本轮没有为追求“原样一致”而放宽解析器保护，也不把通用 Python 解析器能够加载文件当成严格结构正确的证明。

### 验证与边界

先执行 `op venv`，运行完整 Tesla 车辆测试、两份 Tesla 原生 safety 测试、CAN DBC/定义/打包解析测试，以及自动灯执行器、card、模型意图和既有 DesireHelper 回归：**764 passed、40 skipped、572 subtests passed**。十一份修改 Python 文件 Ruff、差异空白检查通过。

另外设置 `CARROT_NATIVE_CPU=0`，在不使用可选原生扩展的 Python 解码路径运行 Tesla 车辆、交通灯广播、灯执行器和模型意图回归：**583 passed、23 subtests passed**。这项检查针对车辆端尚未编译原生扩展的实际部署路径，不代表车辆硬件或道路验证。

联合运行明确排除了一项已单独确认的既有失败：`TestCanParserPacker.test_parser_can_valid` 在第 56 行断言初始状态无效，而现有通用解析器注册时设置时间并提供两秒宽限，初始 `can_valid` 为真。此前用未修改的 HEAD 解析器、HEAD 测试和通用 `test.dbc` 已复现相同失败，与 Tesla DBC 无关；本轮没有改动该解析器、删除断言或宣称全仓全绿。40 项平台/基类跳过及三条插件配置警告保持原有条件。

新增回归覆盖全部四位拨杆码、三字节协议拒绝、左右各深度到真实变道方向、CRC/SNA/来源消歧、数字油门释放、真实八字节胎压及来源/过期/错误长度、HW4 广播初始化和真实 CAN 到交通灯 JSON。未发送车辆 CAN、未刷写或实车验收；持续物理闪灯仍需通过实际 TX 回显和车辆灯反馈确认，不能由 DBC 替换或桌面通过数代替。
