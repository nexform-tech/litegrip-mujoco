# litegrip-mujoco

**LiteGrip 平行两指夹爪**的官方 MuJoCo 仿真环境，API 与 `litegrip` SDK 完全兼容
—— 把 `LiteGrip` 换成 `MujocoGripper`，同一段控制代码在仿真与真机上行为一致。

[English](README.md) · **简体中文**

| 属性 | 值 |
| --- | --- |
| 产品 | LiteGrip 平行两指夹爪 |
| 仓库定位 | MuJoCo 仿真环境 |
| 模型 | 两个平行的移动副手指，各 42.726 mm 行程 |
| 钳口开度 | 闭合 1.548 mm … 张开 87.000 mm |
| 行程基准 | 85.452 mm，URDF 实测值 —— 不是 SDK 名义的 120 mm |
| 物理引擎 | MuJoCo 3.0+，`dt = 1 ms`，θ 空间 PD，增益取自 SDK 本身 |
| Python | 3.10+ |

## 安装

```bash
# 纯仿真（不需要硬件）
python3 -m pip install litegrip-mujoco

# 镜像 / 双控模式（需要真机 SDK）
python3 -m pip install "litegrip-mujoco[mirror]"
```

从源码安装：

```bash
git clone https://github.com/nexform-tech/litegrip-mujoco.git
cd litegrip-mujoco
python3 -m pip install -e ".[dev]"
```

## 快速上手

```python
from litegrip_mujoco import MujocoGripper

with MujocoGripper(render=True) as gripper:
    gripper.open(duration=1.0)
    gripper.close(duration=1.0)

    # 位置控制，单位是真实毫米。0 = 闭合，85.452 = 全开。
    gripper.goto(20.0, duration=0.5)

    # 力控：先合拢到堵转，再保持指定的夹持力。
    gripper.grasp(force_n=10.0)

    state = gripper.get_state()
    print(state.position_mm, state.force_n)
```

演示场景额外带地板和一个被夹具托住的 20 mm 工件，因此抓取可以端到端验证：

```python
from litegrip_mujoco import MujocoGripper

with MujocoGripper(model_path="scene.xml", render=True) as gripper:
    gripper.grasp(force_n=10.0)
    gripper.release_fixture()   # 之后工件只靠摩擦力留在指间
```

### 镜像（仿真跟随真机）

```python
from litegrip_mujoco import MujocoGripper, MirrorMode, apply_calibration, require_sdk

# 上位机标定工具为这台夹爪写出的文件
CALIBRATION = "~/.litegrip/litegrip_calibration.json"

sdk = require_sdk()
real = sdk.LiteGrip(channel="can0", can_id=0x08)
real.connect()
apply_calibration(real, CALIBRATION)   # 必须在 enable() 之前
real.enable()

with MujocoGripper(render=True) as sim:
    with MirrorMode(real, sim, rate_hz=50.0):
        real.goto(20.0)     # 仿真实时跟随
```

`apply_calibration()` 必须排在 `connect()` 之后、`enable()` 之前。`enable()` 是第一个
会给电机上电的调用，标定没载上就必须在它上面被拦下。

### 双控（一条指令，两边同时动）

```python
from litegrip_mujoco import DualGripper

dual = DualGripper(channel="can0", can_id=0x08, render=True,
                   calibration="~/.litegrip/litegrip_calibration.json")
dual.start()

dual.open()
dual.grasp(force_n=10.0)
print(dual.compare())       # {'real_frac': …, 'sim_frac': …, 'delta': …}

dual.disconnect()           # 注意：close() 是合拢夹爪，不是释放资源
```

## 标定

标定文件记录**一台**真机的两个 θ 端点 —— `zero_position_rad`（闭合端）、
`max_position_rad`（张开端）和 `rad_to_mm`。它由上位机（GUI）的标定工具写出，
本包自己产不出标定文件。

**任何会让真机动起来的代码，都必须在运行时选定一份标定文件。** 没有谁会替你选，
默认的那份也绝不会被隐式使用。原因见[为什么要有这道闸](#为什么要有这道闸)；
一句话版本是：SDK 的 `load_calibration()` 无法告诉你它到底读了哪个文件。

### 怎么选

`04_mirror_real.py` 与 `05_dual_control.py` 接受两种形式：

| 调用方式 | 行为 |
| --- | --- |
| `--calibration PATH` | 直接用这份文件，不弹提示。脚本和非交互运行走这条。 |
| 不带参数、交互式终端 | 列出候选及其端点、行程，然后让你挑。 |
| 不带参数、非交互终端 | 以状态码 2 退出，并打印该用的 `--calibration` 写法。 |

```bash
# 先看看这台机器上有哪些，再挑一份 —— 在仓库根目录执行
python3 examples/05_dual_control.py --list-calibrations
python3 examples/05_dual_control.py --calibration ~/.litegrip/litegrip_calibration.json
```

`--dry-run` 不动真机，完全跳过标定选择。

从 Python 调时，`calibration=` 是关键字参数，`apply_calibration()` 是显式调用：

```python
from litegrip_mujoco import DualGripper, MirrorMode, apply_calibration

CALIBRATION = "~/.litegrip/litegrip_calibration.json"

dual = DualGripper(channel="can0", can_id=0x08, calibration=CALIBRATION)
MirrorMode(real, sim, rate_hz=50.0, calibration=CALIBRATION)

real.connect()
apply_calibration(real, CALIBRATION)   # connect() 之后、enable() 之前
real.enable()
```

顺序是有意义的。`enable()` 是第一个给电机上电的调用，标定必须在它上面套好并校验完。
`apply_calibration()` 只抛异常、不给警告：读不出的文件、自相矛盾的文件、SDK 的出厂文件、
以及载入后 `config` 端点与文件对不上的情况，一律直接报错。

### 候选从哪来

不给显式路径时，扫描目录是「SDK 默认标定路径所在目录」（`$LITEGRIP_CALIB`，否则
`~/.litegrip/litegrip_calibration.json`）和当前工作目录，取其中的 `*.json` 普通文件。
SDK 自带的出厂文件永远不出现在候选里。可用的标定排在不可用的 JSON 前面；可用的那些里，
「SDK 默认路径所在目录」排在当前工作目录前面，同一个目录内最新的排最前。

显式给的路径永远优先，也永远不会被质疑 —— 包括 SDK 的默认路径。那份文件在列表里会标上
`⚠ SDK 默认路径`，让选择可见，但**刻意选它是允许的**。被拒绝的只是「不选就用」。

### 这道闸检查什么

三层，因为任何单独一层都能被绕过：

1. 在 SDK 拿到文件**之前**先解析并校验 —— 必填键齐全、取值是有限数、`rad_to_mm > 0`，
   且闭合端在数值上**大于**张开端。不过关的文件根本不会交给 SDK。
2. 按 `realpath` 认身份，拒绝 SDK 的出厂标定，所以改名或软链接都蒙混不过去。
3. 载入之后，把 `config.pos_closed_rad` / `pos_open_rad` 与文件逐字段比对。对不上就报错，
   并把「请求的端点」和「实际读到的端点」一起打出来 —— 那正是静默回落到出厂值的特征。

来源这一道闸，外加「`config` 是否仍与已套用的标定一致」这一项，在每次
`read_frac_open()` / `write_frac_open()` 和每次 `DualGripper`、`MirrorMode` 运动之前都会
再跑一遍。所以**跑着跑着丢掉标定**的设备同样会被抓住，不只是从没标定过的。

### 这道闸检查不了什么

一份格式完全正确、但属于**同型号另一台夹爪**的标定文件，与正确的那份无法区分。
这里没有任何办法识别它。唯一可做的检查是算术：选择器会打印每份文件隐含的行程，
动手之前先把那个数看一眼，与这台夹爪的真实行程对一下。

### 换算函数默认严格

`read_frac_open()` 与 `write_frac_open()` 默认 `strict=True`。在没有可信标定的设备上，
它们抛 `UncalibratedDeviceError`，而不是退回到 SDK 的毫米路径 —— 那条路径正是用缺失的
端点定义的。

旧行为仍然可达：`read_frac_open(device, warn=True)` 会发一条 `DeprecationWarning`，
隐含 `strict=False`，保留原来的「警告并回落」路径。想要同样的行为又不想要警告，传
`strict=False`。

仿真设备豁免，因为它们的端点来自模型而不是标定 —— `MujocoGripper` 与 `DryRunGripper`
都设了 `IS_SIMULATED = True`。其它设备可以用
`mark_calibrated(device, None, reason="…")` 显式登记豁免理由。

### 为什么要有这道闸

SDK 的 `load_calibration(path)` 会构造 `sources = [path, _FACTORY_CALIB]`，吞掉
`FileNotFoundError` 与 `json.JSONDecodeError`，一路回落到出厂文件，然后对**两个来源都**
返回 `True`。于是路径打错一个字母的后果，是一台自称载入成功、随后按**别人的坐标**运动的
夹爪。这条路径上另外两个缺陷见[已知行为](#已知行为)。

## 架构

```text
┌──────────────────────────────────────────────────────┐
│                    你的 Python 程序                    │
│                                                       │
│   g = MujocoGripper()    ← 替换 litegrip.LiteGrip      │
│   g.grasp(force_n=10.0)                               │
│   g.get_state()                                       │
└──────────┬─────────────────────────┬──────────────────┘
           │                         │
    ┌──────▼───────┐         ┌───────▼────────────┐
    │   独立仿真    │         │   双控 / 镜像       │
    │              │         │                    │
    │  MuJoCo      │         │  MuJoCo + CAN      │
    │  物理引擎     │         │  → litegrip SDK    │
    │  θ 空间 PD   │         │                    │
    │  接触求解     │         │  真机 + 仿真同时    │
    └──────────────┘         └────────────────────┘
```

物理模型完全由仓库自带的 URDF 与 STL 网格构建。它建模了**单台** DM4310 电机经刚性
耦合驱动两指的机构，并复刻了 SDK 的控制语义 —— 包括那些会让人意外的部分，见下方
「已知行为」。

## 例程

| 例程 | 方向 | 需要硬件？ |
| --- | --- | --- |
| [`examples/01_hello_sim.py`](examples/01_hello_sim.py) | 只读状态，不运动 | 否 |
| [`examples/02_move_sim.py`](examples/02_move_sim.py) | 位置 / 速度 / 力控 | 否 |
| [`examples/03_trajectory.py`](examples/03_trajectory.py) | 录制、存盘、加载、回放 | 只有录制和放给真机时需要（`--dry-run`、`--play` 不要） |
| [`examples/04_mirror_real.py`](examples/04_mirror_real.py) | 真机 → 仿真 | `--dry-run`，否则要 `--calibration` |
| [`examples/05_dual_control.py`](examples/05_dual_control.py) | 键盘 → 真机，仿真同步显示 | `--dry-run`，否则要 `--calibration` |

03、04、05 不给 `--dry-run` 时走的是真机，因此必须先给一份标定文件 —— 见[标定](#标定)。

每个例程的命令行、终端分节，以及上真机之前必须走完的清单，都写在
**[`examples/README.zh-CN.md`](examples/README.zh-CN.md)**（[English](examples/README.md)）。

```bash
# 在仓库根目录跑，且先 `pip install -e ".[dev]"` —— 例程 import 的是
# litegrip_mujoco，而 src 布局的包没装上是 import 不到的。
python3 examples/01_hello_sim.py
python3 examples/02_move_sim.py
python3 examples/03_trajectory.py
python3 examples/04_mirror_real.py --dry-run
python3 examples/05_dual_control.py --dry-run
```

开了窗口的运行如果在打印 `✅ 完成` **之后**才报 `Segmentation fault (core dumped)`，
这次运行本身是成功的 —— 那是进程退出阶段的上游 GL 崩溃，不是仿真失败。
`--no-render` 不受影响，退出码 0。详见开发者指南。

## API 对照

`litegrip.LiteGrip` 的 44 个公开成员全部都在 `MujocoGripper` 上存在，名字与含义相同。

| `litegrip.LiteGrip` | `MujocoGripper` | 说明 |
| --- | --- | --- |
| `LiteGrip(channel, can_id)` | `MujocoGripper(render=True)` | 构造函数 |
| `connect()` / `disconnect()` | 同名 | ✅ 一致 |
| `enable()` / `disable()` / `clear_fault()` | 同名 | ✅ 一致 |
| `open()` / `close()` | 同名 | ✅ 一致 |
| `goto(mm)` / `goto_rad(rad)` | 同名 | ✅ 一致 |
| `move_to(rad)` / `move_at_speed(mm, mm_s)` | 同名 | ✅ 一致 |
| `grasp(force_n)` | 同名 | ✅ 一致，包括超出请求值的部分 |
| `set_force(force_n)` | 同名 | ✅ 一致，目标 = 当前位置 |
| `home()` | 同名 | ⚠️ 见「已知行为」 |
| `get_state()` / `get_position()` / `get_position_rad()` | 同名 | ✅ 一致（见「状态新鲜度」） |
| `refresh_status(timeout_s)` | 同名 | ✅ 恒返回 `True` —— 仿真状态随时可读 |
| `get_force()` / `get_torque()` / `get_error()` | 同名 | ✅ 一致 |
| `get_temperature()` / `get_info()` | 同名 | ✅ 用仿真热模型 |
| `is_moving()` / `is_grasped()` / `wait_for_ready()` | 同名 | ✅ 一致 |
| `send_mit_frame(q, kp, kd, dq, tau)` | 同名 | ✅ 一致 —— 更新控制律 |
| `poll(timeout)` | 同名 | ✅ 仿真中为空操作 |
| `read_param(rid)` | 同名 | ❌ 抛 `NotImplementedError` |
| `stop()` | 同名 | ✅ 一致 |
| `enter_zero_gravity()` / `exit_zero_gravity()` | 同名 | ✅ 一致 |
| `calibrate*()` / `save_calibration()` / `load_calibration()` | 同名 | ✅ 仿真标定 |
| `channel` / `can_id` / `mst_id` / `config` | 同名 | ✅ 记录但仿真不使用 |

仿真独有：`step()`、`settle()`、`reset()`、`release_fixture()`、`hold_fixture()`、
`gap_mm()`、`frac_open()`、`set_frac_open()`、`launch_viewer()`、`sync_viewer()`、
`model`、`data`、`model_path`。

循环骨架与查看器层（与 PyBullet 例程同形）：`pump()`、`connected()`、`sim_time`、
`command_fraction()`、`gui`、`keyboard_events()`、`mouse_events()`（恒为空表 —— 被动
查看器不给鼠标事件）、`status_text()`、`focus_camera()`。

`status_text()` 用 MuJoCo 内置的位图字体画字，那套字体没有中文字形——一个汉字画出来
是一个实心方块。所以窗口叠字一律 ASCII，中文留在终端。详见[例程
README](examples/README.zh-CN.md#为什么窗口里的字只有英文)。

世界查询：`box_slots()`、`add_box()`、`contacts()`、`link_aabb()`、`pad_aabbs()`、
`grasp_center()`。

为例程另导出：`window` 里的 `pressed()`、`clicked()`、`held()`、`key_label()`、
`key_codes()`、`KeyQueue`、`QUIT_KEYS`、`CONFIRM_KEYS`、`ZERO_GRAVITY_KEYS`、
`TELEOP_KEYS`；`trajectory` 里的 `Trajectory`、`TrajectorySample`、
`trajectory_dir()`、`resolve_path()`，以及异常 `TrajectoryError`、
`TrajectoryBusyError`、`TrajectoryEmptyError`、`TrajectoryFormatError`、
`TrajectoryNotActiveError`、`TrajectoryRecordingError`；`world` 里的
`MujocoContact`。

另导出：`DualGripper`、`MirrorMode`、`DryRunGripper`、`read_frac_open()`、
`write_frac_open()`、`constants`、`require_sdk()`、`HAS_SDK`。

标定相关导出：`apply_calibration()`、`require_calibration()`、`select_calibration_for()`、
`resolve_calibration_path()`、`discover_calibrations()`、`load_calibration_file()`、
`default_calibration_path()`、`sdk_factory_calibration_path()`、`mark_calibrated()`、
`applied_calibration()`、`is_calibrated()`、`is_simulated_device()`、
`require_usable_device()`、`format_selection()`、`describe_candidate()`，
`Calibration` 数据类，以及异常 `CalibrationError`、`CalibrationRequiredError`、
`CalibrationFileError`、`CalibrationVerificationError`、`UncalibratedDeviceError`。

## 状态

哪些验过、哪些没验过：

| 能力 | 状态 | 证据 |
| --- | --- | --- |
| 模型几何与单位 | ✅ 已验证 | `tests/test_mujoco_gripper.py::TestGeometryAndUnits` —— 87.000 mm 开口、网格单位、全行程无自穿透 |
| 那些反直觉行为的物理保真 | ✅ 已验证 | `TestForceSemantics` 与 `TestActuator` 把「已知行为」逐条固化成测试 |
| 与 `LiteGrip` 的 API 对等 | ✅ 已验证 | `TestApiParity` 持有一份冻结的成员清单；装了 SDK 时会对着真类比，2026-09-28 那次报出缺 `refresh_status` —— 由 PR #4 补上 |
| 标定闸 | ✅ 已验证 | `tests/test_calibration.py`，84 个用例，对手是一个能让 `load_calibration()` 复现 SDK 静默回落的替身 |
| 例程 01–03 | ✅ 已验证 | 01、02 带 `--headless` 退出码 0；03 用 `--dry-run` 无头跑完录制、存盘、回放，用 `--play` 无头回放已存的文件。不需要硬件、不需要 SDK |
| 例程 04 / 05 的 `--dry-run` | ✅ 已验证 | 两个无头都退出码 0；04 镜像一个脚本驱动的替身，05 走完脚本目标并在目标之间保持位置 |
| 例程命令行契约 | ✅ 已验证 | `--list-calibrations` 退出码 0；非交互且没有可用标定时退出码 1 并打印指引。2 留给 argparse 自己的用法错误 |
| 真机运动 | ⚠️ **未验证** | 手上没有 CAN 硬件。真机那条路径只经由 `DryRunGripper` 跑过，它报的是一份 2026-09-24 标定的 θ 端点；本包从未在这里驱动过 SDK 本身 |
| CAN 探测（`ensure_can_link` / `--no-can-setup`） | ⚠️ **部分验证** | 「读」这一半验证过：解析器钉的是 `ip -details link show` 的真输出（含一份每个标志位都正常、其实是 bus-off 的样本），并且 `probe_can_link("can0")` 对着一只活着的接口只读地跑过、读得对。「拉起」那一半——那串把不对的接口配好的 `sudo ip` 命令——只对着替身 `run` 跑过，**没人看着它修好过一个真接口** |

## 已知行为

下面这些是**对 SDK 的忠实复刻**，不是仿真缺陷。不要在没有同步改动真机行为的前提
下"修好"它们 —— 一旦分叉，仿真就不再能预测真机。

- **`close(force_n=…)` 限不住力。** 目标一路指向闭合位，位置误差 `kp·Δθ` 压倒前馈
  力矩并让执行器饱和。对着 20 mm 工件实测：`force_n = 0 / 5 / 10 / 20` 全都得到
  精确的 100 N。要力控请用 `grasp()` 或 `set_force()`。
- **`grasp(force_n)` 的实际夹持力大于请求值。** 堵转确认窗口（5 × 10 ms）里指爪
  还在往前走，`q_target` 被改写到窗口末尾的位置，于是留下一段固定位置误差。
  实测：请求 10 N → 实际 15.5 N。力对 `force_n` 仍单调，可当带偏置的开环力控用。
- **`set_force(N)` 需要连续下发。** 目标取的是**调用瞬间**的当前位置，所以第一次
  调用会带上一次受力的位置误差（请求 2 N → 第一次 2.88 N）。重复下发即可收敛到
  0.5% 以内。
- **空夹时 `grasp()` 也返回 `True`。** 指爪顶到闭合硬限位同样是"停住了"。这与 SDK
  一致。
- **`get_force()` 是由力矩推算的，不是测量值。** 它就是 `torque_nm × 10`，与 SDK
  逐字相同。两指之间什么都没有时它照样会报数。
- **`home()` 与 SDK 不同。** SDK 的 `home()` 目标取常量 `POS_CLOSED_RAD`（= `0.0`），
  再叠加 `goto_rad()` 的 clamp bug，实际会驱动夹爪**张开**，与它自己的 docstring
  相反。仿真走的是闭合位，与文档一致。这是刻意的偏离 —— 例程不该示范这个 bug。

### 状态新鲜度

SDK 的 `GripperState` 带 `data_age_s`（这批数值来自多久以前的那一帧）、`has_data`
与 `is_stale`（默认阈值 `STALE_AFTER_S = 0.5 s`）。失能的电机不主动发状态帧，
所以真机上 `get_state()` 完全可能返回一个几秒前的快照，或使能前的构造默认值 ——
这正是 `refresh_status()` 存在的理由。

仿真里没有这个问题：每次 `get_state()` 都是**当场**从物理状态算出来的，
所以 `data_age_s` 恒为 `0.0`，`is_stale` 恒为 `False`。这是刻意的——
它反映的是"仿真没有 CAN 链路"这个事实，不是把 SDK 的语义改掉了。
真机代码如果靠 `is_stale` 判断要不要重读，在仿真上会一直走"新鲜"分支，
这是对的。

### `litegrip` SDK 里发现的三个 bug

在此列出以供知悉。`litegrip-mujoco` 不修补 SDK。第三个它选择**拦在前面不让跑** ——
见[标定](#标定) —— 因为那一个的后果不是抛异常，而是一台**在错误坐标里运动**的夹爪。

1. **`goto_rad()` 忽略入参。** `constants.py` 里 `POS_OPEN_RAD = +1.14`、
   `POS_CLOSED_RAD = 0.0`，违反了 SDK 自己文档写的不变量（闭合值应数值更大）。
   `gripper.py` 的 clamp `max(pos_open, min(pos_closed, x))` 于是对**任意**输入都返回
   `+1.14`。实测 `goto_rad(0.0)`、`goto_rad(0.5)`、`goto_rad(-1.14)`、`goto_rad(3.0)`
   全部得到 `+1.1400`。加载标定 JSON 会覆盖两端点，把这个 bug 掩盖掉。
2. **`calibrate_guided()` 写死 120 刻度。** 它算的是 `rad_to_mm = 120.0 / travel`，
   忽略 `config.max_stroke_mm`，而 `calibrate()` 与 `calibrate_manual()` 都正确使用它。
   因此走引导式标定必然产出 120 刻度的夹爪。
3. **`load_calibration()` 无法告知它读了哪个文件。** 它先试传入的路径，再试自带的出厂
   标定，途中吞掉 `FileNotFoundError` 与 `json.JSONDecodeError`，最后对**两个来源都**
   返回 `True`。所以路径拼错与载入成功在返回值上完全一样。另外，文件能解析但缺
   `rad_to_mm` 时，抛出的 `KeyError` 落在处理其它缺键的那段守卫之外。

## 相关仓库

| 仓库 | 定位 |
| --- | --- |
| [litegrip-urdf](https://github.com/nexform-tech/litegrip-urdf) | URDF/xacro 描述包 —— 本模型的几何来源 |
| [litegrip-pybullet](https://github.com/nexform-tech/litegrip-pybullet) | 同一款夹爪的 PyBullet 仿真环境 |
| [litearm-mujoco](https://github.com/nexform-tech/litearm-mujoco) | LiteArm 的 MuJoCo 环境，本包的形态参照它 |
| [lite-grip](https://gitee.com/yudao_hz_1/lite-grip) | 本包对标的真机 SDK |

## 开发

```bash
python3 -m pip install -e ".[dev]"
python3 -m pytest tests/ -v
```

测试套件是纯仿真的 —— 不需要 CAN 接口、不需要硬件、不需要 `litegrip` SDK。
需要 SDK 的用例在 SDK 缺席时会自行 skip。

### Dev 容器

仓库自带预装全部依赖的 dev 容器配置（[.devcontainer/](.devcontainer/)）。在 VS Code 中打开
仓库，执行 `F1 → Dev Containers: Rebuild and Reopen in Container`，即可得到现成的环境
—— Python 3.11、MuJoCo、测试套件、查看器所需的 GL 运行时。交互式查看器在 Windows 上需要的
X11 配置见 [.devcontainer/README.zh-CN.md](.devcontainer/README.zh-CN.md)。

模型结构、毫米标定口径、标定闸与碰撞几何的设计取舍见
[docs/DEVELOPER_GUIDE_zh-CN.md](docs/DEVELOPER_GUIDE_zh-CN.md)。

## 仓库规范

本仓库遵循 NEXFORM ROBOTICS 的共享仓库规范：代理操作规则见 [AGENTS.md](AGENTS.md)，
提交信息用 Conventional Commits，每次合并进 `main` 由 semantic-release 自动发版。

## 许可证

专有 —— `pyproject.toml` 里声明的是 `LicenseRef-Proprietary`。但本仓库的 `LICENSE` 文件
装的是 Apache License 2.0 全文，两者尚未统一，再分发之前请先确认以哪一份为准。
