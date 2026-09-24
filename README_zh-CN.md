# litegrip-mujoco

**LiteGrip 平行两指夹爪**的官方 MuJoCo 仿真环境。
API 与 `litegrip` SDK 完全兼容 —— 把 `LiteGrip` 换成 `MujocoGripper`，
同一段控制代码在仿真与真机上行为一致。

## 特性

- 🔄 **可直接替换的 API** —— `LiteGrip` 的 43 个公开成员，名字与语义逐一对应。
- 🖥️ **三种运行模式** —— 独立仿真 / 镜像跟随 / 双控。
- 🎮 **原生 MuJoCo 渲染** —— 实时显示指爪运动、接触与抓取。
- 🧪 **不需要硬件** —— 镜像与双控例程带 `--dry-run`，无 CAN 也能跑通全流程。
- 📏 **真实毫米** —— 行程基准是 URDF 实测几何的 85.452 mm，不是 SDK 名义的 120 mm 刻度。

## 安装

```bash
# 纯仿真（不需要硬件）
pip install litegrip-mujoco

# 镜像 / 双控模式（需要真机 SDK）
pip install "litegrip-mujoco[mirror]"
```

从源码安装：

```bash
git clone https://github.com/nexform-tech/litegrip-mujoco.git
cd litegrip-mujoco
pip install -e ".[dev]"
```

## 快速上手

### 模式 1 —— 独立仿真

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

### 模式 2 —— 镜像（仿真跟随真机）

```python
from litegrip_mujoco import MujocoGripper, MirrorMode, require_sdk

sdk = require_sdk()
real = sdk.LiteGrip(channel="can0", can_id=0x08)
real.connect()
real.enable()

with MujocoGripper(render=True) as sim:
    with MirrorMode(real, sim, rate_hz=50.0):
        real.goto(20.0)     # 仿真实时跟随
```

### 模式 3 —— 双控（一条指令，两边同时动）

```python
from litegrip_mujoco import DualGripper

dual = DualGripper(channel="can0", can_id=0x08, render=True)
dual.start()

dual.open()
dual.grasp(force_n=10.0)
print(dual.compare())       # {'real_frac': …, 'sim_frac': …, 'delta': …}

dual.disconnect()           # 注意：close() 是合拢夹爪，不是释放资源
```

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
"已知行为"。

## 例程

| 例程 | 内容 | 需要硬件 |
| --- | --- | :---: |
| `01_hello_sim.py` | 建立仿真、读状态、开合一次 | ❌ |
| `02_move_sim.py` | 位置 / 速度 / 力控，以及抓取验证 | ❌ |
| `03_trajectory.py` | 位置轨迹录制、存盘、加载与回放 | ❌ |
| `04_mirror_real.py` | 真机开度实时驱动仿真 | `--dry-run` |
| `05_dual_control.py` | 同一条指令同时下发仿真与真机 | `--dry-run` |

```bash
# 在仓库根目录跑，且先 `pip install -e ".[dev]"` —— 例程 import 的是
# litegrip_mujoco，而 src 布局的包没装上是 import 不到的。
python3 examples/01_hello_sim.py
python3 examples/02_move_sim.py
python3 examples/03_trajectory.py
python3 examples/04_mirror_real.py --dry-run
python3 examples/05_dual_control.py --dry-run
```

## API 对照

`litegrip.LiteGrip` 的每一个公开成员都在 `MujocoGripper` 上存在，名字与含义相同。

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
| `home()` | 同名 | ⚠️ 见"已知行为" |
| `get_state()` / `get_position()` / `get_position_rad()` | 同名 | ✅ 一致 |
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

另导出：`DualGripper`、`MirrorMode`、`DryRunGripper`、`read_frac_open()`、
`write_frac_open()`、`constants`、`require_sdk()`、`HAS_SDK`。

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

### `litegrip` SDK 里发现的两个 bug

在此列出以供知悉。`litegrip-mujoco` 既不修补也不规避它们。

1. **`goto_rad()` 忽略入参。** `constants.py` 里 `POS_OPEN_RAD = +1.14`、
   `POS_CLOSED_RAD = 0.0`，违反了 SDK 自己文档写的不变量（闭合值应数值更大）。
   `gripper.py` 的 clamp `max(pos_open, min(pos_closed, x))` 于是对**任意**输入都返回
   `+1.14`。实测 `goto_rad(0.0)`、`goto_rad(0.5)`、`goto_rad(-1.14)`、`goto_rad(3.0)`
   全部得到 `+1.1400`。加载标定 JSON 会覆盖两端点，把这个 bug 掩盖掉。
2. **`calibrate_guided()` 写死 120 刻度。** 它算的是 `rad_to_mm = 120.0 / travel`，
   忽略 `config.max_stroke_mm`，而 `calibrate()` 与 `calibrate_manual()` 都正确使用它。
   因此走引导式标定必然产出 120 刻度的夹爪。

## 开发

```bash
pip install -e ".[dev]"
python -m pytest tests/ -v
```

测试套件是纯仿真的 —— 不需要 CAN 接口、不需要硬件、不需要 `litegrip` SDK。
需要 SDK 的用例在 SDK 缺席时会自行 skip。

模型结构、毫米标定口径与碰撞几何的设计取舍见
[docs/DEVELOPER_GUIDE_zh-CN.md](docs/DEVELOPER_GUIDE_zh-CN.md)。

## 许可证

Proprietary

---

[English](README.md) | [开发者指南](docs/DEVELOPER_GUIDE_zh-CN.md)
