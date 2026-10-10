# 开发者指南

LiteGrip 的 MuJoCo 模型是怎么搭起来的，以及为什么这么搭。改
`assets/litegrip.xml` 或 `src/litegrip_mujoco/` 下的任何东西之前，先读这篇。

## 包结构

```text
src/litegrip_mujoco/
├── __init__.py          公开导出
├── constants.py         单位、行程基准、换算。刻意不 import SDK。
├── controller.py        θ 空间 PD + min-jerk / 线性斜坡
├── calibration.py       标定的来源：发现、校验、套用并复核
├── gripper.py           MujocoGripper —— 与 LiteGrip 对等的门面
├── mirror.py            DualGripper、MirrorMode、跨设备 frac_open 换算
├── dryrun.py            DryRunGripper —— 符合 LiteGrip 形态的虚拟夹爪
├── _litegrip/           惰性探测真 SDK；缺席时回落到本地 dataclass 镜像
└── assets/
    ├── litegrip.xml     夹爪模型
    ├── scene.xml        litegrip.xml + 地板 + 灯光 + 被夹具托住的工件
    └── meshes/          从 litegrip-urdf 逐字节拷贝的 3 个 STL
```

## 数字从哪来

| 来源 | 提供什么 |
| --- | --- |
| `litegrip-urdf/urdf/litegrip_urdf.urdf.xacro` + 3 个 STL | 全部几何、惯量、关节原点 |
| `lite-grip`（SDK） | 控制语义、单位、`grasp()` 堵转逻辑、异常类型 |
| `litearm-mujoco` | 包形态、例程编号、控制器结构 |

URDF 是**几何**的权威。SDK 是**控制语义**的权威。两者打架时，分歧写在 XML 注释里，
而不是抹平。

## 毫米标定口径

存在两套毫米刻度，相差 40%：

| 口径 | 数值 | 出处 |
| --- | --- | --- |
| 真实行程 | **85.452 mm** | 两指实测位移 `2 × 0.042726 m` |
| SDK 名义值 | 120 mm | `calibrate_guided()` 里写死的 `120.0 / travel` |

**本包一律上报真实毫米。** `constants.MM_SCALE = 85.452`，你读到的每一个
`position_mm` 都是真实毫米。

SDK 那套 120 刻度应当**在真机侧消除，而不是在仿真侧补偿**：把真机的
`GripperConfig(max_stroke_mm=85.452)` 设好并重新标定，真机也直接报真实毫米，
两侧零换算。在仿真里加偏移量是双重计数，只会让 SDK 的 bug 变成永久事实。

### θ 是设备相关的，`frac_open` 不是

电机角不适合做交换量：它的**零点和量程都随标定而变**。

| 设备 | θ 闭合 | θ 张开 | 量程 |
| --- | --- | --- | --- |
| 仿真 | `0.0` | `-1.14` | 1.14 |
| 真机（已标定） | `1.775959` | `-0.064279` | 1.8402 |

真机那一行是 **2026-09-24 那一次标定**的取值，作为参照冻结在这里。重新标定会同时改变
这两个数，所以请把它当成示例，而不是"这台机器当前的端点"。`constants.REAL_POS_CLOSED_RAD`
/ `REAL_POS_OPEN_RAD` 存的就是这一对，`--dry-run` 的虚拟夹爪也报这一对，好让它算 θ 的
路径与真机同形 —— 除此之外没有别处读它们。

同一个 `position_rad` 交给两台设备，落点完全不同。无量纲的
`frac_open ∈ [0, 1]` 与标定无关，是唯一安全的交换量。`MirrorMode` 与
`DualGripper` 交换的都是 `frac_open`，见 `mirror.py`。

### URDF 里两处已知的不一致

两处都是已知的、刻意的，且都被测试钉住：

1. **行程：网格理论 0.0435 m vs 实测 0.042726 m。** 模型取实测值，因为那才是真实
   机械硬限位。用 0.0435 会让 `close()` 越过物理限位 0.77 mm，并给每一个上报的
   毫米数带上固定偏移。
2. **张开 87.000 mm 与总行程 85.452 mm 互相矛盾 0.04 mm。** URDF 注释里写的闭合
   间隙 1.508 mm，是按"实测张开 86.960 mm"倒算的，不是从网格量的。本模型用
   **网格几何**，所以闭合间隙恒为 1.548 mm。

关系式为 `gap_mm = travel_mm + 1.548`。`get_position()` 报的是行程（0 = 闭合），
`gap_mm()` 报的是绝对开口。

## 标定的选择与来源

真机运动前必须有一份标定文件。不给路径时用的那份就是 SDK 的出厂标定
（`factory_calibration.json`，紧挨着 `litegrip` 包），规则只在
`resolve_calibration_path()` 里实现一次。这条规则归 `calibration.py` 管；`mirror.py` 与
例程 04/05 通过调用它来强制。没有任何地方去扫盘找标定，也没有任何地方问你要哪一份。

**不要把「扫盘 + 挑一份」重新加回** `apply_calibration()` 或例程里。SDK 自己在给定路径
读不出来时会静默装载出厂文件并且照样返回 `True`；在它上面再叠一个选择器，只是多出一条
「用别人的角度驱动电机」的路。要换标定就点名：Python 里传 `calibration=`，命令行上传
`--calibration`。

### 两道闸

设备要能动，两道闸必须同时过：

| 闸 | 判据 | 不过时 |
| --- | --- | --- |
| 来源 | `is_simulated_device(d)` 或 `_provenance(d) is not None` | `UncalibratedDeviceError` |
| 有效 | `_endpoints(d) is not None` —— 闭合端数值大于张开端 | `UncalibratedDeviceError` |

第一道问的是"这些数从哪来"，第二道问的是"这些数讲不讲得通"。SDK 的出厂常量
（`pos_closed_rad = 0.0`、`pos_open_rad = +1.14`）**类型对、次序反**，而这个反转正是
"没标定过"的识别特征：只要 `pos_closed_rad <= pos_open_rad`，`_endpoints()` 就返回
`None`。真实标定永远满足这个不变量。

来源被记成一个冻结的 `Provenance(path, calibration, reason)`，存在
`device._litegrip_mujoco_calibration` 上；对拒绝 `setattr` 的对象（`__slots__`）退到
`WeakKeyDictionary`。`mark_calibrated(device, None, reason="…")` 是给"端点可信但没有文件"
的设备留的口子 —— 理由会被记下来，于是豁免是**显式**的，review 时看得见。

`require_usable_device()` 先过第一道闸，再走 `_check_device_still_holds()`：`config`
与已套用的标定不一致时抛 `CalibrationVerificationError`。这后一步抓的是**跑着跑着丢掉
标定**的设备，不只是从没标定过的。

### 两层防线

`apply_calibration(device, path)` 在两个地方设防，因为只有第一层看不出 SDK 的静默回落：

1. **在 SDK 拿到文件之前先校验。** 存在性、是普通文件、是 JSON 对象、三个必填键
   （`zero_position_rad`、`max_position_rad`、`rad_to_mm`）、取值有限、`rad_to_mm > 0`、
   `closed > open`，以及跨度不小于 `MIN_SPAN_RAD = 1e-3`。SDK 在"文件能解析但缺
   `rad_to_mm`"时会抛出没被守卫的 `KeyError`；而 `closed == open` 的文件会在下游除零。
2. **载入后把 `config` 端点与文件逐字段比对。** 这是承重的一层 —— 只有它抓得住 SDK 的
   静默回落 —— 报错时同时给出"请求的端点"和"实际读到的端点"，那正是回落的特征。

SDK 的出厂文件在这里没有任何特殊通道：它和别的文件一样被校验、被核实。以前它按
`realpath` 认身份被拒（要 `allow_factory=True` 才放行），结果是「库层唯一不肯接受的文件，
恰好就是那个默认标定」。

⚠ **绝对不要把 `apply_calibration()` 挪到 `enable()` 下面。** `enable()` 是第一个给电机
上电的调用。SDK 文档给的顺序是 `connect() → load_calibration() → enable()`，
`apply_calibration()` 就是中间那一步的替代品，两个例程都按这个顺序写。

### `read_frac_open` 为什么默认严格

毫米回落路径是用**缺失的那两个端点**定义的，所以它返回一个看起来像读数、实际不是读数的
数。设备过不了任何一道闸时，`read_frac_open()` 与 `write_frac_open()` 抛
`UncalibratedDeviceError`，而不是返回那个数。

`read_frac_open(device, warn=True)` 作为废弃别名保留：它发一条 `DeprecationWarning`
并隐含 `strict=False`。保留而不是删除，是因为删掉就破坏了调用方，而按本仓库的发布策略，
破坏性变更意味着大版本 —— 这不该作为一次安全修复的副作用发生。用 `strict=False`
可以直接拿到旧路径而不带警告。

### `DryRunGripper` 的坑

`DryRunGripper.load_calibration()` 与 `save_calibration()` 委托给 `self._inner`，所以
`dry.config` 永远不会变，委托的返回值也说明不了外层对象的任何事。`DryRunGripper` 靠
`IS_SIMULATED` 免除第一道闸，因此这里没有任何东西依赖那两个委托 —— 但**不要**读
`dry.config` 来指望拿到刚载入文件的端点。这是本闸之前就有的问题，与它无关；记录在案，
不在此处修。

## 为什么碰撞用 box 而不是网格

MuJoCo **没有凹网格碰撞** —— 每个碰撞网格都会被凸化成凸包。在这台夹爪上实测：
`base_link` 凸包体积是网格的 2.334×，finger 是 2.075×。凸化后 base 填满了指的滑道，
**任意关节值下都恒定 6 个接触、5.25 mm 穿透**。这不是调参能救的，是结构性的。

所以：视觉 geom 用原始 STL 网格并关闭碰撞，碰撞用每指两个 box 代理（滑座 + 夹持面），
从 finger STL 分层扫描导出。夹持面 box 的 `+x` 面精确落在 mesh `x = 0.0235`，
即 `q = 0` 时的世界坐标 `x = -0.0435` —— 这就是 87.000 mm 开口精确成立的原因。
base 本体完全不加碰撞几何，与 `litearm7.xml` 一致。

这四个碰撞 box 都带 `group="3"`，而 group **只影响渲染** —— 碰撞只看 `contype` /
`conaffinity`，从不看 group，所以动力学与接触都不变。这条不是可选项：box 是刻意超出网格的
（夹持面 box 要覆盖网格上 `x ≥ 0.0230` 的全部顶点），留在默认组里就会从白色 STL 中透出来，
在查看器里显示成指尖上方两块灰色方块，看着像模型画错了。查看器默认只画第 0/1/2 组
（`mujoco.MjvOption()` 默认就是 `[1 1 1 0 0 0]`，`launch_passive` 用的正是这个默认值），
要在界面上看碰撞体，Rendering 面板里勾上第 3 组。

⚠ **绝对不要给 mesh geom 写 `pos`/`quat`。** MuJoCo 在编译期对网格做主惯性轴对齐，
并把 geom 的默认 `pos`/`quat` 设成同一个变换来抵消它。覆盖掉就等于取消抵消，
零件会歪。有回归测试守着这条。

## 执行器链路

```text
ctrl  ──(gear=10)──►  qfrc_actuator  ──►  关节力  ──►  夹持面法向力
 τ [Nm]                 10·τ [N]          (每指)
```

`gear="10"` 是为了让 `data.ctrl` 的数值**就等于** SDK 的 `tau`(Nm)。
`ctrlrange=±10` 对应 DM4310 的 `TAU_MAX`，夹持力上限由真实电机决定。

两个坑：

- **`actuator_force` 是 gear 之前的量**，`qfrc_actuator` 是之后的。混用等于 10× 误差。
- **`qfrc_actuator` 是指令力，不是测量值。** 空夹顶到限位时它读 100 N，而真实接触力
  是 0。也不能改用 `qfrc_constraint` 来测夹持力：工件落在地板上、`ctrl = 1.0` 时它
  仍然精确读到 `-5.000 N`，因为闭合关节限位提供了反力。

因此 `get_force()` 与 SDK 逐字一致：`force_n = torque_nm × 10`。它是由力矩推算的
估计值，两指之间什么都没有时照样报数。

### `armature` 不是可选项

DM4310 的转子惯量约 `1.2e-5 kg·m²`。经 10:1 折算得 `1.2e-3 kg·m²`，再经螺距比
`dθ/dx = 26.68 rad/m` 折算成约 **0.85 kg 的等效质量** —— 是指自身 0.0355 kg 的 24 倍。
不写 `armature="0.85"` 时，`dt = 2 ms` 下 `vmax` 会冲到 2.93 m/s 且**永不收敛**
（1 s 后 `q` 还停在 0.0399）。

`timestep="0.001"` 同理：`dt = 2 ms` 时 `kd ≥ 5` 不收敛。

## 控制器

PD 在 **θ 空间**里用 SDK 原始的 `kp`/`kd` 计算，所以 `kp = 150` 在真机和仿真上对
同样的电机角误差产生同样的电机力矩，不需要移植增益；有效关节空间增益自动跟随
（`kp_q = GEAR · R · kp`）。

用 `<motor>` 配外部 Python PD 环，而不是 `<position>`，是因为 MIT 帧本身就长成
`f(q, dq, kp, kd, tau)` —— kp 和 kd 在那套协议里是**每次调用的载荷**，`<position>`
会把它们藏进模型里。

默认增益 `kp = 100, kd = 2`，取自 `GripperParams`（不是 litearm 的 260/5）。
θ 空间阻尼比 ≈ 2.8，是过阻尼 —— 减速传动本该如此。不要"修正"它。

`duration` 按**真实时钟秒**计算。仿真比实时快得多，照抄 SDK 的 "sleep 1 s" 会让
`open(duration=1.0)` 在几毫秒内返回，与真机时序分叉。控制循环按绝对截止时间调度，
误差不累积。

位置族调用（`open`/`close`/`goto`/`move_to`/`move_at_speed`）用 min-jerk 目标斜坡；
`grasp()` 与 `set_force()` 用**恒定**目标 —— 斜坡会糊掉堵转检测。

## 力控：承重细节

力控只在位置误差归零时才成立。

```text
ctrl = kp·(θ_des − θ) + kd·(−θ̇) + τ_ff
       └──────┬──────┘
       这项必须消失，ctrl 才等于 τ_ff
```

所以 `grasp()` 分两步：

1. 以 `kp = 150`、目标指向闭合位合拢 —— 位置误差压倒前馈并让执行器饱和
   （这就是 `close(force_n=…)` 限不住力的原因）。
2. 检测堵转（连续 5 个 10 ms 采样 `|Δθ| < 0.001 rad`），然后把 `q_target`
   **改写到当前位置**，只留 `τ_ff = force_n × 0.1`。

第 2 步才是力成真的地方。`set_force()` 一次性做同样的事。

堵转阈值是**电机角**量，必须换算不能照搬：`Δq = 0.001 / R = 3.748e-5 m`。
而且在 **10 ms 控制节拍**上计数，不是每个物理步 —— `dt = 1 ms` 时逐步 `Δq` 比真机
窗口小 10 倍，会在运动中误判堵转。

`grasp()` 空夹时返回 `True`（指爪顶到闭合硬限位也算"停住了"）是 SDK 既有语义，
照抄而非"改进"；改了仿真与真机就分叉。

## 演示场景

`scene.xml` 加了 `z = -0.08` 的地板、灯光，以及一个 20 × 20 × 30 mm 的
带 freejoint 工件，位于 `z = 0.096735`。

**工件靠 `<weld>` 夹具而不是靠台面托着，这不是偷懒。** 夹爪固定在世界坐标系上，
不能移动去够零件，所以零件必须一开始就在两指之间。但它不能坐在台子上：扫描右指
内侧面（`world_x = -0.067 + mesh_x_max + q`）可见，完全闭合时两指在
`z ∈ [0.024, 0.109]` 的**每一个**高度上都会收敛到 `|x| ≈ 0.0008–0.0045`。
不存在任何一个高度能让支撑柱从闭合的两指之间穿过。这在几何上就是不可能的。

所以例程先 `close()` 再 `release_fixture()`。之后工件完全靠摩擦力挂在指间 ——
这才使它成为真正的端到端抓取验证，而不是一个由力矩推算出来的数字。
`open()` 会让它掉到 `z = -0.065`（地板 `-0.08` + 半高 15 mm）。

### 备用槽位，以及 `add_box()` 的代价

`scene.xml` 里还停着四个方块：`spawn_box_0` … `spawn_box_3`，各是一个 freejoint 加一个
box geom，停在离夹爪很远的地方。MuJoCo 的模型只编译一次，所以运行期的 `add_box()`
变不出新的 body —— 它是把某个槽位**搬**到 `pos`（默认 `grasp_center()`，也就是两指
之间），再改它的尺寸。`box_slots()` 给名字；`litegrip.xml` 一个槽位都没有，向它要槽位
会抛 `IndexError`。

- **是搬运，不是新建。** 槽位里原先的东西 —— 尺寸、质量、速度 —— 全没了。反复用同一个
  槽位就是覆盖上一个方块。`reset()` 会把所有槽位放回地板上的停放位。
- **改尺寸必须走 `mj_setConst`。** MuJoCo 的 `body_mass` 和 `body_inertia` 是**编译期**
  由 geom 尺寸算出来的，运行期改了它不会自己发现，所以 `add_box()` 写完这两个数组之后
  要调一次 `mj_setConst(model, data)`。
- **`mj_setConst` 会把 `qpos` 复位成 `qpos0`。** 在 MuJoCo 3.11 上实测：`qvel`、`ctrl`
  和 `time` 不动，但每个关节位置都回到模型默认值。不管它的话，每加一个方块就会悄悄把
  夹爪和工件摆回默认位姿。所以 `world.spawn_box()` 在改尺寸之前先存一份 `qpos`，围绕
  `mj_setConst` 那次调用恢复回来；新方块的位姿是在恢复**之后**写的，两者不会打架。

同一个模块只读的那一半便宜得多，什么都不用操心：`link_aabb()` 和 `pad_aabbs()` 给碰撞
geom 的世界系包围盒，`contacts()` 把接触点的名字和力解析好，`grasp_center()` 给两个指面
的中点。那些包围盒是**碰撞**盒、刻意做得粗 —— 钳口开度绝不能从它读，用 `gap_mm()`。

## 查看器与线程模型

物理跑在 `connect()` 起的后台线程（`litegrip_sim`）里，一把 RLock 守着对 `MjData` 的每一次
访问。由此有三条规则，破任何一条的表现都是**间歇性**故障 —— 正因如此才要写下来：

**1. 先建查看器，再启动仿真线程。** `launch_passive()` 内部会对你的 `MjData` 调
`mj_forward(model, data)`。如果仿真线程已经在 `mj_step()` 同一份数据，两个线程会同时进它的
arena，`mj_makeConstraint` 无法扩容，于是报
`mj_makeConstraint: nefc under-allocation` —— 更常见的是直接段错误。这里曾经真犯过：
`_open_viewer()` 原先先调 `connect()`。已修，`test_no_sim_thread_when_viewer_is_created`
会在它复发时失败。

**2. `sync()` 必须在锁里调。** `sync()` 会把 `MjData` 拷进查看器内部的副本，而主线程的
`close()`/`goto()` 斜坡循环也在锁里 mj_step，放在锁外就是两个线程一起碰同一份 `MjData` ——
和规则 1 是同一种病。`litearm-mujoco` 也是这么写的。

**3. 仿真线程还活着时不要关查看器。** `disconnect()` 先置 `_running = False`，再把查看器摘下来
（循环就拿不到了），然后 join 线程，**只有线程确实退出才关窗**。join 超时说明它可能正卡在
`sync()` 里，此时关窗会和它抢 `MjData`。宁可把窗口漏给进程退出，也不要在这里段错误。

**4. 按键回调跑在查看器自己的线程里，它只能往队列里塞。** `launch_passive()` 是在查看器
线程里调 `key_callback` 的，而那一刻物理线程正在 `mj_step()` 里。所以
`MujocoGripper._on_key()` 只做一件事：`self._keys.feed(int(keycode))`，往一个 `deque`
里 append；主线程在自己的循环里用 `keyboard_events()` 取走。**别**在那个回调里碰
`MjData`、别开关窗口、别打印 —— 回调里抛出的异常会堆在查看器线程上，紧挨着 MuJoCo 的
内部状态，而那里没有任何人在看着。读输入是主线程的事，`pressed()` / `held()` 吃的是
`keyboard_events()` 返回的那张表。

`key_codes()` 惰性解析 glfw 的按键常量，import 不到 `glfw` 时退回字面量，所以这个模块
在没窗口、没显示、没查看器的环境里照样能 import —— `--help`、`--headless`、CI 要的就是
这个。回调本身只有 MuJoCo ≥ 3.1 才收：`_accepts_key_callback()` 查的是 `launch_passive`
的签名，而不是「先调一次、TypeError 就退回两参数版」——后者看着简单，但 `TypeError`
也可能来自 `launch_passive` **内部**，那时窗口已经建好了，重试会在屏幕上多留一个没人
sync 的孤儿窗口。老版本 MuJoCo 上窗口照开，只是 `keyboard_events()` 永远为空。

**叠字字体只有西文。** `status_text()` 是通过 `mjr_overlay` 画到窗口上的，用的就是
MuJoCo 内置的位图字体。那套字体没有中文字形——一个汉字画出来是一个实心矩形，一行中文
就是一行方块。本机在同一条调用上实测（`mjFONTSCALE_150`）：`'A'` 70 个笔画像素、字形
可辨；`'真'` 是 12×10 的实心矩形；`'开'` 是 24×15 的实心矩形外加一条溢出横杠。所以窗口
里的字写 ASCII，中文留给终端——终端有字体。库这一层**不做**过滤：`lines` 原样交给
`set_texts()`，所以这条规矩靠 `tests/test_example_overlay_text.py` 守，它把 `examples/`
里每个 `status_text()` 调用扫一遍，出现非 ASCII 字面量就报错。

**窗口被关掉 = 断开。** `_sim_loop()` 每一拍都在锁里查一次 `viewer.is_running()`；它变
成 false 就说明操作者把窗口关了，于是循环置上 `_abort`（阻塞在 `open()` / `close()` 里的
主线程会因此醒过来）并退出。它**刻意不清** `self._viewer`：关窗口和关查看器句柄是两件
事，把引用摘掉就等于销毁了 `disconnect()` 判断「现在关窗安不安全」的唯一依据
（规则 3）。收尾统一交给 `disconnect()`。`pump()` 是主线程看到的同一个信号：连接没了它
就返回 `False`，例程里的常驻循环就是这么结束的。

### 这里管不了的部分

在某些 Linux 环境下 —— 已确认的一种是「Wayland + 远程桌面 + NVIDIA 专有驱动」——
开过 MuJoCo 查看器的进程可能在**解释器退出阶段核心转储**，而且是在所有活儿干完、结果都打印
出来之后。用裸 MuJoCo、内联 box 模型、不含本仓库任何代码也能复现，`viewer.close()` 关与不关
都能复现，所以这是上游 + 环境的问题。`--no-render` 不受影响，退出码 0。

症状很迷惑：例程打印完 `✅ 完成`，**然后** shell 才报
`Segmentation fault (core dumped)`。看到这个说明这次运行是成功的，崩的是退出阶段。
设一个非空的 `MUJOCO_GL`（`egl` 或 `glfw`）有时能改变它发不发生；启动时那句 `0x502`
同样只是警告。

## 未标定的自由参数

**URDF 里没有任何摩擦数据。** 下表全是保守占位值，不是实测。在真机上相信绝对力
数值之前请先标定。

| 参数 | 取值 | 依据 |
| --- | --- | --- |
| `friction` | `0.9 0.02 0.0001` | 硅胶/TPU 夹持面压在塑料上的量级估计 |
| `solref` | `0.002 1` | 选定值，非实测 —— 见 XML 里的对照表 |
| `solimp` | `0.98 0.999 0.001` | MuJoCo 默认量级的刚性接触 |
| `damping` / `frictionloss` | `0.05` / `0.02` | 占位值 |
| `armature` | `0.85` | 由 DM4310 转子惯量算出 —— 这一个是有推导的，不是猜的 |

`solref` 决定夹持面有多软，进而决定 `grasp(force_n)` 超出请求值多少。对着演示工件实测：

| `solref` | 请求 10 N | 请求 20 N | 请求 40 N | 每指压入量 |
| --- | --- | --- | --- | --- |
| `0.006 1` | 21.84 N | 29.33 N | 45.61 N | 287 / 349 / 442 µm |
| `0.002 1` | 15.51 N | 24.86 N | 43.59 N | 27 / 43 / 75 µm |
| `0.001 1` | 15.51 N | 24.86 N | 43.59 N | 27 / 43 / 75 µm |

比 `0.002` 更硬没有意义 —— `0.001` 结果完全相同，说明剩下的超调来自 SDK 那个
5 × 10 ms 的堵转确认窗口，而不是接触柔度。真机有同样的窗口，所以这个超调是忠实的，
不是仿真缺陷。

## SDK shim

`_litegrip/` **惰性且只探测一次**真 `litegrip` 包。装了就把它的 dataclass、枚举、
异常和辅助函数再导出；没装就用 `_fallback.py` 里逐字段对齐的本地镜像，这样
`get_state()` 仍然返回真正的 `GripperState`。

不 vendor SDK，因为 import 它会拖进 SocketCAN。

### 跟得有多紧

耦合分三层，**不是每一层都会自动跟随 SDK**：

1. **数据类型 —— 自动跟随。** shim 在 import 时从 SDK 解析每个名字，所以
   `GripperState`、`GripperConfig`、`ErrorCode`、`DM_Motor_Type`、`Control_Mode`、
   `ERROR_DESCRIPTIONS`、`describe_error`、`STALE_AFTER_S` 和 7 个异常类都是
   已安装 SDK 里的那个。`isinstance(state, litegrip.GripperState)` 成立。
2. **控制语义 —— 手工复刻，不跟随。** 控制器增益、堵转窗口、min-jerk 斜坡、
   `grasp()` 改写目标那一步。这些是读 SDK 源码照抄的；上游改了，这里必须跟着改。
3. **常量 —— 有意不 import。** `constants.py` 自己拥有仿真侧的数字（85.452 mm、
   θ 端点），不能继承 SDK 的默认值。

异常属于第 1 层，但有个值得知道的讲究：shim 把这些名字绑到 SDK 的**类对象**上，
而不是本地同名类。`except litegrip.LiteGripError:` 必须接得住仿真抛出的异常，而
仅仅同名的两个类接不住。

### 刻意的偏离

`_fallback.py` 只在**一组数值**上偏离 SDK，散落在四个类型里，全部记在测试文件的
`DELIBERATE_DIVERGENCE` 中：

- **`GripperConfig`、`CalibrationData`、`GripperParams`、`UnitConversion`** ——
  同样两个事实抄了四份：`max_stroke_mm` 是 85.452（不是名义的 120），`rad_to_mm`
  由它推出，`pos_open_rad` 是 `-1.14` 而不是 `+1.14`，因为 SDK 那一对违反了它自己
  文档写的不变量（闭合端数值应当更大）。见"毫米标定口径"。注意 `pos_closed_rad`
  两边都是 `0.0`，**不**在表里 —— 守住这张表的用例要求列出的键确实不同。

两处**不是**偏离、但看着像的命名细节：

- **枚举类名**在 `_fallback.py` 里用 SDK 的真名 —— `MotorType`、`ControlMode` ——
  另在模块级保留 `DM_Motor_Type` / `Control_Mode` 两个别名。SDK 就是这么做的；
  跟着做 `repr()` 才一致，而 `GripperParams.MOTOR_TYPE` 的 repr 是会进日志的。
- **`home()`** 目标是闭合位，而 SDK 自己的 clamp bug 会把它驱动向**张开**。
  这处偏离在 `gripper.py` 而不是 shim 里，见"已知行为"。

其余全部由 `test_mirrors_match_installed_sdk` 对着已安装的 SDK 逐字段比对，
包括字段顺序（dataclass 可以按位置构造）和 `GripperState.is_stale` 这类公开成员。
`test_deliberate_divergences_still_diverge` 是它的镜像面：上面那几组值一旦被谁
"顺手对齐"回 SDK，它会失败。

### 状态新鲜度

SDK 在每个 `GripperState` 上记 `data_age_s`（这批数值来自多久以前的那一帧），
并由此推出 `has_data` 与 `is_stale`（`STALE_AFTER_S`，默认 0.5 s）。这在真机上
有意义：失能的电机不主动发状态帧，`get_state()` 可能返回几秒前的快照或使能前的
默认值 —— `refresh_status()` 就是为这个存在的。

仿真没有 CAN 链路，所以 `MujocoGripper.get_state()` 与 `DryRunGripper.get_state()`
都传 `data_age_s=0.0`：数值是当场算出来的。于是 `is_stale` 恒为 `False`。
`refresh_status()` 保持 SDK 的形状 —— 先 `_check_connected()`，未连接时抛
`NotInitializedError` —— 然后直接返回 `True`，因为没有任何东西需要去问。

## 测试

```bash
python -m pytest tests/ -v
```

分三层：

- **几何与单位** —— 直接读 STL 与 MJCF 数据，绕开 `MujocoGripper`。这层钉的是模型
  本身：网格单位、87.000 mm 开口、全行程无自穿透、网格对齐不变量。
- **行为** —— 公开 API，对照 SDK 语义。
- **物理保真** —— 上面那些反直觉的行为被固化成测试，理由写在 docstring 里，
  以免以后被人"顺手修好"。
- **SDK 对等** —— `TestApiParity` 这一类。`test_matches_installed_sdk` 钉住
  44 个公开成员的快照，`test_mirrors_match_installed_sdk` 把每个镜像的 dataclass、
  枚举和常量类对着已安装的 SDK 逐字段比对，`test_exception_family_is_the_sdk_family`
  检查 7 个异常名解析到的是 SDK 的**类对象**。这一层专门抓上游漂移：SDK 多一个
  字段、多一个错误码、多一个方法，它会响亮地失败，而不是让仿真悄悄缺一块。
- **标定闸** —— `tests/test_calibration.py` 覆盖解析（出厂标定、显式路径，以及两者都没有
  时的拒绝）、发现、校验、来源标记与两个严格换算，对象是一个鸭子类型的 `LiteGrip` 替身，
  可以让它的 `load_calibration()` 复现 SDK 的静默回落。
- **例程命令行契约** —— 同一个文件把例程 04/05 当子进程跑：`--list-calibrations` 是纯查询，
  退出码 0；没有可用标定又没给 `--calibration` 时退出码 1 并打印指引；`--dry-run` 根本
  不需要标定。这里有三个环境细节是必须的：`LITEGRIP_CALIB` 指向用例自己的 home，免得
  开发者自己的标定替它作答；`LITEGRIP_SDK_DIR` 指向一份自造的 SDK 检出；`PYTHONPATH`
  也指向它 —— 因为 `litegrip` 是个**包名**，开发机上装着的那份会替库层的
  `import litegrip` 探测作答、把用例正想否认掉的那份出厂标定递过来（`no_sdk()` 用一份
  import 就抛错的包把它顶掉）。常驻循环都带 `--duration`：不带的话它们会一直跑到 Esc
  或关窗，在测试里就是永远。

测试套件不需要 CAN 接口、不需要硬件、不需要 `litegrip` SDK。需要 SDK 的用例在
SDK 缺席时自行 skip。

推送前**两种配置都要跑** —— 装了 SDK 和没装 SDK 走的是不同代码路径
（`shim → SDK` 与 `shim → _fallback`），而且只有装了的那次能看见漂移：

```bash
# 装了 SDK
python -m pytest tests/ -q

# 没装 —— 把 PYTHONPATH 指向 src，用一个没有 litegrip 的解释器
PYTHONPATH=src python3 -m pytest tests/ -q
```

如果改了接触参数，`test_grasp_force_exceeds_request` 会失败，并把你指向
`litegrip.xml` 里的对照表。两处要一起改。

---

[English](DEVELOPER_GUIDE.md)
