# 液压轴线挂车：B.1.2 闭式解真正接入 ＋ 多轴转向运动学与绘制 —— 修改方案

> 针对评审与实测暴露的缺口：液压轴线挂车当前只把 B.1.2 闭式解当作**圆弧弯道扫空宽度兜底**，
> `steering_type` / `locked_axles` 从未参与计算，θ 被固定钳制在 45°，**转弯通道宽度（B.1.2-1）未实现**，
> 两车型（重型机械挂车 / 液压轴线挂车）的**行驶轨迹是同一条铰接尾随模型生成的、看不出行为区别**，
> 弯道图上整车被画成**单一刚体矩形**，看不到"车板多轴转向"的几何与工程效果。
>
> 本文档依据 **《公路大件运输安全通行评价技术规范》** 附录 B.1.1/B.1.2 与 **`肖-公路运输-通过能力/第六章 道路通过能力`**
> 两套参考文献修订，给出**两阶段、可独立交付**的改造方案。核心目标是让**重型机械挂车（被动）与液压轴线挂车（主动/全轮）在驾驶行为上真正区分**。
>
> 编制日期：2026-09-23　·　基准版本：V1.2.1（`HAILPVS/`）　·　目标版本：V1.3

---

## 0. 读者决策记录（已与需求方确认）

| # | 决策点 | 依据 | 结论 |
| --- | --- | --- | --- |
| D1 | `locked_axles` 如何处理？ | 规范 B.1.2 明示锁死轴=**中间不转轴**，`d`=首末轴距，**中间锁死不改变数值** | **双口径并存**：默认严格守规范（中间锁死不参与计算，仅校验+展示）；仅当首/末轴被锁死时走**规范外推**，`d_A=L(last_active−first_active)` 并告警 |
| D2 | θ 放开到什么程度？ | 规范只给"第一轴线内侧轮胎转向角"，**未设 45° 上限**；45° 是第六章典型值（表6-1）、由挂车结构决定（表1-2） | **默认 45°、界面可手动调整**：`θ=min(axle_max_steering_angle_deg, 工程上限)`，不再硬限 45°，下端 1° 防除零 |
| D3 | B.1.2-4/-5 的 `D总` 怎么取？ | 规范 OCR"动力机组长度"存疑；第六章用"挂车总长"；耦合段仅绘制(D8)、不进判定 | **`D总 = trailer.length`（车板长），不随耦合段变化**；机械车不变 |
| D4 | 是否补"转弯通道宽度"？ | B.1.2-1 的 `W=R外−R内+0.5 ≤ 横向路面宽` 与扫空 `W=d` 是**两套概念** | **补齐** `R内/R外`、`W_turn` 指标并接入判定 |
| D5 | 液压车的行驶轨迹要不要和机械车区分？ | 行为本质：机械=被动铰接尾随（B.1.1）、液压=主动全轮绕瞬心（B.1.2） | **轨迹按 B.1.2 瞬心生成**：液压车行驶/扫掠用瞬心圆周法，机械车保持铰接尾随——达成两车型行为区别 |
| D6 | 是否纳入"原地回转"？ | 工程保守；原地回转需 θ→近90°、tan90° 奇异处理，属强外推 | **暂不纳入**：θ 留工程上限（默认 60°，field 可配，≤89°），不做 base→0 原地回转 |
| D7 | 车长语义 | `trailer.length` 原语义含糊 | **`length`=车板长（不含牛颈/连接杆）**；与牵引车的连接关系（牛颈/连接杆）由 D8 定义，**不进入 B.1.2 判定(D3)**；机械车不变 |
| D8 | 连接方式与连接部分长度 | 需显式控制液压车挂车相对牵引车的画法与间距 | **`TrailerParams` 新增** `coupling_type∈{"鹅颈","连接杆"}` + `coupling_length`(挂车车板前边沿→牵引车后边沿距离,m) —— **仅影响绘图，不参与任何计算** |

> D2/D4/D5/D7/D8 为本方案重点；D1 双口径避免"为接入而偏离规范"；D3=`D总=车板长`(不随耦合段变)；D8 仅绘图、不参与计算；D6 保守不扩能力边界。

---

## 1. 现状诊断（对照代码逐条）

| 位置 | 现状代码 | 问题 |
| --- | --- | --- |
| `kinematics.py` | `TractorTrailerKinematics` 单一铰接尾随模型，机械/液压共用；`hydraulic_turn_radii` 只输出扫空兜底 | **两车型轨迹同源**：图上机械与液压外轮廓几乎一样，无行为区别（D5 缺位）；θ 口径错、无 `W_turn` |
| `passability.py::run_arc_scene` | `W_swept = max(仿真, B.1.2)`；`_is_hydraulic` 仅加 `B1_2_*` 指标 | ①转弯通道宽度（B.1.2-1）缺失（D4）；②两法取大作为结论口径混用 |
| `datamodel.py::TrailerParams` | `steering_type`、`locked_axles` 无计算使用；无挂车轴线最大转向角字段 | D1/D2 所需字段/接口未接 |
| `sweep_view.py::draw_sweep_view` L227-235 | 只画 `trailer_geom`/`cargo_geom` 轮廓 | 无轴轮细节，看不到车板转向 |

**根因**：算法以"单刚体铰接尾随（附录 C 仿真）"为准，B.1.1（机械）与 B.1.2（液压）两套模型只通过 `max()` 兜底相连，既没落到轨迹，也没落到完整判定；液压车全轮转向的**几何本体（绕瞬心的刚体圆周运动）从未进入轨迹生成**。

---

## 2. 规范依据与两车型模型的界定

### 2.1 两车型行为差异（本方案核心）

| 车型 | 转向方式 | 转弯半径决定因素 | 轨迹/扫掠来源 |
| --- | --- | --- | --- |
| 重型机械挂车 | 被动：牵引车拖拽，挂车后轮无助力转向 | 牵引车前轮转向角 + 铰接几何（B.1.1 模型） | **铰接尾随积分**（现状，符合 B.1.1），半径较大 |
| 液压轴线挂车 | 主动/液压：全轮强制转向，各轮绕同一瞬时中心 | 挂车结构 + `θ`（B.1.2，`base=d/(2tanθ)`），半径小、更灵活 | **绕瞬时中心 O 的刚体圆周**（本方案新增，B.1.2） |

- 规范 B.1.2 条文说明 3627-3628："转弯半径由所牵引液压悬挂挂车决定，牵引车因车身较短基本不增加整体转弯半径"——即液压车半径**不由牵引车决定**，由挂车+θ 决定，是 D5 的直接依据；
- 第六章"定点转向原理（各轮绕同一瞬时中心纯滚动）+ 稳定转向阶段"给出液压车刚体圆周扫掠与各轴转向角的理论基础。

### 2.2 液压车 B.1.2 公式

设 $L_{\text{轮挂}}$ 为相邻轴线间距（`axle_spacing`）、$N_a$ 为轴数、$D_{\text{总}}$ 为挂车总长（`trailer.length`，D3）、$d$ 为**首末轴距 $=L_{\text{轮挂}}(N_a-1)$**、$\theta$ 为**第一轴线内侧轮胎转向角度**（D2）。

**A. 转弯通道宽度（轮胎通道，B.1.2-1，判定"≤ 横向路面宽"）**：
$$
W_{\text{turn}} = R_{\text{外}} - R_{\text{内}} + 0.5 \qquad\qquad\qquad \texttt{(B.1.2-1)}
$$
⚠ 规范 OCR 中 B.1.2-2/-3 两式残缺，无法唯一还原差分形式；工程锚点取轮胎最外/最内轨迹差 $\approx d$，
$$
W_{\text{turn}} \approx d + 0.5
$$
作为 B.1.2-1 工程锚点接入判定，公式待原 PDF 复核后替换。

**B. 车货总体扫空空间宽度（B.1.2-4/-5/-6，判定"扫空范围内无障碍"）**：
$$
base = \frac{L_{\text{轮挂}}(N_a-1)}{2\tan\theta}
\qquad
R_{\min} = base + \frac{D_{\text{总}} - d}{2} \tag{B.1.2-4}
\qquad
R_{\max} = base + \frac{D_{\text{总}} + d}{2} \tag{B.1.2-5}
$$
$$
W_{\text{swept}} = R_{\max} - R_{\min} = d \tag{B.1.2-6}
$$

---

## 3. 阶段一：B.1.2 闭式解真正接入（判定口径改造）

### 3.1 数据模型 —— `app/core/datamodel.py`

```python
class TrailerParams(BaseModel):
    # ...现有字段...
    steering_type: str = "全轮强制转向"
    locked_axles: list[int] = Field(
        default_factory=list,
        description="锁死轴列表(1..Na,1=最前轴)。规范基准：仅中间不转轴;若含首/末轴，走规范外推 d 并告警(D1)")
    axle_max_steering_angle_deg: float = Field(
        45.0, gt=0, le=89,
        description="液压轴线每轴最大转向角(°,D2 θ 来源,默认45=第六章典型值,界面可手动调整;D6 上限须<90,不承诺原地回转)")
    coupling_type: str = "鹅颈"        # D8: 连接方式,取值 "鹅颈"|"连接杆";仅绘图定位
    coupling_length: float = Field(
        0.0, ge=0,
        description="连接部分长度(m):挂车车板前边沿→牵引车后边沿距离;仅绘图,不参与计算(D8)")
```

- `axle_max_steering_angle_deg`：θ 来源，替代错误引用牵引车前轮角（D2/D6）；
- `locked_axles`：默认严格守规范（中间锁死不影响 `d`），仅含首/末轴时外推（D1）；
- `coupling_type` / `coupling_length`：**仅阶段二绘图**——决定液压车挂车相对牵引车的画法与间距，`coupling_length`=车板前边沿→牵引车后边沿距离；两者**不进入任何 B.1.2 判定**（D8）。

### 3.2 闭式解实现 —— `app/core/kinematics.py::hydraulic_turn_radii`

```python
def hydraulic_turn_radii(bundle, turn_angle_theta=None):
    tl = bundle.trailer
    na = max(int(tl.axle_count), 2)
    L  = tl.axle_spacing                          # L轮挂
    # 判定口径(D3/D8): length=车板长; 牛颈/连接杆耦合段仅绘图、不进 B.1.2 判定
    D_total  = max(float(tl.length), L * (na - 1))  # D3: 液压车挂车总长(车板长)

    locked = sorted({int(a) for a in tl.locked_axles if 1 <= int(a) <= na})
    first, last, spec_mode = 1, na, True
    if (1 in locked) or (na in locked):           # D1 外推：active 首末
        active = [i for i in range(1, na + 1) if i not in locked]
        first, last = min(active), max(active)
        spec_mode = False
    d = L * (last - first)
    if d <= 0: d = L * (na - 1); spec_mode = True

    theta_max_deg = min(float(tl.axle_max_steering_angle_deg),
                        AXLE_STEER_ABS_LIMIT_DEG)                # D2/D6
    theta = (math.radians(turn_angle_theta) if turn_angle_theta is not None
             else math.radians(theta_max_deg))
    theta = max(math.radians(MIN_TURN_ANGLE_DEG),
                min(theta, math.radians(AXLE_STEER_ABS_LIMIT_DEG)))

    base     = d / (2.0 * math.tan(theta))
    r_min    = base + (D_total - d) / 2.0         # B.1.2-4
    r_max    = base + (D_total + d) / 2.0         # B.1.2-5
    w_swept  = r_max - r_min                      # B.1.2-6 = d
    w_turn   = d + 0.5                            # B.1.2-1 工程锚点
    # 瞬心几何（D5 轨迹层用）：绕轴组中心横向偏距 rho≈base，纵向居中
    wagon_g  = D_total / 2.0
    warnings_ = []
    if not spec_mode:
        warnings_.append(f"首/末轴锁死({locked}):d 按 active 首末={d}m 外推,属规范外推、保守")
    if len([i for i in range(1,na+1) if i not in locked]) < 2:
        warnings_.append("active 转向轴<2,回退规范基准")
    return {"R_in": r_min, "R_out": r_max,
            "W_swept": w_swept, "W_turn": w_turn,
            "base": base, "theta_deg": round(math.degrees(theta), 2),
            "d": d, "D_total": D_total, "spec_mode": spec_mode,
            "locked": locked, "sweep_center_dist": base,  # 瞬心横向偏距(D5)
            "warnings": warnings_}
```

### 3.3 新增常量 —— `config.py`

```python
MIN_TURN_ANGLE_DEG = 1.0                # 防除零/无穷
AXLE_STEER_ABS_LIMIT_DEG = 60.0         # D2/D6 工程上限(可配,≤89);D6 不承诺原地回转
TURN_SAFE_CLEARANCE_B12 = 0.5           # B.1.2-1 安全宽度 a=0.5m
HYDRAULIC_AXLE_DEFAULT_STEER_DEG = 45.0 # 默认(第六章典型值)，界面可手动调整(D2)
```

### 3.4 判定联动 —— `app/core/passability.py`

- `run_arc_scene`：`_is_hydraulic` 时 `b12 = hydraulic_turn_radii(bundle)`：
  - 液压车型判定口径 = **B.1.2**：`W_swept` 判"扫空无障碍"，`W_turn ≤ 横向路面宽`（B.1.2-1）判转弯通道；仿真降级为附录 C 附注，不再无条件 `max`；
  - `b12["warnings"]` 非空时结论文本追加提示；
- `_verify_arc_b` / `_attach_common_checks`：`W_turn` 并入 `width_ok`（复用 `transverse_space_width`）；
- `run_right_angle_scene`：阶段一保持普通铰接，阶段二再按 D5 复用瞬心几何、判定 5.2.1 不动。

### 3.5 耦合段：连接方式与绘制布局（D8，仅液压、仅绘图）

**语义约定**：液压轴线车 `trailer.length` = **车板（承载板）长度，不含牛颈/连接杆**；判定 `D_total = trailer.length`（D3）。牛颈/连接杆由 `coupling_type` + `coupling_length` 描述，**仅影响绘图**，**不进入任何 B.1.2 计算**（半径、扫空、`W_turn` 均不含）。

**字段（`TrailerParams`）**：
- `coupling_type ∈ {"鹅颈", "连接杆"}`：连接方式，默认"牛颈"；
- `coupling_length`：连接部分长度 = **挂车车板前边沿 → 牵引车后边沿** 距离 (m)，默认 0。

**绘制定位**（阶段二，相对牵引车几何）：
- 牵引车后边沿 = 后轴中心 + `tractor.length/2`·牵引车航向（牵引车矩形后端）；
- 挂车车板前端定位 = 牵引车后边沿 + `coupling_length`·挂车航向；
- 车板自前端向后延伸 `length` → 车尾；
- 轴位定位（相对铰接点 P，仍用于画轴/货物，独立于耦合字段）：
  - 第一轴：`P + front_overhang`；第 i 轴：`P + front_overhang + (i−1)·axle_spacing`；车尾：`+ rear_overhang`。

**画法（`coupling_type` 二态）**：

| `coupling_type` | 图面表现 | 说明 |
| --- | --- | --- |
| 鹅颈 | 车板前端与牵引车后部间画**弧形鹅颈**，车板可轻微覆盖牵引车后端（`coupling_length` 取小/0 时贴合鞍座） | 半挂样式 |
| 连接杆 | 牵引车后边沿→车板前端画**直连接杆**，车板前端距牵引车后沿 `coupling_length`，二者**显式留空隙、不重叠** | 全挂样式 |

```python
def hydraulic_coupling_draw_anchor(bundle, tractor_rear_x, tractor_rear_y, psi):
    """D8: 仅绘图。返回挂车车板前端锚点(相对牵引车后沿偏移 coupling_length)。
    不参与 D_total/R/W 计算。"""
    tl = bundle.trailer
    off = float(tl.coupling_length or 0.0)
    return (tractor_rear_x + off * math.cos(psi),
            tractor_rear_y + off * math.sin(psi))
```

### 3.6 阶段一关键回归点
1. `locked_axles=[]`/纯中间锁定 → `d=L(Na−1)`、扫空/半径与 V1.2.1 **逐位一致**；
2. θ 不再引用 `tractor.max_steering_angle_deg`，且受工程上限（D6，默认 60°）；
3. 新增 `W_turn` 判定接入后不破坏原扫空结论；
4. `coupling_type`/`coupling_length`（D8）**不改动任何判定数值**（`D_total` 仍为车板长）。

---

## 4. 阶段二：两车型行为区分 —— 液压车瞬心轨迹 ＋ 多轴转向绘制

**定位升级（来自 D5）**：阶段二不再是"仅绘制转向轮"，而是**让液压车的行驶轨迹/扫掠按 B.1.2 瞬心几何生成**，与机械车（铰接尾随）形成真正的行为区别；同时绘制各轴偏转车板。

### 4.1 液压车瞬心轨迹 —— `hydraulic_arc_trajectory`（`kinematics.py` 新增）

液压车在弯道内视为**绕瞬时中心 O 的刚体圆周运动**（第六章定点转向原理）：

```python
def hydraulic_arc_trajectory(bundle, b12, max_speed_kmh, turn_steps, center=(0,0)):
    """液压车专属轨迹：以 R_in(瞬心横向偏距≈base)为半径，整车刚体绕 O 做圆周，
    每个集点同时算各轴 δ_i 与刚体包络点，得到 [(x,y,geom,trailer,axi,axle_geoms), ...]。
    机械车不调用本函数，仍走 TractorTrailerKinematics 铰接尾随。
    """
    tl = bundle.trailer
    rho = b12["sweep_center_dist"]            # 瞬心横向偏距≈base(D5)
    omega = (max_speed_kmh / 3.6) / rho        # 角速度
    for k in range(turn_steps):
        psi = k * omega * dt                   # 刚体转角(绕瞬心)
        # 车板刚体位姿 = 首尾随绕瞬心转 psi
        #   各轴：先由瞬心求该轴理论 δ_i，锁死轴置0、非锁死轴受±axle_max_steering 钳制
        #   包络：货物/车板最外内点绕瞬心的轨迹点集
        ...

def axle_steering_geoms(tail_x, tail_y, psi, trailer, rho, want=("first","last","midlock"), ...):
    """局部车板坐标(纵轴e、横轴n、首 active 轴中点为原点，瞬心弯内侧 -n 向)：
        a_i = x_i - x_first ; φ_i = atan2(-rho, a_i) ; δ_i = 90° + φ_i
    锁死轴 δ=0；非锁死轴受 ±axle_max_steering_angle_deg 钳制。
    返回 [(轴序号, 轴中点, δ_i, 轮距半宽, 是否锁死), ...]。

    绘制选轴(want)：默认只出 首轴 + 末轴 + 中间闭锁轴——
        want 由轴数决定：奇数 N_a → 含中间闭锁轴((N_a+1)//2)；
                        偶数 N_a → 无闭锁轴，只出首末（中间两轴转角最小、非锁死）。
    """
```

要点：

- **轨迹来源不同**：机械车＝铰接尾随积分（半径由牵引车决定，较大）；液压车＝绕瞬心 O 圆周（半径由挂车结构+θ 决定，较小且 `base` 可控）——**行为区别落到轨迹与扫掠**；
- `axle_max_steering_angle_deg` 越大，`base=d/(2tanθ)` 越小，液压车转弯越"灵"（但 D6 上限<90°，不做原地回转）；
- 各帧同时携带 `axle_geoms`，供绘制车板转向。

### 4.2 稳态一致性说明（不破坏判定）
液压车刚体绕瞬心的包络扫掠，与"同半径下的单刚体铰接（假设能绕那么小半径）"外轮廓一致；但**半径来源不同**（B.1.2 vs 铰接），故液压车轨迹/扫掠是专属的。`measure_steady_turn` 仅用于机械车及整车横向稳定核对的径向量测。

### 4.3 帧结构扩展 —— `build_trajectory_geoms`
- 机械车返回旧 5 元组 `(x,y,geom,trailer_geom,cargo_geom)`（向后兼容）；
- 液压车返回 6 元组，追加 `axle_geoms`（≥任意选中抽帧）。

### 4.4 工程图绘制 —— `app/rendering/sweep_view.py`

**图面原则**：**重型机械挂车画面完全不变**（沿用现状 `trailer_geom`/`cargo_geom`，不画轴线层）；仅液压轴线挂车追加轴线层。

**液压车轴线绘制规则（参照第六章图式）**：

| 轴数 $N_a$ | 中间闭锁轴 | 图上必须画出的轴轮 | 各轴车轮δ |
| --- | --- | --- | --- |
| 奇数 | 中间第 $\frac{N_a+1}{2}$ 轴为闭锁轴 | 首轴轮 · 中间闭锁轴轮 · 末轴轮（至少） | 首/末轴按瞬心转向（$\delta$ 最大且方向朝瞬时中心）；闭锁轴 $\delta=0$ |
| 偶数 | 无闭锁轴（全部可转，中间两根转角最小） | 首轴轮 · 末轴轮（至少） | 首/末轴按瞬心转向；无 $\delta=0$ 的闭锁轴可画 |

- **转向角度与方向正确**：首/末转向轴的 $\delta$ 由瞬心几何精确给出 `δ_i = 90° + atan2(−rho, a_i)`（见 §4.1），方向**朝瞬时转动中心（弯内）偏转**；闭锁轴车轮保持与车板纵轴平行（$\delta=0$）；
- **每轴轮数**：液压轴线按 `TRAILER_TIRE_PER_AXLE`（每轴轮胎数）描绘轮胎实体（可伸出车板边缘），机械车仍按现状矩形轮廓；
- 中间转向轴（未画出的）在图上可省略或轻量化示意，用户要求"至少"首末 + 中间闭锁三组即可。

```python
for _x,_y,geom,trailer_geom,cargo_geom,*axle in res.vehicle_frames:
    _plot_ring(ax, list(trailer_geom.exterior.coords), VEHICLE_COLOR, 1.1)  # 机械/液压通用
    if axle and axle[0] is not None:                       # 仅液压车带轴线层
        for (cx,cy,dd,halfl,locked) in axle[0]:
            _plot_axle(ax, cx, cy, dd, halfl, locked)      # 轴段 + 按 δ 偏转轮胎
    _plot_ring(ax, list(cargo_geom.exterior.coords), CARGO_COLOR, 1.2)
```

其中 `axle[0]` 由 `axle_steering_geoms` 按 §4.4 的选轴规则生成（奇数轴含中间闭锁轴、偶数轴不含），`dd` 为该轴当前转向角。

图面效果：液压车首尾轴车轮朝转弯中心正确偏转、奇数轴时中间闭锁轴笔直，整体轨迹小于机械车同参数轨迹；机械车画面与现状逐点一致——两车型一眼可分。

### 4.5 直角弯与圆弧弯共用
- 阶段二在 `_simulate_right_angle` / `_build_arc_display` 复用 `axle_steering_geoms` 与瞬心法，判定 5.2.1 不动；
- 直角弯液压车可选输出 `W_turn` 参照指标（B.1.2-1 仅圆弧弯道正式判定）。

---

## 5. 报告 / 指标 / 界面联动

| 载体 | 改动 |
| --- | --- |
| `res.metrics`（圆弧） | 新增 `B1_2_W_turn`、`B1_2_spec_mode`、`B1_2_warnings`、`trajectory_source`（= "B.1.2瞬心" / "铰接尾随"） |
| `res.verdict_lines` | 追加 `θ 来源=挂车轴线结构角/转弯通道宽度 W_turn/轨迹来源/是否规范外推告警` |
| 工程图主线 | 液压车追加轴线/转向轮胎图元（首末转向轴轮 + 奇数轴时中间闭锁轴轮，δ 朝瞬心）；图例注明两车型轨迹来源差异 |
| UI/参数面板 | 液压车新增 `θ`（首轴最大转向角）手动输入控件，默认45°，范围(1°,89°)；新增 `连接方式`(鹅颈/连接杆)下拉 + `连接部分长度`(m)输入，仅影响绘图；**机械车画面不变** |
| `计算方法说明.md` | §3.6 由"预留字段"改为"参与判定/绘制"；§6 补 `W_turn`(B.1.2-1)、两套概念、D3/D2/D5 决策 |
| README / 合规声明 | 注明：首/末轴锁死为规范外推；液压车轨迹按 B.1.2 瞬心生成；B.1.2-2/-3 OCR 残缺待复核；D6 不承诺原地回转 |

---

## 6. 测试更新 —— `tests/test_passability.py`

| 用例 | 断言 |
| --- | --- |
| `test_b12_theta_src` | θ 取自 `trailer.axle_max_steering_angle_deg`，**不读** `tractor.max_steering_angle_deg`（D2） |
| `test_b12_theta_upperbound` | θ 被钳到 `AXLE_STEER_ABS_LIMIT_DEG`（默认60），**不含原地回转**（D6） |
| `test_b12_theta_min` | θ<1° → 钳到 `MIN_TURN_ANGLE_DEG`，无除零 |
| `test_b12_locked_middle_spec` | `[2,3]` 纯中间锁死 → `spec_mode=True`、`d=L(Na−1)`、与 V1.2.1 一致 |
| `test_b12_locked_ends_extrap` | `[1]`/`[Na]` → `spec_mode=False`、`d` 缩短、`warnings` 非空 |
| `test_b12_D_total` | `D_total=length`（车板长），与 coupling 字段无关（D3/D8） |
| `test_hydra_coupling_draw` | 连接杆：车板前端=牵引车后沿+`coupling_length`、不重叠；鹅颈：弧形贴合、可覆盖；判定 `D_total/R/W` 不受 coupling 影响（D8） |
| `test_hydra_coupling_intensity` | 机械车画面不变；`coupling_length` 变化仅改车板前端锚点、不改任何指标（D7/D8 边界） |
| `test_b12_W_turn` | `W_turn=d+0.5` 且 `run_arc_scene` 输出并参与"≤路面宽"（D4） |
| `test_b12_active_lt2` | 可转向轴<2 → 回退规范基准+告警 |
| **`test_two_vehicle_trajectory_diff`** | **同一几何下液压车（瞬心法）最大外轨迹半径 < 机械车（铰接尾随）对应值，且两者扫掠不同**（D5 行为区别） |
| `test_axle_steering_geoms` | 中间锁死轴 `δ=0`；非锁死轴 δ 方向一致且坐标在车板内 |
| `test_axle_draw_odd_axles` | 奇数 `N_a` → `axle_steering_geoms` 含首轴+末轴+中间闭锁轴（δ=0）三组 |
| `test_axle_draw_even_axles` | 偶数 `N_a` → 只含首轴+末轴，**无**闭锁轴（δ=0 仅当用户显式锁死） |
| `test_arc_b12_indicator` | 指标含 `B1_2_W_turn/_d/_D_total/_spec_mode/trajectory_source` |

> 改动前 `pytest tests/` 存基线；阶段一提交全绿；阶段二提交且 `test_two_vehicle_trajectory_diff` 通过。

---

## 7. 实施顺序与验收

| 阶段 | 内容 | 验收 |
| --- | --- | --- |
| **S1（判定口径）** | ①datamodel 加 `axle_max_steering_angle_deg` ②config 常量 ③`hydraulic_turn_radii` 双口径+θ放开+`W_turn` ④passability 接入 B.1.2-1 ⑤指标/结论文本 | `pytest` 全绿；纯中间锁准时数值不变；θ 不引用牵引车前轮角；`W_turn` 出现 |
| **S2（行为区分+可视化）** | ①`hydraulic_arc_trajectory`+`axle_steering_geoms`（瞬心法）②`build_trajectory_geoms` 帧扩展（兼容旧5元组）③`sweep_view` 轴线层 ④直角/圆弧帧按车型分支 ⑤文档/README | `test_two_vehicle_trajectory_diff` 通过；液压车轨迹<机械车并可见各轴转向；机械车画面不回归 |

**可选增强（S3，不阻塞主线）**：第六章"自走式挂车" $R_{\min}=\sqrt{(l/\tan\alpha_{\max}+b)^2+l^2}+\tfrac{B-b}{2}$ 对 `B1_2_Rin` 做交叉校验输出（待原书第1章变量定义核实）。

---

## 8. 风险、边界与回退

1. **θ 放开的合规风险（D2/D6）**：θ 越大越"灵"，数据不可靠会低估通过难度；接口强制给出 `axle_max_steering_angle_deg`（缺省45，保守），工程上限默认60°（≤89°），**不承诺原地回转**，`MIN_TURN_ANGLE_DEG=1°` 防除零。
2. **首/末轴锁死为规范外推（D1）**：双口径确保默认严格守规范，仅显式首末锁死时外推并告警。
3. **B.1.2-2/-3 OCR 残缺（D4）**：`R外−R内≈d` 是工程锚点，**须以原 PDF 复核**后方替换，复核前不写为规范原文。
4. **混用边界**：阶段一把"B.1.2 与仿真 `max()`"改为"车型口径优先"，`计算方法说明` §6 同步更新。
5. **帧扩展兼容**：用"机械 5 元组 / 液压 6 元组"减少改动面，所有解包处（`polylist_to_multipoly`、`sweep_view`、`report_assembler`、`_split_swept_checks`）核对。
6. **原地回转边界（D6）**：θ 上限为非 90°，故本次**不宣称**液压车具备原地回转能力；后续若要支持需处理 tan90° 奇异与瞬心退化（车在车体内转）。文档明确此为能力边界、非缺陷。
7. **回退**：阶段一单 commit 且不触碰 5.2.1/4.3.1 主流程，异常可 `git revert` 恢复；阶段二轴线层独立开关可整层关闭回旧图面。
8. **`length` 语义与耦合绘制（D7/D8 边界）**：液压车 `length`=车板长，牛颈/连接杆字段仅绘图、不入判定；图面上挂车与牵引车间距是视觉性、与判定所用车板长**解耦**——界面与 `计算方法说明` 须明确标注，避免误读为判定含连接段；若后续要统一口径再单列改造。

---

## 9. 与两份参考文档的对照清单

| 本文档内容 | 依据出处 |
| --- | --- |
| 两车型行为区分（机械=铰接被动/液压=瞬心主动），液压半径由挂车决定 | 规范 §B.1.2 条文说明 3627-3633；第六章 §一 |
| `W=R外−R内+0.5 ≤ 横向路面宽`（B.1.2-1） | 规范 §B.1.2-1 |
| `Rmin/Rmax=base±(D总−d)/2`、`W=Rmax−Rmin=d`、`base=d/(2tanθ)` | 规范 §B.1.2-4/-5/-6 |
| θ=第一轴线内侧轮胎转向角；奇数轴中间1根为闭锁轴、偶数轴中间2根转向（锁死不画） | 规范 §B.1.2 条文说明 3631-3633；第六章图式 |
| 45° 仅为典型值、α_max 由挂车结构决定（表1-2、表6-1） | 第六章 §二-1 |
| 各轮绕同一瞬时中心纯滚动（定点转向） → 瞬心圆周轨迹 | 第六章 §一、§二-2/3 |
| 稳定转向阶段通道最大、最省校核 | 第六章 §一-末、§二 |
| 自走式挂车最小转弯半径闭式（S3 交叉校验） | 第六章 §二-1 |
| D3/D7/D8 取 `D总=车板长`（液压），耦合段仅绘制 | 第六章用"挂车总长"；挂车—牵引车连接（牛颈/牵引杆）按第六章半/全挂定义及车辆总长构成 |

---

_本文档与 `计算方法说明.md` §3.6/§6、`代码修改方案.md` §4.2 相互衔接；公式最终以原 PDF《公路大件运输安全通行评价技术规范》B.1.2 复核口径为准，OCR 残缺处在复核前标注为工程锚点。_