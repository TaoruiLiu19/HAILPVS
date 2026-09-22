"""大件运输弯道通过性检验系统 - 常量与配置。

沿用 LasingSystem2.0 的 config.py 结构：车型字典、软件名/版本、登录信息、
口令、重力常量、模型后缀等。内容全部替换为弯道通过性检验相关数据。
"""

# ===== 软件名称与版本号 =====
APP_NAME = "大件运输弯道通过性检验系统"
APP_VERSION = "V1.0"
WINDOW_TITLE = APP_NAME + APP_VERSION

# 弯道检验方法依据说明
ACCEPTANCE_NOTE = (
    "本系统依据《公路大件运输安全通行评价技术规范》(JTG/T 2213—2023)："
    "采用多刚体运动学仿真 + 扫掠路径分析（Swept Path Analysis），对牵引车-挂车-货物组合"
    "在圆弧弯道与直角弯道（平面交叉转弯）下进行水平面扫掠、垂直净空及横向稳定性校验；"
    "转弯通道宽度计入 0.5m 安全宽度，车货外廓边缘与护栏/障碍物间计入按公路等级确定的"
    "横向预留空间，输出所需最小弯道半径、最小弯道宽度、外摆值等判定参数。"
)

# ===== 判定常量（对标 JTG/T 2213—2023）=====
TURN_SAFE_CLEARANCE = 0.5        # 转弯通道宽度安全宽度(m)，B.1.1-1 / B.1.2-1 中的 0.5
LATERAL_CLEARANCE_HIGH = 0.5     # 高速/一级/二级公路横向预留空间(m)，4.2.1
LATERAL_CLEARANCE_LOW = 0.25     # 三级/四级公路横向预留空间(m)，4.2.1
VERT_CLEARANCE_MARGIN = 0.1      # 垂直净空附加余量(m)——仅用于普通跨线构造物/上跨桥“宜0.1”
INTERSECTION_RESERVE = 0.5       # 平面交叉转弯单侧预留空间(m)，5.2.1 可取 0.5

# ---- 垂直净空分场景阈值（P0-6：只做 H总+0.1 过于简化）----
# 上跨桥/跨线构造物 6.4.1、竖曲线跨线 4.4.2：不宜 <0.10，不应 <0.05
VERT_CROSS_STRUCTURE = {"desirable": 0.10, "minimum": 0.05}
# 隧道 9.2.1
TUNNEL_CROWN_CLEARANCE = 0.20    # 顶部距洞顶(含设备) ≥0.20
TUNNEL_SIDE_CLEARANCE   = 0.50    # 距隧道侧壁(含检修道) ≥0.50
TUNNEL_HIGH_REQ_BOTTOM  = 0.20    # 货物底部高于检修道顶面 ≥0.20
TUNNEL_GROUND_CLEARANCE = 0.20    # 车/货底缘距路面 ≥0.20
# 匝道 6.3.2：外廓边缘与护栏等设施预留，不宜 <0.50，不应 <0.25
RAMP_CLEARANCE          = {"desirable": 0.50, "minimum": 0.25}

# 净空场景划分：'跨线构造物'/'上跨桥'/'斜坡' 走跨线阈值；'隧道' 走隧道阈值；'匝道' 走匝道阈值
VERT_SCENARIO = {
    "跨线构造物": "cross",
    "上跨桥": "cross",
    "隧道": "tunnel",
    "匝道": "ramp",
}

# 公路等级 → 横向预留空间 σ 映射（4.2.1）
HIGHWAY_GRADES = ["高速公路", "一级公路", "二级公路", "三级公路", "四级公路"]
LATERAL_CLEARANCE_BY_GRADE = {
    "高速公路": LATERAL_CLEARANCE_HIGH,
    "一级公路": LATERAL_CLEARANCE_HIGH,
    "二级公路": LATERAL_CLEARANCE_HIGH,
    "三级公路": LATERAL_CLEARANCE_LOW,
    "四级公路": LATERAL_CLEARANCE_LOW,
}
DEFAULT_SUPERELEVATION = 0.02    # 圆曲线超高默认值 i_h

# ===== 登录信息（支持多账号）=====
LOGIN_USERS = {
    "中远海运": "0000",
    "admin": "admin",
}
LOGIN_USER = "中远海运"
LOGIN_PWD = "0000"

# ===== Word 导出口令 =====
EXPORT_PWD = "565656"

# ===== 重力加速度 m/s² =====
g = 9.81

# ===== 模型保存后缀 =====
MODEL_SUFFIX = ".model"

# ===== 挂车类型配置 =====
TRAILER_TYPES = ["重型机械挂车", "液压轴线挂车"]
# 各挂车类型每轴轮胎数（用于绘图层示意）
TRAILER_TIRE_PER_AXLE = {
    "重型机械挂车": 4,
    "液压轴线挂车": 8,
}
# 轴线纵列数可选
TRAILER_COLUMNS = ["单纵列", "2纵列", "3纵列", "4纵列"]

# ===== 转向类型 =====
STEERING_TYPES = ["全轮强制转向", "自控随动转向", "锁死轴"]

# ===== 道路类型 =====
ROAD_TYPES = ["圆弧弯道", "直角弯道"]

# ===== 衬垫摩擦系数（弯道检验中用于离心抗滑辅助判断）=====
PAD_FRICTION = {
    "干燥的木板": 0.4,
    "干燥的橡胶垫": 0.7,
    "潮湿的木板": 0.3,
    "潮湿的橡胶垫": 0.4,
}

# ===== 仿真参数（运动学引擎）=====
SIM_DT = 0.05          # 仿真时间步长 s
SIM_DRAW_STEP = 40     # 绘图抽帧步数（每 N 步绘制一辆车轮廓）
SIM_APPROACH_SPEED = 2.0  # 直线段行驶速度 m/s（低速，用于仿真推进）
SIM_TURN_SPEED = 1.5      # 转弯段速度 m/s

# ===== 绘图常量 =====
PLOT_DPI = 300

# ===== 结果状态徽章颜色 =====
STATUS_STYLE = {
    "none": ("未运算", "#d9d9d9", "#000000"),
    "ok": ("通过", "#2f855a", "#ffffff"),
    "ng": ("不通过", "#c53030", "#ffffff"),
}