"""输入数据模型 - Pydantic 严格约束。

定义牵引车/挂车/货物/道路四组输入，界面层采集后据此构建仿真输入。
所有字段带默认值，便于模型校验与缺省推断。
"""
from __future__ import annotations
from pydantic import BaseModel, Field, model_validator


class CargoParams(BaseModel):
    """装载货物参数。"""
    weight_t: float = Field(60.0, gt=0, description="货物重量(t)，仅存档展示")
    length: float = Field(gt=0, description="货物长度(m)")
    width: float = Field(gt=0, description="货物宽度(m)")
    height: float = Field(gt=0, description="货物高度(m)")
    cg_height: float = Field(0.0, ge=0, description="重心高度(m)")
    cg_offset_x: float = 0.0
    cg_offset_y: float = 0.0
    is_overhang: bool = False          # 货物是否超出车板边缘(决定包络取货物还是车板)
    cargo_bearing: str = "干燥的木板"  # 衬垫
    rigging_swl_t: float = Field(0.0, ge=0, description="单根绑扎索具SWL(t)")


class TrailerParams(BaseModel):
    """承载挂车参数。"""
    trailer_type: str = "重型机械挂车"
    column_type: str = "单纵列"
    length: float = Field(gt=0, description="挂车长度(m)")
    width: float = Field(gt=0, description="挂车宽度(m)")
    deck_height: float = Field(gt=0, description="车板上表面离地高度(m)")
    axle_count: int = Field(gt=0, description="轴数")
    axle_spacing: float = Field(1.5, gt=0, description="轴距(m)")
    track_width: float = Field(gt=0, description="轮距(m)")
    front_overhang: float = Field(0.0, ge=0, description="前悬:铰接点→第一轴(m)")
    rear_overhang: float = Field(0.0, ge=0, description="后悬:最后一轴→车尾(m)")
    steering_type: str = "全轮强制转向"
    locked_axles: list[int] = Field(
        default_factory=list,
        description="锁死轴列表(1..Na,1=最前轴)。规范基准:仅中间不转轴、d=L(Na−1)不变;"
                    "若含首/末轴则走规范外推 d=L(last_active−first_active)并告警(D1)")
    axle_max_steering_angle_deg: float = Field(
        45.0, gt=0, le=89,
        description="液压轴线每轴最大转向角(°,D2 θ 来源,默认45=第六章典型值;"
                    "D6 工程上限须<90°,不承诺原地回转)")
    coupling_type: str = Field(
        "鹅颈",
        description='连接方式:仅绘图定位,取值 "鹅颈"|"连接杆";不参与任何判定(D8)')
    coupling_length: float = Field(
        0.0, ge=0,
        description="连接部分长度(m):挂车车板前边沿→牵引车后边沿距离;仅绘图,不参与计算(D8)")
    power_unit_length: float = Field(
        0.0, ge=0,
        description="动力机组长度 C(m),B.1.2-5 横项 +C;界面未给时默认 0(退化为无 C)")


class TractorParams(BaseModel):
    """牵引车参数。"""
    wheelbase: float = Field(gt=0, description="轴距(m)")
    max_steering_angle_deg: float = Field(gt=0, le=90, description="最大转向角(°)")
    length: float = Field(8.0, gt=0, description="牵引车长度(m)，绘图用")
    width: float = Field(gt=0, description="牵引车宽度(m)")
    height: float = Field(0.0, ge=0)
    fifth_wheel_pos: float = Field(0.0, description="鞍座/牵引销→后轴距离(m),负数在前悬外")
    drawbar_length: float = Field(0.0, ge=0, description="牵引杆长度(m)，全挂车用")


class ArcCorner(BaseModel):
    """圆弧弯道：R_min = 最大扫空半径。"""
    outer_radius: float = Field(gt=0, description="道路外半径(m)，内半径 = 外半径 − 车道宽")
    lane_width: float = Field(gt=0, description="车道宽度(m)")


class RightAngleCorner(BaseModel):
    """直角弯道。"""
    entry_width: float = Field(gt=0, description="入口宽(m)")
    exit_width: float = Field(gt=0, description="出口宽(m)")
    corner_radius: float = Field(0.0, ge=0, description="弯道内半径(m)")
    intersection_angle_deg: float = Field(90.0, gt=0, lt=180)
    # 5.2.1 左转专用可通行宽度。缺省时回退到入口/出口宽度（界面未显式给定时）。
    W_section: float | None = Field(
        None, ge=0, description="左转时平面交叉物理区可通行宽度(m)，5.2.1-3")
    W_lturn: float | None = Field(
        None, ge=0, description="平面交叉左转处可通行宽度(m)，5.2.1-4")


class RoadParams(BaseModel):
    """道路/弯道参数。"""
    road_type: str = "圆弧弯道"
    arc: ArcCorner | None = None
    right_angle: RightAngleCorner | None = None
    vertical_clearance: float = Field(7.5, gt=0, description="垂直净空高度(m)")
    highway_grade: str = "三级公路"        # 4.2.1 确定横向预留空间 σ
    lateral_clearance: float | None = Field(
        None, ge=0, description="横向预留空间 σ(m)，显式给定则优先于按公路等级取值")
    # 4.2.2 / 4.3.1 横向空间宽度相关（替代“lane − 2σ”的错误口径）
    hard_shoulder_w: float = Field(0.0, ge=0, description="硬路肩宽度(m)")
    left_edgeline_w: float = Field(0.0, ge=0, description="左侧路缘带宽度(m)")
    median_type: str = Field("划线", description="对向分隔方式：物理隔离/划线，4.2.2")
    has_curve_widening: bool = Field(False, description="是否有圆曲线加宽")
    curve_widening_w: float = Field(0.0, ge=0, description="圆曲线加宽值(m)，4.3.1第2款")
    # 垂直净空分场景（6.4.1 上跨桥 / 9.2.1 隧道 / 6.3.2 匝道 / 普通跨线构造物）
    clearance_structure_type: str = Field(
        "跨线构造物", description="上跨桥/隧道/匝道/跨线构造物，决定净空阈值")

    @model_validator(mode="after")
    def _check_geometry(self):
        if self.road_type == "圆弧弯道" and self.arc is None:
            raise ValueError("圆弧弯道必须提供 outer_radius / lane_width")
        if self.road_type == "直角弯道" and self.right_angle is None:
            raise ValueError("直角弯道必须提供 entry_width / exit_width")
        return self


class InputBundle(BaseModel):
    """一次弯道检验的完整输入。"""
    cargo: CargoParams
    trailer: TrailerParams
    tractor: TractorParams
    road: RoadParams
    max_speed_kmh: float = Field(10.0, gt=0, description="最高行驶速度(km/h)")
    load_factor: float = Field(1.0, gt=0, description="横向载荷系数(离心)，默认1.0")

    @model_validator(mode="after")
    def _cargo_fits(self):
        if (self.cargo.is_overhang is False
                and (self.cargo.length > self.trailer.length
                     or self.cargo.width > self.trailer.width)):
            raise ValueError("货物长宽大于挂车长宽，且未勾选超宽，无法生成。")
        return self