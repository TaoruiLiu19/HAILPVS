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
    locked_axles: list[int] = Field(default_factory=list, description="锁死轴列表(如[2,3])")


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


class RoadParams(BaseModel):
    """道路/弯道参数。"""
    road_type: str = "圆弧弯道"
    arc: ArcCorner | None = None
    right_angle: RightAngleCorner | None = None
    vertical_clearance: float = Field(7.5, gt=0, description="垂直净空高度(m)")
    highway_grade: str = "三级公路"        # 4.2.1 确定横向预留空间 σ
    lateral_clearance: float | None = Field(
        None, ge=0, description="横向预留空间 σ(m)，显式给定则优先于按公路等级取值")

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