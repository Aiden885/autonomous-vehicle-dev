"""Source-referenced teaching diagrams for the original APA Simulink model."""

from pathlib import Path
import hashlib
import importlib.util
import json
import math
import re
import xml.etree.ElementTree as ET
import zipfile


HERE = Path(__file__).resolve().parent
APA = HERE.parents[1]
SOURCE = APA / "APA-Simulink/APA"
spec = importlib.util.spec_from_file_location("apa_state_figures", HERE.parent / "state_management_illustrated/build_diagrams.py")
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
INK, MUTED, BLUE, LINE = base.INK, base.MUTED, base.BLUE, base.LINE
PALE, WARM, RED = base.PALE, base.WARM, "#AD493C"


class Diagram(base.Figure):
    def __init__(self, name, height, title, subtitle, sources, kind="teaching"):
        super().__init__(name, 1120, height, title, subtitle)
        self.sources, self.kind = sources, kind

    def line(self, points, color=LINE, width=2, dashed=False):
        attrs = {"points": " ".join("%g,%g" % tuple(p) for p in points),
                 "fill": "none", "stroke": color, "stroke-width": str(width)}
        if dashed:
            attrs["stroke-dasharray"] = "6 5"
        ET.SubElement(self.root, "polyline", attrs)

    def polygon(self, points, fill=PALE, color=BLUE, opacity=1):
        ET.SubElement(self.root, "polygon", {"points": " ".join("%g,%g" % tuple(p) for p in points),
            "fill": fill, "stroke": color, "stroke-width": "2", "opacity": str(opacity)})

    def dot(self, x, y, r=5, color=BLUE):
        ET.SubElement(self.root, "circle", {"cx": str(x), "cy": str(y), "r": str(r), "fill": color})

    def math_text(self, x, y, value, **kwargs):
        self.text(x, y, value, **kwargs)
        node = self.root[-1]
        node.text = ""
        last, start = None, 0
        for match in re.finditer(r"([A-Za-zκψδ])_([A-Za-zψ])", value):
            segment = value[start:match.start()]
            if last is None:
                node.text += segment
            else:
                last.tail = (last.tail or "") + segment
            ET.SubElement(node, "tspan", {"font-style": "italic"}).text = match.group(1)
            last = ET.SubElement(node, "tspan", {"baseline-shift": "sub", "font-size": "70%"})
            last.text = match.group(2)
            start = match.end()
        if last is None:
            node.text = value
        else:
            last.tail = value[start:]

    def save(self):
        result = super().save(HERE)
        result.update(kind=self.kind, sources=self.sources)
        return result


def rotate(point, yaw):
    x, y = point
    return (math.cos(yaw) * x - math.sin(yaw) * y,
            math.sin(yaw) * x + math.cos(yaw) * y)


def transform(point, pose):
    x, y = rotate(point, pose[2])
    return (x + pose[0], y + pose[1])


def screen(origin, scale):
    return lambda p: (origin[0] + scale * p[0], origin[1] - scale * p[1])


def arc(center, radius, a, b, count=61):
    return [(center[0] + radius * math.cos(a + (b-a)*i/(count-1)),
             center[1] + radius * math.sin(a + (b-a)*i/(count-1))) for i in range(count)]


def footprint(pose, front=3.8, rear=1.0, width=1.8):
    return [transform(p, pose) for p in [(-rear, -width/2), (front, -width/2),
                                        (front, width/2), (-rear, width/2)]]


def car(fig, mapping, pose, front=1.4, rear=.5, width=.8, opacity=1):
    fig.polygon([mapping(p) for p in footprint(pose, front, rear, width)], opacity=opacity)
    fig.dot(*mapping(pose[:2]), r=5)
    fig.arrow([mapping(pose[:2]), mapping(transform((front*.85, 0), pose))])


def overview():
    f = Diagram("02-01-system-coordination", 720, "整体协同：信息形成路线，反馈约束执行", "按原模型职责整理；不是逐线接线图，也不是所有模块同周期串行运行。", ["model:root"], "responsibility")
    f.node(40, 130, 240, 76, "输入与定位", "Vehicle_IO / APA_Navi1")
    f.node(425, 130, 240, 76, "近场环境", "Do_UltraSonic_FS")
    f.node(810, 130, 260, 76, "车位与目标", "Slot_Process_1000_1204")
    f.arrow([(280, 168), (425, 168)])
    f.arrow([(665, 168), (810, 168)])
    f.text(355, 150, "位姿等", size=18, anchor="middle")
    f.text(735, 150, "环境证据", size=18, anchor="middle")
    f.rect(820, 290, 250, 123)
    f.text(945, 320, "两路规划候选", bold=True, anchor="middle")
    f.text(945, 354, "Geo_Planning1", size=17, anchor="middle")
    f.text(945, 384, "Local_Planner", size=17, anchor="middle")
    f.arrow([(940, 206), (940, 290)])
    f.node(560, 318, 240, 76, "比较与历史管理", "Planner_Core")
    f.node(300, 318, 240, 76, "运动执行", "Moco3")
    f.node(40, 318, 240, 76, "底盘协调", "ChassisCtrl1")
    for x in (820, 560, 300):
        f.arrow([(x, 356), (x-20, 356)])
    f.node(40, 530, 240, 76, "车辆运动与反馈", "实际位姿 / 速度 / 状态")
    f.arrow([(160, 394), (160, 530)])
    f.arrow([(40, 568), (18, 568), (18, 168), (40, 168)], color=MUTED)
    f.node(425, 530, 240, 76, "任务阶段与许可", "APA_StateMgmet1", warm=True)
    f.node(810, 530, 260, 76, "持续风险判断", "MEB", warm=True)
    f.arrow([(545, 530), (545, 472), (420, 472), (420, 394)], dashed=True)
    f.arrow([(940, 530), (940, 450), (452, 450), (452, 394)], dashed=True)
    f.text(730, 440, "保护约束（省略其他接收方）", size=18, anchor="middle")
    f.text(32, 666, "实线示意主要信息与执行反馈；虚线示意任务 / 安全约束，图中省略部分交互。", size=20)
    f.text(32, 700, "后续各章分别展开定位、环境、车位、规划、控制与安全，不是新增一套系统。", size=18, color=MUTED)
    return f.save()


def coordinates():
    f = Diagram("03-01-coordinate-transform", 695, "同一个点，在车体系和参考系中有不同坐标", "教学约定：参考系 x 向右、y 向上，航向逆时针为正；不替代各接口轴向定义。", ["GeoPlanningSubFcn/geo_axis_trans.m"])
    m = screen((115, 545), 66)
    f.arrow([m((0, 0)), m((7, 0))], color=LINE)
    f.arrow([m((0, 0)), m((0, 6))], color=LINE)
    f.math_text(590, 551, "x_g（m）", size=18)
    f.math_text(45, 131, "y_g（m）", size=18)
    pose = (2, 1.5, math.pi/6)
    local = (3, 1)
    point = transform(local, pose)
    inverse = rotate((point[0]-pose[0], point[1]-pose[1]), -pose[2])
    assert math.dist(local, inverse) < 1e-12
    car(f, m, pose)
    origin = m(pose[:2])
    f.arrow([origin, m(transform((3.7, 0), pose))])
    f.arrow([origin, m(transform((0, 2.4), pose))])
    f.math_text(530, 331, "x_b", size=19)
    f.math_text(146, 302, "y_b", size=19)
    f.dot(*m(point), r=7, color=RED)
    f.text(m(point)[0]+13, m(point)[1]-10, "固定环境点 P", size=20)
    f.line([origin, m(transform((3, 0), pose)), m(point)], dashed=True)
    f.line([m((0, 0)), origin], dashed=True, color=MUTED)
    f.math_text(130, 589, "O_g", size=18)
    f.math_text(origin[0]-28, origin[1]+48, "O_b：车辆参考点", size=18)
    f.text(700, 174, "车体系：相对车辆的位置", size=22, bold=True)
    f.text(700, 222, "参考系：旋转后，再加车辆位置", size=20)
    f.math_text(700, 280, "p_g = t_e + R(ψ_e) p_b", size=22, color=BLUE)
    f.text(700, 347, "反向转换：先减位置，再反旋转", size=20)
    f.math_text(700, 393, "p_b = Rᵀ(ψ_e) (p_g − t_e)", size=21, color=BLUE)
    f.text(700, 470, "P 没有变，只是描述它的基准变了。", size=19)
    f.text(32, 651, "参考点定义必须一致；本图用旋转和平移解释原理，车体简图不按原车型尺寸缩放。", size=19)
    return f.save()


def time_alignment():
    f = Diagram("03-02-time-alignment", 565, "坐标相同还不够：观测必须对应正确时刻", "教学示意，无真实时间数值；说明时间配对，不宣称所有输入均已实现插值补偿。", ["chart:30112", "chart:30337"])
    for y, label in [(185, "车辆位姿"), (315, "感知结果")]:
        f.text(32, y+7, label, size=22, bold=True)
        f.arrow([(200, y), (1060, y)], color=LINE)
    for x, label in [(290, "较早时刻 t_a"), (620, "结果到达"), (960, "当前时刻 t_b")]:
        f.line([(x, 150), (x, 370)], dashed=True)
        f.text(x, 127, label, size=19, anchor="middle")
    f.dot(290, 185, r=8)
    f.dot(960, 185, r=8)
    f.dot(290, 315, r=8, color=RED)
    f.arrow([(290, 315), (620, 315)], color=MUTED)
    f.text(451, 351, "计算 / 传输后才收到", size=19, anchor="middle")
    f.arrow([(290, 303), (290, 197)])
    f.text(330, 252, "观测属于 t_a，应先与 t_a 的位姿配对", size=21, color=BLUE)
    f.text(32, 431, "把旧观测直接当成当前车体系数据，会把车辆这段运动混进环境位置。", size=22, bold=True)
    f.text(32, 482, "正确时间标记与位姿关系，是坐标转换、历史缓存和跨帧车位跟踪共同的基础。", size=20)
    f.text(32, 530, "本图讲同步目标；原模型各支路的计数、标志和补偿方式仍按实际实现区分。", size=18, color=MUTED)
    return f.save()


def environment():
    f = Diagram("04-01-points-to-boundary", 686, "从点到边界：分组与拟合解决不同问题", "相同一组教学点依次展示；不是真实传感器记录，也不表示识别了全部物体。", ["USSubFcn/processFSPoints.m", "USSubFcn/ultrasonicCorrection.m"])
    offsets = (40, 415, 790)
    titles = ("① 输入零散点", "② 按邻近关系分组", "③ 提取线段与轮廓")
    points = [(25+i*23, 100+(i%3-1)*7) for i in range(10)]
    cluster = [(160, 190), (180, 178), (204, 185), (211, 208), (185, 222), (164, 213)]
    noise = [(60, 235), (290, 145)]
    for j, x in enumerate(offsets):
        f.text(x+150, 137, titles[j], size=21, bold=True, anchor="middle")
        f.rect(x, 162, 300, 300, stroke="#D5DEE5", radius=0)
        for px, py in points:
            f.dot(x+px, 162+py, r=4, color=BLUE if j else MUTED)
        for px, py in cluster:
            f.dot(x+px, 162+py, r=4, color="#A8792E" if j else MUTED)
        for px, py in noise:
            f.dot(x+px, 162+py, r=4, color=MUTED)
        if j == 1:
            f.line([(x+18, 244), (x+245, 244)], color=BLUE, dashed=True)
            f.text(x+16, 194, "分成两组，孤立点另作处理", size=18)
        if j == 2:
            f.line([(x+25, 262), (x+232, 262)], color=BLUE, width=4)
            f.polygon([(x+px, 162+py) for px, py in cluster], fill="none", color="#A8792E")
    f.arrow([(345, 310), (405, 310)])
    f.arrow([(720, 310), (780, 310)])
    f.text(40, 507, "聚类：把相邻点归组。拟合：用支持点估计墙的方向和范围。凸包：描述点群外缘。", size=20)
    f.rect(40, 541, 1050, 88, fill=PALE)
    f.text(60, 575, "超声证据还要先匹配对象，再核对或修正相关几何；不是把两个来源直接求平均。", size=20)
    f.text(60, 607, "最终还需携带类别、数量、有效性与更新信息，供规划和安全链采用。", size=19)
    f.text(40, 666, "本图只展示几何整理；神经网络识别、不可见表面和完整空间可用性不由这三步自动保证。", size=18, color=MUTED)
    return f.save()


def slot_tracking():
    f = Diagram("05-01-slot-memory", 765, "跨帧跟踪：车动了，不等于车位动了", "沿用图3-1的参考系思想；本例仅沿 x 平移，方向不变，数值用于说明原理。", ["chart:37199"])
    for y, pos, distance, label in [(208, 0, 5, "上一帧"), (385, 1, 4, "当前帧")]:
        f.text(40, y-49, label, size=22, bold=True)
        ox, scale = 230, 115
        f.arrow([(ox-30, y), (ox+690, y)], color=LINE)
        car(f, lambda p: (ox+scale*p[0], y-scale*p[1]), (pos, 0, 0), front=.8, rear=.3, width=.55)
        px = ox+5*scale
        f.rect(px, y-53, 126, 85, fill=PALE, radius=0)
        f.text(px+63, y-69, "同一个静止车位角点", size=18, anchor="middle")
        f.dot(px, y, r=7, color=RED)
        f.arrow([(ox+scale*pos, y+52), (px, y+52)], color=MUTED)
        f.text((ox+scale*pos+px)/2, y+83, "相对车辆 %d m" % distance, size=20, anchor="middle")
    assert 5-0 == 5 and 5-1 == 4
    f.text(40, 518, "车辆前进 1 m 后，车位角点的局部距离从 5 m 变为 4 m；参考位置仍相同。", size=21, bold=True)
    stages = [("运动补偿", "统一比较基准"), ("关联", "接到正确历史对象"), ("滤波与几何", "更新位置和形状"), ("ID / 有效状态", "交给规划使用")]
    for i, (cn, en) in enumerate(stages):
        f.node(40+i*275, 569, 245, 78, cn, en)
        if i < 3:
            f.arrow([(285+i*275, 608), (315+i*275, 608)])
    f.text(40, 699, "先确认是同一个车位，再融合观测；匹配错误不能靠滤波自动纠正。", size=21)
    f.text(40, 738, "几何与角点画法仅作示意，未执行原模型滤波，也不代表车位永不失效。", size=18, color=MUTED)
    return f.save()


def geometric_connection():
    f = Diagram("07-01-tangent-arc", 740, "圆弧连接：起点朝向与目标方向共同限制形状", "单圆几何教学示意，不是当前场景的规划输出；参考点采用后轴类定义。", ["GeoPlanningSubFcn/geo_circle_1pt1line.m"])
    m = screen((205, 550), 43)
    radius, center = 6, (0, 6)
    points = arc(center, radius, -math.pi/2, 0)
    assert math.dist(points[0], (0, 0)) < 1e-12
    assert math.dist(points[-1], (6, 6)) < 1e-12
    assert all(abs(math.dist(p, center)-radius) < 1e-12 for p in points)
    assert abs(points[0][0]-center[0]) < 1e-12  # Initial radius is normal to the x tangent.
    assert abs(points[-1][1]-center[1]) < 1e-12  # Final radius is normal to the y tangent.
    f.line([m(p) for p in arc(center, radius, -math.pi/2, .3)], color=LINE, dashed=True)
    f.line([m((6, -1)), m((6, 10))], color=MUTED)
    f.line([m(center), m((0, 0))], color=LINE, dashed=True)
    f.line([m(center), m((6, 6))], color=LINE, dashed=True)
    f.line([m(p) for p in points], color=BLUE, width=5)
    car(f, m, (0, 0, 0))
    car(f, m, (6, 6, math.pi/2))
    f.dot(*m(center), r=6, color=RED)
    f.text(85, 285, "圆心 C", size=20)
    f.text(230, 416, "半径 R", size=20)
    f.text(83, 608, "起点：切线沿车头方向", size=20)
    f.text(465, 126, "目标直线", size=20)
    f.text(640, 176, "① 起点切线与当前朝向相容", size=22, bold=True)
    f.text(640, 233, "圆心位于起点方向的一侧法线上。", size=20)
    f.text(640, 313, "② 圆弧与目标直线相切", size=22, bold=True)
    f.text(640, 370, "圆心到目标线的距离等于半径。", size=20)
    f.text(640, 452, "③ 再检查半径和可用空间", size=22, bold=True)
    f.text(640, 507, "几何接得上，不等于整车能通过。", size=20)
    f.text(32, 675, "κ = tan(δ) / L，转角有限意味着半径不能任意小；直线与圆弧只是候选构造。", size=21)
    f.text(32, 717, "本图下一步由图7-2补充车身扫掠；不承诺单圆可解决所有起始姿态或车位。", size=18, color=MUTED)
    return f.save()


def swept_body():
    f = Diagram("07-02-body-sweep", 740, "后轴轨迹不碰，车头仍可能扫到障碍", "接续图7-1的恒曲率圆弧；车身尺寸为教学设定，不是原车型标定。", ["GeoPlanningSubFcn/geo_circle_1pt1line.m", "PlanningSubFcn/collision_Check_SAT.m"])
    m = screen((155, 575), 42)
    center, radius = (0, 6), 6
    theta = math.pi/4
    mid = (radius*math.sin(theta), radius*(1-math.cos(theta)), theta)
    obstacle = transform((2.7, -.6), mid)
    # This obstacle lies inside the example body, but away from its rear-axle circle.
    local = rotate((obstacle[0]-mid[0], obstacle[1]-mid[1]), -mid[2])
    assert -1 < local[0] < 3.8 and abs(local[1]) < .9
    clearance = abs(math.dist(obstacle, center)-radius)
    assert clearance > .5
    for yaw in (0, math.pi/4, math.pi/2):
        pose = (radius*math.sin(yaw), radius*(1-math.cos(yaw)), yaw)
        car(f, m, pose, front=3.8, rear=1, width=1.8, opacity=.35 if yaw != theta else .8)
    f.line([m(p) for p in arc(center, radius, -math.pi/2, 0)], color=BLUE, width=4)
    ox, oy = m(obstacle)
    f.dot(ox, oy, r=9, color=RED)
    f.arrow([(665, 338), (ox+15, oy)], color=RED)
    f.text(685, 335, "这个障碍侵入示例车身", size=22, bold=True, color=RED)
    f.text(685, 383, "但没有落在蓝色后轴轨迹上。", size=20)
    f.text(685, 460, "车辆外形随位置和朝向一起变化。", size=20)
    f.text(685, 507, "车头、车尾走出的区域不同。", size=20)
    f.text(70, 633, "蓝线：后轴参考轨迹；浅色框：沿路三个姿态的车身外廓。", size=20)
    f.text(32, 689, "实际检查还需覆盖姿态之间的运动，不能只检查图中三个采样姿态。", size=21, bold=True)
    f.text(32, 724, "此图只证明这个教学几何例子的区别，不是原场景碰撞回放或完整扫掠验算。", size=18, color=MUTED)
    return f.save()


def propagate(pose, distance, steer, wheelbase=2.8):
    x, y, yaw = pose
    curvature = math.tan(steer)/wheelbase
    end_yaw = yaw+curvature*distance
    if abs(curvature) < 1e-10:
        return (x+distance*math.cos(yaw), y+distance*math.sin(yaw), end_yaw)
    return (x+(math.sin(end_yaw)-math.sin(yaw))/curvature,
            y-(math.cos(end_yaw)-math.cos(yaw))/curvature, end_yaw)


def search_expansion():
    f = Diagram("08-01-search-expansion", 730, "搜索扩展：节点保存姿态，分支对应车辆小动作", "教学动作树，不是原算法运行日志；未标为受阻的分支也不表示已通过完整验收。", ["PlanningSubFcn/VehicleDynamic.m", "PlanningSubFcn/getTotalCost.m"])
    m = screen((105, 380), 133)
    f.arrow([m((0, 0)), m((4.2, 0))], color=LINE)
    f.arrow([m((0, -1.35)), m((0, 1.55))], color=LINE)
    f.text(673, 392, "x（m）", size=18)
    f.text(53, 148, "y（m）", size=18)
    root = (0, 0, 0)
    car(f, m, root, front=.75, rear=.25, width=.45)
    f.text(45, 590, "起始姿态 S", size=21)
    blocked = (2.9, -.22, 3.4, .22)
    box = [m((blocked[0], blocked[1])), m((blocked[2], blocked[1])),
           m((blocked[2], blocked[3])), m((blocked[0], blocked[3]))]
    f.polygon(box, fill=WARM, color=RED)
    f.text(485, 452, "障碍示意", size=19, color=RED)
    rejected = 0
    for first in (-.45, 0, .45):
        parent = propagate(root, 1.6, first)
        f.line([m(propagate(root, 1.6*i/30, first)[:2]) for i in range(31)], color=BLUE)
        f.dot(*m(parent[:2]))
        for second in (-.45, 0, .45):
            end = propagate(parent, 1.6, second)
            unsafe = blocked[0] < end[0] < blocked[2] and blocked[1] < end[1] < blocked[3]
            color = RED if unsafe else MUTED
            f.line([m(propagate(parent, 1.6*i/30, second)[:2]) for i in range(31)], color=color)
            f.dot(*m(end[:2]), r=4, color=color)
            rejected += int(unsafe)
    assert rejected >= 1
    f.text(735, 170, "小动作：给定方向、转角和步长", size=20, bold=True)
    f.text(735, 227, "运动模型推算下一个位置与朝向。", size=19)
    f.text(735, 304, "Open：保存待扩展节点", size=20, bold=True)
    f.text(735, 355, "按代价排序，优先尝试有希望的分支。", size=19)
    f.text(735, 432, "碰撞 / 约束检查：淘汰受阻动作", size=20, bold=True)
    f.text(735, 484, "本例红点的参考点已落入障碍。", size=19)
    f.text(735, 537, "完整车辆检查接着看图8-2。", size=19)
    f.text(32, 662, "f = g + λh：已走代价与后续估计帮助排序；相似状态查重可减少重复尝试。", size=21)
    f.text(32, 706, "本图只画前进动作的局部展开；原规划还涉及倒车、解析连接、预算和结果回溯。", size=18, color=MUTED)
    return f.save()


def separating_axis():
    f = Diagram("08-02-separating-axis", 680, "分离轴：一个方向能分开，就足以说明不相交", "凸多边形教学示意；图中矩形轴向简单，实际车辆旋转后需相应的边法向。", ["PlanningSubFcn/collision_Check_SAT.m", "PlanningSubFcn/collision_Check_SAT_Octagon.m"])
    f.text(285, 135, "例 A：x 方向已经分开", size=23, bold=True, anchor="middle")
    f.text(840, 135, "例 B：x 重叠，仍不能判相交", size=23, bold=True, anchor="middle")
    a1, b1 = (70, 230, 130, 80), (350, 230, 130, 80)
    a2, b2 = (660, 210, 130, 80), (720, 350, 130, 80)
    for x, y, w, h in (a1, a2):
        f.rect(x, y, w, h, fill=PALE, stroke=BLUE, radius=0)
        f.text(x+w/2, y+h/2+7, "A", size=23, anchor="middle")
    for x, y, w, h in (b1, b2):
        f.rect(x, y, w, h, fill=WARM, stroke="#A8792E", radius=0)
        f.text(x+w/2, y+h/2+7, "B", size=23, anchor="middle")
    for x, y, w, h in (a1, b1, a2, b2):
        f.line([(x, y+h), (x, 470)], dashed=True)
        f.line([(x+w, y+h), (x+w, 470)], dashed=True)
    f.arrow([(40, 470), (525, 470)], color=LINE)
    f.arrow([(610, 470), (1070, 470)], color=LINE)
    for x, y, w, _ in (a1, a2):
        f.line([(x, 492), (x+w, 492)], color=BLUE, width=7)
    for x, y, w, _ in (b1, b2):
        f.line([(x, 519), (x+w, 519)], color="#A8792E", width=7)
    assert a1[0]+a1[2] < b1[0]
    assert max(a2[0], b2[0]) < min(a2[0]+a2[2], b2[0]+b2[2])
    assert a2[1]+a2[3] < b2[1]
    f.line([(620, 323), (1040, 323)], dashed=True, color=RED)
    f.text(890, 307, "沿 y 可以分开", size=19, color=RED)
    f.text(285, 559, "投影区间不重叠 → 已找到分离轴", size=20, anchor="middle")
    f.text(840, 559, "x 投影重叠 → 继续检查其他候选轴", size=20, anchor="middle")
    f.text(32, 625, "不能把某一个轴重叠直接当作碰撞；接触、容差和运动区间另按实际实现处理。", size=21)
    return f.save()


def path_management():
    f = Diagram("09-01-candidate-and-memory", 826, "路径管理：不是新路一来，就立即替换正在走的路", "按原模型职责归纳的选择过程；不覆盖每种泊车分支，也不是固定统一评分公式。", ["chart:52669", "PlanningSubFcn/ClassPath.m"], "responsibility")
    for x, cn, en in [(40, "几何候选", "geo_path_list"), (430, "搜索候选", "search_path_list"), (820, "历史路径", "mem_path_list")]:
        f.node(x, 130, 260, 76, cn, en)
    for x in (170, 560, 950):
        f.arrow([(x, 206), (x, 255), (560, 255), (560, 290)])
    f.node(160, 290, 800, 78, "确认身份、更新、有效性与相关空间条件", "属于当前目标，结果可采用")
    f.arrow([(560, 368), (560, 418)])
    f.node(160, 418, 800, 78, "结合方向、段数、场景及当前执行进度比较", "新结果与历史协调：保留，或在条件允许时更新")
    f.arrow([(560, 496), (560, 548)])
    f.node(160, 548, 800, 78, "当前执行路径 + 状态 + 更新编号", "一起交给 MoCo，不只替换 x / y 数组", warm=True)
    f.text(32, 695, "历史路径连接着当前段、参考点和停车点；更新必须让执行方知道版本已经改变。", size=21)
    f.text(32, 745, "历史失效或不安全时不能为保持连续而强行保留；必要时要停车或重规划。", size=21, bold=True)
    f.text(32, 796, "图2-1中的 Planner_Core 在此展开；没有把候选选择写成所有场景统一的最短路规则。", size=18, color=MUTED)
    return f.save()


def preview_geometry():
    f = Diagram("10-01-preview-and-heading", 740, "预瞄位置与航向偏差，是两种不同的信息", "前进几何教学例：车头沿车体纵向正向；预瞄项是反馈，不直接等同于曲率前馈。", ["chart:42652"])
    m = screen((205, 525), 72)
    point = (4, 1)
    radius = (point[0]**2+point[1]**2)/(2*point[1])
    center = (0, radius)
    end_angle = math.atan2(point[1]-radius, point[0])
    curved = arc(center, radius, -math.pi/2, end_angle)
    assert math.dist(curved[-1], point) < 1e-12
    assert abs(1/radius - 2*point[1]/(point[0]**2+point[1]**2)) < 1e-12
    f.arrow([m((-.6, 0)), m((5.5, 0))], color=LINE)
    f.arrow([m((0, -.4)), m((0, 4.6))], color=LINE)
    f.math_text(605, 547, "x_b（m）", size=18)
    f.math_text(122, 178, "y_b（m）", size=18)
    car(f, m, (0, 0, 0))
    f.line([m((0, 0)), m(point)], dashed=True, color=MUTED)
    f.line([m(p) for p in curved], color=BLUE, width=4)
    f.line([m((4, 0)), m(point)], dashed=True, color=RED)
    f.dot(*m(point), r=7, color=RED)
    f.text(435, 405, "预瞄点 P", size=20, color=RED)
    ref_yaw = .35
    f.arrow([m(point), m((point[0]+1.3*math.cos(ref_yaw), point[1]+1.3*math.sin(ref_yaw)))], color=RED)
    f.math_text(559, 395, "参考航向 ψ_r", size=18, color=RED)
    f.line([m(p) for p in arc((0, 0), .85, 0, math.atan2(1, 4))], color=MUTED)
    f.text(280, 482, "α", size=21)
    f.arrow([m((0, -.8)), m((4, -.8))], color=MUTED)
    f.math_text(351, 620, "x_p：前后位置", size=19, anchor="middle")
    f.math_text(517, 495, "y_p", size=21, color=RED)
    f.text(710, 170, "蓝弧：追向预瞄点的理想圆弧", size=21, bold=True)
    f.math_text(710, 229, "κ_p = 2 y_p / (x_p² + y_p²)", size=22, color=BLUE)
    f.text(710, 299, "α 是看向 P 的视线夹角。", size=21)
    f.math_text(710, 365, "e_ψ 是参考航向与车头的差。", size=21)
    f.math_text(710, 421, "e_ψ = wrap(ψ_r − ψ_e)", size=22, color=RED)
    f.math_text(710, 489, "α 与 e_ψ 不能混用。", size=23, bold=True)
    f.math_text(710, 552, "y_p 也不必等于最近点横向误差。", size=19)
    f.text(32, 676, "原分支还处理方向、增益、死区和转向尺度；倒车不能只照搬这个前进示意。", size=20)
    f.text(32, 718, "本图说明跟踪几何；图10-2再把它放回执行请求、底盘和实际反馈的闭环。", size=18, color=MUTED)
    return f.save()


def tracking_loop():
    f = Diagram("10-02-tracking-loop", 705, "跟踪闭环：请求不等于车辆已经做到", "原模型职责示意，省略部分信号与接收方；不是后续 C/CARLA 版本的踏板执行器框图。", ["model:root", "chart:42652"], "responsibility")
    for x, cn, en in [(40, "当前执行路径", "Planning_Out"), (435, "分段执行与跟踪", "Moco3"), (820, "底盘协调与适配", "ChassisCtrl1")]:
        f.node(x, 150, 250, 78, cn, en)
    f.arrow([(290, 189), (435, 189)])
    f.arrow([(685, 189), (820, 189)])
    f.text(750, 132, "控制请求", size=19, anchor="middle")
    f.node(820, 440, 250, 78, "车辆实际运动", "位姿 / 速度 / 挡位等")
    f.node(435, 440, 250, 78, "状态整理与反馈", "定位及车辆信号")
    f.node(40, 440, 250, 78, "任务与安全条件", "主状态 / MEB", warm=True)
    f.arrow([(945, 228), (945, 440)])
    f.text(975, 329, "实际执行", size=20)
    f.arrow([(820, 479), (685, 479)])
    f.text(750, 430, "实际反馈", size=19, anchor="middle")
    f.arrow([(560, 440), (560, 228)])
    f.text(590, 330, "误差与进度重新计算", size=20)
    f.arrow([(290, 479), (365, 479), (365, 189), (435, 189)], dashed=True)
    f.text(40, 333, "阶段 / 保护约束", size=21, bold=True)
    f.text(32, 598, "每次用实际反馈修正请求，并根据准备、换向、路径更新和风险决定能否继续。", size=21)
    f.text(32, 654, "图6-4说明任务内运动阶段的配合；本图说明这些阶段中的反馈如何闭合。", size=20, color=MUTED)
    return f.save()


def braking_distance():
    f = Diagram("11-01-stopping-distance", 765, "停车空间：延迟、减速和余量分别占一段距离", "采用原函数某分支的参数作算术示例；不是实测刹停距离，也不是全部 MEB 触发条件。", ["chart:15046"])
    speed, deceleration, delay, margin = 3.6/3.6, 4, .4, .2
    response, braking = speed*delay, speed**2/(2*deceleration)
    total = response+braking+margin
    assert abs(total-.725) < 1e-12
    f.text(40, 151, "示例输入 3.6 km/h = 1 m/s，减速度假设 4 m/s²，延迟假设 0.4 s", size=22, bold=True)
    x, y, scale = 105, 226, 1100
    lengths = [response, braking, margin]
    for i, (length, color) in enumerate(zip(lengths, [PALE, WARM, "#F0F2F4"])):
        width = length*scale
        f.rect(x, y, width, 84, fill=color, stroke=LINE, radius=0)
        f.text(x+width/2, y+51, str(i+1), size=26, bold=True, anchor="middle")
        x += width
    f.arrow([(105, 341), (x, 341)], color=BLUE)
    f.text((105+x)/2, 386, "示例比较范围：0.725 m", size=25, bold=True, anchor="middle")
    rows = [("1  延迟期间继续运动", "vτ = 1 × 0.4 = 0.400 m"),
            ("2  实际减速所需距离", "v² / (2a_b) = 1² / 8 = 0.125 m"),
            ("3  该分支附加的余量", "0.200 m")]
    for i, (label, value) in enumerate(rows):
        f.text(50, 461+i*64, label, size=22, bold=True)
        f.math_text(480, 461+i*64, value, size=22)
    f.text(32, 686, "速度归零的请求不能使车辆瞬间停下；真实响应更慢时，所需空间还会增加。", size=21)
    f.text(32, 735, "条带从抽象距离参考点起算，不自行规定接口的测距原点；保护链可结合图10-2理解。", size=18, color=MUTED)
    return f.save()


def multirate_schedule():
    f = Diagram("12-01-multirate-memory", 738, "多周期闭环：控制更新时，规划未必刚好产生新路", "按 5 ms 基础调用假设说明名义频率；图中相位为教学排列，不是实测调度记录。", ["chart:14937", "model:config"])
    x0, cell = 245, 36
    for k in range(21):
        x = x0+k*cell
        f.line([(x, 160), (x, 475)], color="#D8E0E5")
        if k % 4 == 0:
            f.text(x, 136, str(k*5), size=18, anchor="middle")
    f.text(1010, 137, "ms", size=18)
    labels = [(210, "基础调用", 1), (320, "约 20 ms 组", 4), (430, "约 50 ms 组", 10)]
    for y, label, multiple in labels:
        f.text(28, y+8, label, size=22, bold=True)
        for k in range(1, 21):
            if k % multiple == 0:
                f.rect(x0+k*cell-7, y-12, 14, 24, fill=BLUE, stroke=BLUE, radius=0)
        f.line([(x0, y), (x0+20*cell, y)])
    f.text(40, 521, "搜索计数组：在同一基础调用假设下约 800 ms，本图不展开该时间尺度。", size=21)
    f.rect(40, 556, 1040, 94, fill=PALE)
    f.text(60, 592, "控制在两次路径更新之间，仍可使用最近一次有效路径与跨周期执行进度。", size=21, bold=True)
    f.text(60, 626, "新路径到来时，再按版本和任务条件接续；不是所有任务都等下一次搜索才能运行。", size=19)
    f.text(32, 703, "频率来自原调度计数条件的推算；初始相位、实际触发、顺序和停用支路仍按模型确认。", size=18, color=MUTED)
    assert 4*.005 == .02 and 10*.005 == .05 and 160*.005 == .8
    return f.save()


def delayed_memory():
    f = Diagram("13-01-delay-and-edge", 718, "UnitDelay 的作用：把电平变成可识别的新事件", "布尔教学例，上一周期记忆的初值设为 0；不是实际按键日志，也不规定某个端口的初值。", ["chart:17000"], "teaching")
    values = [0, 1, 1, 0, 1, 1]
    previous = [0]+values[:-1]
    rising = [int(now and not old) for now, old in zip(values, previous)]
    assert rising == [0, 1, 0, 0, 1, 0]
    x0, step = 280, 120
    for k in range(7):
        f.line([(x0+k*step, 133), (x0+k*step, 565)], color="#D8E0E5")
    for row, (name, series) in enumerate([("当前输入 u(k)", values), ("记忆 u(k−1)", previous), ("上升沿事件", rising)]):
        low = 235+row*145
        f.text(32, low-27, name, size=22, bold=True)
        f.text(243, low+5, "0", size=16)
        f.text(243, low-52, "1", size=16)
        points = []
        for k, value in enumerate(series):
            x = x0+k*step
            y = low-58*value
            if points:
                points.append((x, points[-1][1]))
            points.extend([(x, y), (x+step, y)])
        f.line(points, color=BLUE if row != 1 else MUTED, width=3)
    for k in range(6):
        f.text(x0+(k+.5)*step, 590, "k=%d" % k, size=18, anchor="middle")
    f.text(32, 640, "u(k) AND NOT u(k−1)：只在新上升时成立，持续按下不会每周期都当成一次新操作。", size=20)
    f.text(32, 687, "这解释图6-3恢复确认需要历史值的原因；Delay 存储的是值，不会自动完成整项任务仲裁。", size=18, color=MUTED)
    return f.save()


def source_evidence(figures):
    model = SOURCE / "F515_APA_Seres_V3p24_offline.slx"
    digest = hashlib.sha256(model.read_bytes()).hexdigest()
    assert digest == "2d6788b3ee229ce24e337728dfbe7a971baa580b9f4cfcd20eb845e777ffbede"
    with zipfile.ZipFile(model) as archive:
        stateflow = ET.fromstring(archive.read("simulink/stateflow.xml"))
        blockdiagram = ET.fromstring(archive.read("simulink/blockdiagram.xml"))
        config = ET.fromstring(archive.read("simulink/configSet1.xml"))
        config_info = ET.fromstring(archive.read("simulink/configSetInfo.xml"))
    assert any(c.get("Active") == "true" and c.text == "Configuration1" for c in config_info.iter("ConfigSet"))
    config_values = [(p.get("Name"), p.text) for p in config.iter("P")]
    assert ("Name", "Configuration1") in config_values and ("FixedStep", "0.005") in config_values
    charts = {c.get("id"): c for c in stateflow.iter("chart")}
    nodes = {n.get("Name") for n in blockdiagram.iter("Block")}
    for name in ["APA_Navi1", "Do_UltraSonic_FS", "Slot_Process_1000_1204", "Geo_Planning1", "Local_Planner", "Planner_Core", "Moco3", "ChassisCtrl1", "MEB"]:
        assert name in nodes, name
    evidence = {}
    for reference in sorted({r for f in figures for r in f["sources"]}):
        if reference.startswith("chart:"):
            chart = charts[reference.split(":")[1]]
            name = base.prop(chart, "name")
            blob = ET.tostring(chart, encoding="utf-8")
            evidence[reference] = {"chart_name": name, "sha256": hashlib.sha256(blob).hexdigest()}
        elif reference.startswith("model:"):
            evidence[reference] = {"model_sha256": digest, "meaning": "responsibilities or conditional schedule; not literal wiring"}
            if reference == "model:config":
                evidence[reference].update(configuration="Configuration1", fixed_step_sec=.005,
                                           source="simulink/configSet1.xml")
        else:
            path = SOURCE / reference
            assert path.is_file(), reference
            evidence[reference] = {"sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
    schedule = "\n".join(p.text or "" for p in charts["14937"].iter("P"))
    assert all(token in schedule for token in ["j%%4", "i%%10", "n%%160", "k%%4"])
    braking = "\n".join(p.text or "" for p in charts["15046"].iter("P") if p.get("Name") == "script")
    assert all(token in braking for token in ["k_MEB_braking_deceleration = 4", "k_MEB_braking_delay_time = 0.4", "k_MEB_braking_dist_safety_offset = 0.2", "V_Spd/3.6"])
    result = {"model_sha256": digest, "figures": figures, "sources": evidence,
              "checks": ["coordinate roundtrip", "arc radius and endpoints", "point inside example vehicle but outside rear-axle route", "illustrative motion propagation", "SAT counterexample", "pure-pursuit circle", "braking arithmetic", "nominal period arithmetic", "delay and edge sequence", "text bounds and overlaps"],
              "not_claimed": ["actual input data", "original planner replay", "full collision proof", "simulation or physical performance", "complete sufficient guard conditions"]}
    (HERE / "source_evidence.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n")


def main():
    figures = []
    for draw in [overview, coordinates, time_alignment, environment, slot_tracking,
                 geometric_connection, swept_body, search_expansion, separating_axis,
                 path_management, preview_geometry, tracking_loop, braking_distance,
                 multirate_schedule, delayed_memory]:
        print("Drawing", draw.__name__, flush=True)
        figures.append(draw())
    source_evidence(figures)
    print(json.dumps({"figures": len(figures), "all_source_references_exist": True,
                      "text_bounds_checked": True}, ensure_ascii=False))


if __name__ == "__main__":
    main()
