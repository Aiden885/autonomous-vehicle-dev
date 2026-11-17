"""Build chapter-six illustrations after checking the original Stateflow graph."""

from pathlib import Path
import hashlib
import json
import math
import xml.etree.ElementTree as ET
import zipfile

import cairo
import gi

gi.require_version("Rsvg", "2.0")
gi.require_version("Pango", "1.0")
gi.require_version("PangoCairo", "1.0")
from gi.repository import Pango, PangoCairo, Rsvg


HERE = Path(__file__).resolve().parent
MODEL = HERE.parents[1] / "APA-Simulink/APA/F515_APA_Seres_V3p24_offline.slx"
INK, MUTED, BLUE, LINE = "#183047", "#526577", "#14678A", "#78909F"
PALE, WARM = "#EAF3F7", "#FAF0DE"


def prop(node, key):
    return next((p.text or "" for p in node.findall("P") if p.get("Name") == key), "")


def source_graph():
    digest = hashlib.sha256(MODEL.read_bytes()).hexdigest()
    assert digest == "2d6788b3ee229ce24e337728dfbe7a971baa580b9f4cfcd20eb845e777ffbede"
    with zipfile.ZipFile(MODEL) as archive:
        root = ET.fromstring(archive.read("simulink/stateflow.xml"))
    chart = next(c for c in root.iter("chart") if c.get("id") == "17000")
    parent = {child: node for node in chart.iter() for child in node}
    states = {}
    for state in chart.iter("state"):
        if prop(state, "type") != "OR_STATE":
            continue
        name = prop(state, "labelString").splitlines()[0]
        owner = parent.get(state)
        while owner is not None and owner.tag != "state":
            owner = parent.get(owner)
        states[name] = {"ssid": state.get("SSID"),
                        "parent": prop(owner, "labelString").splitlines()[0] if owner is not None else None}
    transitions = {}
    for transition in chart.iter("transition"):
        transitions[transition.get("SSID")] = {
            "source": prop(transition.find("src"), "SSID"),
            "target": prop(transition.find("dst"), "SSID"),
            "guard": prop(transition, "labelString"),
            "execution_order": prop(transition, "executionOrder"),
        }
    return digest, states, transitions


class Figure:
    def __init__(self, name, width, height, title, subtitle):
        self.name, self.width, self.height = name, width, height
        self.root = ET.Element("svg", {"xmlns": "http://www.w3.org/2000/svg",
            "width": str(width), "height": str(height), "viewBox": f"0 0 {width} {height}",
            "role": "img", "aria-labelledby": "title desc"})
        ET.SubElement(self.root, "title", {"id": "title"}).text = title
        ET.SubElement(self.root, "desc", {"id": "desc"}).text = subtitle
        self.boxes = []
        self.text_bounds = []
        self.rect(0, 0, width, height, fill="white", stroke="none", radius=0)
        self.text(32, 44, title, size=29, bold=True)
        self.text(32, 78, subtitle, size=18, color=MUTED)

    def rect(self, x, y, w, h, fill="white", stroke=LINE, radius=9, dashed=False):
        attrs = {"x": str(x), "y": str(y), "width": str(w), "height": str(h),
                 "rx": str(radius), "fill": fill, "stroke": stroke, "stroke-width": "1.5"}
        if dashed:
            attrs["stroke-dasharray"] = "7 5"
        ET.SubElement(self.root, "rect", attrs)

    def text(self, x, y, value, size=20, color=INK, bold=False, anchor="start", max_width=None):
        layout = PangoCairo.create_layout(cairo.Context(cairo.ImageSurface(cairo.FORMAT_ARGB32, 1, 1)))
        font = Pango.FontDescription()
        font.set_family("Noto Sans CJK SC")
        font.set_absolute_size(size * Pango.SCALE)
        font.set_weight(Pango.Weight.BOLD if bold else Pango.Weight.NORMAL)
        layout.set_font_description(font)
        layout.set_text(value, -1)
        ink, logical = layout.get_pixel_extents()
        baseline = layout.get_baseline() / Pango.SCALE
        bounds = (ink.x, ink.y - baseline, ink.x + ink.width, ink.y + ink.height - baseline)
        width = logical.width
        left = x - (width / 2 if anchor == "middle" else width if anchor == "end" else 0)
        assert max_width is None or width <= max_width, (self.name, value, width, max_width)
        assert left >= 0 and left + width <= self.width, (self.name, value, left, width)
        assert 0 <= y + bounds[1] and y + bounds[3] <= self.height, (self.name, value, y)
        self.text_bounds.append((left, y + bounds[1], left + width, y + bounds[3], value))
        attrs = {"x": str(x), "y": str(y), "font-family": "Noto Sans CJK SC",
                 "font-size": str(size), "fill": color, "text-anchor": anchor}
        if bold:
            attrs["font-weight"] = "700"
        ET.SubElement(self.root, "text", attrs).text = value

    def node(self, x, y, w, h, cn, en, warm=False):
        self.rect(x, y, w, h, fill=WARM if warm else "white")
        self.text(x + w / 2, y + 28, cn, size=21, bold=True, anchor="middle", max_width=w - 16)
        self.text(x + w / 2, y + 52, en, size=16, color=MUTED, anchor="middle", max_width=w - 16)
        self.boxes.append((x, y, w, h))

    def arrow(self, points, color=BLUE, dashed=False):
        attrs = {"points": " ".join(f"{x},{y}" for x, y in points),
                 "fill": "none", "stroke": color, "stroke-width": "2"}
        if dashed:
            attrs["stroke-dasharray"] = "6 4"
        ET.SubElement(self.root, "polyline", attrs)
        (px, py), (x, y) = points[-2:]
        angle = math.atan2(y - py, x - px)
        left = (x - 10 * math.cos(angle) + 4 * math.sin(angle),
                y - 10 * math.sin(angle) - 4 * math.cos(angle))
        right = (x - 10 * math.cos(angle) - 4 * math.sin(angle),
                 y - 10 * math.sin(angle) + 4 * math.cos(angle))
        ET.SubElement(self.root, "polygon", {"points": f"{x},{y} {left[0]},{left[1]} {right[0]},{right[1]}", "fill": color})

    def save(self, output_dir=None):
        output_dir = Path(output_dir) if output_dir is not None else HERE
        for i, a in enumerate(self.text_bounds):
            for b in self.text_bounds[i + 1:]:
                overlap = min(a[2], b[2]) - max(a[0], b[0]) > 1 and min(a[3], b[3]) - max(a[1], b[1]) > 1
                assert not overlap, (self.name, "text overlap", a[4], b[4])
        svg = ET.tostring(self.root, encoding="utf-8", xml_declaration=True)
        (output_dir / (self.name + ".svg")).write_bytes(svg)
        handle = Rsvg.Handle.new_from_data(svg)
        surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, int(self.width * 1.5), int(self.height * 1.5))
        context = cairo.Context(surface)
        context.scale(1.5, 1.5)
        viewport = Rsvg.Rectangle()
        viewport.x = viewport.y = 0
        viewport.width, viewport.height = self.width, self.height
        assert handle.render_document(context, viewport)
        surface.write_to_png(str(output_dir / (self.name + ".png")))
        return {"name": self.name, "text_items": len(self.text_bounds), "text_bounds_checked": True}


def hierarchy(states):
    fig = Figure("06-01-state-hierarchy", 1080, 990, "状态层级：外层管理功能，内层管理任务", "框内包含关系来自原模型；本图不表示执行先后，也不画转换箭头。")
    outer = [("功能关闭", "Off"), ("异常关闭", "FailOff"), ("异常保持", "KeepIPM")]
    for i, (cn, en) in enumerate(outer):
        assert states[en]["parent"] is None
        fig.node(60 + i * 330, 113, 300, 70, cn, en)
    fig.rect(40, 211, 1000, 691, fill=PALE, stroke=BLUE)
    fig.text(60, 246, "Act：功能激活范围，不等同于正在移动车辆", size=23, bold=True)
    top = [("待命", "Standby_Inner"), ("启动准备", "RTA"),
           ("驾驶员搜位", "Driver_Search"), ("泊出准备", "Wait2PrkOut")]
    for i, (cn, en) in enumerate(top):
        assert states[en]["parent"] == "Act"
        fig.node(60 + i * 245, 272, 225, 70, cn, en)
    fig.rect(60, 374, 960, 380, fill="white", stroke=BLUE)
    fig.text(80, 410, "APA_Ctrl：自动泊车任务的执行管理范围", size=23, bold=True)
    children = [("执行前准备", "StpTAct"), ("等待底盘", "ChassisWait"),
                ("执行中", "Processing"), ("任务完成", "Finish"),
                ("路径等待", "WaitOnPath"), ("重规划", "RePlan"),
                ("暂停", "Pause"), ("制动介入等待", "HangOn"),
                ("地锁等条件等待", "GrdLckWait"), ("紧急保护", "EmergencyBrk"),
                ("结束挡位等待", "WaitGear")]
    for i, (cn, en) in enumerate(children):
        assert states[en]["parent"] == "APA_Ctrl"
        fig.node(80 + i % 4 * 235, 439 + i // 4 * 97, 215, 72, cn, en,
                 warm=en in ("Pause", "EmergencyBrk"))
    for i, (cn, en) in enumerate([("接管或取消退出", "OverRide"), ("任务失败", "FAIL"), ("界面等保持", "KeepIVI")]):
        assert states[en]["parent"] == "Act"
        fig.node(60 + i * 325, 794, 310, 70, cn, en)
    assert states["Act"]["parent"] is None and states["APA_Ctrl"]["parent"] == "Act"
    assert len(states) == 23
    fig.text(40, 943, "当前位于 Processing 时，同时位于其父层 APA_Ctrl 和 Act。", size=21, bold=True)
    fig.text(40, 974, "同层运行状态互斥；图形函数、注释分组与 junction 不作为运行状态列入。", size=17, color=MUTED)
    return fig.save()


def normal_path(check):
    fig = Figure("06-02-normal-parkin", 1030, 1210, "正常泊入主线：先准备，再执行，最后确认结束", "仅选取一条正常通路；箭头文字概括主要条件，不代替全部守卫和优先顺序。")
    nodes = [("功能关闭", "Off"), ("待命", "Standby_Inner"), ("启动准备", "RTA"),
             ("驾驶员搜位", "Driver_Search"), ("执行前准备", "StpTAct"),
             ("等待底盘", "ChassisWait"), ("执行中", "Processing"),
             ("结束挡位等待", "WaitGear"), ("任务完成", "Finish")]
    edges = [
        ("Off", "Standby_Inner", ["2499", "46"], "功能启用条件满足", "经 Act 的默认入口进入待命"),
        ("Standby_Inner", "RTA", ["2503"], "收到 APA 启动请求", "待命不等于自动行驶"),
        ("RTA", "Driver_Search", ["2363"], "关联系统与新信息满足条件", "含对应感知及更新条件"),
        ("Driver_Search", "StpTAct", ["2386", "2376"], "目标、低速、制动等条件满足", "进入 APA_Ctrl 后取默认子状态"),
        ("StpTAct", "ChassisWait", ["2392"], "执行前确认条件满足", "结合车辆、路径和底盘相关条件"),
        ("ChassisWait", "Processing", ["3042"], "底盘与 MoCo 准备条件满足", "同时检查制动踏板条件"),
        ("Processing", "WaitGear", ["2377"], "MoCo 完成反馈，且速度符合条件", "还未完成全部收尾确认"),
        ("WaitGear", "Finish", ["2489"], "结束挡位与底盘状态确认", "进入任务完成阶段"),
    ]
    for i, (cn, en) in enumerate(nodes):
        fig.node(60, 107 + i * 117, 354, 68, cn, en, warm=en == "Processing")
    for i, (src, dst, ids, label, detail) in enumerate(edges):
        check(src, dst, ids)
        y = 107 + i * 117 + 68
        fig.arrow([(237, y), (237, y + 49)])
        fig.text(448, y + 16, label, size=21, max_width=550)
        fig.text(448, y + 43, detail, size=17, color=MUTED, max_width=550)
    fig.text(40, 1174, "其他转移可令流程暂停、改道或退出；此图不代表每次泊车必经且只经这些状态。", size=18, color=MUTED)
    return fig.save()


def exception_paths(check):
    fig = Figure("06-03-wait-and-exit", 1080, 1120, "停下来之后，等待的条件并不相同", "四组小图分别抽取实际转移；不是完整状态机，也不表示这些通路具有最高优先级。")
    pairs = [
        ("暂停", "Pause", "3104", "3105", "底盘条件及暂停/车辆条件成立", "新恢复确认＋相关车辆条件恢复"),
        ("路径等待", "WaitOnPath", "3047", "3048", "路径侧提出停车要求等条件成立", "路径停车要求解除"),
        ("重规划", "RePlan", "2876", "2419", "后处理的重规划状态及计数满足条件", "规划/控制进展满足＋相关风险解除"),
        ("紧急保护", "EmergencyBrk", "2398", "2404", "碰撞风险或相关保护请求成立", "风险/停车要求解除＋执行条件允许"),
    ]
    for i, (cn, en, outgoing, incoming, first, second) in enumerate(pairs):
        y = 132 + i * 198
        fig.node(40, y, 250, 76, "执行中", "Processing")
        fig.node(760, y, 280, 76, cn, en, warm=True)
        check("Processing", en, [outgoing])
        check(en, "Processing", [incoming])
        fig.arrow([(290, y + 23), (760, y + 23)])
        fig.text(525, y + 10, first, size=19, anchor="middle", max_width=458)
        fig.arrow([(760, y + 61), (290, y + 61)], color=MUTED)
        fig.text(525, y + 98, second, size=19, anchor="middle", max_width=458)
    fig.rect(32, 946, 1016, 104, fill=PALE)
    fig.node(50, 960, 280, 70, "整个执行管理范围", "APA_Ctrl")
    fig.node(760, 960, 270, 70, "接管或取消退出", "OverRide", warm=True)
    check("APA_Ctrl", "OverRide", ["2406"])
    fig.arrow([(330, 995), (760, 995)])
    fig.text(545, 981, "父层接管 / 取消条件成立", size=20, anchor="middle", max_width=420)
    fig.text(40, 1090, "父层退出不是普通暂停恢复；紧急保护还有转入重规划等其他支路，图中省略。", size=18, color=MUTED)
    return fig.save()


def coordination():
    fig = Figure("06-04-task-and-maneuver", 1080, 646, "同一项泊车任务，内部可以停车并换向", "图6-1中 APA_Ctrl 范围的跨层配合示意；非主状态转移图，非实测时序。")
    x0, step = 210, 205
    titles = ["倒车行驶", "制动停下", "换向准备", "前进行驶"]
    for i, label in enumerate(titles):
        x = x0 + i * step
        fig.text(x + 95, 147, label, size=22, bold=True, anchor="middle", max_width=190)
        if i < 3:
            fig.arrow([(x + 170, 176), (x + 215, 176)], color=LINE)
    fig.text(30, 253, "任务层", size=23, bold=True)
    fig.rect(x0, 207, 800, 90, fill=PALE, stroke=BLUE)
    fig.text(610, 242, "APA_Ctrl：仍属于自动泊车任务的执行管理", size=22, bold=True, anchor="middle", max_width=780)
    fig.text(610, 275, "此示例没有触发任务退出；具体子状态仍按反馈变化", size=18, color=MUTED, anchor="middle")
    fig.text(30, 362, "MoCo 职责", size=23, bold=True)
    moco = [("跟踪当前", "倒车段"), ("制动与", "停稳处理"), ("挡位 / 转向", "准备处理"), ("跟踪下一", "前进段")]
    for i, lines in enumerate(moco):
        x = x0 + i * step
        fig.rect(x, 321, 185, 85)
        for k, value in enumerate(lines):
            fig.text(x + 92, 353 + k * 29, value, size=21, anchor="middle", max_width=170)
    fig.text(30, 466, "车辆运动", size=23, bold=True)
    for i, value in enumerate(["向后运动", "减速至停止", "保持停止", "向前运动"]):
        x = x0 + i * step
        fig.rect(x, 432, 185, 70, fill=WARM if i in (1, 2) else "white")
        fig.text(x + 92, 476, value, size=21, anchor="middle", max_width=175)
    fig.text(32, 563, "停车只是当前机动的一部分，不自动表示任务结束。", size=23, bold=True)
    fig.text(32, 600, "图中不规定全部支路的换挡与打盘顺序；运动能否继续取决于实际准备和保护条件。", size=18, color=MUTED)
    return fig.save()


def main():
    digest, states, transitions = source_graph()
    cited = {}
    def check(src, dst, ids):
        current = states[src]["ssid"]
        for tid in ids:
            edge = transitions[tid]
            if edge["source"]:
                assert edge["source"] == current, (src, dst, tid)
            else:
                owner = next(name for name, value in states.items() if value["ssid"] == current)
                target = next(name for name, value in states.items() if value["ssid"] == edge["target"])
                assert states[target]["parent"] == owner, (src, dst, tid, "default owner")
            current = edge["target"]
            cited[tid] = edge
        assert current == states[dst]["ssid"], (src, dst, ids)
    results = [hierarchy(states), normal_path(check), exception_paths(check), coordination()]
    check("EmergencyBrk", "RePlan", ["2399"])
    evidence = {"model_sha256": digest, "chart_id": 17000, "runtime_states": states,
                "cited_transitions": cited, "figures": results,
                "guard_policy": "Labels summarize guards, not sufficient conditions; original guards and priority retained here.",
                "coordination_policy": "Conceptual example only; no measured times or universal sequencing claimed."}
    (HERE / "source_evidence.json").write_text(json.dumps(evidence, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({"runtime_states_checked": len(states), "transitions_checked": len(cited),
                      "figures": results}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
