import { CollapsibleSection, H1, H2, Text, useCanvasState, useHostTheme } from "cursor/canvas";

type Branch = { name: string; purpose: string; source?: string; children?: Branch[] };

const branches: Branch[] = [
  { name: "Decision", purpose: "任务与恢复决策", source: "ApaModule.cc / apa_recovery_context.* / ApaRecoveryDecision",
    children: [
      { name: "ManageTaskLifecycle", purpose: "识别新任务、初始化、故障锁存" },
      { name: "CheckPlanningReadiness", purpose: "匹配停稳编号、持续时间与起点安全" },
      { name: "AssessRecoveryContext", purpose: "风险分类、场景变化、跨次恢复进展" },
      { name: "SelectRecoveryAction", purpose: "保持、观察、等待、规划、查询或终止" },
      { name: "ManagePlanningJob", purpose: "GAASD 内异步提交、完成和取消；不是外部 worker" },
      { name: "AdmitPlanningResult", purpose: "核对身份与实际起点，当前场景复核后更新路径版本" },
    ] },
  { name: "Planning", purpose: "异步路径规划", source: "apa_planning_job.cpp / apaParkingPathPlanning.c / apa_parking_search.cpp",
    children: [
      { name: "PreparePlanningScene", purpose: "一次求解使用独立、不可变的场景快照" },
      { name: "ResolvePlanningRequest", purpose: "车位几何、入口、坐标约定与目标约束" },
      { name: "GenerateParkingCandidates", purpose: "生成并在求解中检查候选", children: [
        { name: "GenerateGeometricCandidates", purpose: "垂直尾入、平行、斜列、垂直头入" },
        { name: "SearchParkingRoute", purpose: "搜索根、车辆运动扩展、RS 连接、路径恢复" },
        { name: "PlanParkingExit", purpose: "从实际库内位姿向库外目标求解" },
      ] },
      { name: "EvaluateParkingCandidates", purpose: "不是全部候选生成后才开始的单向流水线", children: [
        { name: "DiscretizeParkingPath", purpose: "位置、航向、有符号区间里程与有效点数" },
        { name: "CheckPathStructure", purpose: "容量、方向及坐标与里程一致性" },
        { name: "CheckCandidateSweep", purpose: "完整车身运动扫掠与目标约束" },
        { name: "CompareCandidateQuality", purpose: "换向、短段、几何和近障等代价" },
        { name: "PreviewCandidateExecution", purpose: "独立实例中的控制、执行器和名义车辆预演" },
        { name: "ImproveAcceptedCandidate", purpose: "改善后重新检查，不直接删除短段" },
      ] },
      { name: "PlanClearanceRecovery", purpose: "许可、有界前缀、连接及完整路线复核" },
      { name: "BuildPlanningResult", purpose: "候选、请求身份和准确的失败原因" },
    ] },
  { name: "Control", purpose: "安全运动执行", source: "apa_tracking_execution.cpp / apaMoco*V2.c / apaMocoActuator.c",
    children: [
      { name: "ExecuteSafeTrackingStep", purpose: "先试算约束，再正式提交一次控制状态", children: [
        { name: "ManageManeuverExecution", purpose: "方向段、挡位、转向准备、停车及结束" },
        { name: "ProjectPathProgress", purpose: "当前方向段与历史窗口内的连续投影" },
        { name: "PrepareTrackingReference", purpose: "参考位置、航向、曲率和响应预看" },
        { name: "BuildSpeedEnvelope", purpose: "曲率、短段、近障和段末限制" },
        { name: "ComputeLongitudinalRequest", purpose: "速度 PI、加速度与 jerk 约束" },
        { name: "ComputeLateralRequest", purpose: "曲率前馈、横偏/航向反馈及受限积分" },
        { name: "ConstrainAvailableMotion", purpose: "候选控制预测下的可行运动距离" },
      ] },
      { name: "CheckOnlineMotionSafety", purpose: "当帧实际运动、响应制动预测和扫掠" },
      { name: "ArbitrateMotionCommand", purpose: "本帧安全、故障、恢复和仅规划覆盖" },
      { name: "ActuateVehicle", purpose: "最终油门、制动、手刹和挡位；Bridge 不叠加 PID" },
      { name: "BuildControlDiagnostic", purpose: "跟踪进度、误差、状态及输出诊断" },
    ] },
];

function TreeBranch({ branch, depth = 0 }: { branch: Branch; depth?: number }) {
  const theme = useHostTheme();
  if (!branch.children) return <div style={{ padding: "8px 0", borderBottom: `1px solid ${theme.stroke.tertiary}` }}>
    <div style={{ fontSize: 13, fontWeight: 600, overflowWrap: "anywhere" }}>{branch.name}</div>
    <div style={{ color: theme.text.secondary, fontSize: 13, lineHeight: 1.6 }}>{branch.purpose}</div>
  </div>;
  return <CollapsibleSection title={`${branch.name} · ${branch.purpose}`} defaultOpen={depth < 1} count={branch.children.length}>
    {branch.source && <div style={{ color: theme.text.tertiary, fontSize: 12, marginBottom: 10, overflowWrap: "anywhere" }}>源码依据：{branch.source}</div>}
    {branch.children.map(child => <TreeBranch key={child.name} branch={child} depth={depth + 1} />)}
  </CollapsibleSection>;
}

export default function ApaArchitecture() {
  const theme = useHostTheme();
  const [tab, setTab] = useCanvasState("apa-architecture-tab", "structure");
  const steps = [
    "接收输入，核对身份、时序和新任务，优先处理保护反馈与输入故障。",
    "更新任务与恢复决策；异步规划完成后，核对起点并用当前场景复核。",
    "到达新控制步时：状态副本试算 → 可用距离检查 → 正式控制状态更新一次。",
    "按本帧预测检查在线安全；必要时当帧停车、恢复或故障，不统一延后到下一帧。",
    "完成指令覆盖 → C 执行器生成踏板 → 发布匹配帧输出；同帧重发复用缓存。",
  ];
  return <main style={{ color: theme.text.primary, background: theme.bg.editor, padding: 24, maxWidth: 1320, margin: "0 auto", fontSize: 14, lineHeight: 1.65 }}>
    <Text size="small" tone="secondary">2026-11-17 · 基于 apa_1112.zip 与 carla_bridge_1112.zip</Text>
    <H1>APA 的 GAASD 层级架构</H1>
    <p style={{ color: theme.text.secondary }}>这是供审核的设计预览，不是已创建的 GAASD 画布。算法全部在 GAASD 内执行，默认具备异步能力；Bridge 的职责与协议不变。</p>
    <nav style={{ display: "flex", gap: 8, margin: "18px 0 24px" }}>
      {[["structure", "查看模块层级"], ["semantics", "查看时序与边界"]].map(([key, title]) => <button key={key} onClick={() => setTab(key)} style={{ cursor: "pointer", padding: "8px 14px", borderRadius: 5, border: `1px solid ${theme.stroke.primary}`, background: tab === key ? theme.accent.control : theme.fill.tertiary, color: tab === key ? theme.text.onAccent : theme.text.primary }}>{title}</button>)}
    </nav>
    {tab === "structure" ? <>
      <H2>Main 顶层</H2>
      <p>ValidateInput → 任务决策与异步结果处理 → 已接纳路径的周期控制 → PublishOutput。此箭头表示信息衔接，不表示同步等待规划求解。</p>
      <div style={{ display: "grid", gridTemplateColumns: "repeat(auto-fit, minmax(270px, 1fr))", gap: 24, alignItems: "start" }}>
        {branches.map(branch => <section key={branch.name} style={{ borderTop: `2px solid ${theme.stroke.primary}`, paddingTop: 12, minWidth: 0 }}><TreeBranch branch={branch} /></section>)}
      </div>
      <section style={{ marginTop: 28 }}><H2>安全按检查位置分布，底层算法共用</H2><p>任务准入、搜索扩展、候选检查、接收复核、在线控制各自调用相应安全能力。共用坐标、包络、碰撞、扫掠和制动预测定义；不同任务和预演不共享可写状态。</p></section>
      <section><H2>当前预置库不能只按名称复用</H2><p>两份本机库均有 483 条组件记录，含 144 条 Test 项，自动泊车分类为 13 条。已发现 AStar、CollisionCheck 等同名条目使用标量演示模板；APA 包装虽有真实 C 调用，也仍需核对版本、场景上下文和状态行为。基础运算优先复用，复杂算法按实际实现分类评估。</p></section>
    </> : <>
      <H2>单帧处理与异步边界</H2>
      <ol style={{ paddingLeft: 22 }}>{steps.map(step => <li key={step} style={{ padding: "8px 0" }}>{step}</li>)}</ol>
      <H2>三项不能省略的实现契约</H2>
      <p><strong>异步任务：</strong>请求身份、快照、取消、完成及过期结果拒收必须明确；没有假定某个官方任务组件已经存在。</p>
      <p><strong>状态隔离：</strong>在线控制、候选预演和搜索任务拥有各自状态；不得通过父子端口注入内部 State。控制与踏板执行器都只能按同一时间步推进一次；同帧新增安全事件可立即覆盖停车，但不能再推进滤波、偏置和积分。</p>
      <p><strong>场景与容量：</strong>100 点路径、64 条边界与 4096 个凸片不是同一数据层；不截断环境来迁就接口，不把请求级资源 ID 当作跨线程有效的地址。</p>
      <H2>Bridge 保持不动</H2><p>地图与车位、占用判断、目标构造、CARLA 数据读取、命令与身份核验、转向适配和独立保护仍由当前 Bridge 承担。GAASD 保留原对外协议和 C 踏板所有权。</p>
    </>}
    <footer style={{ marginTop: 28, paddingTop: 12, borderTop: `1px solid ${theme.stroke.primary}`, color: theme.text.tertiary, fontSize: 12 }}>
      详细设计：/data/aiden/文档/Pro/APA/APA_GAASD模块架构设计_20261117.md。当前未生成组件、改动画布或验证 CARLA 物理闭环。
    </footer>
  </main>;
}
