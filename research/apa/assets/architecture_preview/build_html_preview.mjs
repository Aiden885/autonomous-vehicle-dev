import { readFileSync, writeFileSync } from 'node:fs';
import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { runInNewContext } from 'node:vm';

const sourcePath = process.argv[2] || '/home/aiden/.cursor/projects/home-aiden-Modularization/canvases/APA-GAASD-module-architecture.canvas.tsx';
const outputPath = process.argv[3] || resolve(dirname(fileURLToPath(import.meta.url)), '../../APA_GAASD模块架构预览_20261117.html');
const source = readFileSync(sourcePath, 'utf8');
const match = source.match(/const branches: Branch\[\] = (\[[\s\S]*?\n\]);/);
if (!match) throw new Error('Cannot locate the architecture tree in the source TSX.');
const branches = runInNewContext(`(${match[1]})`, Object.create(null), { timeout: 1000 });
const escape = value => String(value).replace(/[&<>"']/g, char => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[char]));
let nodeCount = 0;
function renderNode(node, depth = 0) {
  nodeCount += 1;
  const label = `<span class="node-name">${escape(node.name)}</span><span class="node-purpose">${escape(node.purpose)}</span>`;
  if (!node.children) return `<li class="leaf" data-module="${escape(node.name)}">${label}</li>`;
  return `<li class="branch" data-module="${escape(node.name)}"><details${depth === 0 ? ' open' : ''}>
    <summary>${label}<span class="count">${node.children.length} 项</span></summary>
    ${node.source ? `<p class="source">源码依据：${escape(node.source)}</p>` : ''}
    <ul class="tree">${node.children.map(child => renderNode(child, depth + 1)).join('\n')}</ul>
  </details></li>`;
}
const columns = branches.map((branch, index) => `<section class="column" aria-label="${escape(branch.purpose)}"><div class="column-number">0${index + 1}</div><ul class="tree root-tree">${renderNode(branch)}</ul></section>`).join('\n');

const html = `<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="light dark">
<meta http-equiv="Content-Security-Policy" content="default-src 'none'; script-src 'unsafe-inline'; style-src 'unsafe-inline'; img-src data:; connect-src 'none'; object-src 'none'; base-uri 'none'; form-action 'none'">
<title>APA 的 GAASD 层级架构</title>
<style>
:root{--bg:#f5f7fa;--paper:#fff;--ink:#172a3d;--muted:#55677a;--faint:#758498;--line:#dce3ec;--accent:#175a9b;--selected:#e8f0f9;--hover:#f0f4f9}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font-family:"Noto Sans CJK SC","Microsoft YaHei",sans-serif;font-size:15px;line-height:1.75}
main{max-width:1440px;margin:auto;padding:38px 32px 28px}.eyebrow{font-size:12px;letter-spacing:.08em;color:var(--muted)}h1{font-size:30px;line-height:1.35;font-weight:650;margin:10px 0 14px}h2{font-size:19px;font-weight:650;margin:0 0 10px}p{margin:8px 0}.intro{max-width:1000px;color:var(--muted)}.status{display:inline-block;margin:12px 0 0;padding:2px 10px;border:1px solid var(--line);border-radius:4px;font-size:12px;color:var(--accent)}
button{font:inherit;cursor:pointer}button:focus-visible,summary:focus-visible,a:focus-visible{outline:3px solid var(--accent);outline-offset:3px}.tabs{display:flex;gap:24px;flex-wrap:wrap;margin:28px 0 26px;border-bottom:1px solid var(--line)}.tab{padding:11px 0;background:none;color:var(--muted);border:0;border-bottom:3px solid transparent;font-weight:600}.tab[aria-selected=true]{color:var(--accent);border-bottom-color:var(--accent)}[hidden]{display:none!important}
.section-head{display:flex;justify-content:space-between;align-items:center;gap:16px;flex-wrap:wrap}.tools{display:flex;gap:8px;flex-wrap:wrap}.tool{background:var(--paper);border:1px solid var(--line);color:var(--ink);border-radius:5px;padding:5px 12px;font-size:13px}.tool:hover{background:var(--hover)}.flow{color:var(--muted);margin-bottom:20px}.flow strong{color:var(--ink)}.columns{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:24px;align-items:start}.column{min-width:0;border-top:2px solid var(--accent);padding:14px 18px 18px;background:var(--paper)}.column-number{color:var(--faint);font:13px ui-monospace,monospace;margin-bottom:10px}
.tree{list-style:none;margin:0;padding:0}.tree .tree{margin-left:7px;border-left:1px solid var(--line);padding-left:14px}.root-tree>.branch>details>summary{font-size:17px}.branch{min-width:0;margin:6px 0}.leaf{border-bottom:1px solid var(--line);padding:10px 0}.node-name{display:block;overflow-wrap:anywhere;line-height:1.5;font-size:13px;font-weight:650}.node-purpose{display:block;font-size:13px;color:var(--muted);font-weight:400;line-height:1.7;margin-top:3px;overflow-wrap:anywhere}summary{cursor:pointer;position:relative;padding:8px 8px 8px 20px;border-radius:3px;list-style:none}summary::-webkit-details-marker{display:none}summary::before{content:'›';position:absolute;left:2px;top:6px;font-size:19px;transform:rotate(0deg);color:var(--accent)}details[open]>summary::before{transform:rotate(90deg)}summary:hover{background:var(--hover)}.count{display:inline-block;font-size:11px;color:var(--faint);margin-top:4px}.source{font:11px/1.7 ui-monospace,monospace;overflow-wrap:anywhere;color:var(--faint);padding-left:20px;margin:4px 0 12px}
.notes{margin-top:30px;padding-top:22px;border-top:1px solid var(--line)}.notes p{max-width:1120px;color:var(--muted)}.steps{counter-reset:steps;list-style:none;margin:18px 0 30px;padding:0}.steps li{position:relative;counter-increment:steps;padding:16px 16px 16px 56px;border-bottom:1px solid var(--line);background:var(--paper)}.steps li::before{content:counter(steps,decimal-leading-zero);position:absolute;left:16px;color:var(--accent);font:16px ui-monospace,monospace;top:21px}.contracts{max-width:1100px}.contracts p{padding:8px 0}footer{margin-top:32px;border-top:1px solid var(--line);padding-top:16px;font-size:12px;color:var(--faint);overflow-wrap:anywhere}a{color:var(--accent)}
@media(max-width:1050px){main{padding:28px 22px}.columns{grid-template-columns:1fr}.column{padding:14px 18px}.root-tree>.branch>details>.tree{display:block}h1{font-size:26px}}
@media(max-width:480px){main{padding:22px 14px}h1{font-size:24px}.column{padding:12px}.tabs{gap:18px}.node-name{font-size:12px}.tree .tree{padding-left:10px;margin-left:3px}}
@media(prefers-color-scheme:dark){:root{--bg:#111923;--paper:#18232f;--ink:#e5edf7;--muted:#b1bfce;--faint:#8d9db0;--line:#344455;--accent:#9bc8ff;--selected:#23384f;--hover:#233346}}
@media print{body{background:white;color:black}main{max-width:none;padding:0}.tabs,.tools{display:none}.columns{display:block}.column{break-inside:avoid;background:none;margin-bottom:20px}summary::before{display:none}footer{font-size:10px}}
</style>
</head>
<body>
<main>
<header><div class="eyebrow">APA / GAASD · 2026-11-17</div><h1>APA 的 GAASD 层级架构</h1>
<p class="intro">基于 apa_1112.zip 与 carla_bridge_1112.zip。算法全部在 GAASD 内执行，默认具备异步能力；Bridge 的职责与协议保持不变。</p>
<span class="status">供审核的设计预览 · 不是已创建的 GAASD 画布</span></header>
<nav class="tabs" role="tablist" aria-label="架构预览内容">
<button type="button" class="tab" role="tab" id="tab-structure" aria-controls="panel-structure" aria-selected="true" tabindex="0">模块层级</button>
<button type="button" class="tab" role="tab" id="tab-semantics" aria-controls="panel-semantics" aria-selected="false" tabindex="-1">时序与边界</button>
</nav>
<section id="panel-structure" role="tabpanel" aria-labelledby="tab-structure">
<div class="section-head"><h2>Main 顶层</h2><div class="tools"><button type="button" class="tool" id="expand-all">全部展开</button><button type="button" class="tool" id="collapse-all">全部折叠</button></div></div>
<p class="flow"><strong>ValidateInput</strong> → 任务决策与异步结果处理 → 已接纳路径的周期控制 → <strong>PublishOutput</strong><br>箭头表示信息衔接，不表示同步等待规划求解。点击模块标题可展开或折叠。</p>
<div class="columns">${columns}</div>
<section class="notes"><h2>安全按检查位置分布，底层算法共用</h2><p>任务准入、搜索扩展、候选检查、接收复核、在线控制各自调用相应安全能力。共用坐标、包络、碰撞、扫掠和制动预测定义；不同任务和预演不共享可写状态。</p></section>
<section class="notes"><h2>当前预置库不能只按名称复用</h2><p>两份本机库均有 483 条组件记录，含 144 条 Test 项，自动泊车分类为 13 条。已发现 AStar、CollisionCheck 等同名条目使用标量演示模板；APA 包装虽有真实 C 调用，也仍需核对版本、场景上下文和状态行为。基础运算优先复用，复杂算法按实际实现分类评估。</p></section>
</section>
<section id="panel-semantics" role="tabpanel" aria-labelledby="tab-semantics" hidden>
<h2>单帧处理与异步边界</h2><ol class="steps">
<li>接收输入，核对身份、时序和新任务，优先处理保护反馈与输入故障。</li>
<li>更新任务与恢复决策；异步规划完成后，核对起点并用当前场景复核。</li>
<li>到达新控制步时：状态副本试算 → 可用距离检查 → 正式控制状态更新一次。</li>
<li>按本帧预测检查在线安全；必要时当帧停车、恢复或故障，不统一延后到下一帧。</li>
<li>完成指令覆盖 → C 执行器生成踏板 → 发布匹配帧输出；同帧重发复用缓存。</li>
</ol>
<div class="contracts"><h2>三项不能省略的实现契约</h2>
<p><strong>异步任务：</strong>请求身份、快照、取消、完成及过期结果拒收必须明确；没有假定某个官方任务组件已经存在。</p>
<p><strong>状态隔离：</strong>在线控制、候选预演和搜索任务拥有各自状态；不得通过父子端口注入内部 State。控制与踏板执行器都只能按同一时间步推进一次；同帧新增安全事件可立即覆盖停车，但不能再推进滤波、偏置和积分。</p>
<p><strong>场景与容量：</strong>100 点路径、64 条边界与 4096 个凸片不是同一数据层；不截断环境来迁就接口，不把请求级资源 ID 当作跨线程有效的地址。</p>
<section class="notes"><h2>Bridge 保持不动</h2><p>地图与车位、占用判断、目标构造、CARLA 数据读取、命令与身份核验、转向适配和独立保护仍由当前 Bridge 承担。GAASD 保留原对外协议和 C 踏板所有权。</p></section></div>
</section>
<noscript><p>当前浏览器禁用了 JavaScript，模块仍可展开；时序说明请参阅同目录的设计文档。</p></noscript>
<footer>详细设计：<a href="APA_GAASD模块架构设计_20261117.md">APA_GAASD模块架构设计_20261117.md</a>。当前未生成组件、改动画布或验证 CARLA 物理闭环。<br>此 HTML 可离线打开，无外部脚本、字体或网络依赖。模块树由原 TSX 数据生成，共 ${nodeCount} 个节点。</footer>
</main>
<script>
'use strict';
const tabs = Array.from(document.querySelectorAll('[role="tab"]'));
function activateTab(tab) {
  tabs.forEach(item => {
    const active = item === tab;
    item.setAttribute('aria-selected', String(active));
    item.tabIndex = active ? 0 : -1;
    document.getElementById(item.getAttribute('aria-controls')).hidden = !active;
  });
}
tabs.forEach((tab, index) => {
  tab.addEventListener('click', () => activateTab(tab));
  tab.addEventListener('keydown', event => {
    const indexes = {ArrowRight: (index + 1) % tabs.length, ArrowLeft: (index + tabs.length - 1) % tabs.length, Home: 0, End: tabs.length - 1};
    if (!(event.key in indexes)) return;
    event.preventDefault();
    const next = tabs[indexes[event.key]];
    activateTab(next);
    next.focus();
  });
});
document.getElementById('expand-all').addEventListener('click', () => document.querySelectorAll('details').forEach(item => item.open = true));
document.getElementById('collapse-all').addEventListener('click', () => document.querySelectorAll('details').forEach(item => item.open = false));
</script>
</body>
</html>`;
writeFileSync(outputPath, html, 'utf8');
console.log(JSON.stringify({ outputPath, nodes: nodeCount, bytes: Buffer.byteLength(html) }));
