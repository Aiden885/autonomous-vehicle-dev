const state = {
  scenarios: [],
  selectedId: null,
  cursor: 0,
  renderedLog: "",
};

const LOG_POLL_MS = 1000;
const SCENARIO_REFRESH_MS = 5000;
const HEALTH_POLL_MS = 3000;

const $ = (id) => document.getElementById(id);

function escapeHtml(value) {
  return String(value ?? "")
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;")
    .replaceAll("'", "&#039;");
}

function line(entry) {
  const icon = entry.level === "error" ? "✕" : entry.level === "ok" ? "✓" : "·";
  return `[${entry.time}] ${icon} ${entry.message}`;
}

function stateLabel(raw) {
  const labels = {
    idle: "Idle",
    running: "Running",
    ready: "Ready",
    restored: "Restored",
    error: "Error",
  };
  return labels[raw] || raw || "Idle";
}

function setStatus(raw) {
  const pill = $("statusPill");
  pill.className = `status-pill status-${raw || "idle"}`;
  $("statusText").textContent = stateLabel(raw);
  updateActionState(raw || "idle");
}

function setButtonLoading(btn, loading) {
  if (loading) {
    btn.classList.add("loading");
  } else {
    btn.classList.remove("loading");
  }
}

function updateActionState(raw) {
  const busy = raw === "running";
  const hasScenario = Boolean(state.selectedId);

  const restoreBtn = $("restoreBtn");
  const startBtn   = $("startBtn");
  const stopBtn    = $("stopBtn");

  restoreBtn.disabled = busy || !hasScenario;
  startBtn.disabled   = busy || !hasScenario;
  stopBtn.disabled    = busy || !hasScenario;
  $("healthBtn").disabled   = busy || !hasScenario;
  $("fixGeneratedBtn").disabled = busy || !hasScenario;
  $("copyPathBtn").disabled = !hasScenario;
  $("readmeBtn").disabled   = !hasScenario;
  $("forceRestore").disabled = busy || !hasScenario;

  if (!busy) {
    setButtonLoading(restoreBtn, false);
    setButtonLoading(startBtn, false);
  }
}

async function requestJson(url, options = {}) {
  const res = await fetch(url, options);
  const data = await res.json();
  if (!res.ok) {
    throw new Error(data.error || data.message || `HTTP ${res.status}`);
  }
  return data;
}

function selectedScenario() {
  return state.scenarios.find((item) => item.id === state.selectedId) || null;
}

function renderScenarioList() {
  const root = $("scenarioList");
  root.innerHTML = "";
  if (state.scenarios.length === 0) {
    root.innerHTML = `
      <div class="scenario-item scenario-empty">
        <span class="scenario-icon" aria-hidden="true">—</span>
        <span class="scenario-info">
          <span class="scenario-name">暂无场景</span>
          <span class="scenario-date">请检查 scenarios/ 目录</span>
        </span>
      </div>`;
    updateActionState("idle");
    return;
  }
  for (const item of state.scenarios) {
    const btn = document.createElement("button");
    btn.className = `scenario-item ${item.id === state.selectedId ? "active" : ""}`;
    btn.innerHTML = `
      <span class="scenario-icon" aria-hidden="true">
        <svg viewBox="0 0 24 24" fill="none">
          <path d="M5.5 13.5h13M7.5 9.5h9M8 17h8" stroke="currentColor" stroke-width="1.7" stroke-linecap="round"/>
          <path d="M6.2 18.5 4.8 20H3.5v-2.2l1.3-1.3M19.2 16.5l1.3 1.3V20h-1.3l-1.4-1.5" stroke="currentColor" stroke-width="1.5" stroke-linecap="round" stroke-linejoin="round"/>
          <path d="M7.2 5.5h9.6c1.1 0 1.9.6 2.3 1.6l1.4 4.2c.2.5.3 1.1.3 1.6v2.6c0 1.1-.9 2-2 2H5.2c-1.1 0-2-.9-2-2v-2.6c0-.5.1-1.1.3-1.6l1.4-4.2c.4-1 1.2-1.6 2.3-1.6Z" stroke="currentColor" stroke-width="1.5"/>
        </svg>
      </span>
      <span class="scenario-info">
        <span class="scenario-name">${escapeHtml(item.name)}</span>
        <span class="scenario-date">${escapeHtml(item.created_at || item.id)}</span>
      </span>`;
    btn.onclick = () => selectScenario(item.id);
    root.appendChild(btn);
  }
}

function renderDetails() {
  const item = selectedScenario();
  if (!item) return;
  $("scenarioName").textContent = item.name;
  $("scenarioDescription").textContent = item.description || "无描述";
  $("restorePath").textContent = item.restore_path || "-";
  $("carlaInfo").textContent = `${item.environment.carla_version || "CARLA"} · ${item.environment.python || "python"}`;
  $("mapInfo").textContent = `${item.scene.map || "-"} · spawn ${item.scene.ego_spawn_index || "-"}`;
  $("leadInfo").textContent = `${item.scene.lead_behavior || "-"} · ${item.scene.lead_placement || "-"} · ${item.scene.lead_speed_mps || "-"} m/s`;
  const signals = Array.isArray(item.signals) ? item.signals : [];
  $("signalInfo").innerHTML = signals.length > 0
    ? signals.map((signal) => `<span class="signal-tag">${escapeHtml(signal)}</span>`).join("")
    : `<span class="signal-tag muted">-</span>`;
  setStatus(item.state?.status || "idle");
}

async function loadScenarios() {
  const data = await requestJson("/api/scenarios");
  state.scenarios = data.scenarios || [];
  if (!state.selectedId && state.scenarios.length > 0) {
    state.selectedId = state.scenarios[0].id;
  }
  renderScenarioList();
  renderDetails();
}

async function selectScenario(id) {
  state.selectedId = id;
  state.cursor = 0;
  state.renderedLog = "";
  $("logBox").textContent = "等待操作...";
  renderScenarioList();
  renderDetails();
  await pollLogs();
  await healthCheck(false);
}

async function runAction(action, body = {}) {
  const id = state.selectedId;
  if (!id) return;

  updateActionState("running");
  const actionBtn = action === "restore" ? $("restoreBtn") : action === "start" ? $("startBtn") : null;
  if (actionBtn) setButtonLoading(actionBtn, true);

  appendLocalLog(`开始执行: ${action}`);
  try {
    await requestJson(`/api/scenarios/${id}/${action}`, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify(body),
    });
    await pollLogs();
    await loadScenarios();
  } catch (err) {
    appendLocalLog(`操作失败: ${err.message}`);
    updateActionState("idle");
  }
}

function appendLocalLog(message) {
  const box = $("logBox");
  const prefix = state.renderedLog ? "\n" : "";
  state.renderedLog += `${prefix}[local] ${message}`;
  box.textContent = state.renderedLog;
  box.scrollTop = box.scrollHeight;
}

async function pollLogs() {
  const id = state.selectedId;
  if (!id) return;
  try {
    const data = await requestJson(`/api/scenarios/${id}/logs?since=${state.cursor}`);
    state.cursor = data.cursor;
    if (data.logs && data.logs.length > 0) {
      const text = data.logs.map(line).join("\n");
      state.renderedLog = state.renderedLog && state.renderedLog !== "等待操作..."
        ? `${state.renderedLog}\n${text}`
        : text;
      const box = $("logBox");
      box.textContent = state.renderedLog;
      box.scrollTop = box.scrollHeight;
    }
    setStatus(data.state?.status || "idle");
  } catch {
    // Keep the UI quiet during server restarts.
  }
}

function updateHealthChip(service, online, port) {
  const row = document.querySelector(`[data-service="${service}"]`);
  if (!row) return;
  row.classList.toggle("online",  online);
  row.classList.toggle("offline", !online);
  const name = row.querySelector(".health-name");
  const portNode = row.querySelector(".health-port");
  const label = service === "carla"
    ? "CARLA Server"
    : service === "bridge_pub"
      ? "Bridge PUB"
      : "Bridge CONTROL";
  if (name) name.textContent = `${label} ${online ? "在线" : "离线"}`;
  if (portNode) portNode.textContent = port ? `:${port}` : "";
}

async function healthCheck(writeLog = true) {
  const id = state.selectedId;
  if (!id) return;
  try {
    const data = await requestJson(`/api/scenarios/${id}/health?log=${writeLog ? "1" : "0"}`);
    updateHealthChip("carla",          data.carla,          data.ports?.carla);
    updateHealthChip("bridge_pub",     data.bridge_pub,     data.ports?.bridge_pub);
    updateHealthChip("bridge_control", data.bridge_control, data.ports?.bridge_control);
    if (writeLog) await pollLogs();
  } catch (err) {
    if (writeLog) appendLocalLog(`健康检查失败: ${err.message}`);
  }
}

async function copyProjectPath() {
  const item = selectedScenario();
  if (!item || !item.restore_path) {
    appendLocalLog("没有可复制的工程路径");
    return;
  }
  if (navigator.clipboard) {
    await navigator.clipboard.writeText(item.restore_path);
  } else {
    const textarea = document.createElement("textarea");
    textarea.value = item.restore_path;
    textarea.style.cssText = "position:fixed;left:-9999px;";
    document.body.appendChild(textarea);
    textarea.select();
    document.execCommand("copy");
    document.body.removeChild(textarea);
  }
  appendLocalLog(`已复制工程路径: ${item.restore_path}`);
}

function openReadme() {
  const id = state.selectedId;
  if (id) window.open(`/api/scenarios/${id}/readme`, "_blank");
}

function bindEvents() {
  $("restoreBtn").onclick = () => runAction("restore", {force: $("forceRestore").checked});
  $("startBtn").onclick   = () => runAction("start");
  $("stopBtn").onclick    = () => runAction("stop");
  $("healthBtn").onclick  = () => healthCheck(true);
  $("fixGeneratedBtn").onclick = () => runAction("fix-generated");
  $("copyPathBtn").onclick = copyProjectPath;
  $("readmeBtn").onclick  = openReadme;
  $("clearLogBtn").onclick = () => {
    state.renderedLog = "";
    $("logBox").textContent = "显示已清空，后台日志仍保留。";
  };
}

function bindVisualEffects() {
  const reduceMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  const spotlight = $("spotlight");
  const hero = $("heroCard");

  $("restorePath").onclick = copyProjectPath;

  if (reduceMotion) return;

  window.addEventListener("mousemove", (event) => {
    if (!spotlight) return;
    spotlight.style.left = `${event.clientX}px`;
    spotlight.style.top = `${event.clientY}px`;
  }, {passive: true});

  if (!hero) return;
  hero.addEventListener("mousemove", (event) => {
    const rect = hero.getBoundingClientRect();
    const dx = (event.clientX - rect.left) / rect.width - 0.5;
    const dy = (event.clientY - rect.top) / rect.height - 0.5;
    hero.style.transform = `perspective(1100px) rotateX(${(-dy * 2.2).toFixed(2)}deg) rotateY(${(dx * 2.2).toFixed(2)}deg)`;
  });
  hero.addEventListener("mouseleave", () => {
    hero.style.transform = "";
  });
}

async function boot() {
  bindEvents();
  bindVisualEffects();
  await loadScenarios();
  await healthCheck(false);
  setInterval(pollLogs, LOG_POLL_MS);
  setInterval(() => loadScenarios().catch(() => {}), SCENARIO_REFRESH_MS);
  setInterval(() => healthCheck(false).catch(() => {}), HEALTH_POLL_MS);
}

boot().catch((err) => appendLocalLog(err.message));
