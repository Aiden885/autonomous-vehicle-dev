const state = {
  scenarios: [],
  selectedId: null,
  cursor: 0,
  renderedLog: "",
  lksDriver: {brakePressed: false, steerNorm: 0},
  lksInputTimer: null,
  lksRequestBusy: false,
  lksPendingRequest: null,
  lksPressedKeys: new Set(),
  accLevelKey: null,
  accCommandQueue: Promise.resolve(),
  lksParameters: {schema: [], values: {}},
};

const LOG_POLL_MS = 1000;
const SCENARIO_REFRESH_MS = 5000;
const HEALTH_POLL_MS = 3000;

const $ = (id) => document.getElementById(id);

function bindClick(id, handler) {
  const element = $(id);
  if (element) element.onclick = handler;
}

function setDisabled(id, disabled) {
  const element = $(id);
  if (element) element.disabled = disabled;
}

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
  setDisabled("kickstartBtn", busy || !hasScenario);
  setDisabled("lksReleaseBtn", busy || !hasScenario);
  setDisabled("lksResultBtn", busy || !hasScenario);
  setDisabled("lksParameterSaveBtn", busy || !hasScenario);
  setDisabled("lksParameterResetBtn", busy || !hasScenario);
  document.querySelectorAll("[data-lks-param]").forEach((input) => {
    input.disabled = busy || !hasScenario;
  });
  document.querySelectorAll("[data-command]").forEach((button) => {
    button.disabled = busy || !hasScenario;
  });
  document.querySelectorAll("[data-lks-steer], [data-lks-brake]").forEach((button) => {
    button.disabled = busy || !hasScenario;
  });

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
  if (!root) return;
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
  const profile = item.control_profile || "acc";
  const accDriverCard = $("accDriverCard");
  const lksDriverCard = $("lksDriverCard");
  const lksParameterCard = $("lksParameterCard");
  if (accDriverCard) accDriverCard.hidden = profile !== "acc";
  if (lksDriverCard) lksDriverCard.hidden = profile !== "lks";
  if (lksParameterCard) lksParameterCard.hidden = profile !== "lks" || !item.has_parameters;
  if (profile !== "lks") releaseLksDriverState(false);
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
  await loadLksParameters();
  await pollLogs();
  await healthCheck(false);
}

function renderLksParameters() {
  const root = $("lksParameterGroups");
  if (!root) return;
  const groups = new Map();
  for (const item of state.lksParameters.schema) {
    const group = item.group || "参数";
    if (!groups.has(group)) groups.set(group, []);
    groups.get(group).push(item);
  }
  root.innerHTML = [...groups.entries()].map(([group, items]) => `
    <section class="parameter-group">
      <h3>${escapeHtml(group)}</h3>
      <div class="parameter-grid">
        ${items.map((item) => `
          <label class="parameter-field" title="${escapeHtml(item.description || "")}">
            <span class="parameter-label">${escapeHtml(item.label || item.key)}</span>
            <span class="parameter-input-row">
              <input type="number"
                     data-lks-param="${escapeHtml(item.key)}"
                     value="${escapeHtml(state.lksParameters.values[item.key] ?? item.default)}"
                     min="${escapeHtml(item.min)}"
                     max="${escapeHtml(item.max)}"
                     step="${escapeHtml(item.step || "any")}">
              <span>${escapeHtml(item.unit || "")}</span>
            </span>
            <small>${escapeHtml(item.description || "")}</small>
          </label>`).join("")}
      </div>
    </section>`).join("");
}

async function loadLksParameters() {
  const item = selectedScenario();
  if (!item || item.control_profile !== "lks" || !item.has_parameters) {
    state.lksParameters = {schema: [], values: {}};
    return;
  }
  try {
    const data = await requestJson(`/api/scenarios/${item.id}/parameters`);
    state.lksParameters = {schema: data.schema || [], values: data.values || {}};
    renderLksParameters();
  } catch (err) {
    appendLocalLog(`LKS参数加载失败: ${err.message}`);
  }
}

function collectLksParameters() {
  const values = {};
  document.querySelectorAll("[data-lks-param]").forEach((input) => {
    values[input.dataset.lksParam] = input.value;
  });
  return values;
}

async function saveLksParameters(writeLog = true) {
  const item = selectedScenario();
  if (!item || item.control_profile !== "lks" || !item.has_parameters) return true;
  try {
    const data = await requestJson(`/api/scenarios/${item.id}/parameters`, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({values: collectLksParameters()}),
    });
    state.lksParameters.values = data.values || {};
    renderLksParameters();
    if (writeLog) appendLocalLog("LKS参数已保存，将在下次启动时生效");
    return true;
  } catch (err) {
    appendLocalLog(`LKS参数保存失败: ${err.message}`);
    return false;
  }
}

async function resetLksParameters() {
  const item = selectedScenario();
  if (!item || item.control_profile !== "lks" || !item.has_parameters) return;
  try {
    const data = await requestJson(`/api/scenarios/${item.id}/parameters`, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({reset: true}),
    });
    state.lksParameters.values = data.values || {};
    renderLksParameters();
    appendLocalLog("LKS参数已恢复默认值");
  } catch (err) {
    appendLocalLog(`恢复默认参数失败: ${err.message}`);
  }
}

async function startSelectedScenario() {
  if (!(await saveLksParameters(false))) return;
  await runAction("start");
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
  const labels = {
    carla: "CARLA Server",
    bridge_pub: "Bridge PUB",
    bridge_control: "Bridge CONTROL",
    pangu_process: "Pangu 业务进程",
  };
  const label = labels[service] || service;
  if (name) name.textContent = `${label} ${online ? "在线" : "离线"}`;
  if (portNode) {
    portNode.textContent = Number.isFinite(Number(port)) ? `:${port}` : (port || "");
  }
}

async function healthCheck(writeLog = true) {
  const id = state.selectedId;
  if (!id) return;
  try {
    const data = await requestJson(`/api/scenarios/${id}/health?log=${writeLog ? "1" : "0"}`);
    updateHealthChip("carla",          data.carla,          data.ports?.carla);
    updateHealthChip("bridge_pub",     data.bridge_pub,     data.ports?.bridge_pub);
    updateHealthChip("bridge_control", data.bridge_control, data.ports?.bridge_control);
    updateHealthChip("pangu_process",  data.pangu_process,  data.ports?.pangu_process);
    if (writeLog) await pollLogs();
  } catch (err) {
    if (writeLog) appendLocalLog(`健康检查失败: ${err.message}`);
  }
}

async function sendDriverCommand(key) {
  const id = state.selectedId;
  if (!id) return;
  appendLocalLog(`发送驾驶指令: ${key.toUpperCase()}`);
  try {
    await requestJson(`/api/scenarios/${id}/driver-command`, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({key}),
    });
    await pollLogs();
  } catch (err) {
    appendLocalLog(`驾驶指令失败: ${err.message}`);
  }
}

function queueDriverCommand(key) {
  state.accCommandQueue = state.accCommandQueue
    .catch(() => {})
    .then(() => sendDriverCommand(key));
  return state.accCommandQueue;
}

async function kickstartAcc() {
  const id = state.selectedId;
  if (!id) return;
  appendLocalLog("执行辅助启控: boost + E");
  $("kickstartBtn").disabled = true;
  try {
    await requestJson(`/api/scenarios/${id}/kickstart`, {method: "POST"});
    await pollLogs();
    await healthCheck(false);
  } catch (err) {
    appendLocalLog(`辅助启控失败: ${err.message}`);
  } finally {
    $("kickstartBtn").disabled = false;
  }
}

async function sendLksDriverState(writeLog = false) {
  const id = state.selectedId;
  const item = selectedScenario();
  if (!id || item?.control_profile !== "lks") return;
  const request = {
    brakePressed: state.lksDriver.brakePressed,
    steerNorm: state.lksDriver.steerNorm,
    writeLog,
  };
  if (state.lksRequestBusy) {
    // Always keep the newest state. In particular, a release must not be
    // discarded while the preceding press request is still in flight.
    state.lksPendingRequest = request;
    return;
  }
  state.lksRequestBusy = true;
  try {
    let current = request;
    while (current) {
      state.lksPendingRequest = null;
      try {
        await requestJson(`/api/scenarios/${id}/lks-driver-state`, {
          method: "POST",
          headers: {"Content-Type": "application/json"},
          body: JSON.stringify({
            brake_pressed: current.brakePressed,
            driver_steer_norm: current.steerNorm,
          }),
        });
        if (current.writeLog) {
          appendLocalLog(`LKS驾驶输入: brake=${Number(current.brakePressed)}, steer=${current.steerNorm.toFixed(2)}`);
        }
      } catch (err) {
        if (current.writeLog) appendLocalLog(`LKS驾驶输入失败: ${err.message}`);
      }
      current = state.lksPendingRequest;
    }
  } finally {
    state.lksRequestBusy = false;
  }
}

function refreshLksButtonState() {
  document.querySelectorAll("[data-lks-steer]").forEach((button) => {
    button.classList.toggle("active", Number(button.dataset.lksSteer) === state.lksDriver.steerNorm);
  });
  document.querySelectorAll("[data-lks-brake]").forEach((button) => {
    button.classList.toggle("active", state.lksDriver.brakePressed);
  });
}

function startLksDriverState({steerNorm = 0, brakePressed = false}) {
  state.lksDriver.steerNorm = steerNorm;
  state.lksDriver.brakePressed = brakePressed;
  refreshLksButtonState();
  sendLksDriverState(true);
  if (state.lksInputTimer) clearInterval(state.lksInputTimer);
  state.lksInputTimer = setInterval(() => sendLksDriverState(false), 250);
}

function releaseLksDriverState(writeLog = true) {
  if (state.lksInputTimer) clearInterval(state.lksInputTimer);
  state.lksInputTimer = null;
  const changed = state.lksDriver.brakePressed || state.lksDriver.steerNorm !== 0;
  state.lksDriver = {brakePressed: false, steerNorm: 0};
  refreshLksButtonState();
  if (changed || writeLog) sendLksDriverState(writeLog);
}

function syncLksKeyboardState(writeLog = true) {
  const brakePressed = state.lksPressedKeys.has("b");
  const leftPressed = state.lksPressedKeys.has("a");
  const rightPressed = state.lksPressedKeys.has("d");
  let steerNorm = 0;
  if (leftPressed !== rightPressed) steerNorm = leftPressed ? -0.3 : 0.3;
  if (brakePressed || steerNorm !== 0) {
    startLksDriverState({steerNorm: brakePressed ? 0 : steerNorm, brakePressed});
  } else {
    releaseLksDriverState(writeLog);
  }
}

function startAccLevelInput(key) {
  if (state.accLevelKey === key) return;
  if (state.accLevelKey) queueDriverCommand("0");
  state.accLevelKey = key;
  document.querySelector(`[data-command="${key}"]`)?.classList.add("active");
  queueDriverCommand(key);
}

function releaseAccLevelInput() {
  if (!state.accLevelKey) return;
  document.querySelector(`[data-command="${state.accLevelKey}"]`)?.classList.remove("active");
  state.accLevelKey = null;
  queueDriverCommand("0");
}

function keyboardTargetIsEditable(event) {
  const target = event.target;
  if (!(target instanceof HTMLElement)) return false;
  const tag = target.tagName.toLowerCase();
  return tag === "input" || tag === "textarea" || target.isContentEditable;
}

async function readLksResult() {
  const id = state.selectedId;
  if (!id) return;
  try {
    const data = await requestJson(`/api/scenarios/${id}/results/latest`);
    const result = data.result || {};
    appendLocalLog(
      `LKS结果: passed=${result.passed}, curve=${result.curve_covered}, ` +
      `碰撞=${result.collision_count}, 弯道最大偏差=${result.curve_max_abs_lateral_offset_m ?? "-"}m, ` +
      `最高车速=${Number(result.max_speed_mps || 0).toFixed(2)}m/s, 样本=${result.sample_count || 0}`
    );
  } catch (err) {
    appendLocalLog(`LKS测试结果尚不可用: ${err.message}`);
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

async function copyLogs() {
  const text = $("logBox").textContent || "";
  if (!text.trim()) {
    appendLocalLog("没有可复制的运行日志");
    return;
  }
  if (navigator.clipboard) {
    await navigator.clipboard.writeText(text);
  } else {
    const textarea = document.createElement("textarea");
    textarea.value = text;
    textarea.style.cssText = "position:fixed;left:-9999px;";
    document.body.appendChild(textarea);
    textarea.select();
    document.execCommand("copy");
    document.body.removeChild(textarea);
  }
  appendLocalLog("运行日志已复制");
}

function openReadme() {
  const id = state.selectedId;
  if (id) window.open(`/api/scenarios/${id}/readme`, "_blank");
}

function bindEvents() {
  bindClick("restoreBtn", () => runAction("restore", {force: $("forceRestore").checked}));
  bindClick("startBtn", startSelectedScenario);
  bindClick("stopBtn", () => runAction("stop"));
  bindClick("healthBtn", () => healthCheck(true));
  bindClick("fixGeneratedBtn", () => runAction("fix-generated"));
  bindClick("copyPathBtn", copyProjectPath);
  bindClick("readmeBtn", openReadme);
  bindClick("copyLogBtn", () => copyLogs().catch((err) => appendLocalLog(`复制日志失败: ${err.message}`)));
  bindClick("kickstartBtn", kickstartAcc);
  bindClick("lksReleaseBtn", () => releaseLksDriverState(true));
  bindClick("lksResultBtn", readLksResult);
  bindClick("lksParameterSaveBtn", () => saveLksParameters(true));
  bindClick("lksParameterResetBtn", resetLksParameters);
  document.querySelectorAll("[data-command]").forEach((button) => {
    const key = button.dataset.command || "";
    const isLevel = button.dataset.commandMode === "level";
    if (!isLevel) {
      button.onclick = () => queueDriverCommand(key);
      return;
    }
    button.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      startAccLevelInput(key);
    });
    button.addEventListener("pointerup", releaseAccLevelInput);
    button.addEventListener("pointercancel", releaseAccLevelInput);
    button.addEventListener("pointerleave", releaseAccLevelInput);
  });
  document.querySelectorAll("[data-lks-steer]").forEach((button) => {
    const press = (event) => {
      event.preventDefault();
      startLksDriverState({steerNorm: Number(button.dataset.lksSteer), brakePressed: false});
    };
    button.addEventListener("pointerdown", press);
    button.addEventListener("pointerup", () => releaseLksDriverState(true));
    button.addEventListener("pointercancel", () => releaseLksDriverState(true));
    button.addEventListener("pointerleave", () => releaseLksDriverState(false));
  });
  document.querySelectorAll("[data-lks-brake]").forEach((button) => {
    button.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      startLksDriverState({steerNorm: 0, brakePressed: true});
    });
    button.addEventListener("pointerup", () => releaseLksDriverState(true));
    button.addEventListener("pointercancel", () => releaseLksDriverState(true));
    button.addEventListener("pointerleave", () => releaseLksDriverState(false));
  });
  window.addEventListener("keydown", (event) => {
    if (keyboardTargetIsEditable(event)) return;
    const profile = selectedScenario()?.control_profile;
    const key = event.key.toLowerCase();
    if (profile === "lks" && ["a", "d", "b"].includes(key)) {
      event.preventDefault();
      if (event.repeat) return;
      state.lksPressedKeys.add(key);
      syncLksKeyboardState(true);
      return;
    }
    if (profile === "acc" && ["e", "q", "t", "r", "c", "w", "s", "0"].includes(key)) {
      event.preventDefault();
      if (event.repeat) return;
      if (["w", "s"].includes(key)) startAccLevelInput(key);
      else if (key === "0") releaseAccLevelInput();
      else queueDriverCommand(key);
    }
  });
  window.addEventListener("keyup", (event) => {
    const key = event.key.toLowerCase();
    if (["a", "d", "b"].includes(key)) {
      state.lksPressedKeys.delete(key);
      syncLksKeyboardState(true);
    }
    if (["w", "s"].includes(key) && state.accLevelKey === key) releaseAccLevelInput();
  });
  window.addEventListener("blur", () => {
    state.lksPressedKeys.clear();
    releaseLksDriverState(false);
    releaseAccLevelInput();
  });
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) {
      state.lksPressedKeys.clear();
      releaseLksDriverState(false);
      releaseAccLevelInput();
    }
  });
  bindClick("clearLogBtn", () => {
    state.renderedLog = "";
    $("logBox").textContent = "显示已清空，后台日志仍保留。";
  });
}

function bindVisualEffects() {
  const reduceMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
  const spotlight = $("spotlight");
  const hero = $("heroCard");

  bindClick("restorePath", copyProjectPath);

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
  await loadLksParameters();
  await healthCheck(false);
  setInterval(pollLogs, LOG_POLL_MS);
  setInterval(() => loadScenarios().catch(() => {}), SCENARIO_REFRESH_MS);
  setInterval(() => healthCheck(false).catch(() => {}), HEALTH_POLL_MS);
}

boot().catch((err) => appendLocalLog(err.message));
