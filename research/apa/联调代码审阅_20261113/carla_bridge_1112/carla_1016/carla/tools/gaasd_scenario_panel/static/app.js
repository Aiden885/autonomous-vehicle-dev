const state = {
  scenarios: [],
  selectedId: null,
  scenarioBusy: false,
  scenarioStatus: "idle",
  cursor: 0,
  logEpoch: 0,
  renderedLog: "",
  lksDriver: {brakePressed: false, steerNorm: 0},
  lksInputTimer: null,
  lksRequestBusy: false,
  lksPendingRequest: null,
  lksPressedKeys: new Set(),
  lksPedalKeys: new Set(),
  lksPedal: null,
  lksVehicleTimer: null,
  lksVehicleBusy: false,
  lksVehiclePending: false,
  accLevelKey: null,
  accLevelTimer: null,
  accCommandQueue: Promise.resolve(),
  accCommandFeedbackTimers: new Map(),
  accTelemetryBusy: false,
  accCommandFeedbackOverride: null,
  accHeld: new Set(),
  accVehicle: {
    ignition: "ON",
    gear: "D",
    apa: false,
    lka: false,
    aeb: false,
    laneChange: "",
    restartPending: false,
  },
  accVehicleTimer: null,
  accVehicleBusy: false,
  accVehiclePending: false,
  accRestartTimer: null,
  accPointerHeld: new Set(),
  scenarioParameters: {schema: [], values: {}},
  apaHeld: new Set(),
  apaVehicle: {ignition: "ON", gear: "D", handBrake: false},
  apaVehicleTimer: null,
  apaVehicleBusy: false,
  apaVehiclePending: false,
  apaTelemetryBusy: false,
  apaSamples: [],
  apaAlgoPath: [],
  apaActualPath: [],
  apaSampleT0: null,
  apaReportSnapshot: null,
  apaTelemetry: {},
  apaTaskBusy: false,
  apaTaskAction: "",
  apaTaskRevision: 0,
  apaTaskUnconfirmed: false, // Awaiting the final receipt, including accepted/running operations.
  apaUnconfirmedEventId: "",
  apaSubmittedCommand: null,
  apaObservedCommand: null, // Identity of an operation submitted by another page; never replay its partial payload.
  apaPendingMap: "",
  apaMapChoicesKey: "",
  apaDraftDirty: false,
  apaSceneKey: "",
  apaHistoryKey: "",
  apaPollRevision: 0,
  apaVehicleKey: "",
  apaMapView: "nearby",
  apaMapLayers: {forbidden: true, static: true, live: true, unknown: true, boundary: true, virtual: true, slots: true},
  apaParkingMap: {key: "", available: false, slots: []},
  apaParkingMapRequest: null,
  apaReplanTaskKey: "",
  apaLastReplanReason: "",
};

const LOG_POLL_MS = 1000;
const SCENARIO_REFRESH_MS = 5000;
const HEALTH_POLL_MS = 3000;
const ACC_TELEMETRY_POLL_MS = 300;

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
  state.scenarioStatus = raw || "idle";
  if (selectedScenario()?.control_profile === "apa") {
    renderApaHeader();
  } else {
    const pill = $("statusPill");
    pill.className = `status-pill status-${raw || "idle"}`;
    $("statusText").textContent = stateLabel(raw);
  }
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
  state.scenarioBusy = raw === "running";
  const busy = state.scenarioBusy || (selectedScenario()?.control_profile === "apa" && state.apaTaskBusy);
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
  setDisabled("accResultBtn", busy || !hasScenario);
  setDisabled("lksReleaseBtn", busy || !hasScenario);
  setDisabled("lksResultBtn", busy || !hasScenario);
  setDisabled("scenarioParameterSaveBtn", busy || !hasScenario);
  setDisabled("scenarioParameterResetBtn", busy || !hasScenario);
  document.querySelectorAll("[data-scenario-param]").forEach((input) => {
    input.disabled = busy || !hasScenario;
  });
  document.querySelectorAll("[data-command], [data-ignition], [data-gear], [data-hold], [data-latch], [data-lane]").forEach((button) => {
    button.disabled = busy || !hasScenario;
  });
  document.querySelectorAll("[data-lks-steer], [data-lks-brake], [data-lks-pedal]").forEach((button) => {
    button.disabled = busy || !hasScenario;
  });
  if (selectedScenario()?.control_profile === "apa") {
    refreshApaConsole();
    updateApaTaskControls();
  }
  document.querySelectorAll('input[name="carlaFrameMode"]').forEach((input) => {
    input.disabled = busy;
  });
  document.querySelectorAll('input[name="carlaQualityMode"]').forEach((input) => {
    input.disabled = busy;
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
    const error = new Error(data.error || data.message || `HTTP ${res.status}`);
    error.status = res.status;
    error.data = data;
    throw error;
  }
  return data;
}

function selectedScenario() {
  return state.scenarios.find((item) => item.id === state.selectedId) || null;
}

function selectedCarlaFrameMode() {
  return document.querySelector('input[name="carlaFrameMode"]:checked')?.value || "stable";
}

function selectedCarlaQualityMode() {
  return document.querySelector('input[name="carlaQualityMode"]:checked')?.value || "low";
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
  const profile = item.control_profile || "acc";
  $("scenarioName").textContent = item.name;
  $("scenarioDescription").textContent = profile === "apa" ? "" : item.description || "无描述";
  $("restorePath").textContent = item.restore_path || "-";
  $("carlaInfo").textContent = `${item.environment.carla_version || "CARLA"} · ${item.environment.python || "python"}`;
  if (profile !== "apa") {
    $("mapInfo").textContent = `${item.scene.map || "-"} · spawn ${item.scene.ego_spawn_index || "-"}`;
    $("mapInfo").removeAttribute("title");
  }
  $("leadInfo").textContent = `${item.scene.lead_behavior || "-"} · ${item.scene.lead_placement || "-"} · ${item.scene.lead_speed_mps || "-"} m/s`;
  const signals = Array.isArray(item.signals) ? item.signals : [];
  $("signalInfo").innerHTML = signals.length > 0
    ? signals.map((signal) => `<span class="signal-tag">${escapeHtml(signal)}</span>`).join("")
    : `<span class="signal-tag muted">-</span>`;
  document.body.classList.toggle("is-apa", profile === "apa");
  const accDriverCard = $("accDriverCard");
  const accTelemetryCard = $("accTelemetryCard");
  const lksDriverCard = $("lksDriverCard");
  const apaDriverCard = $("apaDriverCard");
  const scenarioParameterCard = $("scenarioParameterCard");
  if (accDriverCard) accDriverCard.hidden = profile !== "acc";
  if (accTelemetryCard) accTelemetryCard.hidden = profile !== "acc";
  if (lksDriverCard) lksDriverCard.hidden = profile !== "lks";
  if (apaDriverCard) apaDriverCard.hidden = profile !== "apa";
  if (scenarioParameterCard) scenarioParameterCard.hidden = !item.has_parameters;
  const parameterTitle = $("scenarioParameterTitle");
  if (parameterTitle) parameterTitle.textContent = `${profile.toUpperCase()} 运行参数`;
  const parameterFootnote = $("scenarioParameterFootnote");
  if (parameterFootnote) {
    parameterFootnote.textContent = profile === "lks"
      ? "参数在下次启动时生效；三点权重之和必须为 1.0。车型标定参数修改前请记录原值。"
      : "参数在下次启动时生效；速度阶跃时刻设为 0 可保持固定前车速度。执行器约束只影响本地调试场景。";
  }
  if (profile !== "lks") {
    releaseLksDriverState(false);
    state.lksPedalKeys.clear();
    releaseLksPedal(false);
  }
  if (profile !== "acc") {
    ["w", "s", "a", "d", "space", "f4"].forEach((key) => state.accHeld.delete(key));
    releaseAccLevelInput();
    stopAccVehicleHeartbeat();
  } else {
    refreshAccConsole();
  }
  if (profile === "apa") {
    refreshApaConsole();
    updateApaTaskControls();
    startApaCamera();
    startApaVehicleHeartbeat();
  } else {
    stopApaCamera();
    stopApaVehicleHeartbeat();
  }
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
  if (state.apaTaskBusy) return;
  if (id !== state.selectedId) {
    state.apaTelemetry = {};
    state.apaTaskUnconfirmed = false;
    state.apaUnconfirmedEventId = "";
    state.apaSubmittedCommand = null;
    state.apaObservedCommand = null;
    state.apaPendingMap = "";
    state.apaMapChoicesKey = "";
    state.apaSceneKey = "";
    state.apaDraftDirty = false;
    state.apaHeld.clear();
    state.apaPollRevision += 1;
  }
  state.selectedId = id;
  state.logEpoch += 1;
  state.cursor = 0;
  state.renderedLog = "";
  state.accCommandFeedbackOverride = null;
  $("logBox").textContent = "等待操作...";
  resetAccTelemetryDisplay();
  renderScenarioList();
  renderDetails();
  await loadScenarioParameters();
  await pollLogs();
  await healthCheck(false);
  await pollAccTelemetry();
}

function renderScenarioParameters() {
  const root = $("scenarioParameterGroups");
  if (!root) return;
  const groups = new Map();
  for (const item of state.scenarioParameters.schema) {
    const group = item.group || "参数";
    if (!groups.has(group)) groups.set(group, []);
    groups.get(group).push(item);
  }
  root.innerHTML = [...groups.entries()].map(([group, items]) => {
    const cautionClass = items.some((item) => item.caution) ? " is-caution" : "";
    return `
    <section class="parameter-group${cautionClass}">
      <h3>${escapeHtml(group)}</h3>
      <div class="parameter-grid">
        ${items.map((item) => `
          <label class="parameter-field" title="${escapeHtml(item.description || "")}">
            <span class="parameter-label">${escapeHtml(item.label || item.key)}</span>
            <span class="parameter-input-row">
              <input type="number"
                     data-scenario-param="${escapeHtml(item.key)}"
                     value="${escapeHtml(state.scenarioParameters.values[item.key] ?? item.default)}"
                     min="${escapeHtml(item.min)}"
                     max="${escapeHtml(item.max)}"
                     step="${escapeHtml(item.step || "any")}">
              <span>${escapeHtml(item.unit || "")}</span>
            </span>
            <small>${escapeHtml(item.description || "")}</small>
          </label>`).join("")}
      </div>
    </section>`;
  }).join("");
}

async function loadScenarioParameters() {
  const item = selectedScenario();
  if (!item || !item.has_parameters) {
    state.scenarioParameters = {schema: [], values: {}};
    renderScenarioParameters();
    return;
  }
  try {
    const data = await requestJson(`/api/scenarios/${item.id}/parameters`);
    state.scenarioParameters = {schema: data.schema || [], values: data.values || {}};
    renderScenarioParameters();
  } catch (err) {
    appendLocalLog(`场景参数加载失败: ${err.message}`);
  }
}

function collectScenarioParameters() {
  const values = {};
  document.querySelectorAll("[data-scenario-param]").forEach((input) => {
    values[input.dataset.scenarioParam] = input.value;
  });
  return values;
}

async function saveScenarioParameters(writeLog = true) {
  const item = selectedScenario();
  if (!item || !item.has_parameters) return true;
  try {
    const data = await requestJson(`/api/scenarios/${item.id}/parameters`, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({values: collectScenarioParameters()}),
    });
    state.scenarioParameters.values = data.values || {};
    renderScenarioParameters();
    if (writeLog) appendLocalLog("场景参数已保存，将在下次启动时生效");
    return true;
  } catch (err) {
    appendLocalLog(`场景参数保存失败: ${err.message}`);
    return false;
  }
}

async function resetScenarioParameters() {
  const item = selectedScenario();
  if (!item || !item.has_parameters) return;
  try {
    const data = await requestJson(`/api/scenarios/${item.id}/parameters`, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({reset: true}),
    });
    state.scenarioParameters.values = data.values || {};
    renderScenarioParameters();
    appendLocalLog("场景参数已恢复默认值");
  } catch (err) {
    appendLocalLog(`恢复默认参数失败: ${err.message}`);
  }
}

async function startSelectedScenario() {
  if (!(await saveScenarioParameters(false))) return;
  const frameMode = selectedCarlaFrameMode();
  const qualityMode = selectedCarlaQualityMode();
  appendLocalLog(`CARLA 帧率模式: ${frameMode === "normal" ? "正常 30 FPS" : "稳定 20 FPS"}`);
  appendLocalLog(`CARLA 画质模式: ${qualityMode === "normal" ? "普通画质 (Epic)" : "省资源画质 (Low)"}`);
  await runAction("start", {
    carla_frame_mode: frameMode,
    carla_quality_mode: qualityMode,
  });
}

async function shutdownPanel() {
  const confirmed = window.confirm(
    "关闭平台将停止 CARLA、Bridge、Pangu 和当前测试任务。确定继续吗？"
  );
  if (!confirmed) return;

  const button = $("shutdownPanelBtn");
  button.disabled = true;
  button.classList.add("is-stopping");
  appendLocalLog("正在停止全部联调进程并关闭测试平台...");
  try {
    const data = await requestJson("/api/panel/shutdown", {method: "POST"});
    appendLocalLog(data.message || "平台关闭请求已发送");
    button.querySelector("span").textContent = "正在关闭";
  } catch (err) {
    appendLocalLog(`关闭平台失败: ${err.message}`);
    button.disabled = false;
    button.classList.remove("is-stopping");
  }
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
  const epoch = state.logEpoch;
  if (!id) return;
  try {
    const data = await requestJson(`/api/scenarios/${id}/logs?since=${state.cursor}`);
    if (id !== state.selectedId || epoch !== state.logEpoch) return;
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
    carla: selectedScenario()?.control_profile === "apa" ? "车辆反馈" : "CARLA Server",
    bridge_pub: selectedScenario()?.control_profile === "apa" ? "Bridge 状态端口" : "Bridge PUB",
    bridge_control: selectedScenario()?.control_profile === "apa" ? "算法输出端口" : "Bridge CONTROL",
    pangu_process: "Pangu 业务进程",
  };
  const label = labels[service] || service;
  const apaPort = selectedScenario()?.control_profile === "apa" && ["bridge_pub", "bridge_control"].includes(service);
  if (name) name.textContent = `${label} ${apaPort ? (online ? "已监听" : "未监听") : (online ? "在线" : "离线")}`;
  if (portNode) {
    portNode.textContent = port !== "" && port != null && Number.isFinite(Number(port)) ? `:${port}` : (port || "");
  }
}

async function healthCheck(writeLog = true) {
  const id = state.selectedId;
  if (!id) return;
  try {
    const data = await requestJson(`/api/scenarios/${id}/health?log=${writeLog ? "1" : "0"}`);
    if (state.selectedId !== id) return;
    updateHealthChip("carla", selectedScenario()?.control_profile === "apa" ? apaUiState().ready : data.carla,
      selectedScenario()?.control_profile === "apa" ? "" : data.ports?.carla);
    updateHealthChip("bridge_pub",     data.bridge_pub,     data.ports?.bridge_pub);
    updateHealthChip("bridge_control", data.bridge_control, data.ports?.bridge_control);
    updateHealthChip("pangu_process",  data.pangu_process,  data.ports?.pangu_process);
    const badge = $("accLinkBadge");
    if (badge) {
      badge.textContent = `ACC算法 ${data.pangu_process ? "接入" : "未接入"}`;
      badge.classList.toggle("linked", Boolean(data.pangu_process));
    }
    if (writeLog) await pollLogs();
  } catch (err) {
    if (writeLog) appendLocalLog(`健康检查失败: ${err.message}`);
  }
}

function finiteTelemetryNumber(value) {
  if (value === null || value === undefined || value === "") return null;
  const number = Number(value);
  return Number.isFinite(number) ? number : null;
}

function telemetryValue(value, digits = 1) {
  const number = finiteTelemetryNumber(value);
  return number === null ? "--" : number.toFixed(digits);
}

function signedTelemetryValue(value, digits = 2) {
  const number = finiteTelemetryNumber(value);
  if (number === null) return "--";
  const prefix = number > 0 ? "+" : "";
  return `${prefix}${number.toFixed(digits)}`;
}

function resetAccTelemetryDisplay() {
  const values = {
    accEgoSpeed: "--",
    accEgoSpeedMps: "-- m/s",
    accLeadSpeed: "--",
    accLeadSpeedMps: "-- m/s",
    accLeadDistance: "--",
    accDesiredTimeGap: "1.8",
    accActualTimeGap: "-- s",
    accDesiredDistance: "-- m",
    accDistanceError: "-- m",
    accRelativeSpeed: "-- m/s",
    accTargetAccel: "-- m/s²",
    accActualAccel: "-- m/s²",
    accAccelFeedforward: "-- m/s²",
    accAccelFeedback: "-- m/s²",
    accAccelSaturation: "--",
    accActuator: "油门 -- · 制动 --",
    accControlState: "等待数据",
    accCommandFeedback: "尚未发送",
    accCommandFeedbackHint: "按键闪蓝表示指令已发布",
  };
  for (const [id, value] of Object.entries(values)) {
    const node = $(id);
    if (node) node.textContent = value;
  }
  const status = $("accTelemetryStatus");
  status?.classList.remove("is-live", "is-offline");
  status?.classList.add("is-waiting");
  if ($("accTelemetryStatusText")) $("accTelemetryStatusText").textContent = "等待 Bridge 数据";
  document.querySelector(".telemetry-control-state")?.classList.remove("is-active");
  document.querySelector(".telemetry-command-feedback")?.classList.remove("is-error");
}

function renderAccCommandFeedback(command) {
  const container = document.querySelector(".telemetry-command-feedback");
  const label = $("accCommandFeedback");
  const hint = $("accCommandFeedbackHint");
  const override = state.accCommandFeedbackOverride;
  if (override && override.until > Date.now()) {
    label.textContent = override.text;
    hint.textContent = override.hint;
    container?.classList.toggle("is-error", override.type === "error");
    return;
  }
  state.accCommandFeedbackOverride = null;
  container?.classList.remove("is-error");
  if (!command) {
    label.textContent = "尚未发送";
    hint.textContent = "按键闪蓝表示指令已发布";
    return;
  }
  const age = finiteTelemetryNumber(command.age_sec);
  const ageText = age === null ? "" : age < 1 ? "刚刚" : `${Math.round(age)} 秒前`;
  label.textContent = `${command.label || String(command.key || "").toUpperCase()} · 已发送`;
  if (command.control_active === null || command.control_active === undefined) {
    hint.textContent = `已发布，暂无在控反馈，无法确认设定值变化${ageText ? ` · ${ageText}` : ""}`;
  } else if (command.estimated_applied === false) {
    hint.textContent = `ACC 未在控，设定值未调整${ageText ? ` · ${ageText}` : ""}`;
  } else {
    hint.textContent = `Bridge 控制通道已发布${ageText ? ` · ${ageText}` : ""}`;
  }
}

function renderAccTelemetry(data) {
  const online = Boolean(data.online);
  const egoSpeed = finiteTelemetryNumber(data.ego_speed_mps);
  const leadSpeed = finiteTelemetryNumber(data.lead_speed_mps);
  $("accEgoSpeed").textContent = telemetryValue(egoSpeed === null ? null : egoSpeed * 3.6, 1);
  $("accEgoSpeedMps").textContent = `${telemetryValue(egoSpeed, 2)} m/s`;
  $("accLeadSpeed").textContent = telemetryValue(leadSpeed === null ? null : leadSpeed * 3.6, 1);
  $("accLeadSpeedMps").textContent = `${telemetryValue(leadSpeed, 2)} m/s`;
  $("accLeadDistance").textContent = telemetryValue(data.clearance_m, 1);
  $("accDesiredTimeGap").textContent = telemetryValue(data.desired_time_gap_s, 1);
  $("accActualTimeGap").textContent = `${telemetryValue(data.actual_time_gap_s, 2)} s`;
  $("accDesiredDistance").textContent = `${telemetryValue(data.desired_distance_m, 1)} m`;
  $("accDistanceError").textContent = `${signedTelemetryValue(data.distance_error_m, 1)} m`;
  $("accRelativeSpeed").textContent = `${signedTelemetryValue(data.relative_speed_mps, 2)} m/s`;
  $("accTargetAccel").textContent = `${signedTelemetryValue(data.target_accel_mps2, 2)} m/s²`;
  $("accActualAccel").textContent = `${signedTelemetryValue(data.filtered_accel_mps2, 2)} m/s²`;
  $("accAccelFeedforward").textContent = `${signedTelemetryValue(data.accel_feedforward_mps2, 2)} m/s²`;
  $("accAccelFeedback").textContent = `${signedTelemetryValue(data.accel_feedback_mps2, 2)} m/s²`;
  $("accAccelSaturation").textContent = data.accel_controller_saturated === null || data.accel_controller_saturated === undefined
    ? "--"
    : data.accel_controller_saturated ? "已限幅" : "未限幅";
  const throttle = finiteTelemetryNumber(data.throttle);
  const brake = finiteTelemetryNumber(data.brake);
  const throttleText = throttle === null ? "--" : `${Math.round(throttle * 100)}%`;
  const brakeText = brake === null ? "--" : `${Math.round(brake * 100)}%`;
  $("accActuator").textContent = `油门 ${throttleText} · 制动 ${brakeText}`;

  const controlLabels = {
    active: "ACC 在控",
    standby: "ACC 待命",
    unknown: "状态无反馈",
    offline: "数据离线",
  };
  $("accControlState").textContent = controlLabels[data.control_state] || "状态未知";
  document.querySelector(".telemetry-control-state")?.classList.toggle(
    "is-active", data.control_state === "active"
  );

  const status = $("accTelemetryStatus");
  status.classList.remove("is-waiting", "is-live", "is-offline");
  status.classList.add(online ? "is-live" : "is-offline");
  const age = finiteTelemetryNumber(data.age_sec);
  $("accTelemetryStatusText").textContent = online
    ? `实时 · ${age === null ? "--" : Math.round(age * 1000)} ms`
    : (age === null ? "等待 Bridge 数据" : "Bridge 数据已中断");
  renderAccCommandFeedback(data.last_command);
}

async function pollAccTelemetry() {
  const item = selectedScenario();
  if (!item || item.control_profile !== "acc" || state.accTelemetryBusy) return;
  const id = item.id;
  state.accTelemetryBusy = true;
  try {
    const data = await requestJson(`/api/scenarios/${id}/telemetry`);
    if (state.selectedId === id) renderAccTelemetry(data);
  } catch {
    if (state.selectedId === id) {
      renderAccTelemetry({online: false, control_state: "offline"});
    }
  } finally {
    state.accTelemetryBusy = false;
  }
}

function setCommandFeedback(key, status, message = "") {
  const button = document.querySelector(`[data-command="${key}"]`);
  if (!button) return;
  const timer = state.accCommandFeedbackTimers.get(key);
  if (timer) clearTimeout(timer);
  button.classList.remove("command-pending", "command-ack", "command-error");
  void button.offsetWidth;
  button.classList.add(`command-${status}`);
  if (status !== "pending") {
    const timeout = setTimeout(() => {
      button.classList.remove(`command-${status}`);
      state.accCommandFeedbackTimers.delete(key);
    }, 620);
    state.accCommandFeedbackTimers.set(key, timeout);
  }

  if (status === "pending") {
    state.accCommandFeedbackOverride = {
      type: "pending",
      text: `${key.toUpperCase()} · 发送中`,
      hint: "正在写入 Bridge 控制通道",
      until: Date.now() + 3000,
    };
  } else if (status === "ack") {
    state.accCommandFeedbackOverride = {
      type: "ack",
      text: `${key.toUpperCase()} · 已发送`,
      hint: "Bridge 控制通道已确认发布",
      until: Date.now() + 900,
    };
  } else {
    state.accCommandFeedbackOverride = {
      type: "error",
      text: `${key.toUpperCase()} · 发送失败`,
      hint: message || "请检查 Bridge CONTROL 服务",
      until: Date.now() + 3500,
    };
  }
  renderAccCommandFeedback(null);
}

async function sendDriverCommand(key, writeLog = true) {
  const id = state.selectedId;
  if (!id) return false;
  if (writeLog) appendLocalLog(`发送驾驶指令: ${key.toUpperCase()}`);
  if (writeLog) setCommandFeedback(key, "pending");
  try {
    await requestJson(`/api/scenarios/${id}/driver-command`, {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({key, write_log: writeLog}),
    });
    if (writeLog) setCommandFeedback(key, "ack");
    if (writeLog) await pollLogs();
    if (writeLog) await pollAccTelemetry();
    return true;
  } catch (err) {
    if (writeLog) setCommandFeedback(key, "error", err.message);
    if (writeLog) appendLocalLog(`驾驶指令失败: ${err.message}`);
    return false;
  }
}

function queueDriverCommand(key, writeLog = true) {
  state.accCommandQueue = state.accCommandQueue
    .catch(() => {})
    .then(() => sendDriverCommand(key, writeLog));
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

function lksPedalPayload() {
  const throttle = state.lksPedal === "throttle" ? 0.5 : 0.0;
  const brake = state.lksPedal === "brake" ? 0.9 : 0.0;
  return {
    throttle,
    brake,
    preserve_steer: true,
    hand_brake: false,
    gear: "D",
    ignition_switch: "ON",
  };
}

function refreshLksPedalButtonState() {
  document.querySelectorAll("[data-lks-pedal]").forEach((button) => {
    button.classList.toggle("active", button.dataset.lksPedal === state.lksPedal);
  });
}

async function flushLksVehicle(writeLog = false) {
  const id = state.selectedId;
  const item = selectedScenario();
  if (!id || item?.control_profile !== "lks") return;
  if (state.lksVehicleBusy) {
    state.lksVehiclePending = true;
    return;
  }
  state.lksVehicleBusy = true;
  try {
    do {
      state.lksVehiclePending = false;
      const payload = lksPedalPayload();
      try {
        await requestJson(`/api/scenarios/${id}/vehicle-cmd`, {
          method: "POST",
          headers: {"Content-Type": "application/json"},
          body: JSON.stringify(payload),
        });
        if (writeLog) {
          appendLocalLog(
            `LKS车辆指令: throttle=${payload.throttle.toFixed(2)}, brake=${payload.brake.toFixed(2)}`
          );
        }
      } catch (err) {
        if (writeLog) appendLocalLog(`LKS车辆指令失败: ${err.message}`);
      }
    } while (state.lksVehiclePending);
  } finally {
    state.lksVehicleBusy = false;
  }
}

function startLksVehicleHeartbeat() {
  if (state.lksVehicleTimer) return;
  state.lksVehicleTimer = setInterval(() => flushLksVehicle(false), 100);
}

function stopLksVehicleHeartbeat() {
  if (state.lksVehicleTimer) {
    clearInterval(state.lksVehicleTimer);
    state.lksVehicleTimer = null;
  }
}

function startLksPedal(pedal, writeLog = true) {
  state.lksPedal = pedal;
  refreshLksPedalButtonState();
  startLksVehicleHeartbeat();
  flushLksVehicle(writeLog);
}

function releaseLksPedal(writeLog = true) {
  const changed = Boolean(state.lksPedal);
  state.lksPedal = null;
  stopLksVehicleHeartbeat();
  refreshLksPedalButtonState();
  if (changed || writeLog) flushLksVehicle(writeLog);
}

function syncLksPedalKeyboard(writeLog = true) {
  if (state.lksPedalKeys.has("s")) {
    startLksPedal("brake", writeLog);
  } else if (state.lksPedalKeys.has("w")) {
    startLksPedal("throttle", writeLog);
  } else {
    releaseLksPedal(writeLog);
  }
}

function startAccLevelInput(key) {
  if (state.accLevelKey === key) return;
  if (state.accLevelTimer) clearInterval(state.accLevelTimer);
  state.accLevelTimer = null;
  if (state.accLevelKey) queueDriverCommand("0", false);
  state.accLevelKey = key;
  document.querySelector(`[data-command="${key}"]`)?.classList.add("active");
  queueDriverCommand(key);
  state.accLevelTimer = setInterval(() => {
    if (state.accLevelKey === key) queueDriverCommand(key, false);
  }, 250);
}

function releaseAccLevelInput() {
  if (state.accLevelTimer) clearInterval(state.accLevelTimer);
  state.accLevelTimer = null;
  if (!state.accLevelKey) return;
  document.querySelector(`[data-command="${state.accLevelKey}"]`)?.classList.remove("active");
  state.accLevelKey = null;
  queueDriverCommand("0");
}

function accAnalog() {
  const held = state.accHeld;
  const ignition = state.accVehicle.ignition;
  const brake = held.has("s") || state.accVehicle.restartPending ? 1 : 0;
  let throttle = 0;
  if (brake <= 0 && held.has("w") && (ignition === "ON" || ignition === "START")) {
    throttle = 0.6;
  }
  let steer = 0;
  if (held.has("a") && !held.has("d")) steer = -0.6;
  else if (held.has("d") && !held.has("a")) steer = 0.6;
  const handBrake = held.has("space") || ignition === "OFF";
  return {throttle, brake, steer, handBrake};
}

function refreshAccConsole() {
  const analog = accAnalog();
  document.querySelectorAll("[data-ignition]").forEach((button) => {
    button.classList.toggle("active", button.dataset.ignition === state.accVehicle.ignition);
  });
  document.querySelectorAll("[data-gear]").forEach((button) => {
    button.classList.toggle("active", button.dataset.gear === state.accVehicle.gear);
  });
  document.querySelectorAll("[data-hold]").forEach((button) => {
    const key = button.dataset.hold;
    button.classList.toggle("active", state.accHeld.has(key) || (key === "f4" && state.accVehicle.ignition === "START"));
  });
  document.querySelectorAll("[data-latch]").forEach((button) => {
    button.classList.toggle("active", Boolean(state.accVehicle[button.dataset.latch]));
  });
  document.querySelectorAll("[data-lane]").forEach((button) => {
    button.classList.toggle("active", state.accVehicle.laneChange === button.dataset.lane);
  });
  document.querySelector("[data-restart]")?.classList.toggle("active", state.accVehicle.restartPending);
  const box = $("accStatusBox");
  if (box) {
    box.textContent = (
      `油门 ${analog.throttle.toFixed(2)}　制动 ${analog.brake.toFixed(2)}　转向 ${analog.steer >= 0 ? "+" : ""}${analog.steer.toFixed(2)}　` +
      `挡位 ${state.accVehicle.gear}　手刹 ${Number(analog.handBrake)}　点火 ${state.accVehicle.ignition}\n` +
      `APA ${state.accVehicle.apa ? "开" : "关"}　LKA ${state.accVehicle.lka ? "开" : "关"}　AEB ${state.accVehicle.aeb ? "开" : "关"}　` +
      `换道 ${state.accVehicle.laneChange || "无"}\n` +
      "W/S/A/D/空格直接到 Bridge；E/Q/T/R/C 进入 ACC 算法指令。"
    );
  }
}

async function flushAccVehicle(writeLog = false) {
  const id = state.selectedId;
  const item = selectedScenario();
  if (!id || item?.control_profile === "lks") return;
  if (state.accVehicleBusy) {
    state.accVehiclePending = true;
    return;
  }
  state.accVehicleBusy = true;
  try {
    do {
      state.accVehiclePending = false;
      const analog = accAnalog();
      const payload = {
        throttle: analog.throttle,
        brake: analog.brake,
        steer: analog.steer,
        steering_angle: analog.steer * 720.0,
        hand_brake: analog.handBrake,
        gear: state.accVehicle.gear,
        ignition_switch: state.accVehicle.ignition,
        apa_enable: state.accVehicle.apa,
        lka_enable: state.accVehicle.lka,
        aeb_enable: state.accVehicle.aeb,
        lc_command: state.accVehicle.laneChange,
      };
      try {
        await requestJson(`/api/scenarios/${id}/vehicle-cmd`, {
          method: "POST",
          headers: {"Content-Type": "application/json"},
          body: JSON.stringify(payload),
        });
        if (writeLog) {
          appendLocalLog(
            `车辆指令 ignition=${payload.ignition_switch} gear=${payload.gear} ` +
            `T=${payload.throttle.toFixed(2)} B=${payload.brake.toFixed(2)} S=${payload.steer.toFixed(2)}`
          );
        }
      } catch (err) {
        if (writeLog) appendLocalLog(`车辆指令失败: ${err.message}`);
      }
    } while (state.accVehiclePending);
  } finally {
    state.accVehicleBusy = false;
  }
  refreshAccConsole();
}

function startAccVehicleHeartbeat() {
  if (state.accVehicleTimer) return;
  state.accVehicleTimer = setInterval(() => flushAccVehicle(false), 80);
}

function stopAccVehicleHeartbeat() {
  if (state.accVehicleTimer) {
    clearInterval(state.accVehicleTimer);
    state.accVehicleTimer = null;
  }
}

function setAccHold(key, pressed, writeLog = false) {
  if (pressed) state.accHeld.add(key);
  else state.accHeld.delete(key);
  if (key === "f4") {
    state.accVehicle.ignition = pressed ? "START" : "ON";
  }
  if (pressed && key === "w" && !["ON", "START"].includes(state.accVehicle.ignition)) {
    appendLocalLog("请先按 F3 点火或 F4 启动，再按住 W 加油门");
  }
  if (key === "w" || key === "s") {
    if (pressed) startAccLevelInput(key);
    else if (state.accLevelKey === key) releaseAccLevelInput();
  }
  refreshAccConsole();
  startAccVehicleHeartbeat();
  flushAccVehicle(writeLog);
}

function setAccIgnition(value) {
  state.accVehicle.ignition = value;
  if (value !== "START") state.accHeld.delete("f4");
  refreshAccConsole();
  startAccVehicleHeartbeat();
  flushAccVehicle(true);
}

function setAccGear(value) {
  state.accVehicle.gear = value;
  refreshAccConsole();
  startAccVehicleHeartbeat();
  flushAccVehicle(true);
}

function toggleAccLatch(name) {
  state.accVehicle[name] = !state.accVehicle[name];
  refreshAccConsole();
  startAccVehicleHeartbeat();
  flushAccVehicle(true);
}

function pulseAccLane(direction) {
  state.accVehicle.laneChange = direction;
  refreshAccConsole();
  flushAccVehicle(true);
  setTimeout(() => {
    if (state.accVehicle.laneChange === direction) {
      state.accVehicle.laneChange = "";
      refreshAccConsole();
      flushAccVehicle(false);
    }
  }, 600);
}

function startAccRestart() {
  if (state.accRestartTimer) clearTimeout(state.accRestartTimer);
  state.accVehicle.restartPending = true;
  setAccHold("s", true, true);
  appendLocalLog("0: 刹车退出 ACC，随后自动重开");
  state.accRestartTimer = setTimeout(() => {
    setAccHold("s", false, false);
    queueDriverCommand("0").then(() => queueDriverCommand("e"));
    state.accVehicle.restartPending = false;
    refreshAccConsole();
    appendLocalLog("0: 已重新发送 E 启控");
  }, 700);
}

function keyboardTargetIsEditable(event) {
  const target = event.target;
  if (!(target instanceof HTMLElement)) return false;
  const tag = target.tagName.toLowerCase();
  if (tag === "select" && target.closest("#apaSetupForm")) return true;
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

function formatResultNumber(value, digits = 2, suffix = "") {
  if (value === null || value === undefined || value === "") return "-";
  const number = Number(value);
  return Number.isFinite(number) ? `${number.toFixed(digits)}${suffix}` : "-";
}

async function readAccResult() {
  const id = state.selectedId;
  if (!id) return;
  try {
    const data = await requestJson(`/api/scenarios/${id}/results/latest`);
    const result = data.result || {};
    appendLocalLog(
      `ACC结果: 最小/末端间距=${formatResultNumber(result.clearance_min_m, 2, "m")}/` +
      `${formatResultNumber(result.clearance_last_m, 2, "m")}, ` +
      `速度误差RMS=${formatResultNumber(result.speed_error_rms_mps, 2, "m/s")}, ` +
      `稳定时间=${formatResultNumber(result.speed_settling_time_sec, 1, "s")}, ` +
      `加速度范围=${formatResultNumber(result.target_accel_min_mps2, 2, "m/s²")}~` +
      `${formatResultNumber(result.target_accel_max_mps2, 2, "m/s²")}, ` +
      `饱和占比=${formatResultNumber(Number(result.accel_saturation_ratio) * 100, 1, "%")}, ` +
      `踏板切换=${result.throttle_brake_switch_count ?? "-"}, 样本=${result.sample_count || 0}`
    );
  } catch (err) {
    appendLocalLog(`ACC测试结果尚不可用: ${err.message}`);
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

const APA_STEER_RAD = 0.8;
const APA_CAMERA_VIEWS = [
  {image: "apaRearCamera", status: "apaCameraStatus", view: "rear"},
  {image: "apaOverheadCamera", status: "apaOverheadCameraStatus", view: "overhead"},
];
const apaCameraRequests = new Map();

function renderApaCameraStatus(streams) {
  if (!streams) return;
  const labels = {waiting: "等待影像", live: "已连接", stale: "连接中断 · 保留画面", unavailable: "未连接"};
  APA_CAMERA_VIEWS.forEach((camera) => {
    const image = $(camera.image);
    const status = $(camera.status);
    const request = apaCameraRequests.get(camera.view);
    const cameraState = request?.hasFrame
      ? (performance.now() - request.lastFrameAt <= 1500 ? "live" : "stale")
      : (image.dataset.streamError ? "unavailable" : "waiting");
    const label = request && cameraState === "stale"
      ? (image.dataset.streamError ? "取图中断 · 保留画面" : "相机未更新 · 保留画面")
      : (labels[request ? cameraState : streams[camera.view]] || "等待影像");
    if (status.textContent !== label) status.textContent = label;
  });
}

function apaAnalog() {
  const held = state.apaHeld;
  const ignition = state.apaVehicle.ignition;
  const gear = state.apaVehicle.gear;
  if (state.apaTelemetry.parking_enabled) {
    return {throttle: 0, brake: 0, steerRad: 0, handBrake: false, gear, ignition};
  }
  const brake = held.has("s") ? 1 : 0;
  let throttle = 0;
  if (brake <= 0 && held.has("w") && (ignition === "ON" || ignition === "START")) throttle = 0.45;
  let steerRad = 0;
  if (held.has("a") && !held.has("d")) steerRad = -APA_STEER_RAD;
  else if (held.has("d") && !held.has("a")) steerRad = APA_STEER_RAD;
  const handBrake = Boolean(state.apaVehicle.handBrake) || ignition === "OFF";
  if (gear === "P" || gear === "N" || ignition === "OFF" || ignition === "ACC" || handBrake) throttle = 0;
  return {throttle, brake: handBrake ? 1 : brake, steerRad, handBrake, gear, ignition};
}

function updateApaLinkBadge(linked) {
  const badge = $("apaLinkBadge");
  if (!badge) return;
  const connected = Boolean(linked);
  const text = connected ? "算法已连接" : "算法未连接";
  if (badge.textContent !== text) badge.textContent = text;
  badge.classList.toggle("linked", connected);
  badge.classList.toggle("offline", !connected);
}

function renderApaHeader(data = state.apaTelemetry) {
  if (selectedScenario()?.control_profile !== "apa") return;
  const online = Boolean(data.online && data.session_id);
  const map = $("mapInfo");
  const physicalSlotId = data.physical_slot_id ?? data.slot_id;
  const mapText = online && data.map_name
    ? `${data.map_name}${Number.isInteger(physicalSlotId) ? ` · 车位 ${physicalSlotId}` : ""}` : "等待连接";
  if (map.textContent !== mapText) map.textContent = mapText;
  map.title = mapText;
  updateApaLinkBadge(online && data.algorithm_linked);
  const raw = state.scenarioStatus;
  const status = raw === "running" || raw === "error" ? raw : online ? "ready" : "idle";
  const pillClass = `status-pill status-${status}`;
  if ($("statusPill").className !== pillClass) $("statusPill").className = pillClass;
  const label = raw === "running" ? "环境处理中" : raw === "error" ? "环境异常"
    : online ? "状态在线" : "状态未连接";
  if ($("statusText").textContent !== label) $("statusText").textContent = label;
}

// Task labels and action gates share this state; status alone does not prove vehicle feedback.
function apaUiState(data = state.apaTelemetry) {
  const online = Boolean(data.online && data.session_id);
  const feedbackAvailable = data.vehicle_feedback_available !== false;
  const ready = Boolean(online && data.initial_pose && feedbackAvailable);
  const geometryPreparing = Boolean(data.geometry?.preparing ?? data.slot_occupancy?.preparing_geometry);
  const geometryPending = Number(data.geometry?.pending || 0) > 0 ||
    ["checking", "sampling", "failed"].includes(data.geometry?.progress?.state);
  const mapLoading = apaMapLoading();
  const receipt = data.last_task_command;
  const executing = online && receipt?.state === "running" && receipt.source_session_id === data.session_id &&
    (receipt.task_id === data.task_id || receipt.source_task_id === data.task_id);
  const commandPending = state.apaTaskBusy || state.apaTaskUnconfirmed || executing;
  const busy = state.scenarioBusy || commandPending || mapLoading || geometryPreparing;
  const safety = data.safety_state || {};
  const stopping = Boolean(safety.reason && safety.state !== "stopped");
  const needsReset = Boolean((safety.reason && safety.reason !== "cancelled") || data.sync_error);
  const parking = Boolean(data.parking_enabled);
  const planningOnly = parking && Boolean(data.planning_only);
  const linked = online && Boolean(data.algorithm_linked);
  const speed = feedbackAvailable ? finiteTelemetryNumber(data.ego_speed_mps) : null;
  const startSpeedLimit = finiteTelemetryNumber(data.action_switch_max_speed_mps) ?? 0.03;
  const stationary = speed !== null && Math.abs(speed) <= startSpeedLimit;
  const parked = speed !== null && Math.abs(speed) <= 0.02 && !parking && !stopping;
  const completed = parking && linked && stationary && data.control_status === "complete" && !needsReset && !stopping;
  const previewReady = planningOnly && linked && stationary && data.control_status === "plan_ready" && !needsReset && !stopping;
  const notice = apaTaskNotice(data);
  const replanInfo = apaReplanInfo(data);
  const actionName = Number(data.parking_action) === 2 ? "泊出" : "泊入";
  const loadingMap = state.apaPendingMap || data.map_transition?.map_name || "地图";
  let label = "待机";
  if (!online) label = mapLoading ? `正在加载 ${loadingMap} · 等待状态` : "车辆状态未连接";
  else if (mapLoading) label = `正在加载 ${loadingMap}`;
  else if (data.control_status === "map_failed") label = "地图加载失败";
  else if (notice?.error) label = `${notice.label}：${notice.reason}`;
  else if (safety.reason) label = stopping ? "正在制动 · 等待停稳确认"
    : safety.reason === "cancelled" ? "已退出 · 已停稳" : "保护停车 · 已停稳";
  else if (!feedbackAvailable) label = "车辆反馈不可用";
  else if (executing) label = ({confirming_stop: "正在确认停稳", applying_pose: "正在应用设置",
    settling: "等待车辆稳定", checking_geometry: "正在读取离线底图", loading_map: "正在加载离线底图", preparing_geometry: "旧版 Bridge 正在采样",
    preparing_map: "正在准备车位周边地图",
    finalizing: "正在确认设置完成"})[receipt.stage] || "正在执行操作";
  else if (geometryPreparing) label = "正在准备地图";
  else if (!data.initial_pose) label = "正在初始化车辆";
  else if (commandPending) label = "等待操作确认";
  else if (geometryPending) label = "离线底图不可用";
  else if (!linked) label = "等待算法连接";
  else if (notice) label = `${notice.label}：${notice.reason}`;
  else if (previewReady) label = "路径已就绪 · 驻车";
  else if (completed) label = replanInfo.isReplan ? `${actionName}完成 · 重规划` : `${actionName}完成`;
  else if (parking) {
    const planningLabel = replanInfo.isReplan
      ? `正在重规划${replanInfo.reasonText ? `：${replanInfo.reasonText}` : ""}`
      : "正在规划";
    const stoppingLabel = replanInfo.isReplan
      ? `停车重规划${replanInfo.reasonText ? `：${replanInfo.reasonText}` : ""}`
      : "停车重规划";
    const trackingLabel = replanInfo.isReplan ? `正在${actionName} · 重规划` : `正在${actionName}`;
    const holdingLabel = replanInfo.isReplan ? "停车等待 · 重规划" : "停车等待";
    label = ({planning: planningLabel, replan_stopping: stoppingLabel, complete: "确认停稳中",
      replan_waiting: "等待场景变化", tracking: trackingLabel, holding: holdingLabel})[data.control_status] || "等待算法响应";
  }
  const manualReason = !online ? "车辆状态未连接，手动输入暂停"
    : !feedbackAvailable ? "车辆反馈不可用，手动输入暂停"
    : busy ? "操作执行或确认中，手动输入暂停"
    : !data.initial_pose ? "车辆初始化中，手动输入暂停"
    : stopping ? "正在确认停稳，手动输入暂停"
    : needsReset ? "保护停车已锁存，请重置后操作"
    : parking ? "APA 控制中，退出并停稳后可手动驾驶" : "";
  const mode = !online ? "状态未更新" : stopping ? "正在停车" : needsReset ? "保护停车"
    : notice?.error ? "APA 停车"
    : !feedbackAvailable ? "反馈不可用" : !ready || busy ? "场景处理中"
    : completed || planningOnly ? "APA 驻车保持"
    : parking ? `自动${actionName}` : "手动驾驶";
  return {online, feedbackAvailable, ready, geometryPreparing, geometryPending, mapLoading, busy,
    safety, stopping, needsReset, parking, planningOnly, previewReady, linked, stationary, parked, completed, notice, label, mode,
    manualReason, canStart: ready && !busy && !geometryPending && linked && stationary &&
      !stopping && !needsReset && (!parking || completed || previewReady)};
}

function refreshApaConsole() {
  const analog = apaAnalog();
  const view = apaUiState();
  const locked = apaManualLocked();
  document.querySelectorAll("[data-apa-ignition]").forEach((button) => {
    button.classList.toggle("active", button.dataset.apaIgnition === state.apaVehicle.ignition);
    button.disabled = locked;
  });
  document.querySelectorAll("[data-apa-gear]").forEach((button) => {
    button.classList.toggle("active", button.dataset.apaGear === state.apaVehicle.gear);
    button.disabled = locked;
  });
  document.querySelectorAll("[data-apa-hold]").forEach((button) => {
    button.classList.toggle("active", state.apaHeld.has(button.dataset.apaHold));
    button.disabled = locked;
  });
  document.querySelectorAll("[data-apa-handbrake]").forEach((button) => {
    button.classList.toggle("active", Boolean(state.apaVehicle.handBrake));
    button.disabled = locked;
  });
  const box = $("apaStatusBox");
  if (box) {
    box.textContent = view.manualReason || (!$("apaReport").hidden ? "报告查看中，手动输入暂停"
      : `手动输入：油门 ${analog.throttle.toFixed(2)}　制动 ${analog.brake.toFixed(2)}　转向 ${analog.steerRad >= 0 ? "+" : ""}${analog.steerRad.toFixed(2)} rad　挡位 ${analog.gear}　驻车 ${analog.handBrake ? "拉起" : "松开"}　点火 ${analog.ignition}`);
  }
  updateApaLinkBadge(view.linked);
}

function apaManualLocked() {
  return Boolean(apaUiState().manualReason) || !$("apaReport").hidden;
}

function toggleApaHandBrake() {
  if (apaManualLocked()) return;
  state.apaVehicle.handBrake = !state.apaVehicle.handBrake;
  refreshApaConsole();
  flushApaVehicle(true);
  appendLocalLog(state.apaVehicle.handBrake ? "请求拉起驻车制动" : "请求松开驻车制动");
}

function setApaHold(key, pressed) {
  if (pressed && apaManualLocked()) return;
  if (pressed) state.apaHeld.add(key);
  else state.apaHeld.delete(key);
  if (key === "f4") state.apaVehicle.ignition = pressed ? "START" : "ON";
  refreshApaConsole();
  startApaVehicleHeartbeat();
  flushApaVehicle(false);
}

function setApaIgnition(value) {
  if (apaManualLocked()) return;
  state.apaVehicle.ignition = value;
  refreshApaConsole();
  flushApaVehicle(true);
}

function setApaGear(value) {
  if (apaManualLocked()) return;
  state.apaVehicle.gear = value;
  refreshApaConsole();
  flushApaVehicle(true);
}

async function flushApaVehicle(writeLog = false) {
  const id = state.selectedId;
  if (!id || selectedScenario()?.control_profile !== "apa") return;
  if (apaManualLocked()) return;
  if (state.apaVehicleBusy) {
    state.apaVehiclePending = true;
    return;
  }
  state.apaVehicleBusy = true;
  try {
    do {
      if (state.selectedId !== id || selectedScenario()?.control_profile !== "apa" || apaManualLocked()) break;
      state.apaVehiclePending = false;
      const analog = apaAnalog();
      const payload = {
        throttle: analog.throttle,
        brake: analog.brake,
        steer_rad: analog.steerRad,
        hand_brake: analog.handBrake,
        gear: analog.gear,
        ignition_switch: analog.ignition,
        apa_phase: "manual",
        apa_enable: false,
        apa_exit: false,
        session_id: state.apaTelemetry.session_id,
        task_id: state.apaTelemetry.task_id,
      };
      await requestJson(`/api/scenarios/${id}/vehicle-cmd`, {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(payload),
        signal: AbortSignal.timeout(2000),
      });
      if (state.selectedId !== id || payload.session_id !== state.apaTelemetry.session_id ||
          payload.task_id !== state.apaTelemetry.task_id) break;
      if (writeLog) appendLocalLog(`手动指令已发送：挡位 ${payload.gear} 转向 ${payload.steer_rad.toFixed(2)} rad 驻车制动 ${payload.hand_brake ? "拉起" : "松开"}`);
    } while (state.apaVehiclePending);
  } catch (err) {
    if (writeLog) appendLocalLog(`车辆指令失败: ${err.message}`);
  } finally {
    state.apaVehicleBusy = false;
  }
}

function startApaVehicleHeartbeat() {
  if (state.apaVehicleTimer || !$("apaReport").hidden || document.hidden || !document.hasFocus()) return;
  state.apaVehicleTimer = setInterval(() => flushApaVehicle(false), 80);
}

function stopApaVehicleHeartbeat() {
  if (!state.apaVehicleTimer) return;
  clearInterval(state.apaVehicleTimer);
  state.apaVehicleTimer = null;
}

function releaseApaInput() {
  state.apaHeld.clear();
  if (state.apaVehicle.ignition === "START") state.apaVehicle.ignition = "ON";
  stopApaVehicleHeartbeat();
  if (selectedScenario()?.control_profile !== "apa") return;
  refreshApaConsole();
  flushApaVehicle(false);
}

const APA_SLOT_LABELS = {vertical: "垂直", parallel: "平行", diagonal: "斜列"};

const APA_REPLAN_REASONS = {
  endpoint_deviation: "段终点位姿超差",
  tracking_deviation: "路径跟踪偏差触发重规划",
  tracking_blocked: "运动受限触发重规划",
  path_blocked: "行驶路径被障碍物阻挡",
  body_predicted: "障碍物预计侵入当前车身",
  planning_clearance: "当前车身安全距离不足",
  clearance_deficit: "恢复车身安全净距",
  clearance_predicted: "预计运动侵入安全余量",
  motion_blocked: "预计运动碰障或压线",
  clearance_recovery_deviation: "安全净距恢复偏差",
  standstill_lost: "等待车辆重新停稳",
  obstacle_clear_confirmation: "等待障碍物离开确认",
  stale_plan_start: "规划起点变化重试",
  candidate_blocked: "候选路径被当前障碍物阻挡",
  unsafe_candidate: "候选路径未通过安全检查",
};

function apaReplanInfo(data = state.apaTelemetry) {
  const replan = data?.replan || {};
  const trajectory = data?.trajectory || {};
  const prevCount = Array.isArray(trajectory.previous_plans) ? trajectory.previous_plans.length : 0;
  const pathVersion = Math.max(Number(replan.path_version) || 0, Number(trajectory.path_version) || 0, prevCount > 0 ? prevCount + 1 : 0);
  const recoveryId = Number(replan.recovery_id) || 0;
  const attempt = Number(replan.attempt) || 0;
  const activePlanning = ["planning", "replan_stopping", "replan_waiting"].includes(data?.control_status);
  const rawReason = String(replan.reason || "");
  const taskKey = data?.session_id && data?.task_id !== undefined ? `${data.session_id}:${data.task_id}` : "";
  if (taskKey && state.apaReplanTaskKey !== taskKey) {
    state.apaReplanTaskKey = taskKey;
    state.apaLastReplanReason = "";
  }
  if (rawReason && rawReason !== "initial_plan") {
    state.apaLastReplanReason = rawReason;
  } else if (recoveryId <= 1 && pathVersion <= 1 && prevCount === 0 && !activePlanning) {
    state.apaLastReplanReason = "";
  }
  const baseCount = activePlanning
    ? Math.max(recoveryId, pathVersion + (rawReason === "initial_plan" && pathVersion === 0 ? 0 : (pathVersion > 0 ? 1 : 0)), 1)
    : Math.max(pathVersion, recoveryId, prevCount > 0 ? prevCount + 1 : 0);
  const retryOffset = activePlanning && attempt > 1 ? attempt - 1 : 0;
  const count = baseCount > 0 ? baseCount + retryOffset : 0;
  const isReplan = count > 1 || prevCount > 0 || (Boolean(rawReason) && rawReason !== "initial_plan");
  const displayCount = isReplan ? Math.max(count, 2) : (count > 0 ? 1 : 0);
  const stageLabel = displayCount <= 0 ? "" : isReplan ? `第 ${displayCount} 次重规划` : "第 1 次规划";
  const effectiveReason = isReplan ? (rawReason && rawReason !== "initial_plan" ? rawReason : state.apaLastReplanReason) : "";
  const reasonText = effectiveReason ? (APA_REPLAN_REASONS[effectiveReason] || "") : "";
  return {count: displayCount, hasPlan: displayCount > 0, isReplan, activePlanning, stageLabel, reasonText, rawReason};
}

const APA_PLANNING_FAILURES = {
  no_solution: ["规划失败", "未找到可执行路径"],
  planning_timeout: ["规划失败", "路径搜索超时"],
  planner_process_failed: ["规划失败", "规划进程异常退出"],
  planning_scene_capacity: ["规划失败", "场景约束超出规划容量"],
  planning_refinement_limit: ["规划失败", "场景约束处理达到上限"],
  unsafe_candidate: ["路径安全检查失败", "候选路径未通过安全检查"],
  candidate_blocked: ["路径安全检查失败", "候选路径被当前障碍物阻挡"],
  stale_plan_start: ["规划失败", "规划期间车辆起点发生变化"],
  recovery_timeout: ["规划失败", "重规划等待超时"],
  replan_exhausted: ["规划失败", "重规划次数已用尽"],
  recovery_no_progress: ["泊车修正停止", "连续修正未改善泊车位置"],
};

// Display only causes established by telemetry. A clearance stop is not a
// collision event, and no_solution does not establish which solver gate failed.
function apaTaskNotice(data) {
  if (!data.online) return null;
  const safety = data.safety_state || {};
  const replan = data.replan || {};
  const status = data.control_status;
  const stopReason = String(safety.reason || data.sync_error || "");
  const notice = (label, reason, error = true) => ({label, reason, error});
  const planningFailure = (reason) => notice(...(Object.prototype.hasOwnProperty.call(APA_PLANNING_FAILURES, reason)
    ? APA_PLANNING_FAILURES[reason] : ["规划失败", "未得到可执行路径"]));
  if (["cancelled", "shutdown"].includes(stopReason)) return null;
  if (stopReason) {
    const stopState = safety.state === "stopped" ? "已确认停稳"
      : safety.state === "unconfirmed" ? "尚未确认停稳" : "正在制动";
    if (/^collision(?::|$)/.test(stopReason))
      return notice("碰撞停止", `检测到实际碰撞，${stopState}`);
    if (stopReason.startsWith("clearance_stop:")) {
      if (replan.state === "failed" && replan.reason === "recovery_no_progress")
        return notice("泊车修正停止", `连续修正未改善泊车位置，${stopState}`);
      const cause = /\bclearance_predicted\b/.test(stopReason)
        ? "预计运动将侵入安全余量" : "当前安全余量不足";
      return notice("安全距离停车", `${cause}，${stopState}`);
    }
    if (stopReason.startsWith("obstacle_stop:")) {
      const cause = /\bbody_blocked\b/.test(stopReason) ? "当前车身与障碍物重叠"
        : /\bbody_predicted\b/.test(stopReason) ? "障碍物预计侵入当前车身"
        : /\bplanning_clearance\b/.test(stopReason) ? "当前车身安全距离不足"
        : /\bpath_blocked\b/.test(stopReason) ? "行驶路径被障碍物阻挡" : "检测到障碍物";
      return notice("安全检查停车", `${cause}，${stopState}`);
    }
    if (stopReason.startsWith("replan_failed:"))
      return planningFailure(stopReason.slice("replan_failed:".length).trim());
    if (stopReason.startsWith("actual_overspeed:"))
      return notice("超速保护停车", `实际车速超过安全上限，${stopState}`);
    if (/^APA input rejected:|^invalid_(ego_feedback|plan_request):/.test(stopReason))
      return notice("输入异常停车", `输入数据未通过校验，${stopState}`);
    if (/RPC|timeout|synchroniz|timing_error/.test(stopReason))
      return notice("通信异常停车", `控制或仿真通信异常，${stopState}`);
    return notice("保护停车", `触发安全保护，${stopState}`);
  }
  if (status === "tracking_fault") {
    const causes = {3: "输入或参数无效", 4: "路径参考丢失或偏差过大", 5: "未达到段终点位姿",
      6: "档位确认超时", 7: "路径进度超时", 8: "终点车身边界检查失败"};
    return notice("跟踪保护停车", causes[data.control_diagnostic?.stop_reason] || "触发跟踪保护");
  }
  if (status === "obstacle_stop") return notice("安全检查停车", "检测到障碍物，已触发制动");
  if (status === "safety_stop") return notice("保护停车", "触发安全保护");
  if (status === "timing_error") return notice("通信异常停车", "控制与仿真时序不一致");
  if (status === "input_error") return notice("输入异常停车", "输入数据未通过校验");
  if (status === "plan_failed" || (data.parking_enabled && replan.state === "failed"))
    return planningFailure(replan.reason);
  if (status === "replan_waiting") {
    const cause = APA_REPLAN_REASONS[replan.reason] || "暂未得到可执行路径";
    return notice("等待场景变化", cause, false);
  }
  if (status === "replan_stopping" && replan.reason !== "initial_plan") {
    if (Object.prototype.hasOwnProperty.call(APA_REPLAN_REASONS, replan.reason))
      return notice("停车重规划", APA_REPLAN_REASONS[replan.reason], false);
  }
  return null;
}

function apaFeedback(message, tone = "") {
  const element = $("apaSetupFeedback");
  element.textContent = message;
  element.className = `apa-form-status${tone ? ` is-${tone}` : ""}`;
}

function clearApaHistory() {
  state.apaSamples = [];
  state.apaAlgoPath = [];
  state.apaActualPath = [];
  state.apaSampleT0 = null;
  state.apaReplanTaskKey = "";
  state.apaLastReplanReason = "";
  closeApaReport();
  ["apaReportWheel", "apaReportAccel", "apaReportSpeed", "apaReportPath", "apaReportTable"].forEach((id) => {
    const element = $(id);
    if (element) { element.replaceChildren(); element._hits = []; }
  });
}

function writeApaPose(pose) {
  for (const [id, key] of [["apaStartX", "x"], ["apaStartY", "y"], ["apaStartYaw", "yaw_deg"]]) {
    $(id).value = Number.isFinite(pose?.[key]) ? Number(pose[key].toFixed(6)) : "";
    $(id).removeAttribute("aria-invalid");
  }
}

function apaMapLoading() {
  return Boolean(state.apaPendingMap || state.apaTelemetry.map_transition?.state === "loading");
}

function apaMapChoices(data = state.apaTelemetry) {
  return data.map_choices?.length ? data.map_choices : data.map_name
    ? [{map_name: data.map_name, available: true, slot_choices: data.slot_choices || [], default_slot_id: data.slot_id}]
    : [];
}

function apaDraftSlots() {
  const data = state.apaTelemetry;
  return $("apaMap").value === data.map_name ? (data.slot_choices || [])
    : (apaMapChoices().find((row) => row.map_name === $("apaMap").value)?.slot_choices || []);
}

function apaSlotOccupancy(slot) {
  if (finiteTelemetryNumber(slot?.occupancy_frame) !== null &&
      ["free", "occupied", "unknown"].includes(slot?.occupancy_state)) return slot.occupancy_state;
  return "unknown";
}

function apaSlotKindLabel(slot = {}, withDirection = true) {
  const kind = APA_SLOT_LABELS[slot.kind ?? slot.slot_kind] || slot.kind || slot.slot_kind || "--";
  if (!withDirection) return kind;
  return slot.parking_direction === "head_in" ? `${kind}头入`
    : slot.parking_direction === "reverse_in" ? `${kind}尾入` : kind;
}

function apaPhysicalSlotId(slot) {
  return Number(slot?.physical_slot_id ?? slot?.slot_id);
}

function apaSlotDirection(slot) {
  return slot?.parking_direction || ((slot?.kind ?? slot?.slot_kind) === "parallel" ? "parallel" : "reverse_in");
}

function apaSlotGroups(slots) {
  const groups = new Map();
  slots.forEach((slot) => {
    const physical = apaPhysicalSlotId(slot);
    if (!groups.has(physical)) groups.set(physical, []);
    groups.get(physical).push(slot);
  });
  return [...groups.values()];
}

function apaGroupOccupancy(variants) {
  const states = variants.map(apaSlotOccupancy);
  return states.includes("free") ? "free" : states.includes("unknown") ? "unknown" : "occupied";
}

function updateApaEntryDirection() {
  const slots = apaDraftSlots();
  const selected = slots.find((slot) => slot.slot_id === Number($("apaTargetSlot").value));
  const variants = selected ? slots.filter((slot) => apaPhysicalSlotId(slot) === apaPhysicalSlotId(selected)) : [];
  const direction = selected ? apaSlotDirection(selected) : "";
  const names = {reverse_in: "尾入", head_in: "头入"};
  document.querySelectorAll('input[name="apaEntryDirection"]').forEach((input) => {
    const target = variants.find((slot) => apaSlotDirection(slot) === input.value);
    input.checked = direction === input.value;
    input.disabled = $("apaTargetSlot").disabled || !target || apaSlotOccupancy(target) === "occupied";
    input.parentElement.title = !target ? `该车位暂未配置${names[input.value]}目标`
      : apaSlotOccupancy(target) === "occupied" ? `${names[input.value]}目标位置已占用` : "";
  });
  const headIn = variants.find((slot) => apaSlotDirection(slot) === "head_in");
  $("apaEntryDirectionHint").textContent = !selected ? "选择车位后可设置泊入方式"
    : direction === "parallel" ? "平行车位无需选择头入或尾入"
    : apaSlotOccupancy(selected) === "occupied" ? "当前泊入方式的目标位置已占用"
    : !headIn ? "该车位暂未配置头入目标"
    : variants.some((slot) => apaSlotOccupancy(slot) === "occupied") ? "已占用的泊入方式暂不可选" : "";
}

function renderApaSlotChoices(slots, slotId, preferredDirection) {
  const select = $("apaTargetSlot");
  const selected = slots.find((slot) => slot.slot_id === Number(slotId));
  const direction = preferredDirection ?? apaSlotDirection(selected);
  const groups = apaSlotGroups(slots);
  const targets = groups.map((variants) => variants.find((slot) => apaSlotDirection(slot) === direction)
    || variants.find((slot) => apaSlotDirection(slot) === "reverse_in") || variants[0]);
  const ids = targets.map((slot) => String(slot.slot_id));
  if (JSON.stringify([...select.options].map((option) => option.value)) !== JSON.stringify(ids)) {
    select.replaceChildren(...ids.map((id) => new Option("", id)));
  }
  const labels = {free: "空闲", occupied: "已占用", unknown: "待确认"};
  targets.forEach((slot, index) => {
    const variants = groups[index];
    const occupancy = apaGroupOccupancy(variants);
    const option = select.options[index];
    option.disabled = occupancy === "occupied";
    const blocked = occupancy === "occupied" && variants.every((row) => row.occupancy_reason === "parking_envelope_blocked");
    const edge = occupancy === "free" && slot.occupancy_edge_state !== "free" && slot.occupancy_edge_obstacle_id;
    const partial = variants.some((row) => apaSlotOccupancy(row) !== occupancy);
    const detail = blocked ? "目标停车位置存在固定障碍，请选择其他车位"
      : occupancy === "occupied" ? "车辆、行人或物体与目标停车位置重叠"
      : edge ? "边缘有障碍或尚待确认，目标停车位置未被占据；泊入仍需通过完整路径安全检查"
      : occupancy === "free" ? "目标停车位置已确认无障碍，泊入仍需通过完整路径安全检查"
      : "底图覆盖、静态遮挡或现场障碍尚未确认；切换车位不会重新采样";
    const variantStates = variants.length > 1 ? variants.map((row) =>
      `${apaSlotDirection(row) === "head_in" ? "头入" : "尾入"}：${labels[apaSlotOccupancy(row)]}`).join("；") : "";
    option.title = variantStates ? `${variantStates}。${detail}` : detail;
    const availability = blocked ? "障碍阻挡" : partial && occupancy === "free" ? "部分可用"
      : edge ? "空闲 · 边缘受限" : labels[occupancy];
    const label = `${apaPhysicalSlotId(slot)} 号 · ${apaSlotKindLabel(slot, false)} · ${availability}`;
    if (option.textContent !== label) option.textContent = label;
  });
  const target = selected ? targets.find((slot) => apaPhysicalSlotId(slot) === apaPhysicalSlotId(selected)) : null;
  if (select.value !== String(target?.slot_id ?? "")) select.value = String(target?.slot_id ?? "");
  updateApaEntryDirection();
}

function changeApaMapDraft() {
  const choice = apaMapChoices().find((row) => row.map_name === $("apaMap").value);
  const slots = apaDraftSlots();
  const slot = slots.find((row) => row.slot_id === choice?.default_slot_id && apaSlotOccupancy(row) !== "occupied")
    || slots.find((row) => apaSlotOccupancy(row) !== "occupied");
  renderApaSlotChoices(slots, slot?.slot_id);
  $("apaStartMode").value = "slot";
  changeApaDraft(true);
}

function changeApaDraft(loadDefault = false) {
  state.apaDraftDirty = true;
  if (loadDefault && $("apaStartMode").value === "slot") {
    const slot = apaDraftSlots().find((row) => row.slot_id === Number($("apaTargetSlot").value));
    writeApaPose(slot?.default_pose);
  }
  apaFeedback("设置未应用。");
  updateApaTaskControls();
}

function updateApaTaskControls() {
  const data = state.apaTelemetry;
  const view = apaUiState(data);
  const {mapLoading, geometryPreparing, busy, ready, stopping, needsReset, parking, parked, completed, previewReady} = view;
  const changingMap = Boolean($("apaMap").value && $("apaMap").value !== data.map_name);
  const draftSlot = apaDraftSlots().find((row) => row.slot_id === Number($("apaTargetSlot").value));
  const activeSlot = (data.slot_choices || []).find((row) => row.slot_id === data.slot_id);
  const parkOutAvailable = Boolean(activeSlot?.park_out_available);
  const returningToStart = activeSlot?.park_out_target?.source === "park_in_start";
  const occupancy = apaSlotOccupancy(activeSlot);
  const canStart = view.canStart && !state.apaDraftDirty && Boolean(activeSlot);
  setDisabled("apaPlanOnlyBtn", !canStart || occupancy !== "free" || (parking && !previewReady));
  setDisabled("apaParkInBtn", !canStart || occupancy !== "free" || (parking && !previewReady && data.parking_action === 1));
  setDisabled("apaParkOutBtn", !canStart || !parkOutAvailable || (parking && data.parking_action === 2));
  const canCancel = Boolean(data.session_id && !state.scenarioBusy && !mapLoading &&
    !(state.apaTaskBusy && state.apaTaskAction === "cancel") &&
    (parking || state.apaTaskBusy || state.apaTaskUnconfirmed));
  setDisabled("apaCancelBtn", !canCancel);
  const receipt = apaCommandReceipt(data);
  $("apaRetryCommandBtn").hidden = !state.apaTaskUnconfirmed ||
    state.apaSubmittedCommand?.event_id !== state.apaUnconfirmedEventId ||
    ["running", "loading"].includes(receipt?.state);
  setDisabled("apaRetryCommandBtn", state.apaTaskBusy || state.scenarioBusy || !data.online);
  setDisabled("apaResetBtn", !ready || busy || stopping);
  setDisabled("apaPrepareMapBtn", !ready || busy || !parked || changingMap || state.apaDraftDirty ||
    !(data.geometry?.map_preparation?.current_available ?? data.geometry?.map_preparation?.available));
  setDisabled("apaApplyBtn", !ready || busy || stopping || !draftSlot || apaSlotOccupancy(draftSlot) === "occupied" || (changingMap && !parked));
  setDisabled("apaMap", !ready || busy || !parked);
  setDisabled("apaTargetSlot", !ready || busy);
  updateApaEntryDirection();
  setDisabled("apaStartMode", !ready || busy);
  setDisabled("apaUseCurrentBtn", !ready || busy || changingMap);
  $("apaApplyBtn").textContent = mapLoading ? "正在加载地图…" : changingMap ? "切换地图并重置" : "应用设置并重置";
  $("apaMap").title = ready && !parked ? "退出泊车并停稳后可切换地图" : "";
  $("apaMapHint").textContent = "";
  $("apaMapHint").hidden = true;
  $("apaSavedPose").hidden = !state.apaDraftDirty || !data.initial_pose;
  setDisabled("apaReportBtn", busy || !state.apaSamples.length);
  for (const id of ["apaStartX", "apaStartY", "apaStartYaw"]) {
    $(id).readOnly = $("apaStartMode").value !== "custom";
    $(id).disabled = !ready || busy;
  }
  $("apaSetupForm").setAttribute("aria-busy", String(busy));
  const blockedReason = state.scenarioBusy ? "环境处理中"
    : !view.online ? "请先连接环境"
    : stopping ? "停稳后可应用设置或重置"
    : !view.feedbackAvailable ? "等待车辆反馈"
    : geometryPreparing ? "正在准备地图"
    : busy ? "等待操作确认"
    : needsReset ? "请先重置任务"
    : !ready ? "正在初始化车辆"
    : view.geometryPending ? "请检查地图错误信息"
    : state.apaDraftDirty ? "请先应用设置"
    : !view.linked ? "等待算法连接"
    : !view.stationary ? "请先停稳车辆"
    : data.control_status === "plan_failed" ? "退出后可调整起点重试"
    : data.control_status === "tracking_fault" ? "请重置后重试"
    : previewReady ? ""
    : completed ? ""
    : ["planning", "replan_stopping", "replan_waiting"].includes(data.control_status) ? ""
    : parking ? ""
    : occupancy === "occupied" ? "目标车位被占用，请选择其他车位"
    : occupancy === "unknown" ? "等待车位状态确认"
    : "";
  $("apaSlotHint").textContent = view.online && !parking && occupancy === "occupied" ? "车位已占用" : "";
  $("apaSlotHint").hidden = !$("apaSlotHint").textContent;
  const startBlockedReason = blockedReason || "当前任务执行中，结束或退出后再开始。";
  $("apaParkInBtn").title = !canStart ? startBlockedReason
    : parking && !previewReady && data.parking_action === 1 ? "再次泊入请先重置任务"
    : occupancy === "unknown" ? "车位状态尚未确认，暂不能开始泊入"
    : occupancy === "occupied" ? "目标停车位置被占用或有障碍" : "";
  $("apaPlanOnlyBtn").title = !canStart ? startBlockedReason
    : parking && !previewReady ? "退出当前任务后可仅规划路径"
    : occupancy !== "free" ? "车位确认空闲后可规划" : "仅预览路径，车辆保持驻车";
  $("apaParkOutBtn").title = !canStart ? startBlockedReason
    : parking && data.parking_action === 2 ? "再次泊出请先重置任务"
    : !parkOutAvailable ? "当前没有可用的泊出目标"
    : returningToStart ? "返回本次泊入前的起点和朝向" : "从当前位置驶向配置的库外目标";
}

function renderApaGeometry(data) {
  const label = $("apaGeometryLabel"), detail = $("apaGeometryDetail");
  if (!label || !detail) return;
  const panel = label.closest(".apa-geometry");
  const geometry = data.geometry || {}, base = geometry.map_preparation || {};
  // Connection and loading belong to the task bar; this panel shows map issues.
  panel.hidden = true;
  label.textContent = "";
  detail.textContent = "";
  if (!data.online || !data.initial_pose || geometry.preparing ||
      data.map_transition?.state === "loading" || state.apaPendingMap) return;
  if (base.scope !== "offline_map") {
    label.textContent = "地图尚未就绪";
    detail.textContent = "请更新并重启 Bridge。";
  } else if (base.state !== "ready") {
    label.textContent = "地图不可用";
    detail.textContent = base.message || "请确认地图文件已准备好，再重新加载。";
  } else if (base.coverage_quality?.missing_layers > 0) {
    label.textContent = "地图覆盖不完整";
    detail.textContent = "部分区域暂不可用，请补齐地图数据后重新加载。";
  } else {
    return;
  }
  panel.hidden = false;
}

function renderApaScene(data) {
  renderApaHeader(data);
  renderApaGeometry(data);
  const view = apaUiState(data);
  const transition = data.map_transition || state.apaTelemetry.map_transition;
  const loadingMap = state.apaPendingMap || (transition?.state === "loading" ? transition.map_name : "");
  if (!data.online || !data.session_id) {
    if ($("apaOccupancySummary")) {
      $("apaOccupancySummary").textContent = "";
      $("apaOccupancySummary").hidden = true;
    }
    const failed = transition?.state === "failed";
    $("apaTaskState").textContent = failed && !loadingMap ? "地图加载失败 · 未连接" : view.label;
    $("apaTaskContext").textContent = loadingMap ? "地图切换后保持驻车，请稍候" : failed ? transition.message : "未收到车辆状态";
    document.querySelector(".apa-task-state").classList.remove("is-active");
    document.querySelector(".apa-task-state").classList.toggle("is-error", Boolean(failed && !loadingMap));
    updateApaTaskControls();
    return;
  }
  const maps = apaMapChoices(data);
  const mapsKey = JSON.stringify(maps.map(({map_name, available, reason}) => ({map_name, available, reason})));
  if (mapsKey !== state.apaMapChoicesKey) {
    const selectedMap = $("apaMap").value;
    $("apaMap").innerHTML = maps.map((row) => `<option value="${escapeHtml(row.map_name)}" ${row.available ? "" : "disabled"}>${escapeHtml(row.map_name)}${row.available ? "" : ` · ${escapeHtml(row.reason || "不可用")}`}</option>`).join("");
    $("apaMap").value = maps.some((row) => row.map_name === selectedMap) ? selectedMap : data.map_name;
    state.apaMapChoicesKey = mapsKey;
  }
  const key = `${data.session_id}:${data.map_name}:${data.slot_id}:${JSON.stringify(data.initial_pose)}`;
  const sessionChanged = state.apaSceneKey.split(":")[0] !== data.session_id;
  if (data.initial_pose && (sessionChanged || (state.apaSceneKey !== key && !state.apaDraftDirty))) {
    $("apaMap").value = data.map_name;
    renderApaSlotChoices(data.slot_choices || [], data.slot_id);
    $("apaStartMode").value = data.start_mode || "slot";
    writeApaPose(data.initial_pose);
    state.apaSceneKey = key;
    state.apaDraftDirty = false;
    if (!state.apaTaskBusy && !state.apaTaskUnconfirmed) apaFeedback("");
  }
  // Refresh availability without rebuilding an open select or losing a draft.
  renderApaSlotChoices(apaDraftSlots(), $("apaTargetSlot").value);
  const occupancySummary = $("apaOccupancySummary");
  if (occupancySummary) {
    const summary = data.slot_occupancy || {};
    const counts = {free: 0, occupied: 0, unknown: 0, blocked: 0};
    apaSlotGroups(data.slot_choices || []).forEach((variants) => {
      const occupancy = apaGroupOccupancy(variants);
      counts[occupancy] += 1;
      if (occupancy === "occupied" && variants.every((slot) => slot.occupancy_reason === "parking_envelope_blocked"))
        counts.blocked += 1;
    });
    occupancySummary.textContent = $("apaMap").value !== data.map_name
      ? "待加载所选地图"
      : summary.frame ? `空闲 ${counts.free} · 占用 ${counts.occupied - counts.blocked}${counts.blocked ? ` · 受阻 ${counts.blocked}` : ""}${counts.unknown ? ` · 待确认 ${counts.unknown}` : ""}${summary.preparing_geometry ? " · 准备中" : ""}`
      : "";
    occupancySummary.hidden = !occupancySummary.textContent;
  }
  const pose = data.initial_pose;
  $("apaSavedPose").textContent = pose ? `生效起点：X ${pose.x.toFixed(2)}, Y ${pose.y.toFixed(2)}, ${pose.yaw_deg.toFixed(1)}°` : "生效起点：--";
  const replanInfo = apaReplanInfo(data);
  if ($("apaTaskState").textContent !== view.label) $("apaTaskState").textContent = view.label;
  const elapsedText = replanInfo.activePlanning ? ` · ${Number(data.replan?.elapsed_sec || 0).toFixed(1)} s` : "";
  const progress = replanInfo.stageLabel && (view.parking || replanInfo.isReplan || replanInfo.activePlanning)
    ? ` · ${replanInfo.stageLabel}${elapsedText}` : "";
  $("apaTaskContext").textContent = `任务 ${data.task_id}${progress}`;
  document.querySelector(".apa-task-state").classList.toggle("is-active", view.parking && view.linked &&
    !view.completed && !view.previewReady && !view.safety.reason && !view.notice?.error);
  document.querySelector(".apa-task-state").classList.toggle("is-error", Boolean(view.notice?.error || data.control_status === "map_failed"));
  updateApaTaskControls();
}

function apaCommandReceipt(data) {
  const command = state.apaSubmittedCommand || state.apaObservedCommand;
  if (!command) return null;
  return [data.last_task_command, data.map_transition].find((receipt) => receipt &&
    receipt.event_id === command.event_id && receipt.source_session_id === command.session_id &&
    receipt.source_task_id === command.task_id && receipt.action === command.action &&
    (command.action !== "configure" || !command.map_name || receipt.map_name === command.map_name)) || null;
}

function finishApaCommand(result) {
  // Polling and HTTP responses share this terminal transition. Invalidate the
  // outstanding HTTP handler so a late 202/timeout cannot reopen this event.
  state.apaTaskRevision += 1;
  state.apaPollRevision += 1;
  state.apaTaskBusy = false;
  state.apaTaskAction = "";
  state.apaTaskUnconfirmed = false;
  state.apaUnconfirmedEventId = "";
  state.apaSubmittedCommand = null;
  state.apaObservedCommand = null;
  state.apaPendingMap = "";
  if (result.ok && ["reset", "configure"].includes(result.action)) {
    state.apaDraftDirty = false;
    state.apaSceneKey = "";
    state.logEpoch += 1;
    state.renderedLog = "";
    $("logBox").textContent = "";
  }
  // Command receipts and live motion/completion are separate states.
  const acknowledgements = {configure: "设置已应用。", reset: "任务已重置。", cancel: "退出请求已确认。",
    prepare_map: "离线泊车底图已加载，车辆保持驻车。",
    plan_only: "规划请求已确认。", park_in: "泊入请求已确认。", park_out: "泊出请求已确认。"};
  return [result.ok ? (acknowledgements[result.action] || "操作已确认。")
    : (result.message || "操作未完成，请查看当前状态。"), result.ok ? "success" : "error"];
}

async function runApaTask(action, extra = {}, retry = false) {
  const cancel = action === "cancel";
  updateApaTaskControls();
  const buttonId = {plan_only: "apaPlanOnlyBtn", park_in: "apaParkInBtn", park_out: "apaParkOutBtn", cancel: "apaCancelBtn",
    reset: "apaResetBtn", configure: "apaApplyBtn", prepare_map: "apaPrepareMapBtn"}[action];
  if ((!retry && (!buttonId || $(buttonId).disabled)) || (retry && !state.apaSubmittedCommand)) return;
  if ((!retry && state.apaPendingMap) || state.scenarioBusy || (!retry && !cancel && state.apaTaskUnconfirmed) ||
      (state.apaTaskBusy && (!cancel || state.apaTaskAction === "cancel")) ||
      (!state.apaTelemetry.online && !(cancel && state.apaTelemetry.session_id))) return;
  const replacingUnconfirmed = cancel && (state.apaTaskBusy || state.apaTaskUnconfirmed);
  const id = state.selectedId;
  const revision = ++state.apaTaskRevision;
  const command = retry ? state.apaSubmittedCommand : {
    action,
    event_id: globalThis.crypto?.randomUUID?.() || `${Date.now()}-${Math.random().toString(16).slice(2)}`,
    session_id: state.apaTelemetry.session_id, task_id: state.apaTelemetry.task_id, ...extra,
  };
  const eventId = command.event_id;
  state.apaSubmittedCommand = command;
  state.apaObservedCommand = null;
  state.apaTaskBusy = true;
  state.apaTaskAction = action;
  state.apaTaskUnconfirmed = false;
  state.apaUnconfirmedEventId = eventId;
  const changingMap = action === "configure" && command.map_name !== state.apaTelemetry.map_name;
  state.apaPendingMap = changingMap ? command.map_name : "";
  state.apaPollRevision += 1;
  state.apaHeld.clear();
  updateActionState(state.scenarioBusy ? "running" : "idle");
  apaFeedback(retry ? "正在重新确认上次操作…" : changingMap ? `正在加载 ${command.map_name}，完成后保持驻车…` : "等待操作确认…");
  try {
    const result = await requestJson(`/api/scenarios/${id}/apa-command`, {
      method: "POST", headers: {"Content-Type": "application/json"},
      body: JSON.stringify(command),
      signal: AbortSignal.timeout(changingMap ? 490000 : 12000),
    });
    if (revision !== state.apaTaskRevision || state.selectedId !== id) return;
    renderApaTelemetry(result.telemetry);
    if (revision !== state.apaTaskRevision || state.selectedId !== id) return;
    if (result.pending) {
      state.apaTaskUnconfirmed = true;
      apaFeedback(apaCommandReceipt(state.apaTelemetry)?.message || result.message || "操作已接收，等待执行完成…");
      return;
    }
    // An HTTP success without this event's receipt is not execution evidence.
    state.apaTaskUnconfirmed = true;
    apaFeedback("尚未收到匹配的操作结果，继续等待确认…");
  } catch (err) {
    if (revision !== state.apaTaskRevision || state.selectedId !== id) return;
    if (err.data?.pending_command && err.data.pending_event_id !== eventId) {
      state.apaObservedCommand = err.data.pending_command;
      state.apaSubmittedCommand = null;
      state.apaUnconfirmedEventId = err.data.pending_event_id;
      state.apaPendingMap = "";
    }
    if (err.data?.telemetry) renderApaTelemetry(err.data.telemetry);
    if (revision !== state.apaTaskRevision || state.selectedId !== id) return;
    const unconfirmed = err.status >= 500 || err.name === "TimeoutError" || !err.status ||
      Boolean(err.data?.pending_event_id) || replacingUnconfirmed;
    state.apaTaskUnconfirmed = unconfirmed;
    if (!unconfirmed) {
      apaFeedback(...finishApaCommand({ok: false, action, message: err.message}));
      updateActionState(state.scenarioBusy ? "running" : "idle");
      return;
    }
    state.apaUnconfirmedEventId = err.data?.event_id || err.data?.pending_event_id || eventId;
    const progress = apaCommandReceipt(state.apaTelemetry);
    const message = progress && ["running", "loading"].includes(progress.state)
      ? progress.message : err.data?.pending_event_id ? err.message : "连接或响应超时，正在等待这次操作的最终结果…";
    apaFeedback(message);
    appendLocalLog(`APA 操作反馈：${message}`);
  } finally {
    if (revision === state.apaTaskRevision && state.selectedId === id) {
      state.apaTaskBusy = false;
      state.apaTaskAction = "";
      state.apaPollRevision += 1;
      updateActionState(state.scenarioBusy ? "running" : "idle");
      await pollApaTelemetry();
    }
  }
}

function applyApaSetup(event) {
  event.preventDefault();
  const slot = apaDraftSlots().find((row) => row.slot_id === Number($("apaTargetSlot").value));
  if (!slot || apaSlotOccupancy(slot) === "occupied") {
    apaFeedback(slot ? "当前泊入方式的目标位置已占用，请调整车位或泊入方式。" : "请选择目标车位。", "error");
    $("apaTargetSlot").focus();
    return;
  }
  const extra = {map_name: $("apaMap").value, slot_id: Number($("apaTargetSlot").value), start_mode: $("apaStartMode").value};
  if (extra.start_mode === "custom") {
    const fields = ["apaStartX", "apaStartY", "apaStartYaw"];
    const invalid = fields.filter((id) => !$(id).value.trim() || !Number.isFinite(Number($(id).value)));
    fields.forEach((id) => $(id).setAttribute("aria-invalid", String(invalid.includes(id))));
    if (invalid.length) {
      apaFeedback("请输入有效的 X、Y 和航向。", "error");
      $(invalid[0]).focus();
      return;
    }
    extra.initial_pose = {x: Number($("apaStartX").value), y: Number($("apaStartY").value), yaw_deg: Number($("apaStartYaw").value)};
  }
  runApaTask("configure", extra);
}

function beginApa(action) { return runApaTask(action); }
function cancelApaAssist() { return runApaTask("cancel"); }

function formatMetric(value, digits, unit) {
  const number = finiteTelemetryNumber(value);
  if (number === null) return "--";
  return `${number.toFixed(digits)}${unit || ""}`;
}

function zipXy(xs, ys) {
  const left = Array.isArray(xs) ? xs : [];
  const right = Array.isArray(ys) ? ys : [];
  const count = Math.min(left.length, right.length);
  const points = [];
  for (let index = 0; index < count; index += 1) {
    const x = Number(left[index]);
    const y = Number(right[index]);
    if (Number.isFinite(x) && Number.isFinite(y)) points.push({x, y});
  }
  return points;
}

function renderApaInputLayers(input, toSvg, line) {
  if (!input.available) return "";
  const layers = state.apaMapLayers;
  const polygon = (item) => {
    const points = item.points.map(([x, y]) => toSvg({x, y}))
      .map((p) => `${p.x.toFixed(3)},${p.y.toFixed(3)}`).join(" ");
    const velocity = item.velocity.some((v) => Math.abs(v) > 0.0001)
      ? ` · v=(${item.velocity[0].toFixed(2)}, ${item.velocity[1].toFixed(2)}) m/s` : "";
    const envelope = item.kind === "live" ? " · 原始输入包围盒投影，显示附加膨胀 0" : "";
    return `<polygon class="map-${item.kind}" points="${points}"><title>${escapeHtml(item.id + velocity + envelope)}</title></polygon>`;
  };
  const geometry = ["forbidden", "static", "unknown", "live"].map((kind) => layers[kind]
    ? input.obstacles.filter((item) => item.kind === kind).map(polygon).join("") : "");
  for (const [key, kind, label] of [["boundary_lines", "boundary", "边界线"],
    ["virtual_lines", "virtual", "车位约束"], ["stopper_lines", "boundary", "轮挡线"]]) {
    if (layers[kind]) geometry.push(input[key].map((points, index) =>
      `<g><title>${label} ${index}</title>${line(points.map(([x, y]) => ({x, y})), `map-${kind}`)}</g>`).join(""));
  }
  if (layers.boundary) geometry.push(input.constraint_points.map((item) => {
    const [x, y] = item.point;
    const p = toSvg({x, y});
    return `<circle class="map-point" cx="${p.x.toFixed(3)}" cy="${p.y.toFixed(3)}" r="1.5"><title>${escapeHtml(item.id)}</title></circle>`;
  }).join(""));
  if (state.apaMapView === "slots" && input.bounds) {
    const [x1, y1, x2, y2] = input.bounds;
    const points = [{x:x1,y:y1}, {x:x2,y:y1}, {x:x2,y:y2}, {x:x1,y:y2}];
    const projected = points.map(toSvg);
    const x = Math.min(...projected.map(p => p.x)) + 3;
    const y = Math.min(...projected.map(p => p.y)) + 10;
    geometry.push(`<g class="map-input-window"><title>当前任务输入范围，框外车位仅供显示参考</title>${line(points.concat([points[0]]), "map-input-bounds")}<text x="${x.toFixed(2)}" y="${y.toFixed(2)}">当前输入范围</text></g>`);
  }
  return `<defs><pattern id="apaInputForbiddenHatch" width="7" height="7" patternUnits="userSpaceOnUse"><path d="M0 3.5 H7"/></pattern><pattern id="apaInputUnknownHatch" width="6" height="6" patternUnits="userSpaceOnUse"><path d="M-1 1 L1 -1 M0 6 L6 0 M5 7 L7 5"/></pattern></defs>${geometry.join("")}`;
}

function apaParkingMapKey(data) {
  const sha = data.parking_map?.opendrive_sha256;
  return data.online && data.vehicle_feedback_available !== false && data.session_id && data.map_name &&
    data.map_transition?.state !== "loading" && typeof sha === "string" && /^[0-9a-f]{64}$/.test(sha)
    ? `${data.session_id}:${data.map_name}:${sha}` : "";
}

function refreshApaParkingMap(data) {
  const key = apaParkingMapKey(data);
  if (key !== state.apaParkingMap.key) {
    state.apaParkingMapRequest?.abort();
    state.apaParkingMapRequest = null;
    state.apaParkingMap = {key, available: false, slots: []};
  }
  if (!key || state.apaParkingMap.available || state.apaParkingMapRequest ||
      Date.now() < (state.apaParkingMap.retryAt || 0)) return;
  const controller = new AbortController();
  const scenarioId = state.selectedId;
  state.apaParkingMapRequest = controller;
  state.apaParkingMap.loading = true;
  const timeout = setTimeout(() => controller.abort(), 2500);
  requestJson(`/api/scenarios/${encodeURIComponent(scenarioId)}/parking-slots?session_id=${encodeURIComponent(data.session_id)}`,
    {cache: "no-store", signal: controller.signal}).then((catalog) => {
    if (state.selectedId !== scenarioId || state.apaParkingMap.key !== key ||
        apaParkingMapKey(state.apaTelemetry) !== key) return;
    if (!catalog.available || catalog.session_id !== data.session_id || catalog.map_name !== data.map_name ||
        catalog.opendrive_sha256 !== data.parking_map.opendrive_sha256 ||
        catalog.coordinate_frame !== "carla_world" || !Array.isArray(catalog.slots))
      throw new Error("地图车位来源不匹配");
    state.apaParkingMap = {...catalog, key, loading: false};
  }).catch(() => {
    if (state.apaParkingMap.key === key)
      state.apaParkingMap = {key, available: false, slots: [], loading: false, retryAt: Date.now() + 5000};
  }).finally(() => {
    clearTimeout(timeout);
    if (state.apaParkingMapRequest === controller) state.apaParkingMapRequest = null;
    if (state.selectedId === scenarioId && apaParkingMapKey(state.apaTelemetry) === key)
      renderApaTrajectory(state.apaTelemetry);
  });
}

function apaParkingSlots(data) {
  // Only the active Bridge catalog belongs on this map. Form/map drafts and
  // cached offline telemetry must never supply geometry for another scene.
  if (!data.online || data.vehicle_feedback_available === false || !data.session_id ||
      !data.map_name || data.map_transition?.state === "loading") return [];
  const input = data.input_map || {};
  if (input.available && (input.session_id !== data.session_id || input.task_id !== data.task_id)) return [];
  const catalog = state.apaParkingMap;
  const complete = catalog.available && catalog.key === apaParkingMapKey(data);
  const rows = complete ? catalog.slots : data.slot_choices;
  const physicalSlots = new Set();
  return (Array.isArray(rows) ? rows : []).flatMap((slot) => {
    const configured = Number.isInteger(slot?.slot_id);
    if ((!configured && !(complete && slot?.slot_id === null && typeof slot.id === "string")) ||
        !Array.isArray(slot.corners) || (configured ? slot.corners.length !== 4
          : slot.corners.length < 3 || slot.corners.length > 256)) return [];
    const physical = configured ? (slot.physical_slot_id ?? slot.slot_id) : null;
    if (configured && physicalSlots.has(physical)) return [];
    const rawParts = !configured && Array.isArray(slot.parts) ? slot.parts : [slot.corners];
    if (!rawParts.length || rawParts.length > 32 || rawParts.some(part => !Array.isArray(part) ||
        part.length < 3 || part.length > 256)) return [];
    const parts = rawParts.map(part => part.map(point => ({
      x: finiteTelemetryNumber(point?.x), y: finiteTelemetryNumber(point?.y),
    })));
    const corners = parts.flat();
    if (configured && corners.every((point) => point.x !== null && point.y !== null)) physicalSlots.add(physical);
    return corners.every((point) => point.x !== null && point.y !== null)
      ? [{id: slot.id || `slot:${physical}`, slot_id: configured ? physical : null, corners, parts,
          road_id: slot.road_id, object_ids: slot.object_ids}] : [];
  });
}

function renderApaParkingSlots(slots, targetId, toSvg, line, scale) {
  if (!state.apaMapLayers.slots) return "";
  return slots.map((slot) => {
    const target = slot.slot_id === targetId;
    const projected = slot.corners.map(toSvg);
    if (Math.max(...projected.map(p => p.x)) < 0 || Math.min(...projected.map(p => p.x)) > 360 ||
        Math.max(...projected.map(p => p.y)) < 0 || Math.min(...projected.map(p => p.y)) > 240) return "";
    const center = slot.corners.reduce((sum, point) => ({
      x: sum.x + point.x / slot.corners.length, y: sum.y + point.y / slot.corners.length,
    }), {x: 0, y: 0});
    const p = toSvg(center);
    // Keep overview clusters legible. Small spaces retain their outline and
    // source tooltip; the active target always keeps its number.
    const minEdge = Math.min(...slot.corners.map((point, index) => {
      const next = slot.corners[(index + 1) % slot.corners.length];
      return Math.hypot(point.x - next.x, point.y - next.y);
    }));
    const label = Number.isInteger(slot.slot_id) && (target || minEdge * scale >= 9)
      ? `<text class="map-slot-label${target ? " is-target" : ""}" x="${p.x.toFixed(3)}" y="${p.y.toFixed(3)}" text-anchor="middle" dominant-baseline="central">${slot.slot_id}</text>` : "";
    const title = Number.isInteger(slot.slot_id) ? `${slot.slot_id} 号车位${target ? " · 当前目标" : ""}`
      : `地图车位${slot.road_id ? ` · 道路 ${slot.road_id}` : ""} · 标线对象 ${(slot.object_ids || []).join(", ")}`;
    const outlines = slot.parts.map(part => line(part.concat([part[0]]), "map-slot")).join("");
    return `<g class="map-parking-slot" data-outline-id="${escapeHtml(slot.id)}"${Number.isInteger(slot.slot_id) ? ` data-slot-id="${slot.slot_id}"` : ""}><title>${escapeHtml(title)}</title>${outlines}${label}</g>`;
  }).join("");
}

function updateApaInputMapControls(input, slots) {
  const available = Boolean(input.available);
  const hasSlots = slots.length > 0;
  const completeSlots = state.apaParkingMap.available && hasSlots;
  if ((state.apaMapView === "slots" && !completeSlots) || (state.apaMapView === "full" && !available))
    state.apaMapView = "nearby";
  document.querySelectorAll("[data-apa-map-view]").forEach((button) => {
    button.disabled = button.dataset.apaMapView === "slots" ? !hasSlots || !completeSlots
      : button.dataset.apaMapView === "full" ? !available : !available && !hasSlots;
    button.setAttribute("aria-pressed", String(button.dataset.apaMapView === state.apaMapView));
  });
  document.querySelectorAll("[data-apa-map-layer]").forEach((button) => {
    button.disabled = button.dataset.apaMapLayer === "slots" ? !hasSlots : !available;
    button.setAttribute("aria-pressed", String(state.apaMapLayers[button.dataset.apaMapLayer]));
  });
  const readout = $("apaInputMapReadout");
  if (readout) {
    const counts = {forbidden: 0, static: 0, live: 0, unknown: 0};
    (input.obstacles || []).forEach((item) => counts[item.kind]++);
    const unavailable = {offline: "输入地图已断开", feedback_unavailable: "等待有效车辆反馈",
      invalid_geometry: "输入几何无效，暂停绘制障碍与约束"};
    const sceneText = available
      ? `地图帧 ${input.frame} · 禁行区域 ${counts.forbidden} 片 · 实体 ${counts.static} · 物体包络 ${counts.live} · 未知 ${counts.unknown}${input.boundaries_available
        ? ` · 边界 ${input.boundary_lines.length} / 车位约束 ${input.virtual_lines.length}（帧 ${input.boundary_frame}）`
        : " · 尚未收到本任务边界输入"}`
      : unavailable[input.reason] || "等待泊车输入地图";
    const slotText = completeSlots ? `地图车位 ${slots.length} 个（含已占用车位）`
      : state.apaParkingMap.loading ? `地图车位加载中 · 已配置 ${slots.length} 个`
        : hasSlots ? `地图车位尚未加载，仅显示 ${slots.length} 个已配置车位` : "车位边框待接收";
    readout.textContent = `${sceneText} · ${slotText}${state.apaMapView === "slots" ? " · 全部车位总览，障碍仅覆盖当前输入窗口" : ""}`;
  }
  const hint = $("apaInputMapHint");
  if (hint) {
    hint.textContent = !available ? (input.reason === "invalid_geometry" ? "地图几何数据无效" : "")
      : state.apaMapView === "slots" ? "虚线框为规划输入范围"
      : !input.boundaries_available ? "车位约束待接收"
      : !completeSlots ? "车位边框尚未完整加载" : "";
    hint.hidden = !hint.textContent;
  }
}

function renderApaTrajectory(data = {}) {
  const svg = $("apaTrajectorySvg");
  if (!svg) return;
  const algo = zipXy(data.algorithm_path_x, data.algorithm_path_y);
  const actual = zipXy(data.actual_path_x, data.actual_path_y);
  const trajectory = data.trajectory || {};
  const previousPlans = (Array.isArray(trajectory.previous_plans) ? trajectory.previous_plans : [])
    .map((plan) => zipXy(plan.x, plan.y));
  if ($("apaPrevPlanLegend")) $("apaPrevPlanLegend").hidden = !previousPlans.length;
  const yaws = Array.isArray(data.algorithm_path_yaw) ? data.algorithm_path_yaw : [];
  const odos = Array.isArray(data.algorithm_path_odo) ? data.algorithm_path_odo : [];
  const diagnostic = data.control_diagnostic || {};
  const input = data.input_map || {};
  const parkingSlots = apaParkingSlots(data);
  updateApaInputMapControls(input, parkingSlots);
  const mapCorners = input.available && input.target_slot.length
    ? input.target_slot.map(([x, y]) => ({x, y})) : data.slot_corners;
  const corners = (Array.isArray(mapCorners) ? mapCorners : [])
    .map((item) => ({x: Number(item.x), y: Number(item.y)}))
    .filter((point) => Number.isFinite(point.x) && Number.isFinite(point.y));
  const pose = input.available ? input.ego_pose : [data.vehicle_x, data.vehicle_y, data.vehicle_yaw];
  const ego = {x: finiteTelemetryNumber(pose[0]), y: finiteTelemetryNumber(pose[1])};
  const start = {x: finiteTelemetryNumber(data.initial_pose?.x), y: finiteTelemetryNumber(data.initial_pose?.y)};
  const yaw = finiteTelemetryNumber(pose[2]);
  const geometry = data.vehicle_geometry || {};
  const widthM = finiteTelemetryNumber(geometry.width_m);
  const rearToFront = finiteTelemetryNumber(geometry.rear_to_front_m);
  const rearToBack = finiteTelemetryNumber(geometry.rear_to_back_m);
  const fChamLon = Math.max(0, finiteTelemetryNumber(geometry.front_chamfer_lon_m) ?? 0);
  const fChamLat = Math.max(0, finiteTelemetryNumber(geometry.front_chamfer_lat_m) ?? 0);
  const rChamLon = Math.max(0, finiteTelemetryNumber(geometry.rear_chamfer_lon_m) ?? 0);
  const rChamLat = Math.max(0, finiteTelemetryNumber(geometry.rear_chamfer_lat_m) ?? 0);
  const hasBodyDimensions = [widthM, rearToFront, rearToBack].every((value) => value !== null && value > 0);
  const forward = {x: Math.cos(yaw ?? 0), y: Math.sin(yaw ?? 0)};
  const right = {x: -forward.y, y: forward.x};
  const halfW = hasBodyDimensions ? widthM / 2 : 0;
  const localBody = hasBodyDimensions
    ? ((fChamLon > 0 || rChamLon > 0)
        ? [[rearToFront, halfW - fChamLat], [rearToFront, -(halfW - fChamLat)],
           [rearToFront - fChamLon, -halfW], [-rearToBack + rChamLon, -halfW],
           [-rearToBack, -(halfW - rChamLat)], [-rearToBack, halfW - rChamLat],
           [-rearToBack + rChamLon, halfW], [rearToFront - fChamLon, halfW]]
        : [[rearToFront, halfW], [rearToFront, -halfW],
           [-rearToBack, -halfW], [-rearToBack, halfW]])
    : [];
  const body = localBody.length && ego.x !== null && ego.y !== null && yaw !== null
    ? localBody.map(([front, side]) => ({
        x: ego.x + forward.x * front + right.x * side,
        y: ego.y + forward.y * front + right.y * side,
      })) : [];
  const bodyReadout = $("apaBodyReadout");
  if (bodyReadout) {
    let text = body.length
      ? `${(rearToFront + rearToBack).toFixed(2)} × ${widthM.toFixed(2)} m`
      : hasBodyDimensions ? "等待位姿" : "--";
    if (body.length && corners.length === 4) {
      const entry = (Number(data.slot_entry) || 1) - 1;
      const backA = corners[(entry + 2) % 4], backB = corners[(entry + 3) % 4];
      const nx = backB.y - backA.y, ny = backA.x - backB.x;
      const normalLength = Math.hypot(nx, ny);
      if (normalLength > 0) {
        const center = corners.reduce((sum, point) => ({x: sum.x + point.x / 4, y: sum.y + point.y / 4}), {x: 0, y: 0});
        const sign = (center.x - backA.x) * nx + (center.y - backA.y) * ny >= 0 ? 1 : -1;
        const clearance = Math.min(...body.map((point) => sign * ((point.x - backA.x) * nx + (point.y - backA.y) * ny) / normalLength));
        text += ` · 库底净距 ${clearance.toFixed(2)} m`;
      }
    }
    bodyReadout.textContent = text;
  }
  const all = algo.concat(actual, corners, body, ...previousPlans);
  const completeMap = input.available && state.apaMapView === "full";
  const allSlots = parkingSlots.length > 0 && state.apaMapView === "slots";
  if (allSlots) parkingSlots.forEach((slot) => all.push(...slot.corners));
  if (completeMap) {
    input.obstacles.forEach((item) => item.points.forEach(([x, y]) => all.push({x, y})));
    for (const key of ["boundary_lines", "virtual_lines", "stopper_lines"])
      input[key].forEach((points) => points.forEach(([x, y]) => all.push({x, y})));
    input.constraint_points.forEach((item) => all.push({x:item.point[0], y:item.point[1]}));
  }
  if ((completeMap || allSlots) && input.available && input.bounds) {
    const [x1, y1, x2, y2] = input.bounds;
    all.push({x:x1,y:y1}, {x:x1,y:y2}, {x:x2,y:y1}, {x:x2,y:y2});
  }
  if (ego.x !== null && ego.y !== null) all.push(ego);
  if (start.x !== null && start.y !== null) all.push(start);
  const replanInfo = apaReplanInfo(data);
  if ($("apaTrajMeta")) {
    $("apaTrajMeta").textContent = algo.length
      ? [
          replanInfo.stageLabel,
          replanInfo.isReplan && replanInfo.reasonText ? replanInfo.reasonText : "",
          `规划 ${algo.length} 点`,
          trajectory.retained ? "保留轨迹" : "",
        ].filter(Boolean).join(" · ")
      : replanInfo.activePlanning
        ? [replanInfo.stageLabel, replanInfo.reasonText || "正在搜索路径"].filter(Boolean).join(" · ")
        : "等待规划";
  }
  const trackingReadout = $("apaTrackingReadout");
  if (trackingReadout) {
    const errors = trajectory.tracking_error || {};
    trackingReadout.textContent = errors.samples
      ? `横偏 RMS ${formatMetric(errors.rms_cross_track_error_m * 100, 1, " cm")} · 峰值 ${formatMetric(errors.peak_cross_track_error_m * 100, 1, " cm")} · 航偏 ${formatMetric(errors.peak_heading_error_rad * 180 / Math.PI, 1, "°")}`
      : "--";
  }
  const controlReadout = $("apaControlReadout");
  if (controlReadout) {
    const phases = {2: "档位确认", 3: "跟踪", 4: "到点停车", 6: "泊车完成", 7: "减速", 8: "低速接近", 9: "保护停车"};
    controlReadout.textContent = diagnostic.segment_count
      ? `${replanInfo.stageLabel ? `${replanInfo.stageLabel} · ` : ""}第 ${diagnostic.segment}/${diagnostic.segment_count} 段 · ${phases[data.control_phase] || "准备"} · 剩余 ${Number(diagnostic.remaining_distance_m || 0).toFixed(2)} m · 横偏 ${Number(diagnostic.cross_track_error_m || 0).toFixed(2)} m`
      : replanInfo.activePlanning
        ? `${replanInfo.stageLabel || "正在规划"} · ${data.control_status === "replan_stopping" ? "停车准备重规划" : data.control_status === "replan_waiting" ? "等待场景变化" : "正在搜索路径"}${replanInfo.reasonText ? ` · ${replanInfo.reasonText}` : ""}`
        : "--";
  }
  if (!all.length) {
    svg.innerHTML = `<text x="180" y="120" text-anchor="middle" fill="#94a3b8" font-size="12">暂无轨迹</text>`;
    return;
  }
  // Plot and overhead camera share the fixed reset heading, including during replanning.
  const viewYawDeg = finiteTelemetryNumber(data.initial_pose?.yaw_deg);
  if (viewYawDeg === null) {
    svg.innerHTML = `<text x="180" y="120" text-anchor="middle" fill="#94a3b8" font-size="12">等待起点方向</text>`;
    return;
  }
  const viewYaw = viewYawDeg * Math.PI / 180;
  const toView = (point) => {
    const dx = point.x - (start.x ?? 0), dy = point.y - (start.y ?? 0);
    return {x: -Math.sin(viewYaw) * dx + Math.cos(viewYaw) * dy,
            y: -Math.cos(viewYaw) * dx - Math.sin(viewYaw) * dy};
  };
  const projected = all.map(toView);
  const xs = projected.map((point) => point.x);
  const ys = projected.map((point) => point.y);
  let minX = Math.min(...xs), maxX = Math.max(...xs);
  let minY = Math.min(...ys), maxY = Math.max(...ys);
  if ((input.available || parkingSlots.length) && !completeMap && !allSlots) {
    // Nearby view limits scale; complete-map view retains the full input extent.
    const cx = (minX+maxX)/2, cy = (minY+maxY)/2;
    minX = Math.min(minX,cx-9); maxX = Math.max(maxX,cx+9);
    minY = Math.min(minY,cy-6); maxY = Math.max(maxY,cy+6);
  }
  const span = Math.max(maxX - minX, maxY - minY, 4);
  const pad = Math.max(1.6, span * (completeMap || allSlots ? 0.04 : 0.12));
  const width = Math.max(maxX - minX, 0.5) + pad * 2;
  const height = Math.max(maxY - minY, 0.5) + pad * 2;
  const scale = Math.min(360 / width, 240 / height);
  const centerX = (minX + maxX) / 2;
  const centerY = (minY + maxY) / 2;
  const toSvg = (point) => {
    const view = toView(point);
    return {x: 180 + (view.x - centerX) * scale, y: 120 + (view.y - centerY) * scale};
  };
  const line = (points, cssClass) => {
    if (points.length < 2) return "";
    const d = points.map((point, index) => `${index ? "L" : "M"}${toSvg(point).x.toFixed(3)} ${toSvg(point).y.toFixed(3)}`).join(" ");
    // Slot outlines stay readable; constraint centerlines retain a finer stroke.
    const centerline = ["slot", "map-slot", "map-virtual", "map-boundary"].includes(cssClass);
    const strokeWidth = ["slot", "map-slot"].includes(cssClass)
      ? Math.max(0.7, Math.min(1.2, 0.04 * scale))
      : Math.max(0.2, Math.min(0.8, 0.02 * scale));
    const stroke = centerline ? ` style="stroke-width:${strokeWidth.toFixed(3)}"` : "";
    return `<path class="${cssClass}" d="${d}"${stroke}/>`;
  };
  const slot = corners.length >= 2
    ? line(corners.concat([corners[0]]), "slot")
    : "";
  let egoRect = "";
  if (body.length) {
    const centerOffset = (rearToFront - rearToBack) / 2;
    const center = {x: ego.x + forward.x * centerOffset, y: ego.y + forward.y * centerOffset};
    const rect = body.map(toSvg)
      .map((point) => `${point.x.toFixed(3)},${point.y.toFixed(3)}`).join(" ");
    const front = toSvg({x: ego.x + forward.x * rearToFront, y: ego.y + forward.y * rearToFront});
    const centerMark = toSvg(center);
    egoRect = `<polygon class="ego" points="${rect}"><title>车型车身包络${fChamLon > 0 || rChamLon > 0 ? "（含倒角）" : ""}，无附加余量</title></polygon><path class="heading" d="M${centerMark.x.toFixed(1)} ${centerMark.y.toFixed(1)} L${front.x.toFixed(1)} ${front.y.toFixed(1)}"><title>自车车头</title></path>`;
  }
  const startMark = start.x !== null && start.y !== null
    ? `<circle class="start" cx="${toSvg(start).x.toFixed(1)}" cy="${toSvg(start).y.toFixed(1)}" r="2.4"/>` : "";
  const headingMarks = [];
  let previousDirection = 0;
  let cuspCount = 0;
  let lastHeadingSvg = null;
  let lastDirSvg = null;
  const stride = Math.max(1, Math.ceil(algo.length / 10));
  algo.forEach((point, index) => {
    const theta = finiteTelemetryNumber(yaws[index]);
    const odo = finiteTelemetryNumber(odos[index]);
    const direction = odo === null || Math.abs(odo) < 0.0001 ? previousDirection : Math.sign(odo);
    const cusp = previousDirection && direction && direction !== previousDirection;
    const p = toSvg(point);
    if (cusp) {
      cuspCount += 1;
      headingMarks.push(`<circle class="cusp" cx="${p.x.toFixed(1)}" cy="${p.y.toFixed(1)}" r="1.6"><title>换向点 #${cuspCount}（点 ${index} · 切换至 ${direction < 0 ? "R 倒车" : "D 前进"}）</title></circle>`);
    }
    const canDrawHeading = !lastHeadingSvg || Math.hypot(p.x - lastHeadingSvg.x, p.y - lastHeadingSvg.y) >= 10;
    if (theta !== null && canDrawHeading && (index % stride === 0 || cusp || index === algo.length - 1)) {
      const heading = toSvg({x: point.x + Math.cos(theta), y: point.y + Math.sin(theta)});
      const dx = (heading.x - p.x) / scale, dy = (heading.y - p.y) / scale;
      const tip = {x: p.x + dx * 6.5, y: p.y + dy * 6.5};
      const back = {x: tip.x - dx * 2.8, y: tip.y - dy * 2.8};
      headingMarks.push(`<path class="heading" d="M${p.x.toFixed(1)} ${p.y.toFixed(1)} L${tip.x.toFixed(1)} ${tip.y.toFixed(1)} M${(back.x - dy * 2.2).toFixed(1)} ${(back.y + dx * 2.2).toFixed(1)} L${tip.x.toFixed(1)} ${tip.y.toFixed(1)} L${(back.x + dy * 2.2).toFixed(1)} ${(back.y - dx * 2.2).toFixed(1)}"><title>车头朝向 · ${direction < 0 ? "R 倒车" : "D 前进"}</title></path>`);
      lastHeadingSvg = p;
    }
    const canDrawDir = !lastDirSvg || Math.hypot(p.x - lastDirSvg.x, p.y - lastDirSvg.y) >= 12;
    if ((cusp || index === 0) && canDrawDir && direction) {
      headingMarks.push(`<text class="direction" x="${(p.x + 4).toFixed(1)}" y="${(p.y - 5).toFixed(1)}">${direction < 0 ? "R" : "D"}</text>`);
      lastDirSvg = p;
    }
    previousDirection = direction;
  });
  const refX = finiteTelemetryNumber(diagnostic.reference_x);
  const refY = finiteTelemetryNumber(diagnostic.reference_y);
  const reference = diagnostic.reference_valid && refX !== null && refY !== null
    ? (() => { const p = toSvg({x: refX, y: refY}); return `<circle class="reference" cx="${p.x.toFixed(1)}" cy="${p.y.toFixed(1)}" r="2.2"><title>当前预瞄点</title></circle>`; })() : "";
  const goal = input.available && input.goal ? (() => {
    const p = toSvg({x:input.goal[0], y:input.goal[1]});
    return `<circle class="map-goal" cx="${p.x.toFixed(3)}" cy="${p.y.toFixed(3)}" r="1.8"><title>目标后轴中心</title></circle>`;
  })() : "";
  const meters = [1, 2, 5, 10, 20, 50, 100, 200, 500].find((n) => n*scale >= 30) || 500;
  const ruler = input.available || parkingSlots.length ? `<g class="map-scale"><path d="M14 219 V223 H${(14+meters*scale).toFixed(2)} V219"/><text x="14" y="216">${meters} m</text></g>` : "";
  // Keep unchanged map elements alive across telemetry polls so source
  // tooltips remain usable and only the moving path/body layer is rebuilt.
  if (!svg.querySelector(".apa-input-geometry"))
    svg.innerHTML = '<g class="apa-input-geometry"></g><g class="apa-parking-geometry"></g><g class="apa-path-geometry"></g>';
  const mapGroup = svg.querySelector(".apa-input-geometry");
  const mapMarkup = renderApaInputLayers(input,toSvg,line);
  if (mapGroup._markup !== mapMarkup) {
    mapGroup.innerHTML = mapMarkup;
    mapGroup._markup = mapMarkup;
  }
  const parkingGroup = svg.querySelector(".apa-parking-geometry");
  const parkingMarkup = renderApaParkingSlots(parkingSlots, data.physical_slot_id ?? data.slot_id, toSvg, line, scale);
  if (parkingGroup._markup !== parkingMarkup) {
    parkingGroup.innerHTML = parkingMarkup;
    parkingGroup._markup = parkingMarkup;
  }
  svg.querySelector(".apa-path-geometry").innerHTML = [slot, ...previousPlans.map((points) => line(points, "previous-plan")),
    line(actual, "actual"), line(algo, "algo"), headingMarks.join(""), reference, goal, egoRect, startMark, ruler].join("");
}

const APA_REPORT_COLORS = {algo: "#fb7185", actual: "#34d399"};

function recordApaSample(data) {
  if (!data || !data.online || !data.parking_enabled) return;
  const x = finiteTelemetryNumber(data.vehicle_x);
  const y = finiteTelemetryNumber(data.vehicle_y);
  const frame = finiteTelemetryNumber(data.carla_frame_id);
  const simTime = finiteTelemetryNumber(data.sim_time_sec);
  if (x === null || y === null || frame === null || simTime === null) return;
  const last = state.apaSamples[state.apaSamples.length - 1];
  if (last && frame <= last.frame) return;
  if (state.apaSampleT0 === null) state.apaSampleT0 = simTime;
  const algo = zipXy(data.algorithm_path_x, data.algorithm_path_y);
  const actual = zipXy(data.actual_path_x, data.actual_path_y);
  if (algo.length) state.apaAlgoPath = algo;
  if (actual.length) state.apaActualPath = actual;
  const sample = {
    frame, simTime,
    t: simTime - state.apaSampleT0,
    wheelExp: finiteTelemetryNumber(data.expected_steer_wheel_deg),
    wheelAct: finiteTelemetryNumber(data.actual_steer_wheel_deg),
    accelExp: finiteTelemetryNumber(data.expected_accel_mps2),
    accelAct: finiteTelemetryNumber(data.actual_accel_mps2),
    speedExp: finiteTelemetryNumber(data.expected_speed_mps),
    speedAct: finiteTelemetryNumber(data.actual_speed_mps !== undefined ? data.actual_speed_mps : data.ego_speed_mps),
    x,
    y,
  };
  state.apaSamples.push(sample);
}

function reportLayout(groups, width, height, pad, includeZeroY, worldCoordinates = false) {
  const points = groups.flatMap((group) => group.points);
  if (!points.length) return null;
  let minX = Math.min(...points.map((point) => point.x));
  let maxX = Math.max(...points.map((point) => point.x));
  let minY = Math.min(...points.map((point) => point.y));
  let maxY = Math.max(...points.map((point) => point.y));
  if (includeZeroY) {
    minY = Math.min(minY, 0);
    maxY = Math.max(maxY, 0);
  }
  const spanX = Math.max(maxX - minX, 1e-6);
  const spanY = Math.max(maxY - minY, 1e-6);
  const worldScale = Math.min((width - pad * 2) / spanX, (height - pad * 2) / spanY);
  return {
    minX,
    maxX,
    minY,
    maxY,
    to(point) {
      if (worldCoordinates) return {
        sx: width / 2 + (point.x - (minX + maxX) / 2) * worldScale,
        sy: height / 2 + (point.y - (minY + maxY) / 2) * worldScale,
      };
      return {
        sx: pad + ((point.x - minX) / spanX) * (width - pad * 2),
        sy: height - pad - ((point.y - minY) / spanY) * (height - pad * 2),
      };
    },
  };
}

function drawReportChart(svg, groups, options) {
  if (!svg) return;
  const width = options.width;
  const height = options.height;
  svg.setAttribute("viewBox", `0 0 ${width} ${height}`);
  const layout = reportLayout(groups, width, height, 28, Boolean(options.zeroLine), Boolean(options.worldCoordinates));
  if (!layout) {
    svg.innerHTML = `<text x="${width / 2}" y="${height / 2}" text-anchor="middle">暂无数据</text>`;
    svg._hits = [];
    return;
  }
  const parts = [`<rect width="${width}" height="${height}" fill="#0f172a"/>`];
  if (options.worldCoordinates) parts.push('<text x="8" y="15" fill="#cbd5e1" font-size="11">CARLA · +X → · +Y ↓ · 等比例</text>');
  const hits = [];
  if (options.zeroLine && layout.minY <= 0 && layout.maxY >= 0) {
    const left = layout.to({x: layout.minX, y: 0});
    const right = layout.to({x: layout.maxX, y: 0});
    parts.push(`<line stroke="#94a3b8" stroke-dasharray="4 4" x1="${left.sx.toFixed(1)}" y1="${left.sy.toFixed(1)}" x2="${right.sx.toFixed(1)}" y2="${right.sy.toFixed(1)}"/>`);
    parts.push(`<text fill="#e2e8f0" font-size="11" x="8" y="${Math.max(14, left.sy - 4).toFixed(1)}">0</text>`);
  }
  groups.forEach((group) => {
    if (group.points.length < 2) return;
    const mapped = group.points.map((point) => Object.assign({}, point, layout.to(point)));
    const d = mapped.map((point, index) => `${index ? "L" : "M"}${point.sx.toFixed(1)} ${point.sy.toFixed(1)}`).join(" ");
    parts.push(`<path fill="none" stroke="${group.color}" stroke-width="2.2" d="${d}"/>`);
    hits.push({name: group.name, samples: mapped});
    if (options.markStart) {
      const start = mapped[0];
      parts.push(`<circle cx="${start.sx.toFixed(1)}" cy="${start.sy.toFixed(1)}" r="4" fill="#f8fafc"/>`);
      parts.push(`<text fill="#e2e8f0" font-size="11" x="${(start.sx + 6).toFixed(1)}" y="${(start.sy - 6).toFixed(1)}">0</text>`);
    }
  });
  parts.push(`<circle class="pick" r="5" cx="-20" cy="-20"/>`);
  svg.innerHTML = parts.join("");
  svg._hits = hits;
  svg._pinned = false;
}

function nearestReportSegment(hits, sx, sy) {
  let best = null;
  (hits || []).forEach((series) => {
    for (let index = 1; index < series.samples.length; index += 1) {
      const start = series.samples[index - 1];
      const end = series.samples[index];
      const dx = end.sx - start.sx;
      const dy = end.sy - start.sy;
      const length2 = dx * dx + dy * dy || 1;
      const ratio = Math.max(0, Math.min(1, ((sx - start.sx) * dx + (sy - start.sy) * dy) / length2));
      const px = start.sx + dx * ratio;
      const py = start.sy + dy * ratio;
      const dist = Math.hypot(sx - px, sy - py);
      if (!best || dist < best.dist) {
        best = {
          dist,
          sx: px,
          sy: py,
          x: start.worldX + (end.worldX - start.worldX) * ratio,
          y: start.worldY + (end.worldY - start.worldY) * ratio,
          name: series.name,
        };
      }
    }
  });
  return best && best.dist <= 18 ? best : null;
}

function reportSvgPoint(svg, event) {
  const point = svg.createSVGPoint();
  point.x = event.clientX;
  point.y = event.clientY;
  const matrix = svg.getScreenCTM();
  if (!matrix) return {sx: 0, sy: 0};
  const local = point.matrixTransform(matrix.inverse());
  return {sx: local.x, sy: local.y};
}

function showReportHit(svg, hit) {
  const mark = svg.querySelector(".pick");
  const readout = $("apaReportReadout");
  if (!hit) {
    if (mark) {
      mark.setAttribute("cx", "-20");
      mark.setAttribute("cy", "-20");
    }
    return;
  }
  if (mark) {
    mark.setAttribute("cx", hit.sx);
    mark.setAttribute("cy", hit.sy);
  }
  if (readout) readout.textContent = `${hit.name}  x ${hit.x.toFixed(2)}  y ${hit.y.toFixed(2)}`;
}

function bindReportSvg(svg) {
  if (!svg || svg.dataset.bound === "1") return;
  svg.dataset.bound = "1";
  svg.addEventListener("pointermove", (event) => {
    if (svg._pinned) return;
    const local = reportSvgPoint(svg, event);
    showReportHit(svg, nearestReportSegment(svg._hits, local.sx, local.sy));
  });
  svg.addEventListener("click", (event) => {
    const local = reportSvgPoint(svg, event);
    const hit = nearestReportSegment(svg._hits, local.sx, local.sy);
    svg._pinned = Boolean(hit);
    showReportHit(svg, hit);
  });
  svg.addEventListener("pointerleave", () => {
    if (!svg._pinned) showReportHit(svg, null);
  });
}

function reportCell(value, digits) {
  return value === null || value === undefined || Number.isNaN(value) ? "--" : Number(value).toFixed(digits);
}

function renderApaReportTable(samples) {
  const table = $("apaReportTable");
  if (!table) return;
  const firstIndex = Math.max(0, samples.length - 720);
  const rows = samples.slice(firstIndex).map((sample, offset) => `
    <tr data-index="${firstIndex + offset}">
      <td>${firstIndex + offset}</td>
      <td>${sample.t.toFixed(1)}</td>
      <td>${reportCell(sample.wheelExp, 1)}</td>
      <td>${reportCell(sample.wheelAct, 1)}</td>
      <td>${reportCell(sample.accelExp, 2)}</td>
      <td>${reportCell(sample.accelAct, 2)}</td>
      <td>${reportCell(sample.speedExp, 2)}</td>
      <td>${reportCell(sample.speedAct, 2)}</td>
      <td>${sample.x.toFixed(2)}</td>
      <td>${sample.y.toFixed(2)}</td>
    </tr>`).join("");
  table.innerHTML = `
    <caption>显示第 ${firstIndex + 1}–${samples.length} 条，共 ${samples.length} 条；导出包含全部已采集样本。</caption>
    <thead><tr><th>采样序号</th><th>仿真时间 s</th><th>期望转向 °</th><th>施加转向 °</th><th>期望加速度</th><th>实际加速度</th><th>期望速度 m/s</th><th>实际速度 m/s</th><th>X</th><th>Y</th></tr></thead>
    <tbody>${rows || `<tr><td colspan="10">暂无采样，请先运行泊车。</td></tr>`}</tbody>`;
  table.querySelectorAll("tbody tr[data-index]").forEach((row) => {
    row.addEventListener("click", () => {
      table.querySelectorAll("tr.is-selected").forEach((item) => item.classList.remove("is-selected"));
      row.classList.add("is-selected");
      const sample = samples[Number(row.dataset.index)];
      const path = $("apaReportPath");
      if (!path || !sample) return;
      let point = null;
      let best = Infinity;
      (path._hits || []).forEach((series) => {
        if (series.name !== "实际轨迹") return;
        series.samples.forEach((item) => {
          const dist = Math.hypot(item.worldX - sample.x, item.worldY - sample.y);
          if (dist < best) {
            best = dist;
            point = item;
          }
        });
      });
      path._pinned = true;
      showReportHit(path, {
        sx: point ? point.sx : 28,
        sy: point ? point.sy : 28,
        x: sample.x,
        y: sample.y,
        name: `采样 ${row.dataset.index}`,
      });
    });
  });
}

function smoothReportSeries(points, tauSec) {
  let filtered = null;
  let previousTime = null;
  return points.map((point) => {
    if (filtered === null || previousTime === null) {
      filtered = point.y;
      previousTime = point.x;
      return point;
    }
    const dt = Math.max(point.x - previousTime, 1.0e-3);
    const alpha = dt / (tauSec + dt);
    filtered = filtered + alpha * (point.y - filtered);
    previousTime = point.x;
    return Object.assign({}, point, {y: filtered});
  });
}

function downloadApaReport() {
  const samples = state.apaReportSnapshot.samples;
  const header = ["序号", "相对仿真时间s", "期望转向deg", "施加转向deg", "期望加速度", "实际加速度", "期望速度mps", "实际速度mps", "X", "Y", "CARLA帧", "仿真时间s"];
  const lines = [header.join(",")];
  samples.forEach((sample, index) => {
    const cell = (value, digits) => (value === null || value === undefined || Number.isNaN(value) ? "" : Number(value).toFixed(digits));
    lines.push([
      index,
      sample.t.toFixed(2),
      cell(sample.wheelExp, 2),
      cell(sample.wheelAct, 2),
      cell(sample.accelExp, 3),
      cell(sample.accelAct, 3),
      cell(sample.speedExp, 3),
      cell(sample.speedAct, 3),
      sample.x.toFixed(3),
      sample.y.toFixed(3),
      sample.frame,
      sample.simTime.toFixed(3),
    ].join(","));
  });
  const blob = new Blob(["\uFEFF" + lines.join("\n")], {type: "text/csv;charset=utf-8"});
  const link = document.createElement("a");
  link.href = URL.createObjectURL(blob);
  link.download = "apa-simulation-report.csv";
  link.click();
  URL.revokeObjectURL(link.href);
}

function svgToJpegBytes(svg, width, height) {
  return new Promise((resolve, reject) => {
    if (!svg) {
      resolve(null);
      return;
    }
    const clone = svg.cloneNode(true);
    clone.setAttribute("xmlns", "http://www.w3.org/2000/svg");
    clone.setAttribute("width", String(width));
    clone.setAttribute("height", String(height));
    const xml = new XMLSerializer().serializeToString(clone);
    const image = new Image();
    const url = URL.createObjectURL(new Blob([xml], {type: "image/svg+xml;charset=utf-8"}));
    image.onload = () => {
      const canvas = document.createElement("canvas");
      canvas.width = width;
      canvas.height = height;
      const ctx = canvas.getContext("2d");
      ctx.fillStyle = "#0f172a";
      ctx.fillRect(0, 0, width, height);
      ctx.drawImage(image, 0, 0, width, height);
      URL.revokeObjectURL(url);
      const data = canvas.toDataURL("image/jpeg", 0.88).split(",")[1];
      resolve(Uint8Array.from(atob(data), (char) => char.charCodeAt(0)));
    };
    image.onerror = () => {
      URL.revokeObjectURL(url);
      reject(new Error("chart image failed"));
    };
    image.src = url;
  });
}

function jpegBytesToImage(bytes) {
  return new Promise((resolve, reject) => {
    const blob = new Blob([bytes], {type: "image/jpeg"});
    const url = URL.createObjectURL(blob);
    const image = new Image();
    image.onload = () => {
      URL.revokeObjectURL(url);
      resolve(image);
    };
    image.onerror = () => {
      URL.revokeObjectURL(url);
      reject(new Error("jpeg decode failed"));
    };
    image.src = url;
  });
}

function canvasToJpegBytes(canvas) {
  const data = canvas.toDataURL("image/jpeg", 0.88).split(",")[1];
  return Uint8Array.from(atob(data), (char) => char.charCodeAt(0));
}

function concatBytes(parts) {
  const size = parts.reduce((total, part) => total + part.length, 0);
  const out = new Uint8Array(size);
  let offset = 0;
  parts.forEach((part) => {
    out.set(part, offset);
    offset += part.length;
  });
  return out;
}

function jpegPagesToPdf(jpegs, imgW, imgH) {
  const enc = (text) => new TextEncoder().encode(text);
  const pageW = 595;
  const pageH = Math.round(pageW * imgH / imgW);
  const blobs = [enc("%PDF-1.4\n")];
  const offsets = [];
  let pos = blobs[0].length;
  const add = (id, content) => {
    const head = enc(`${id} 0 obj\n`);
    const tail = enc("\nendobj\n");
    offsets[id] = pos;
    blobs.push(head, content, tail);
    pos += head.length + content.length + tail.length;
  };
  const pageIds = jpegs.map((_, index) => 5 + index * 3);
  add(1, enc(`<< /Type /Catalog /Pages 2 0 R >>`));
  add(2, enc(`<< /Type /Pages /Kids [${pageIds.map((id) => `${id} 0 R`).join(" ")}] /Count ${jpegs.length} >>`));
  jpegs.forEach((jpeg, index) => {
    const imageId = 3 + index * 3;
    const contentId = imageId + 1;
    const pageId = imageId + 2;
    const dict = enc(`<< /Type /XObject /Subtype /Image /Width ${imgW} /Height ${imgH} /ColorSpace /DeviceRGB /BitsPerComponent 8 /Filter /DCTDecode /Length ${jpeg.length} >>\nstream\n`);
    add(imageId, concatBytes([dict, jpeg, enc("\nendstream")]));
    const commands = `q ${pageW} 0 0 ${pageH} 0 0 cm /Im${index} Do Q`;
    add(contentId, enc(`<< /Length ${commands.length} >>\nstream\n${commands}\nendstream`));
    add(pageId, enc(`<< /Type /Page /Parent 2 0 R /MediaBox [0 0 ${pageW} ${pageH}] /Contents ${contentId} 0 R /Resources << /XObject << /Im${index} ${imageId} 0 R >> >> >>`));
  });
  const size = 3 + jpegs.length * 3;
  let xref = `xref\n0 ${size}\n0000000000 65535 f \n`;
  for (let id = 1; id < size; id += 1) xref += `${String(offsets[id]).padStart(10, "0")} 00000 n \n`;
  xref += `trailer << /Size ${size} /Root 1 0 R >>\nstartxref\n${pos}\n%%EOF`;
  blobs.push(enc(xref));
  return concatBytes(blobs);
}

async function downloadApaReportPdf() {
  if (!$("apaReport") || $("apaReport").hidden) generateApaReport();
  const samples = state.apaReportSnapshot.samples;
  const charts = [
    ["转向命令", $("apaReportWheel"), 640, 220],
    ["加速度（实际曲线经 0.2 s 平滑，表格为原始反馈）", $("apaReportAccel"), 640, 220],
    ["速度", $("apaReportSpeed"), 640, 220],
    ["轨迹  玫红=算法  绿色=实际  白点=各曲线首点", $("apaReportPath"), 640, 320],
  ];
  // Clone every chart before awaiting image encoding; polling may start a new task.
  const chartImages = await Promise.all(charts.map(async ([title, svg, width, height]) => ({
    title, image: await jpegBytesToImage(await svgToJpegBytes(svg, width, height)), width, height,
  })));
  const pageW = 1000;
  const pageH = 1414;
  const pages = [];
  let canvas = document.createElement("canvas");
  canvas.width = pageW;
  canvas.height = pageH;
  let ctx = canvas.getContext("2d");
  let y = 56;
  const paintPage = () => {
    ctx.fillStyle = "#ffffff";
    ctx.fillRect(0, 0, pageW, pageH);
    ctx.fillStyle = "#0f172a";
  };
  const nextPage = () => {
    pages.push(canvas);
    canvas = document.createElement("canvas");
    canvas.width = pageW;
    canvas.height = pageH;
    ctx = canvas.getContext("2d");
    y = 56;
    paintPage();
  };
  paintPage();
  ctx.font = "32px sans-serif";
  ctx.fillText("泊车报告", 48, y);
  y += 28;
  ctx.font = "16px sans-serif";
  ctx.fillText(`采样 ${samples.length} 点`, 48, y);
  y += 28;
  chartImages.forEach((chart) => {
    const drawW = 904;
    const drawH = Math.round(drawW * chart.height / chart.width);
    if (y + drawH + 36 > pageH - 40) nextPage();
    ctx.font = "20px sans-serif";
    ctx.fillStyle = "#0f172a";
    ctx.fillText(chart.title, 48, y);
    y += 12;
    ctx.drawImage(chart.image, 48, y, drawW, drawH);
    y += drawH + 28;
  });
  const header = ["序号", "仿真时间s", "期望转向°", "施加转向°", "期望加速度", "实际加速度", "期望速度m/s", "实际速度m/s", "X", "Y"];
  const rows = [header].concat(samples.map((sample, index) => [
    String(index),
    sample.t.toFixed(1),
    reportCell(sample.wheelExp, 1),
    reportCell(sample.wheelAct, 1),
    reportCell(sample.accelExp, 2),
    reportCell(sample.accelAct, 2),
    reportCell(sample.speedExp, 2),
    reportCell(sample.speedAct, 2),
    sample.x.toFixed(2),
    sample.y.toFixed(2),
  ]));
  rows.forEach((row, index) => {
    if (y + 22 > pageH - 40) nextPage();
    ctx.font = index === 0 ? "bold 13px sans-serif" : "13px sans-serif";
    ctx.fillStyle = "#0f172a";
    row.forEach((cell, column) => ctx.fillText(cell, 36 + column * 96, y));
    y += 20;
  });
  pages.push(canvas);
  const pdf = jpegPagesToPdf(pages.map((page) => canvasToJpegBytes(page)), pageW, pageH);
  const link = document.createElement("a");
  link.href = URL.createObjectURL(new Blob([pdf], {type: "application/pdf"}));
  link.download = "apa-simulation-report.pdf";
  link.click();
  URL.revokeObjectURL(link.href);
}

function generateApaReport() {
  const root = $("apaReport");
  if (!root || !state.apaSamples.length) return;
  const samples = state.apaSamples.map((sample) => ({...sample}));
  const algo = state.apaAlgoPath.map((point) => ({x: point.x, y: point.y, worldX: point.x, worldY: point.y}));
  const actualPath = (state.apaActualPath.length ? state.apaActualPath : samples.map((sample) => ({x: sample.x, y: sample.y})))
    .map((point) => ({x: point.x, y: point.y, worldX: point.x, worldY: point.y}));
  state.apaReportSnapshot = {samples};
  const wheel = (key) => samples.filter((sample) => sample[key] !== null).map((sample) => ({
    x: sample.t, y: sample[key], worldX: sample.x, worldY: sample.y,
  }));
  const accel = (key) => samples.filter((sample) => sample[key] !== null).map((sample) => ({
    x: sample.t, y: sample[key], worldX: sample.x, worldY: sample.y,
  }));
  const speed = (key) => samples.filter((sample) => sample[key] !== null && sample[key] !== undefined).map((sample) => ({
    x: sample.t, y: sample[key], worldX: sample.x, worldY: sample.y,
  }));
  drawReportChart($("apaReportWheel"), [
    {name: "期望转向命令", color: APA_REPORT_COLORS.algo, points: wheel("wheelExp")},
    {name: "已施加转向命令", color: APA_REPORT_COLORS.actual, points: wheel("wheelAct")},
  ], {width: 640, height: 220, zeroLine: true, markStart: true});
  drawReportChart($("apaReportAccel"), [
    {name: "期望加速度", color: APA_REPORT_COLORS.algo, points: accel("accelExp")},
    {name: "实际加速度（平滑）", color: APA_REPORT_COLORS.actual, points: smoothReportSeries(accel("accelAct"), 0.2)},
  ], {width: 640, height: 220, zeroLine: true, markStart: true});
  drawReportChart($("apaReportSpeed"), [
    {name: "期望速度", color: APA_REPORT_COLORS.algo, points: speed("speedExp")},
    {name: "实际速度", color: APA_REPORT_COLORS.actual, points: speed("speedAct")},
  ], {width: 640, height: 220, zeroLine: true, markStart: true});
  drawReportChart($("apaReportPath"), [
    {name: "算法轨迹", color: APA_REPORT_COLORS.algo, points: algo},
    {name: "实际轨迹", color: APA_REPORT_COLORS.actual, points: actualPath},
  ], {width: 640, height: 320, zeroLine: false, markStart: true, worldCoordinates: true});
  ["apaReportWheel", "apaReportAccel", "apaReportSpeed", "apaReportPath"].forEach((id) => bindReportSvg($(id)));
  renderApaReportTable(samples);
  const readout = $("apaReportReadout");
  if (readout) readout.textContent = `本页采集 ${samples.length} 点 · 相对仿真时间 ${samples[0].t.toFixed(1)}–${samples[samples.length - 1].t.toFixed(1)} s · 白点为各曲线首点`;
  releaseApaInput();
  root.hidden = false;
  root._previousFocus = document.activeElement;
  root._previousOverflow = document.body.style.overflow;
  document.body.style.overflow = "hidden";
  document.querySelector(".dashboard").inert = true;
  $("apaReportClose").focus();
  refreshApaConsole();
}

function closeApaReport() {
  const root = $("apaReport");
  if (!root || root.hidden) return;
  root.hidden = true;
  state.apaReportSnapshot = null;
  document.body.style.overflow = root._previousOverflow || "";
  document.querySelector(".dashboard").inert = false;
  root._previousFocus?.focus();
  refreshApaConsole();
  startApaVehicleHeartbeat();
}

function renderApaTelemetry(data = {}) {
  const previous = state.apaTelemetry;
  const stale = (data.status_received_at && previous.status_received_at &&
      data.status_received_at < previous.status_received_at) ||
    (data.session_id && data.session_id === previous.session_id && data.task_id < previous.task_id);
  let reconciledFeedback = null;
  const receipt = apaCommandReceipt(data);
  const confirmed = receipt && !["running", "loading"].includes(receipt.state) ? receipt : null;
  if (confirmed) reconciledFeedback = finishApaCommand(confirmed);
  if (stale) {
    // An exact terminal receipt may arrive with an older telemetry snapshot.
    // Finish that event without replacing newer vehicle or scene state.
    if (reconciledFeedback) {
      renderApaScene(state.apaTelemetry);
      apaFeedback(...reconciledFeedback);
      updateActionState(state.scenarioBusy ? "running" : "idle");
    }
    return;
  }
  renderApaCameraStatus(data.camera_streams);
  if ((state.apaTaskBusy || state.apaTaskUnconfirmed) && receipt && !confirmed &&
      receipt.message !== previous.last_task_command?.message) {
    reconciledFeedback = [receipt.message || "操作正在执行…", ""];
  }
  if (data.session_id && data.session_id !== previous.session_id &&
      (state.apaTaskBusy || state.apaTaskUnconfirmed) && !receipt &&
      !(state.apaPendingMap && data.map_name === state.apaPendingMap)) {
    reconciledFeedback = finishApaCommand({ok: false, message: "环境已重启，上次操作结果未确认，请核对当前场景。"});
  }
  // Retain the last identity for an explicit cancel even if status is stale.
  // The visible metrics still render the established offline state.
  state.apaTelemetry = data.online ? data : {...previous, ...data, online: false};
  if (data.session_id && (data.session_id !== previous.session_id || data.camera_revision !== previous.camera_revision)) startApaCamera();
  if (!data.online) data = {online: false};
  const historyKey = data.session_id ? `${data.session_id}:${data.history_revision}` : "";
  if (historyKey && historyKey !== state.apaHistoryKey) {
    clearApaHistory();
    state.apaHistoryKey = historyKey;
  }
  if (data.online) {
    const vehicleKey = `${data.session_id}:${data.task_id}`;
    if (vehicleKey !== state.apaVehicleKey) {
      state.apaHeld.clear();
      state.apaVehicle.gear = data.gear || "P";
      state.apaVehicle.handBrake = Boolean(data.hand_brake);
      state.apaVehicle.ignition = data.ignition_switch || "ON";
      state.apaVehicleKey = vehicleKey;
    }
  }
  const speed = finiteTelemetryNumber(data.ego_speed_mps);
  if ($("apaEgoSpeed")) $("apaEgoSpeed").textContent = speed === null ? "--" : (speed * 3.6).toFixed(1);
  if ($("apaEgoSpeedMps")) $("apaEgoSpeedMps").textContent = speed === null ? "-- m/s" : `${speed.toFixed(2)} m/s`;
  if ($("apaGear")) $("apaGear").textContent = data.gear || "--";
  if ($("apaSlotId")) $("apaSlotId").textContent = data.physical_slot_id ?? data.slot_id ?? "--";
  if ($("apaSlotKind")) $("apaSlotKind").textContent = apaSlotKindLabel(data);
  if ($("apaIgnition")) $("apaIgnition").textContent = data.ignition_switch || "--";
  if ($("apaHandBrake")) $("apaHandBrake").textContent = typeof data.hand_brake === "boolean" ? (data.hand_brake ? "拉起" : "松开") : "--";
  const view = apaUiState();
  if (selectedScenario()?.control_profile === "apa") updateHealthChip("carla", view.ready, "");
  if ($("apaControlState")) $("apaControlState").textContent = data.online ? view.mode : "--";
  if ($("apaControlSub")) {
    const phases = {2: "档位确认", 3: "路径跟踪", 4: "到点停车", 6: "泊车完成", 7: "减速接近", 8: "低速接近", 9: "保护停车"};
    const replanInfo = apaReplanInfo(data);
    const phaseLabel = replanInfo.activePlanning
      ? (data.control_status === "replan_stopping" ? "停车重规划"
        : data.control_status === "replan_waiting" ? "等待场景变化"
        : replanInfo.isReplan ? "正在重规划" : "正在规划")
      : (phases[data.control_phase] || view.label);
    $("apaControlSub").textContent = !data.online ? "--"
      : view.parking ? `${phaseLabel}${replanInfo.stageLabel && !phaseLabel.includes(replanInfo.stageLabel) ? ` · ${replanInfo.stageLabel}` : ""}`
      : view.label;
  }
  const formatDelta = (actualVal, expectedVal, digits = 2) => {
    const a = finiteTelemetryNumber(actualVal);
    const e = finiteTelemetryNumber(expectedVal);
    if (a === null || e === null) return "--";
    const diff = a - e;
    const sign = diff > 1e-6 ? "+" : "";
    return `${sign}${diff.toFixed(digits)}`;
  };
  const expGear = data.expected_gear || "";
  const actGear = data.applied_gear || data.gear || "";
  if ($("apaExpectedGear")) $("apaExpectedGear").textContent = expGear || "--";
  if ($("apaActualGear")) $("apaActualGear").textContent = actGear || "--";
  if ($("apaDiffGear")) $("apaDiffGear").textContent = !expGear || !actGear ? "--" : expGear === actGear ? "一致" : `${expGear}→${actGear}`;
  if ($("apaExpectedSpeed")) $("apaExpectedSpeed").textContent = formatMetric(data.expected_speed_mps, 2, "");
  if ($("apaActualSpeed")) $("apaActualSpeed").textContent = formatMetric(data.actual_speed_mps, 2, "");
  if ($("apaDiffSpeed")) $("apaDiffSpeed").textContent = formatDelta(data.actual_speed_mps, data.expected_speed_mps, 2);
  if ($("apaExpectedAccel")) $("apaExpectedAccel").textContent = formatMetric(data.expected_accel_mps2, 2, "");
  if ($("apaActualAccel")) $("apaActualAccel").textContent = formatMetric(data.actual_accel_mps2, 2, "");
  if ($("apaDiffAccel")) $("apaDiffAccel").textContent = formatDelta(data.actual_accel_mps2, data.expected_accel_mps2, 2);
  if ($("apaExpectedBrake")) $("apaExpectedBrake").textContent = formatMetric(data.expected_brake, 2, "");
  if ($("apaActualBrake")) $("apaActualBrake").textContent = formatMetric(data.brake, 2, "");
  if ($("apaDiffBrake")) $("apaDiffBrake").textContent = formatDelta(data.brake, data.expected_brake, 2);
  if ($("apaExpectedThrottle")) $("apaExpectedThrottle").textContent = formatMetric(data.expected_throttle, 2, "");
  if ($("apaActualThrottle")) $("apaActualThrottle").textContent = formatMetric(data.throttle, 2, "");
  if ($("apaDiffThrottle")) $("apaDiffThrottle").textContent = formatDelta(data.throttle, data.expected_throttle, 2);
  if ($("apaExpectedWheel")) $("apaExpectedWheel").textContent = formatMetric(data.expected_steer_wheel_deg, 1, "");
  if ($("apaActualWheel")) $("apaActualWheel").textContent = formatMetric(data.actual_steer_wheel_deg, 1, "");
  if ($("apaDiffWheel")) $("apaDiffWheel").textContent = formatDelta(data.actual_steer_wheel_deg, data.expected_steer_wheel_deg, 1);
  const expectedX = finiteTelemetryNumber(data.expected_x);
  const expectedY = finiteTelemetryNumber(data.expected_y);
  if ($("apaExpectedPose")) {
    $("apaExpectedPose").textContent = expectedX === null || expectedY === null ? "--" : `(${expectedX.toFixed(2)}, ${expectedY.toFixed(2)})`;
  }
  const vx = finiteTelemetryNumber(data.vehicle_x);
  const vy = finiteTelemetryNumber(data.vehicle_y);
  if ($("apaActualPose")) $("apaActualPose").textContent = vx === null || vy === null ? "--" : `(${vx.toFixed(2)}, ${vy.toFixed(2)})`;
  if ($("apaDiffPose")) {
    $("apaDiffPose").textContent = expectedX === null || expectedY === null || vx === null || vy === null
      ? "--" : Math.hypot(vx - expectedX, vy - expectedY).toFixed(2);
  }
  const status = $("apaTelemetryStatus");
  if (status) {
    const feedbackOnline = Boolean(data.online && data.vehicle_feedback_available !== false);
    status.classList.toggle("is-live", feedbackOnline);
    status.classList.toggle("is-waiting", !feedbackOnline);
  }
  if ($("apaTelemetryStatusText")) {
    const age = finiteTelemetryNumber(data.age_sec);
    $("apaTelemetryStatusText").textContent = !data.online ? "反馈离线"
      : data.vehicle_feedback_available === false ? "反馈暂停" : "反馈在线";
    $("apaTelemetryStatusText").title = data.online && age !== null
      ? `状态更新于 ${Math.round(age * 1000)} ms 前` : "";
  }
  refreshApaParkingMap(data);
  renderApaTrajectory(data);
  if (data.vehicle_feedback_available !== false) recordApaSample(data);
  renderApaScene(data);
  if (reconciledFeedback) apaFeedback(...reconciledFeedback);
  if (apaManualLocked()) {
    state.apaHeld.clear();
    state.apaVehiclePending = false;
  }
  updateActionState(state.scenarioBusy ? "running" : "idle");
}

async function pollApaTelemetry() {
  const item = selectedScenario();
  if (!item || item.control_profile !== "apa" || state.apaTelemetryBusy) return;
  const id = item.id;
  const revision = state.apaPollRevision;
  state.apaTelemetryBusy = true;
  try {
    const data = await requestJson(`/api/scenarios/${id}/telemetry`, {signal: AbortSignal.timeout(5000)});
    if (state.selectedId === id && revision === state.apaPollRevision) renderApaTelemetry(data);
  } catch {
    if (state.selectedId === id && revision === state.apaPollRevision) renderApaTelemetry({online: false});
  } finally {
    state.apaTelemetryBusy = false;
  }
}

function startApaCamera() {
  const id = state.selectedId;
  if (!id) return;
  APA_CAMERA_VIEWS.forEach((camera) => {
    const image = $(camera.image);
    if (!image) return;
    const session = state.apaTelemetry.session_id || "";
    const revision = state.apaTelemetry.camera_revision ?? 0;
    const next = `/api/scenarios/${id}/${camera.view}-camera?snapshot=1&session_id=${encodeURIComponent(session)}&revision=${encodeURIComponent(revision)}`;
    if (image.dataset.stream === next) return;
    stopApaCameraView(camera);
    const request = {timer: null, controller: null, objectUrl: "", lastFrameAt: 0, hasFrame: false};
    apaCameraRequests.set(camera.view, request);
    delete image.dataset.streamError;
    $(camera.status).textContent = "等待影像";
    image.dataset.stream = next;
    const refresh = async () => {
      if (apaCameraRequests.get(camera.view) !== request) return;
      const controller = new AbortController();
      request.controller = controller;
      const timeout = setTimeout(() => controller.abort(), 2000);
      let objectUrl = "";
      try {
        const response = await fetch(next, {cache: "no-store", signal: controller.signal});
        if (!response.ok || response.headers.get("Content-Type")?.split(";")[0] !== "image/jpeg") {
          throw new Error("camera frame unavailable");
        }
        objectUrl = URL.createObjectURL(await response.blob());
        const decoded = new Image();
        decoded.src = objectUrl;
        await decoded.decode();
        if (apaCameraRequests.get(camera.view) !== request) return;
        const oldUrl = request.objectUrl;
        request.objectUrl = objectUrl;
        request.hasFrame = true;
        image.src = objectUrl;
        objectUrl = "";
        if (oldUrl) URL.revokeObjectURL(oldUrl);
        if (response.headers.get("X-Camera-State") === "live") request.lastFrameAt = performance.now();
        delete image.dataset.streamError;
      } catch {
        if (apaCameraRequests.get(camera.view) === request) image.dataset.streamError = "1";
      } finally {
        clearTimeout(timeout);
        if (objectUrl) URL.revokeObjectURL(objectUrl);
        if (apaCameraRequests.get(camera.view) === request) {
          renderApaCameraStatus({});
          request.timer = setTimeout(refresh, image.dataset.streamError ? 800 : 125);
        }
      }
    };
    refresh();
  });
}

function stopApaCameraView(camera) {
  const request = apaCameraRequests.get(camera.view);
  apaCameraRequests.delete(camera.view);
  if (request) {
    clearTimeout(request.timer);
    request.controller?.abort();
    if (request.objectUrl) URL.revokeObjectURL(request.objectUrl);
  }
  const image = $(camera.image);
  if (!image) return;
  image.removeAttribute("src");
  delete image.dataset.stream;
  delete image.dataset.streamError;
  $(camera.status).textContent = "等待影像";
}

function stopApaCamera() {
  APA_CAMERA_VIEWS.forEach(stopApaCameraView);
}

function ensureApaReportShell() {
  const targetSelect = $("apaTargetSlot");
  const occSummary = $("apaOccupancySummary");
  const targetLabel = targetSelect?.closest(".apa-field");
  if (targetLabel && occSummary && !targetLabel.querySelector(".apa-field-head")) {
    const headSpan = document.createElement("span");
    headSpan.className = "apa-field-head";
    const titleSpan = targetLabel.querySelector("span") || document.createElement("span");
    titleSpan.textContent = "目标车位";
    occSummary.className = "apa-field-meta";
    headSpan.appendChild(titleSpan);
    headSpan.appendChild(occSummary);
    targetLabel.insertBefore(headSpan, targetSelect);
  }
  const slotIdEl = $("apaSlotId");
  const slotKindEl = $("apaSlotKind");
  const slotRow = slotIdEl?.closest(".telemetry-value-row");
  if (slotRow && slotKindEl && slotKindEl.parentElement !== slotRow) {
    slotKindEl.classList.add("apa-slot-kind-tag");
    slotRow.appendChild(slotKindEl);
  }
  const trajHead = document.querySelector(".apa-traj-head");
  if (trajHead) {
    const h3 = trajHead.querySelector("h3");
    if (h3) h3.textContent = "路径规划与跟踪";
    const meta = $("apaTrajMeta");
    const toolbar = document.querySelector(".apa-map-toolbar");
    if (h3 && meta && !trajHead.querySelector(".apa-traj-title-group")) {
      const group = document.createElement("div");
      group.className = "apa-traj-title-group";
      group.appendChild(h3);
      group.appendChild(meta);
      trajHead.insertBefore(group, trajHead.firstChild);
    }
    if (toolbar && toolbar.parentElement !== trajHead) {
      trajHead.appendChild(toolbar);
    }
  }
  const viewNames = {nearby: "局部视图", full: "规划范围", slots: "全场车位"};
  document.querySelectorAll("[data-apa-map-view]").forEach((btn) => {
    const label = viewNames[btn.dataset.apaMapView];
    if (label) btn.textContent = label;
  });
  document.querySelectorAll(".apa-traj-legend .parking-slots, .apa-camera-label").forEach((el) => el.remove());
  const head = document.querySelector(".apa-traj-head");
  if (head && !$("apaReportBtn")) {
    const button = document.createElement("button");
    button.type = "button";
    button.className = "clear-btn";
    button.id = "apaReportBtn";
    button.textContent = "查看报告";
    const meta = $("apaTrajMeta");
    head.insertBefore(button, meta || null);
  }
  if ($("apaReport")) return;
  const wrap = document.createElement("div");
  wrap.className = "apa-report";
  wrap.id = "apaReport";
  wrap.hidden = true;
  wrap.innerHTML = `
    <div class="apa-report-card" role="dialog" aria-modal="true" aria-labelledby="apaReportTitle">
      <div class="apa-report-head">
        <div>
          <h2 id="apaReportTitle">泊车报告</h2>
          <p id="apaReportReadout">悬停查看坐标，点击固定；白点为各曲线首点。</p>
        </div>
        <div class="apa-report-actions">
          <button type="button" class="clear-btn" id="apaReportDownload">下载 CSV</button>
          <button type="button" class="clear-btn" id="apaReportPdf">下载 PDF</button>
          <button type="button" class="clear-btn" id="apaReportClose">关闭</button>
        </div>
      </div>
      <div class="apa-report-charts">
        <section><h3>转向命令</h3><svg id="apaReportWheel" class="apa-report-svg" viewBox="0 0 640 220"></svg></section>
        <section><h3>加速度（实际曲线经 0.2 s 平滑，表格为原始反馈）</h3><svg id="apaReportAccel" class="apa-report-svg" viewBox="0 0 640 220"></svg></section>
        <section><h3>速度</h3><svg id="apaReportSpeed" class="apa-report-svg" viewBox="0 0 640 220"></svg></section>
        <section class="apa-report-path"><h3>轨迹</h3><svg id="apaReportPath" class="apa-report-svg" viewBox="0 0 640 320"></svg>
          <div class="apa-report-legend"><span class="algo">算法轨迹</span><span class="actual">实际轨迹</span><span class="zero">各曲线首点</span></div>
        </section>
      </div>
      <div class="apa-report-table-wrap"><table class="apa-report-table" id="apaReportTable" aria-label="仿真采样表"></table></div>
    </div>`;
  document.body.appendChild(wrap);
}

function bindEvents() {
  document.querySelectorAll("[data-apa-map-view]").forEach((button) => {
    button.addEventListener("click", () => {
      state.apaMapView = button.dataset.apaMapView;
      renderApaTrajectory(state.apaTelemetry);
    });
  });
  document.querySelectorAll("[data-apa-map-layer]").forEach((button) => {
    button.addEventListener("click", () => {
      const layer = button.dataset.apaMapLayer;
      state.apaMapLayers[layer] = !state.apaMapLayers[layer];
      renderApaTrajectory(state.apaTelemetry);
    });
  });
  ensureApaReportShell();
  bindClick("shutdownPanelBtn", shutdownPanel);
  bindClick("restoreBtn", () => runAction("restore", {force: $("forceRestore").checked}));
  bindClick("startBtn", startSelectedScenario);
  bindClick("stopBtn", () => runAction("stop"));
  bindClick("healthBtn", () => healthCheck(true));
  bindClick("fixGeneratedBtn", () => runAction("fix-generated"));
  bindClick("copyPathBtn", copyProjectPath);
  bindClick("readmeBtn", openReadme);
  bindClick("apaReportBtn", generateApaReport);
  bindClick("apaReportDownload", downloadApaReport);
  bindClick("apaReportPdf", () => downloadApaReportPdf().catch((err) => appendLocalLog(`PDF 导出失败: ${err.message}`)));
  bindClick("apaReportClose", closeApaReport);
  bindClick("copyLogBtn", () => copyLogs().catch((err) => appendLocalLog(`复制日志失败: ${err.message}`)));
  bindClick("kickstartBtn", kickstartAcc);
  bindClick("accResultBtn", readAccResult);
  bindClick("lksReleaseBtn", () => releaseLksDriverState(true));
  bindClick("lksResultBtn", readLksResult);
  bindClick("scenarioParameterSaveBtn", () => saveScenarioParameters(true));
  bindClick("scenarioParameterResetBtn", resetScenarioParameters);
  document.querySelectorAll("[data-command]").forEach((button) => {
    const key = button.dataset.command || "";
    if (button.dataset.restart === "1") {
      button.onclick = startAccRestart;
      return;
    }
    if (key === "w" || key === "s") {
      const press = (event) => {
        event.preventDefault();
        state.accPointerHeld.add(key);
        setAccHold(key, true, true);
      };
      button.addEventListener("pointerdown", press);
      button.addEventListener("pointerup", () => {
        state.accPointerHeld.delete(key);
        setAccHold(key, false, false);
      });
      button.addEventListener("pointercancel", () => {
        state.accPointerHeld.delete(key);
        setAccHold(key, false, false);
      });
      button.addEventListener("pointerleave", () => {
        state.accPointerHeld.delete(key);
        setAccHold(key, false, false);
      });
    } else {
      button.onclick = () => queueDriverCommand(key);
    }
  });
  document.querySelectorAll("[data-ignition]").forEach((button) => {
    button.onclick = () => setAccIgnition(button.dataset.ignition);
  });
  document.querySelectorAll("[data-gear]").forEach((button) => {
    button.onclick = () => setAccGear(button.dataset.gear);
  });
  document.querySelectorAll("[data-hold]").forEach((button) => {
    const key = button.dataset.hold;
    if (key === "w" || key === "s") return;
    button.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      state.accPointerHeld.add(key);
      setAccHold(key, true, true);
    });
    button.addEventListener("pointerup", () => {
      state.accPointerHeld.delete(key);
      setAccHold(key, false, key === "f4");
    });
    button.addEventListener("pointercancel", () => {
      state.accPointerHeld.delete(key);
      setAccHold(key, false, key === "f4");
    });
  });
  document.querySelectorAll("[data-latch]").forEach((button) => {
    button.onclick = () => toggleAccLatch(button.dataset.latch);
  });
  document.querySelectorAll("[data-lane]").forEach((button) => {
    button.onclick = () => pulseAccLane(button.dataset.lane);
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
  document.querySelectorAll("[data-lks-pedal]").forEach((button) => {
    const press = (event) => {
      event.preventDefault();
      startLksPedal(button.dataset.lksPedal, true);
    };
    button.addEventListener("pointerdown", press);
    button.addEventListener("pointerup", () => releaseLksPedal(true));
    button.addEventListener("pointercancel", () => releaseLksPedal(true));
    button.addEventListener("pointerleave", () => releaseLksPedal(false));
  });
  bindClick("apaParkInBtn", () => beginApa("park_in"));
  bindClick("apaPlanOnlyBtn", () => beginApa("plan_only"));
  bindClick("apaParkOutBtn", () => beginApa("park_out"));
  bindClick("apaCancelBtn", cancelApaAssist);
  bindClick("apaRetryCommandBtn", () => runApaTask(state.apaSubmittedCommand.action, {}, true));
  bindClick("apaResetBtn", () => runApaTask("reset"));
  bindClick("apaPrepareMapBtn", () => runApaTask("prepare_map", {preparation_scope: "map"}));
  $("apaSetupForm").onsubmit = applyApaSetup;
  $("apaMap").onchange = changeApaMapDraft;
  $("apaTargetSlot").onchange = () => {
    renderApaSlotChoices(apaDraftSlots(), $("apaTargetSlot").value);
    changeApaDraft(true);
  };
  document.querySelectorAll('input[name="apaEntryDirection"]').forEach((input) => {
    input.onchange = () => {
      if (input.disabled || !input.checked) return;
      renderApaSlotChoices(apaDraftSlots(), $("apaTargetSlot").value, input.value);
      changeApaDraft(true);
    };
  });
  $("apaStartMode").onchange = () => changeApaDraft(true);
  ["apaStartX", "apaStartY", "apaStartYaw"].forEach((id) => {
    $(id).oninput = () => changeApaDraft(false);
    $(id).setAttribute("aria-describedby", "apaSetupFeedback");
  });
  bindClick("apaUseCurrentBtn", () => {
    const data = state.apaTelemetry;
    if (!data.online) return;
    $("apaStartMode").value = "custom";
    writeApaPose({x: data.vehicle_x, y: data.vehicle_y, yaw_deg: data.vehicle_yaw * 180 / Math.PI});
    changeApaDraft(false);
  });
  APA_CAMERA_VIEWS.forEach((camera) => {
    const image = $(camera.image);
    image.onload = () => {
      delete image.dataset.streamError;
      renderApaCameraStatus(state.apaTelemetry.camera_streams);
    };
    image.onerror = () => {
      if (!image.dataset.stream) return;
      image.dataset.streamError = "1";
      $(camera.status).textContent = "未连接";
    };
  });
  $("apaReport").addEventListener("keydown", (event) => {
    if (event.key === "Escape") { event.preventDefault(); closeApaReport(); }
    if (event.key === "Tab") {
      const buttons = [...$("apaReport").querySelectorAll("button:not(:disabled)")];
      const first = buttons[0], last = buttons[buttons.length - 1];
      if (event.shiftKey && document.activeElement === first) { event.preventDefault(); last.focus(); }
      else if (!event.shiftKey && document.activeElement === last) { event.preventDefault(); first.focus(); }
    }
  });
  document.querySelectorAll("[data-apa-ignition]").forEach((button) => {
    button.onclick = () => setApaIgnition(button.dataset.apaIgnition);
  });
  document.querySelectorAll("[data-apa-gear]").forEach((button) => {
    button.onclick = () => setApaGear(button.dataset.apaGear);
  });
  document.querySelectorAll("[data-apa-handbrake]").forEach((button) => {
    button.onclick = () => toggleApaHandBrake();
  });
  document.querySelectorAll("[data-apa-hold]").forEach((button) => {
    const key = button.dataset.apaHold;
    button.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      setApaHold(key, true);
    });
    button.addEventListener("pointerup", () => setApaHold(key, false));
    button.addEventListener("pointercancel", () => setApaHold(key, false));
    button.addEventListener("pointerleave", () => setApaHold(key, false));
    button.addEventListener("keydown", (event) => {
      if (["Enter", " "].includes(event.key)) {
        event.preventDefault();
        event.stopPropagation();
        if (!event.repeat) setApaHold(key, true);
      }
    });
    button.addEventListener("keyup", (event) => {
      if (["Enter", " "].includes(event.key)) {
        event.preventDefault();
        event.stopPropagation();
        setApaHold(key, false);
      }
    });
    button.addEventListener("blur", () => setApaHold(key, false));
  });
  window.addEventListener("keydown", (event) => {
    if (keyboardTargetIsEditable(event)) return;
    const profile = selectedScenario()?.control_profile;
    if (profile === "apa" && !$("apaReport").hidden) return;
    if (profile === "apa" && ["Enter", " "].includes(event.key) && event.target.closest("button, summary")) return;
    let key = event.key.toLowerCase();
    if (event.key === " ") key = "space";
    if (profile === "lks" && ["w", "s"].includes(key)) {
      event.preventDefault();
      if (event.repeat) return;
      state.lksPedalKeys.add(key);
      syncLksPedalKeyboard(true);
      return;
    }
    if (profile === "lks" && ["a", "d", "b"].includes(key)) {
      event.preventDefault();
      if (event.repeat) return;
      state.lksPressedKeys.add(key);
      syncLksKeyboardState(true);
      return;
    }
    if (profile === "acc") {
      const ignitionMap = {f1: "OFF", f2: "ACC", f3: "ON"};
      const gearMap = {1: "P", 2: "R", 3: "N", 4: "D"};
      if (ignitionMap[key] || gearMap[key] || ["w", "s", "a", "d", "space", "e", "q", "t", "r", "c", "0", "p", "l", "b", "z", "x", "f4"].includes(key)) {
        event.preventDefault();
      }
      if (event.repeat) return;
      if (ignitionMap[key]) setAccIgnition(ignitionMap[key]);
      else if (gearMap[key]) setAccGear(gearMap[key]);
      else if (["w", "s", "a", "d", "space", "f4"].includes(key)) setAccHold(key, true, true);
      else if (key === "0") startAccRestart();
      else if (key === "p") toggleAccLatch("apa");
      else if (key === "l") toggleAccLatch("lka");
      else if (key === "b") toggleAccLatch("aeb");
      else if (key === "z") pulseAccLane("left");
      else if (key === "x") pulseAccLane("right");
      else if (["e", "q", "t", "r", "c"].includes(key)) queueDriverCommand(key);
    }
    if (profile === "apa") {
      const ignitionMap = {f1: "OFF", f2: "ACC", f3: "ON"};
      const gearMap = {1: "P", 2: "R", 3: "N", 4: "D"};
      if (ignitionMap[key] || gearMap[key] || ["w", "s", "a", "d", "space", "f4"].includes(key)) event.preventDefault();
      if (event.repeat && key !== "space") return;
      if (key === "space") {
        if (event.repeat) return;
        toggleApaHandBrake();
        return;
      }
      if (ignitionMap[key]) setApaIgnition(ignitionMap[key]);
      else if (gearMap[key]) setApaGear(gearMap[key]);
      else if (["w", "s", "a", "d", "f4"].includes(key)) setApaHold(key, true);
    }
  });
  window.addEventListener("pointerup", () => {
    for (const key of [...state.accPointerHeld]) {
      state.accPointerHeld.delete(key);
      setAccHold(key, false, key === "f4");
    }
  });
  window.addEventListener("pointercancel", () => {
    for (const key of [...state.accPointerHeld]) {
      state.accPointerHeld.delete(key);
      setAccHold(key, false, key === "f4");
    }
  });
  window.addEventListener("keyup", (event) => {
    let key = event.key.toLowerCase();
    if (event.key === " ") key = "space";
    if (["w", "s"].includes(key) && selectedScenario()?.control_profile === "lks") {
      state.lksPedalKeys.delete(key);
      syncLksPedalKeyboard(true);
    }
    if (["a", "d", "b"].includes(key)) {
      state.lksPressedKeys.delete(key);
      syncLksKeyboardState(true);
    }
    if (["w", "s", "a", "d", "space", "f4"].includes(key) && selectedScenario()?.control_profile === "acc") {
      setAccHold(key, false, key === "f4");
    }
    if (["w", "s", "a", "d", "f4"].includes(key) && selectedScenario()?.control_profile === "apa") {
      setApaHold(key, false);
    }
  });
  window.addEventListener("focus", () => {
    if (selectedScenario()?.control_profile === "acc") refreshAccConsole();
    if (selectedScenario()?.control_profile === "apa") startApaVehicleHeartbeat();
  });
  window.addEventListener("blur", () => {
    releaseApaInput();
    state.lksPressedKeys.clear();
    state.lksPedalKeys.clear();
    releaseLksDriverState(false);
    releaseLksPedal(false);
    ["w", "s", "a", "d", "space", "f4"].forEach((key) => state.accHeld.delete(key));
    releaseAccLevelInput();
    stopAccVehicleHeartbeat();
    flushAccVehicle(false);
  });
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) {
      releaseApaInput();
      state.lksPressedKeys.clear();
      state.lksPedalKeys.clear();
      releaseLksDriverState(false);
      releaseLksPedal(false);
      ["w", "s", "a", "d", "space", "f4"].forEach((key) => state.accHeld.delete(key));
      releaseAccLevelInput();
      stopAccVehicleHeartbeat();
      flushAccVehicle(false);
    } else if (selectedScenario()?.control_profile === "acc") {
      refreshAccConsole();
    } else if (selectedScenario()?.control_profile === "apa") {
      startApaVehicleHeartbeat();
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
  await loadScenarioParameters();
  await healthCheck(false);
  await pollAccTelemetry();
  refreshAccConsole();
  setInterval(pollLogs, LOG_POLL_MS);
  setInterval(() => loadScenarios().catch(() => {}), SCENARIO_REFRESH_MS);
  setInterval(() => healthCheck(false).catch(() => {}), HEALTH_POLL_MS);
  setInterval(() => pollAccTelemetry().catch(() => {}), ACC_TELEMETRY_POLL_MS);
  setInterval(() => pollApaTelemetry().catch(() => {}), ACC_TELEMETRY_POLL_MS);
}

boot().catch((err) => appendLocalLog(err.message));
