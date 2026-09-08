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
  accLevelTimer: null,
  accCommandQueue: Promise.resolve(),
  accCommandFeedbackTimers: new Map(),
  accTelemetryBusy: false,
  accCommandFeedbackOverride: null,
  scenarioParameters: {schema: [], values: {}},
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
  setDisabled("accResultBtn", busy || !hasScenario);
  setDisabled("lksReleaseBtn", busy || !hasScenario);
  setDisabled("lksResultBtn", busy || !hasScenario);
  setDisabled("scenarioParameterSaveBtn", busy || !hasScenario);
  setDisabled("scenarioParameterResetBtn", busy || !hasScenario);
  document.querySelectorAll("[data-scenario-param]").forEach((input) => {
    input.disabled = busy || !hasScenario;
  });
  document.querySelectorAll("[data-command]").forEach((button) => {
    button.disabled = busy || !hasScenario;
  });
  document.querySelectorAll("[data-lks-steer], [data-lks-brake]").forEach((button) => {
    button.disabled = busy || !hasScenario;
  });
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
    throw new Error(data.error || data.message || `HTTP ${res.status}`);
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
  const accTelemetryCard = $("accTelemetryCard");
  const lksDriverCard = $("lksDriverCard");
  const scenarioParameterCard = $("scenarioParameterCard");
  if (accDriverCard) accDriverCard.hidden = profile !== "acc";
  if (accTelemetryCard) accTelemetryCard.hidden = profile !== "acc";
  if (lksDriverCard) lksDriverCard.hidden = profile !== "lks";
  if (scenarioParameterCard) scenarioParameterCard.hidden = !item.has_parameters;
  const parameterTitle = $("scenarioParameterTitle");
  if (parameterTitle) parameterTitle.textContent = `${profile.toUpperCase()} 运行参数`;
  const parameterFootnote = $("scenarioParameterFootnote");
  if (parameterFootnote) {
    parameterFootnote.textContent = profile === "lks"
      ? "参数在下次启动时生效；三点权重之和必须为 1.0。车型标定参数修改前请记录原值。"
      : "参数在下次启动时生效；速度阶跃时刻设为 0 可保持固定前车速度。执行器约束只影响本地调试场景。";
  }
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

function bindEvents() {
  bindClick("shutdownPanelBtn", shutdownPanel);
  bindClick("restoreBtn", () => runAction("restore", {force: $("forceRestore").checked}));
  bindClick("startBtn", startSelectedScenario);
  bindClick("stopBtn", () => runAction("stop"));
  bindClick("healthBtn", () => healthCheck(true));
  bindClick("fixGeneratedBtn", () => runAction("fix-generated"));
  bindClick("copyPathBtn", copyProjectPath);
  bindClick("readmeBtn", openReadme);
  bindClick("copyLogBtn", () => copyLogs().catch((err) => appendLocalLog(`复制日志失败: ${err.message}`)));
  bindClick("kickstartBtn", kickstartAcc);
  bindClick("accResultBtn", readAccResult);
  bindClick("lksReleaseBtn", () => releaseLksDriverState(true));
  bindClick("lksResultBtn", readLksResult);
  bindClick("scenarioParameterSaveBtn", () => saveScenarioParameters(true));
  bindClick("scenarioParameterResetBtn", resetScenarioParameters);
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
  await loadScenarioParameters();
  await healthCheck(false);
  await pollAccTelemetry();
  setInterval(pollLogs, LOG_POLL_MS);
  setInterval(() => loadScenarios().catch(() => {}), SCENARIO_REFRESH_MS);
  setInterval(() => healthCheck(false).catch(() => {}), HEALTH_POLL_MS);
  setInterval(() => pollAccTelemetry().catch(() => {}), ACC_TELEMETRY_POLL_MS);
}

boot().catch((err) => appendLocalLog(err.message));
