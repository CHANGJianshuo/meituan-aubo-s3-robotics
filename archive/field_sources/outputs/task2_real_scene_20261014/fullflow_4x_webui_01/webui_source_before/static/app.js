/* Local UI, authored with OpenAI Codex. All task progress comes from snapshots. */
"use strict";

const $ = (id) => document.getElementById(id);
const colorNames = {1: "红", 2: "蓝", 3: "黄", 4: "绿"};
const colorClasses = {1: "red", 2: "blue", 3: "yellow", 4: "green"};
const statusNames = {SUBMITTING: "等待受理", UNKNOWN: "结果未知", REJECTED: "已拒绝", OBSERVED: "外部任务", ACCEPTED: "已受理", EXECUTING: "执行中", CANCELING: "取消中", SUCCEEDED: "已完成", ABORTED: "失败", CANCELED: "已取消"};
const eventNames = {ROS_CONNECTION: "ROS 连接", GOAL_REQUESTED: "提交请求", TASK_STATE: "FSM 状态", ACTION_UNKNOWN: "回执未知", CANCEL_REQUESTED: "取消请求", CANCEL_REPLY: "取消回执", STOP_REQUESTED: "停止请求", STOP_REPLY: "停止回执", ROS_LOG: "ROS 日志", SESSION_STARTED: "会话启动", GOAL_ACCEPTED: "任务受理", FEEDBACK: "执行反馈", PLACEMENT_CONFIRMED: "放置确认", RESULT: "任务结果", CANCEL_DELIVERED: "取消送达", STOP_CONFIRMED: "停稳反馈", TASK_REJECTED: "任务拒绝", DEMO_CONDITION: "演练条件"};
let snapshot = null;
let online = false;
let paused = false;
let busy = false;
let fetching = false;
let awaitingSnapshot = false;
let lastReceived = 0;
let pauseGeneration = 0;
let eventSignature = "";
let historySignature = "";
let hydratedTask = "";
let hardwareFormReady = false;
let cameraLoading = false;
let cameraKey = "";
let cameraUrl = "";
const isMock = () => snapshot?.mode === "mock";
let taskNoticeId = null;
let taskNoticeSignature = "";
const isHardware = () => snapshot?.mode === "hardware_trajectory";
const isCppMock = () => snapshot?.mode === "cpp_mock";

function element(tag, text, className) {
  const node = document.createElement(tag);
  if (text !== undefined) node.textContent = text;
  if (className) node.className = className;
  return node;
}

function timeLabel(value, full = false) {
  if (!value) return "—";
  const date = new Date(value);
  return full ? date.toLocaleString("zh-CN", {hour12: false}) : date.toLocaleTimeString("zh-CN", {hour12: false});
}

function transportFresh() {
  return online && !paused && snapshot && performance.now() - lastReceived < snapshot.freshness_limit_ms;
}

function quality(sample) {
  if (!transportFresh() || (["ros", "cpp_mock", "hardware_trajectory"].includes(snapshot?.mode) && !snapshot?.connection?.connected)) return "离线";
  if (!sample?.available || !sample.data || sample.age_ms === null) return "缺失";
  if (sample.age_ms + performance.now() - lastReceived >= snapshot.freshness_limit_ms) return "过期";
  return "正常";
}

function notify(message, error = false, taskNotice = false) {
  if (!taskNotice) { taskNoticeId = null; taskNoticeSignature = ""; }
  $("notice").textContent = message;
  $("notice").className = "notice" + (error ? " error" : "");
  $("notice").hidden = false;
}

async function api(path, body) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), 2500);
  try {
    const response = await fetch(path, {
      method: body === undefined ? "GET" : "POST",
      headers: body === undefined ? {} : {"Content-Type": "application/json"},
      body: body === undefined ? undefined : JSON.stringify(body),
      cache: "no-store", signal: controller.signal,
    });
    const result = await response.json();
    if (!response.ok) throw new Error(result.message || `HTTP ${response.status}`);
    return result;
  } finally {
    clearTimeout(timer);
  }
}

async function refresh() {
  if (paused || fetching) return;
  fetching = true;
  const generation = pauseGeneration;
  try {
    const data = await api("/api/snapshot");
    if (generation !== pauseGeneration || paused) return;
    if (data.schema_version !== 1 || !["mock", "ros", "cpp_mock", "hardware_trajectory"].includes(data.mode) || !data.operations) {
      throw new Error("当前页面需要 v1 ROS / Mock 数据契约");
    }
    if (snapshot && data.session_id === snapshot.session_id && data.snapshot_seq <= snapshot.snapshot_seq) return;
    if (snapshot && data.session_id !== snapshot.session_id) {
      notify("服务已创建新会话；此前记录保留在原输出目录。任务不会自动重发。");
      $("record-detail").hidden = true;
    }
    snapshot = data;
    awaitingSnapshot = false;
    lastReceived = performance.now();
    online = true;
    render();
  } catch (error) {
    if (generation !== pauseGeneration || paused) return;
    online = false;
    renderHealth();
  } finally {
    fetching = false;
  }
}

function renderHealth() {
  const fresh = transportFresh();
  const mock = isMock();
  const connected = fresh && (mock || snapshot?.connection?.connected);
  const suffix = mock ? "（Mock）" : "";
  $("connection").className = "connection " + (connected ? "online" : "offline");
  $("connection").lastElementChild.textContent = paused ? "接收已暂停" : !fresh ? "服务未连接" : mock ? "Mock 已连接" : connected ? (isHardware() ? "实机已连接" : "ROS 已连接") : (isHardware() ? "实机离线" : "ROS 离线");
  const robotQuality = quality(snapshot?.robot);
  const toolQuality = quality(snapshot?.tool);
  const invalidSample = isCppMock() ? quality(snapshot?.cpp_task) !== "正常" || quality(snapshot?.cpp_pick) !== "正常" : robotQuality !== "正常" || toolQuality !== "正常";
  $("connection-alert").hidden = fresh && !invalidSample && connected;
  $("connection-alert").textContent = !fresh
    ? (paused ? "接收已暂停，显示旧状态；恢复后读取当前任务，不重发指令。" : "无法获取最新快照。当前状态不可确认，操作已禁用。")
    : !connected ? snapshot?.connection?.reason || "等待 ROS 连接"
      : isCppMock() ? "C++ Mock 就绪状态缺失或过期；请检查统一启动日志。" : `设备数据${robotQuality !== "正常" ? robotQuality : toolQuality}。${mock ? "可在接口状态页恢复演练数据。" : "检查发布源与消息时间戳。"}`;
  $("api-status").textContent = fresh ? "已连接 / HTTP 快照" : paused ? "本浏览器暂停接收" : "未连接";
  $("updated").textContent = snapshot ? `快照 ${timeLabel(snapshot.generated_at)}` : "等待快照";
  $("sample-quality").textContent = !fresh ? "离线" : invalidSample ? (robotQuality !== "正常" ? robotQuality : toolQuality) : "状态有效" + suffix;
  $("sample-quality").className = "tag " + (!fresh || invalidSample ? "bad" : "good");
  if (isCppMock()) { $("sample-quality").textContent = "硬件未接入"; $("sample-quality").className = "tag"; }
  function device(id, sample, value, yes, no, goodValue = true) {
    const q = quality(sample);
    const known = q === "正常" && typeof value === "boolean";
    $(id).textContent = known ? (value ? yes : no) + suffix : `未知 · ${q}`;
    $(id).className = known ? (value === goodValue ? "good-text" : "warning-text") : "muted";
  }
  const robot = snapshot?.robot?.data;
  const tool = snapshot?.tool?.data;
  device("robot-communication", snapshot?.robot, robot?.communication_ok, "正常", "未连接");
  device("robot-ready", snapshot?.robot, robot?.robot_ready, "就绪", "未就绪");
  device("robot-motion", snapshot?.robot, robot?.motion_active, "运动中", "未在运动", false);
  device("robot-stop", snapshot?.robot, robot?.stop_confirmed, "已确认", "等待确认");
  device("robot-safety", snapshot?.robot, robot?.safety_normal, "正常", "联锁异常");
  $("robot-controller").textContent = robotQuality === "正常" ? robot?.controller_mode || "未报告" : "未知";
  device("tool-verified", snapshot?.tool, tool?.verified, "已验证", "未验证");
  $("tool-state").textContent = toolQuality === "正常" ? ({0:"未知",1:"已分离",2:"接合中",3:"已附着",4:"释放中",5:"故障"}[tool.state] || "未知") + suffix : `未知 · ${toolQuality}`;
  if (isHardware()) $("tool-state").textContent = "被动 V1 / 无附着传感器";
  const evidence = ["无证据", "仅命令", "几何证据", "传感或视觉证据"][tool?.evidence_level] || "未知";
  $("source-note").textContent = `机器人：${snapshot?.robot?.source || "无数据"}；工具：${snapshot?.tool?.source || "无数据"} · 证据：${toolQuality === "正常" ? evidence + suffix : "未知"}`;
  const active = snapshot?.task && ["SUBMITTING", "ACCEPTED", "EXECUTING", "CANCELING", "UNKNOWN"].includes(snapshot.task.status);
  $("goal-fields").disabled = busy || Boolean(active);
  $("submit").disabled = busy || awaitingSnapshot || !fresh || !snapshot?.operations.submit || invalidSample;
  $("cancel").disabled = busy || awaitingSnapshot || !fresh || !snapshot?.operations.cancel;
  $("software-stop").disabled = busy || awaitingSnapshot || !fresh || !snapshot?.operations.stop;
  $("software-stop").hidden = mock || isCppMock();
  $("confirm-stop").disabled = busy || awaitingSnapshot || !fresh || !snapshot?.operations.confirm_mock_stop;
  $("condition").disabled = busy || !fresh;
  const reasons = snapshot?.operations.reasons || [];
  $("operation-hint").textContent = !fresh ? "恢复连接后重新核对当前任务。"
    : !mock ? reasons.join("；") || (isCppMock() ? "向 C++ 软件 Mock 提交 ROS 任务；未连接硬件。" : isHardware() ? "执行已核验的红电池 → T0 单次轨迹。" : "向 ExecuteTask 提交真实任务。请核对颜色口令和现场设备。")
      : active ? "任务正在运行。取消回执与停稳确认分别显示。"
        : invalidSample ? "恢复有效设备数据后才能开始任务。"
          : !snapshot?.operations.submit ? "等待独立停稳反馈。" : "当前为本地 Mock 演练。";
  $("stop-reply").hidden = !snapshot?.stop_request;
  $("stop-reply").textContent = snapshot?.stop_request?.message || "";
  const state = snapshot?.task_state;
  $("fsm-state").textContent = mock ? snapshot?.task?.state || "待命（Mock）" : quality(state) === "正常" ? `${state.data.state} / ${state.data.substate || "—"}` : "未知 · " + quality(state);
  $("external-task").textContent = snapshot?.external_task ? `外部任务 ${snapshot.external_task.task_id} · 本会话没有 Goal 句柄` : "";
  if (!mock && quality(snapshot?.camera?.[$("camera-stream").value]) !== "正常") {
    $("camera-image").hidden = true;
    $("camera-empty").hidden = false;
  }
  renderJoints();
}

function renderMode() {
  const mock = isMock();
  document.querySelectorAll("[data-mock]").forEach(node => {node.hidden = !mock;});
  $("live-view").hidden = mock;
  $("ros-setup").hidden = mock;
  $("environment-tag").textContent = mock ? "Mock 演练" : "实机模式";
  $("environment-tag").className = "tag " + (mock ? "mock" : "");
  $("environment-note").textContent = mock ? "所有状态和结果均来自本地演示器。" : "真实 ROS 任务与设备反馈 · 软件停止不等同于实体急停";
  $("station-mode").textContent = mock ? "本地 Mock 工作站" : "实机工作站";
  $("sidebar-status").textContent = mock ? "界面演练环境" : snapshot.connection?.connected ? "ROS 2 Jazzy" : "等待 ROS 连接";
  $("submit").textContent = mock ? "开始 Mock 任务 ↗" : "提交";
  $("footer-note").textContent = mock ? "Mock 结果不代表 ROS、物理仿真或实机验收" : "系统集成工作站 · 状态与结果以设备反馈为准";
  $("data-provider").textContent = mock ? "webui.mock_backend" : "ROS 2 / rclpy";
  $("connection-mode").textContent = mock ? "Mock" : "ROS 2 Jazzy";
  $("ros-connection-detail").textContent = mock ? "显式启动的本地演练服务" : snapshot.connection?.reason || "等待 ROS";
  $("domain-id").textContent = snapshot.config?.domain_id ?? "未配置";
  $("control-mode").textContent = mock ? "Mock 演练" : snapshot.config?.read_only ? "只读观察" : "真实任务 / Cancel / 软件停止";
  $("connection-reasons").textContent = snapshot.operations.reasons?.join("；") || "当前满足任务提交条件";
  if (isCppMock()) {
    $("environment-tag").textContent = "C++ Mock · ROS 2";
    $("environment-note").textContent = "真实 C++ Task → PickPlace Action · 软件 Mock，无硬件 / MoveIt";
    $("station-mode").textContent = "C++ Mock 集成工作站";
    $("footer-note").textContent = "结果来自 C++ Action；不代表完整机器人 FSM、MoveIt 或实机验收";
    $("control-mode").textContent = snapshot.config?.read_only ? "只读观察" : "C++ Mock Task / 定向 Cancel";
    $("data-provider").textContent = "C++ Task / PickPlace → ROS 2 → rclpy";
    $("connection-mode").textContent = "C++ Mock / Jazzy";
    if (snapshot.local_preview) {
      $("environment-note").textContent = "C++ 软件 Mock 任务；相机是现场只读画面，页面任务不会移动实机。";
    }
  }
  if (isHardware()) {
    $("environment-tag").textContent = "AUBO 实机 · 已验证轨迹";
    $("environment-note").textContent = "红电池 → T0 · half_time 平滑轨迹 · 进度来自控制柜实际反馈";
    $("station-mode").textContent = "实机轨迹工作站";
    $("footer-note").textContent = "撤离后还有约 3 秒放置核验；请等页面显示“已完成”后再复位电池。";
    $("sidebar-status").textContent = snapshot.connection.connected ? "AUBO SDK 已连接" : "等待实机连接";
    $("data-provider").textContent = "NUC / AUBO SDK / 受保护播放器";
    $("connection-mode").textContent = "真实机械臂 / SSH + SDK";
    $("control-mode").textContent = "单次红 → T0 / 定向取消 / 软件停止";
    $("domain-id").textContent = "此轨迹入口不使用 ROS Action";
    $("submit").textContent = "提交";
    document.querySelector('input[name="mode"][value="2"]').disabled = true;
    document.querySelector('input[name="mode"][value="1"]').checked = true;
    $("color-0").value = "1";
    $("color-0").disabled = true;
    $("timeout").min = "120";
    if (!hardwareFormReady) { updateForm(); hardwareFormReady = true; }
    $("tool-state").textContent = "被动 V1 / 无附着传感器";
  }
  $("connection-config").textContent = JSON.stringify(snapshot.config || {}, null, 2);
  $("discovered-cameras").textContent = "发现的图像话题：" + (snapshot.connection?.image_topics?.join("、") || "无");
  const rows = $("ros-interfaces");
  rows.replaceChildren();
  const append = (name, purpose, status) => {
    const row = document.createElement("tr");
    row.append(element("td", name), element("td", purpose), element("td", status));
    rows.append(row);
  };
  if (mock) { append("webui.mock_backend", "定时演示数据", "未连接 ROS"); return; }
  if (isHardware()) {
    append("NUC / AUBO SDK", "关节、模式、运动、停稳", quality(snapshot.robot));
    append("half_time_package", "10 段平滑轨迹 / 单次执行", snapshot.task?.status || "等待任务");
    append("owned SSH control", "本次任务取消 / 停止", snapshot.operations.stop ? "可请求" : "无活动任务");
    append("V1 44 mm", "安装核验；无抓取传感器", snapshot.tool?.data?.detail || "等待核验");
    return;
  }
  append(snapshot.config.task_action, "任务 Action", snapshot.connection.action_ready ? "唯一服务端已就绪" : "未就绪或服务端不唯一");
  if (isCppMock()) append(snapshot.cpp_pick_action, "C++ PickPlace 下游 Action", snapshot.connection.child_ready ? "唯一 C++ 服务端已就绪" : "未就绪 / 冲突");
  if (!isCppMock()) append(snapshot.config.stop_service, "软件停止 RequestStop", snapshot.connection.stop_ready ? "服务可用" : "不可用或服务端不唯一");
  const labels = {task_state:"任务状态", robot:"机器人 / 停稳", tool:"工具证据", joints:"关节反馈", color:"彩色相机", depth:"深度相机"};
  for (const [kind, label] of Object.entries(labels)) {
    const info = snapshot.connection.publishers?.[kind];
    const sample = ["color", "depth"].includes(kind) ? snapshot.camera[kind] : kind === "task_state" ? snapshot.task_state : snapshot[kind];
    append(info?.topic || snapshot.config[kind + "_topic"] || "等待配置 / 发现", label,
      info?.selected ? `${quality(sample)} · ${info.selected}` : `未选定发布源${info?.candidates.length ? " · " + info.candidates.map(p => p.node).join("、") : ""}`);
  }
}

function renderJoints() {
  const sample = snapshot?.joints;
  const q = quality(sample);
  $("joints-quality").textContent = q === "正常" ? "实时 / °" : q;
  $("joints").replaceChildren();
  if (q !== "正常") { $("joints").append(element("p", "关节位置未知 · " + q, "muted")); return; }
  const {name, position} = sample.data;
  name.forEach((n, i) => {
    const cell = element("div", undefined, "joint-value");
    cell.append(element("small", n), element("strong", Number.isFinite(position[i]) ? (position[i] * 180 / Math.PI).toFixed(2) + "°" : "未知"));
    $("joints").append(cell);
  });
  $("joints-note").textContent = `来源 ${sample.source} · 龄期 ${sample.age_ms} ms`;
}

async function renderCamera() {
  if (isMock()) return;
  const stream = $("camera-stream").value;
  const sample = snapshot?.camera?.[stream];
  const q = quality(sample);
  $("camera-quality").textContent = q === "正常" ? "实时" : q;
  $("camera-message").textContent = sample?.available && (q === "过期" || q === "离线") ? "图像已过期，恢复新帧后继续显示。" : "尚未收到可显示的新鲜图像。";
  $("camera-note").textContent = sample?.data ? `${sample.data.topic} · ${sample.data.width} × ${sample.data.height} · 龄期 ${sample.age_ms ?? "未知"} ms · ${sample.data.preview}` : "相机话题未连接；在接口状态页查看配置与发现结果。";
  const key = `${snapshot?.session_id}/${stream}/${sample?.source_stamp_ns}/${sample?.source}`;
  if (q !== "正常" || cameraLoading || key === cameraKey) return;
  cameraLoading = true;
  const generation = pauseGeneration;
  const session = snapshot.session_id;
  const source = sample.source;
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), 2000);
  try {
    const response = await fetch(`/api/camera/${stream}`, {cache: "no-store", signal: controller.signal});
    if (!response.ok) throw new Error("图像不可用");
    const blob = await response.blob();
    if (generation !== pauseGeneration || session !== snapshot.session_id || source !== snapshot.camera?.[stream]?.source || stream !== $("camera-stream").value || quality(snapshot.camera[stream]) !== "正常") return;
    if (cameraUrl) URL.revokeObjectURL(cameraUrl);
    cameraUrl = URL.createObjectURL(blob);
    $("camera-image").src = cameraUrl;
    $("camera-image").hidden = false;
    $("camera-empty").hidden = true;
    cameraKey = key;
  } catch (error) {
    $("camera-image").hidden = true;
    $("camera-empty").hidden = false;
    $("camera-message").textContent = "当前图像不可用；等待下一帧。";
  } finally { clearTimeout(timer); cameraLoading = false; }
}

function renderScene() {
  const task = snapshot?.task;
  const goal = task?.goal || draftGoal();
  const sequence = goal.mode === 2;
  $("basic-target").toggleAttribute("hidden", sequence);
  $("sequence-targets").toggleAttribute("hidden", !sequence);
  for (const slot of ["T0", "P1", "P2", "P3"]) {
    const index = slot === "T0" ? 0 : Number(slot[1]) - 1;
    const color = goal.ordered_colors[index];
    $("target-label-" + slot).textContent = colorNames[color] ? `${colorNames[color]}色` : "待指定";
    $("slot-" + slot).classList.toggle("complete", Boolean(task?.completed_slots?.includes(slot)));
    $("slot-" + slot).classList.toggle("current", task?.current_slot === slot && ["EXECUTING", "CANCELING"].includes(task.status));
  }
  for (const [color, name] of Object.entries(colorNames)) {
    const selected = task && task.current_object_id === `mock-battery-${color}` && task.status === "EXECUTING";
    const index = goal.ordered_colors.indexOf(Number(color));
    $("battery-" + color).classList.toggle("selected", Boolean(selected));
    $("battery-" + color).classList.toggle("used", Boolean(task && index >= 0 && index < task.completed_count));
  }
}

function renderTask() {
  const external = snapshot?.external_task;
  const task = external && external.state !== "WAIT_TASK" ? {...external, goal: null, status: "OBSERVED", elapsed_ms: null} : snapshot?.task;
  const status = task?.status;
  const signature = task && `${task.task_id}:${status}:${task.detail}`;
  if (task && taskNoticeId === task.task_id && signature !== taskNoticeSignature) {
    taskNoticeSignature = signature;
    notify(`${statusNames[status] || status}：${task.detail || "等待实际反馈"}`,
      ["UNKNOWN", "ABORTED", "REJECTED"].includes(status), true);
  }
  $("task-status").textContent = statusNames[status] || (isMock() || (quality(snapshot?.task_state) === "正常" && snapshot.task_state.data.state === "WAIT_TASK") ? "等待任务" : "状态未知");
  $("task-status").className = "tag " + (status === "SUCCEEDED" ? "good" : status === "ABORTED" ? "bad" : ["EXECUTING", "CANCELING"].includes(status) ? "running" : "");
  $("workcell-description").textContent = task?.detail || (isMock() ? "选择颜色口令，开始一次 Mock 演练" : "实时相机与任务反馈");
  $("progress-label").textContent = task ? `${task.goal ? (task.goal.mode === 1 ? "基础任务 · T0" : "序列任务 · P1–P3") : "外部任务"} / 服务端确认完成数` : "尚未提交任务";
  $("progress-count").textContent = `${task?.completed_count || 0} / ${task?.total_count || 0}`;
  $("progress").max = task?.total_count || 1;
  $("progress").value = task?.completed_count || 0;
  $("task-id").textContent = task?.task_id || "—";
  $("task-id").title = task ? `Goal ID: ${task.goal_id}` : "";
  $("elapsed").textContent = task?.elapsed_ms != null ? `${(task.elapsed_ms / 1000).toFixed(1)} 秒` : "—";
  const stage = !status ? (isMock() || (quality(snapshot?.task_state) === "正常" && snapshot.task_state.data.state === "WAIT_TASK") ? "ready" : "unknown") : ["SUBMITTING", "ACCEPTED"].includes(status) ? "accepted" : ["EXECUTING", "OBSERVED", "UNKNOWN"].includes(status) ? "running" : status === "CANCELING" ? "canceling" : "finished";
  document.querySelector('#lifecycle [data-stage="accepted"]').textContent = status === "SUBMITTING" ? "等待受理" : "已受理";
  for (const node of $("lifecycle").children) {
    const current = node.dataset.stage === stage;
    node.classList.toggle("active", current);
    if (current) node.setAttribute("aria-current", "step"); else node.removeAttribute("aria-current");
  }
  $("task-result").hidden = !task?.result;
  if (task?.result) {
    $("task-result").textContent = `${task.result.message} · 错误码 ${task.result.error_code}`;
    $("task-result").className = "result" + (status === "ABORTED" ? " bad" : "");
  }
  renderScene();
}

function renderEvents() {
  const filter = $("event-filter").value;
  const task = snapshot?.task;
  let events = filter === "current" && task && !snapshot?.external_task ? task.events : snapshot?.events || [];
  if (filter === "errors") events = events.filter(e => ["error", "warning"].includes(e.level));
  const signature = `${snapshot?.session_id}/${snapshot?.event_seq}/${filter}/${task?.task_id}`;
  if (signature === eventSignature) return;
  eventSignature = signature;
  $("event-count").textContent = `${events.length} 条`;
  $("events").replaceChildren();
  if (!events.length) $("events").append(element("li", "暂无对应事件。提交任务后，受理、反馈和结果会显示在这里。", "empty-event"));
  for (const event of [...events].reverse()) {
    const li = element("li", undefined, event.level);
    const time = element("time", timeLabel(event.at));
    time.dateTime = event.at;
    const message = element("span", event.message, "event-message");
    message.title = event.task_id ? `Task: ${event.task_id}\nGoal: ${event.goal_id}` : "会话事件";
    li.append(time, element("span", eventNames[event.kind] || event.kind, "event-kind"), message);
    $("events").append(li);
  }
}

function renderHistory() {
  const history = snapshot?.history || [];
  $("record-count").textContent = history.length;
  $("history-empty").hidden = history.length > 0;
  const signature = JSON.stringify(history);
  if (signature === historySignature) return;
  historySignature = signature;
  $("history").replaceChildren();
  for (const task of history) {
    const row = document.createElement("tr");
    const identity = document.createElement("td");
    identity.append(element("code", task.task_id), element("small", timeLabel(task.created_at, true)));
    const status = document.createElement("td");
    status.append(element("span", statusNames[task.status] || task.status, "tag " + (task.status === "SUCCEEDED" ? "good" : task.status === "ABORTED" ? "bad" : "")));
    const actions = document.createElement("td");
    const inspect = element("button", "查看", "button quiet small");
    inspect.type = "button";
    inspect.addEventListener("click", () => openRecord(task.task_id));
    const download = element("a", "下载 JSON", "button quiet small");
    download.href = `/api/tasks/${encodeURIComponent(task.task_id)}`;
    download.download = `${task.task_id}.json`;
    actions.append(inspect, download);
    row.append(identity, status, element("td", `${task.completed_count} / ${task.total_count}`), element("td", String(task.result?.error_code ?? "—")), actions);
    $("history").append(row);
  }
}

async function openRecord(taskId) {
  try {
    const record = await api(`/api/tasks/${encodeURIComponent(taskId)}`);
    $("record-detail-title").textContent = "任务详情";
    $("record-detail-subtitle").textContent = `${record.task_id} · ${statusNames[record.status]}`;
    $("record-json").textContent = JSON.stringify(record, null, 2);
    $("record-detail").hidden = false;
    $("record-detail").scrollIntoView({block: "start"});
    $("close-record").focus({preventScroll: true});
  } catch (error) { notify(`无法读取记录：${error.message}`, true); }
}

function render() {
  const task = snapshot.task;
  if (task && !hydratedTask && !taskNoticeId) taskNoticeId = task.task_id;
  if (task?.goal && task.task_id !== hydratedTask) {
    hydratedTask = task.task_id;
    document.querySelector(`input[name="mode"][value="${task.goal.mode}"]`).checked = true;
    for (const i of [0, 1, 2]) $("color-" + i).value = task.goal.ordered_colors[i] || "";
    $("timeout").value = task.goal.timeout_ms / 1000;
    $("scenario").value = task.scenario;
    $("timeout").readOnly = task.scenario === "timeout";
    updateForm();
  }
  renderMode();
  renderHealth();
  renderTask();
  renderCamera();
  renderEvents();
  renderHistory();
  $("session-id").textContent = snapshot.session_id;
  if (document.activeElement !== $("condition")) $("condition").value = snapshot.condition;
}

function draftGoal() {
  const mode = Number(document.querySelector('input[name="mode"]:checked').value);
  return {mode, ordered_colors: Array.from({length: mode === 1 ? 1 : 3}, (_, i) => Number($("color-" + i).value))};
}

function updateForm() {
  const sequence = draftGoal().mode === 2;
  document.querySelectorAll(".sequence-field").forEach(node => {node.hidden = !sequence;});
  for (const i of [1, 2]) {
    $("color-" + i).disabled = !sequence;
    $("color-" + i).required = sequence;
  }
  $("slot-label-0").textContent = sequence ? "P1" : "T0";
  $("color-count").textContent = sequence ? "3 个目标" : "1 个目标";
  for (const i of [0, 1, 2]) $("swatch-" + i).className = "color-swatch " + (colorClasses[$("color-" + i).value] || "");
  $("form-error").hidden = true;
  if (!snapshot?.task) renderScene();
}

async function command(path, payload, message) {
  if (!transportFresh() || busy) return;
  busy = true;
  renderHealth();
  try {
    const response = await api(path, payload);
    awaitingSnapshot = true;
    pauseGeneration += 1; // Discard reads started before this command was acknowledged.
    notify(message);
    if (path === "/api/tasks" || path === "/api/stop" || path.endsWith("/cancel")) {
      taskNoticeId = response.task_id || payload.task_id || snapshot?.task?.task_id;
      taskNoticeSignature = "";
    }
    await refresh();
  } catch (error) {
    const uncertain = error.name === "AbortError" || error instanceof TypeError;
    notify(uncertain ? "请求结果暂未确认。请等待快照恢复后检查当前任务；不会自动重发。" : error.message, true);
    if (uncertain) {
      taskNoticeId = payload?.task_id || snapshot?.task?.task_id || null;
      taskNoticeSignature = "";
      online = false;
    }
  } finally {
    busy = false;
    renderHealth();
  }
}

$("task-form").addEventListener("submit", async event => {
  event.preventDefault();
  if ($("submit").disabled) return;
  const goal = draftGoal();
  if (new Set(goal.ordered_colors).size !== goal.ordered_colors.length) {
    $("form-error").textContent = "序列任务需要三种不同颜色，请检查口令。";
    $("form-error").hidden = false;
    return;
  }
  const payload = {...goal, task_id: "ui-" + crypto.randomUUID(), timeout_ms: Number($("timeout").value) * 1000,
    placement_stability_sec: 3};
  if (isMock()) payload.scenario = $("scenario").value;
  await command("/api/tasks", payload, isMock() ? "Mock 任务已受理。" : "任务请求已登记，等待执行端实际回执；不会自动重发。");
});
$("cancel").addEventListener("click", async () => {
  const task = snapshot?.task;
  if (!task || $("cancel").disabled) return;
  await command(`/api/tasks/${encodeURIComponent(task.task_id)}/cancel`, {goal_id: task.goal_id}, "取消请求已发送，等待执行端回执与独立停稳确认。");
});
$("software-stop").addEventListener("click", async () => {
  if (!$("software-stop").disabled) await command("/api/stop", {}, "软件停止已排队。送达回执与停稳反馈分别显示。");
});
$("camera-stream").addEventListener("change", () => {
  cameraKey = "";
  $("camera-image").hidden = true;
  $("camera-empty").hidden = false;
  renderCamera();
});
$("pause").addEventListener("click", () => {
  paused = !paused;
  pauseGeneration += 1;
  online = false;
  $("pause").textContent = paused ? "恢复接收" : "暂停接收";
  renderHealth();
  if (!paused) refresh();
});
$("condition").addEventListener("change", async () => {
  await command("/api/demo/condition", {condition: $("condition").value}, "Mock 数据条件已更新。");
});
$("confirm-stop").addEventListener("click", async () => {
  await command("/api/demo/confirm-stop", {}, "已补发 Mock 停稳反馈；仅用于界面演练。");
});
$("scenario").addEventListener("change", () => {
  const timeout = $("scenario").value === "timeout";
  $("timeout").value = timeout ? 2 : 120;
  $("timeout").readOnly = timeout;
});
document.querySelectorAll('input[name="mode"], .color-row select').forEach(node => node.addEventListener("change", updateForm));
$("event-filter").addEventListener("change", renderEvents);
$("close-record").addEventListener("click", () => {$("record-detail").hidden = true;});

function navigate() {
  const current = ["console", "records", "connections"].includes(location.hash.slice(1)) ? location.hash.slice(1) : "console";
  for (const view of ["console", "records", "connections"]) $("view-" + view).hidden = view !== current;
  document.querySelectorAll("[data-view]").forEach(node => {
    const selected = node.dataset.view === current;
    node.classList.toggle("active", selected);
    if (selected) node.setAttribute("aria-current", "page"); else node.removeAttribute("aria-current");
  });
  $("page-title").textContent = {console: "任务操作台", records: "任务记录", connections: "接口状态"}[current];
}
window.addEventListener("hashchange", navigate);
navigate();
updateForm();
refresh();
setInterval(() => { renderHealth(); refresh(); }, 750);
