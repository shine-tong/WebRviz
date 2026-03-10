import "./style.css";
import { RuntimeConfig, loadConfig, saveConfig } from "./config";
import { decodePointCloud2 } from "./ros/pointcloud";
import { RosClient, TopicInfo } from "./ros/rosClient";
import { loadRvizSyncHints } from "./rviz/rvizConfig";
import { loadRobotModel } from "./visualization/robotLoader";
import { SceneManager } from "./visualization/sceneManager";

interface AppElements {
  appRoot: HTMLElement;
  rosbridgeUrl: HTMLInputElement;
  rvizConfigPath: HTMLInputElement;
  urdfFallbackPath: HTMLInputElement;
  packageRootUrl: HTMLInputElement;
  pointCloudTopic: HTMLSelectElement;
  connectBtn: HTMLButtonElement;
  syncBtn: HTMLButtonElement;
  sidebarToggleBtn: HTMLButtonElement;
  rightSidebarToggleBtn: HTMLButtonElement;
  fixedFrameValue: HTMLElement;
  connectionStatus: HTMLElement;
  logBox: HTMLElement;
  viewport: HTMLElement;
  jointValues: HTMLElement;
  cartesianFrame: HTMLSelectElement;
  cartesianValues: HTMLElement;
  tfTree: HTMLElement;
  trajRecordBtn: HTMLButtonElement;
  trajPlayBtn: HTMLButtonElement;
  trajPauseBtn: HTMLButtonElement;
  trajClearBtn: HTMLButtonElement;
  trajProgress: HTMLInputElement;
  trajInfo: HTMLElement;
  trajTime: HTMLElement;
}

interface TopicSubscriptions {
  jointStates?: any;
  tf?: any;
  tfStatic?: any;
  pointCloud?: any;
}

interface JointSnapshot {
  name: string;
  position: number;
}

interface TrajectoryFrame {
  t: number;
  names: string[];
  positions: number[];
}

const POINTCLOUD2_TYPE = "sensor_msgs/PointCloud2";
const SIDEBAR_STATE_KEY = "webrviz-sidebar-collapsed";
const RIGHT_SIDEBAR_STATE_KEY = "webrviz-right-sidebar-collapsed";
const TRAJ_SAMPLE_INTERVAL_MS = 50;
const TRAJ_MOTION_THRESHOLD = 0.001;
const TRAJ_IDLE_STOP_MS = 800;

function getElements(): AppElements {
  const byId = <T extends HTMLElement>(id: string): T => {
    const element = document.getElementById(id);
    if (!element) {
      throw new Error(`missing element: ${id}`);
    }
    return element as T;
  };

  return {
    appRoot: byId<HTMLElement>("app"),
    rosbridgeUrl: byId<HTMLInputElement>("rosbridgeUrl"),
    rvizConfigPath: byId<HTMLInputElement>("rvizConfigPath"),
    urdfFallbackPath: byId<HTMLInputElement>("urdfFallbackPath"),
    packageRootUrl: byId<HTMLInputElement>("packageRootUrl"),
    pointCloudTopic: byId<HTMLSelectElement>("pointCloudTopic"),
    connectBtn: byId<HTMLButtonElement>("connectBtn"),
    syncBtn: byId<HTMLButtonElement>("syncBtn"),
    sidebarToggleBtn: byId<HTMLButtonElement>("sidebarToggleBtn"),
    rightSidebarToggleBtn: byId<HTMLButtonElement>("rightSidebarToggleBtn"),
    fixedFrameValue: byId<HTMLElement>("fixedFrameValue"),
    connectionStatus: byId<HTMLElement>("connectionStatus"),
    logBox: byId<HTMLElement>("logBox"),
    viewport: byId<HTMLElement>("viewport"),
    jointValues: byId<HTMLElement>("jointValues"),
    cartesianFrame: byId<HTMLSelectElement>("cartesianFrame"),
    cartesianValues: byId<HTMLElement>("cartesianValues"),
    tfTree: byId<HTMLElement>("tfTree"),
    trajRecordBtn: byId<HTMLButtonElement>("trajRecordBtn"),
    trajPlayBtn: byId<HTMLButtonElement>("trajPlayBtn"),
    trajPauseBtn: byId<HTMLButtonElement>("trajPauseBtn"),
    trajClearBtn: byId<HTMLButtonElement>("trajClearBtn"),
    trajProgress: byId<HTMLInputElement>("trajProgress"),
    trajInfo: byId<HTMLElement>("trajInfo"),
    trajTime: byId<HTMLElement>("trajTime")
  };
}

const elements = getElements();
let config: RuntimeConfig = loadConfig();

const rosClient = new RosClient();
const sceneManager = new SceneManager(elements.viewport, config.targetFps);
sceneManager.setFixedFrame(config.fixedFrame);

let topics: TopicInfo[] = [];
let subscriptions: TopicSubscriptions = {};
let lastPointCloudTimestamp = 0;
let sidebarCollapsed = window.localStorage.getItem(SIDEBAR_STATE_KEY) === "1";
let rightSidebarCollapsed = window.localStorage.getItem(RIGHT_SIDEBAR_STATE_KEY) !== "0";
let jointState: JointSnapshot[] = [];
let cartesianFrame = "";
let lastFrameOptions: string[] = [];
let pendingRightPanelUpdate = false;
let trajectory: TrajectoryFrame[] = [];
let isRecording = false;
let isPlaying = false;
let isPlaybackPaused = false;
let recordStartTime = 0;
let lastRecordTime = 0;
let playbackIndex = 0;
let playbackStartTime = 0;
let playbackFrameHandle: number | null = null;
let recordArmed = false;
let recordArmBaseline: Map<string, number> | null = null;
let recordLastMotionTime = 0;
let recordLastPositions: Map<string, number> | null = null;

function formatError(error: unknown): string {
  if (error instanceof Error) {
    return error.message;
  }
  return String(error);
}

function log(message: string): void {
  const now = new Date().toLocaleTimeString();
  const line = `[${now}] ${message}`;
  const existing = elements.logBox.textContent ? `${elements.logBox.textContent}\n` : "";
  const lines = `${existing}${line}`.split("\n");
  elements.logBox.textContent = lines.slice(-120).join("\n");
  elements.logBox.scrollTop = elements.logBox.scrollHeight;
}

function updateStatusLabel(state: string, detail?: string): void {
  elements.connectionStatus.textContent = detail ? `${state} (${detail})` : state;
}

function setSidebarCollapsed(collapsed: boolean): void {
  sidebarCollapsed = collapsed;
  elements.appRoot.classList.toggle("sidebar-collapsed", sidebarCollapsed);
  elements.sidebarToggleBtn.textContent = sidebarCollapsed ? "Show Sidebar" : "Hide Sidebar";
  window.localStorage.setItem(SIDEBAR_STATE_KEY, sidebarCollapsed ? "1" : "0");

  // Trigger canvas resize after layout change.
  window.requestAnimationFrame(() => {
    window.dispatchEvent(new Event("resize"));
  });
}

function setRightSidebarCollapsed(collapsed: boolean): void {
  rightSidebarCollapsed = collapsed;
  elements.appRoot.classList.toggle("right-sidebar-collapsed", rightSidebarCollapsed);
  elements.rightSidebarToggleBtn.textContent = rightSidebarCollapsed ? "<" : ">";
  elements.rightSidebarToggleBtn.setAttribute(
    "aria-label",
    rightSidebarCollapsed ? "Show right sidebar" : "Hide right sidebar"
  );
  window.localStorage.setItem(RIGHT_SIDEBAR_STATE_KEY, rightSidebarCollapsed ? "1" : "0");

  window.requestAnimationFrame(() => {
    window.dispatchEvent(new Event("resize"));
  });
}

function formatNumber(value: number, digits = 3): string {
  if (!Number.isFinite(value)) {
    return "--";
  }
  return value.toFixed(digits);
}

function formatAngleRadians(value: number): string {
  return `${formatNumber(value, 4)} rad`;
}

function formatAngleDegrees(value: number): string {
  return `${formatNumber(value, 1)} deg`;
}

function addDataRow(container: HTMLElement, label: string, value: string): void {
  const row = document.createElement("div");
  row.className = "data-row";
  const labelSpan = document.createElement("span");
  labelSpan.textContent = label;
  const valueStrong = document.createElement("strong");
  valueStrong.textContent = value;
  row.appendChild(labelSpan);
  row.appendChild(valueStrong);
  container.appendChild(row);
}

function renderPlaceholder(container: HTMLElement, label: string): void {
  container.innerHTML = "";
  addDataRow(container, label, "--");
}

function quatToEuler(rotation: { x: number; y: number; z: number; w: number }): {
  roll: number;
  pitch: number;
  yaw: number;
} {
  const { x, y, z, w } = rotation;
  const sinrCosp = 2 * (w * x + y * z);
  const cosrCosp = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinrCosp, cosrCosp);

  const sinp = 2 * (w * y - z * x);
  let pitch = 0;
  if (Math.abs(sinp) >= 1) {
    pitch = Math.sign(sinp) * (Math.PI / 2);
  } else {
    pitch = Math.asin(sinp);
  }

  const sinyCosp = 2 * (w * z + x * y);
  const cosyCosp = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(sinyCosp, cosyCosp);

  return { roll, pitch, yaw };
}

function renderJointValues(): void {
  if (jointState.length === 0) {
    renderPlaceholder(elements.jointValues, "no joint data");
    return;
  }

  elements.jointValues.innerHTML = "";
  for (const joint of jointState) {
    addDataRow(elements.jointValues, joint.name, formatAngleRadians(joint.position));
  }
}

function updateFrameOptions(): void {
  const frames = sceneManager.getLinkList();
  if (frames.length === 0) {
    elements.cartesianFrame.innerHTML = "";
    cartesianFrame = "";
    lastFrameOptions = [];
    return;
  }

  const changed =
    frames.length !== lastFrameOptions.length ||
    frames.some((frame, index) => frame !== lastFrameOptions[index]);

  if (changed) {
    elements.cartesianFrame.innerHTML = "";
    for (const frame of frames) {
      const option = document.createElement("option");
      option.value = frame;
      option.textContent = frame;
      elements.cartesianFrame.appendChild(option);
    }
    lastFrameOptions = frames;
  }

  const snapshot = sceneManager.getTfSnapshot();
  const tfOrder = getTfTreeOrder(snapshot);
  let lastTfLink = "";
  for (let index = tfOrder.length - 1; index >= 0; index -= 1) {
    const frame = tfOrder[index];
    if (frames.includes(frame)) {
      lastTfLink = frame;
      break;
    }
  }

  const preferred =
    cartesianFrame ||
    lastTfLink ||
    sceneManager.getDefaultEndEffectorFrame() ||
    sceneManager.getRobotBaseFrame() ||
    sceneManager.getFixedFrame() ||
    frames[0];
  cartesianFrame = frames.includes(preferred) ? preferred : frames[0];
  elements.cartesianFrame.value = cartesianFrame;
}
function renderCartesianValues(): void {
  if (!cartesianFrame) {
    renderPlaceholder(elements.cartesianValues, "no frame");
    return;
  }

  const transform = sceneManager.getRelativeTransform(cartesianFrame);
  if (!transform) {
    renderPlaceholder(elements.cartesianValues, "unavailable");
    return;
  }

  const position = transform.translation;
  const rotation = transform.rotation;
  const euler = quatToEuler(rotation);

  elements.cartesianValues.innerHTML = "";
  addDataRow(elements.cartesianValues, "X (m)", formatNumber(position.x));
  addDataRow(elements.cartesianValues, "Y (m)", formatNumber(position.y));
  addDataRow(elements.cartesianValues, "Z (m)", formatNumber(position.z));
  addDataRow(elements.cartesianValues, "Roll (deg)", formatAngleDegrees((euler.roll * 180) / Math.PI));
  addDataRow(elements.cartesianValues, "Pitch (deg)", formatAngleDegrees((euler.pitch * 180) / Math.PI));
  addDataRow(elements.cartesianValues, "Yaw (deg)", formatAngleDegrees((euler.yaw * 180) / Math.PI));
}

function buildTfTreeLines(snapshot: Array<{ parent: string; child: string }>): string[] {
  if (snapshot.length === 0) {
    return [];
  }

  const childrenByParent = new Map<string, string[]>();
  const nodes = new Set<string>();
  const childrenSet = new Set<string>();

  for (const edge of snapshot) {
    if (!edge.parent || !edge.child) {
      continue;
    }
    nodes.add(edge.parent);
    nodes.add(edge.child);
    childrenSet.add(edge.child);

    if (!childrenByParent.has(edge.parent)) {
      childrenByParent.set(edge.parent, []);
    }
    childrenByParent.get(edge.parent)!.push(edge.child);
  }

  if (nodes.size === 0) {
    return [];
  }

  for (const [parent, children] of childrenByParent.entries()) {
    children.sort((left, right) => left.localeCompare(right));
    childrenByParent.set(parent, children);
  }

  let roots = Array.from(nodes).filter((node) => !childrenSet.has(node));
  if (roots.length === 0) {
    roots = Array.from(childrenByParent.keys());
  }
  roots.sort((left, right) => left.localeCompare(right));

  const fixedFrame = sceneManager.getFixedFrame();
  const selectedFrame = cartesianFrame;
  const lines: string[] = [];
  const stack = new Set<string>();
  const maxLines = 400;

  const formatLabel = (frame: string): string => {
    let label = frame;
    if (frame === fixedFrame) {
      label += " [fixed]";
    }
    if (frame === selectedFrame) {
      label += " [selected]";
    }
    return label;
  };

  const renderNode = (node: string, depth: number): void => {
    if (lines.length >= maxLines) {
      return;
    }

    lines.push(`${"  ".repeat(depth)}${formatLabel(node)}`);

    if (stack.has(node)) {
      lines.push(`${"  ".repeat(depth + 1)}(loop)`);
      return;
    }

    const children = childrenByParent.get(node);
    if (!children || children.length === 0) {
      return;
    }

    stack.add(node);
    for (const child of children) {
      if (lines.length >= maxLines) {
        break;
      }
      renderNode(child, depth + 1);
    }
    stack.delete(node);
  };

  for (const root of roots) {
    if (lines.length >= maxLines) {
      break;
    }
    renderNode(root, 0);
  }

  if (lines.length >= maxLines) {
    lines.push("...");
  }

  return lines;
}

function getTfTreeOrder(snapshot: Array<{ parent: string; child: string }>): string[] {
  if (snapshot.length === 0) {
    return [];
  }

  const childrenByParent = new Map<string, string[]>();
  const nodes = new Set<string>();
  const childrenSet = new Set<string>();

  for (const edge of snapshot) {
    if (!edge.parent || !edge.child) {
      continue;
    }
    nodes.add(edge.parent);
    nodes.add(edge.child);
    childrenSet.add(edge.child);

    if (!childrenByParent.has(edge.parent)) {
      childrenByParent.set(edge.parent, []);
    }
    childrenByParent.get(edge.parent)!.push(edge.child);
  }

  if (nodes.size === 0) {
    return [];
  }

  for (const [parent, children] of childrenByParent.entries()) {
    children.sort((left, right) => left.localeCompare(right));
    childrenByParent.set(parent, children);
  }

  let roots = Array.from(nodes).filter((node) => !childrenSet.has(node));
  if (roots.length === 0) {
    roots = Array.from(childrenByParent.keys());
  }
  roots.sort((left, right) => left.localeCompare(right));

  const order: string[] = [];
  const stack = new Set<string>();
  const maxNodes = 400;

  const visitNode = (node: string): void => {
    if (order.length >= maxNodes) {
      return;
    }

    order.push(node);

    if (stack.has(node)) {
      return;
    }

    const children = childrenByParent.get(node);
    if (!children || children.length === 0) {
      return;
    }

    stack.add(node);
    for (const child of children) {
      if (order.length >= maxNodes) {
        break;
      }
      visitNode(child);
    }
    stack.delete(node);
  };

  for (const root of roots) {
    if (order.length >= maxNodes) {
      break;
    }
    visitNode(root);
  }

  return order;
}

function renderTfTree(): void {
  const snapshot = sceneManager.getTfSnapshot();
  const lines = buildTfTreeLines(snapshot);
  if (lines.length === 0) {
    elements.tfTree.textContent = "no tf data";
    return;
  }

  elements.tfTree.textContent = lines.join("\n");
}

function getTrajectoryDurationMs(): number {
  if (trajectory.length === 0) {
    return 0;
  }
  return trajectory[trajectory.length - 1].t;
}

function getTrajectoryProgressMs(): number {
  if (trajectory.length === 0) {
    return 0;
  }
  const index = Math.max(0, Math.min(playbackIndex, trajectory.length - 1));
  return trajectory[index].t;
}

function updateTrajectoryUi(): void {
  const durationMs = getTrajectoryDurationMs();
  const progressMs = getTrajectoryProgressMs();
  const progress = durationMs > 0 ? Math.min(100, Math.max(0, Math.round((progressMs / durationMs) * 100))) : 0;
  elements.trajProgress.value = String(progress);

  if (isRecording && recordArmed) {
    elements.trajInfo.textContent = "armed";
  } else {
    elements.trajInfo.textContent = `${trajectory.length} frame${trajectory.length === 1 ? "" : "s"}`;
  }
  elements.trajTime.textContent = `${(durationMs / 1000).toFixed(1)}s`;

  elements.trajPlayBtn.disabled = trajectory.length === 0 || isPlaying;
  elements.trajPauseBtn.disabled = !isPlaying;
  elements.trajClearBtn.disabled = trajectory.length === 0 && !isRecording && !isPlaying;
  elements.trajRecordBtn.disabled = isPlaying;
  elements.trajRecordBtn.textContent = isRecording ? "Stop" : "Record";
  elements.cartesianFrame.disabled = isPlaying;
}
function applyJointSnapshot(names: string[], positions: number[]): void {
  const payload = { name: names, position: positions };
  sceneManager.updateJointStates(payload);

  const next: JointSnapshot[] = [];
  const count = Math.min(names.length, positions.length);
  for (let index = 0; index < count; index += 1) {
    next.push({ name: names[index], position: positions[index] });
  }
  jointState = next;
}

function setPlaybackIndex(nextIndex: number): void {
  if (trajectory.length === 0) {
    playbackIndex = 0;
    updateTrajectoryUi();
    return;
  }

  playbackIndex = Math.max(0, Math.min(nextIndex, trajectory.length - 1));
  const frame = trajectory[playbackIndex];
  applyJointSnapshot(frame.names, frame.positions);
  scheduleRightPanelUpdate();
  updateTrajectoryUi();
}

function stopPlayback(): void {
  isPlaying = false;
  isPlaybackPaused = false;
  sceneManager.setShowOnlyEndEffector(false);
  if (playbackFrameHandle !== null) {
    window.cancelAnimationFrame(playbackFrameHandle);
    playbackFrameHandle = null;
  }
  updateTrajectoryUi();
}

function pausePlayback(): void {
  if (!isPlaying && isPlaybackPaused) {
    return;
  }
  isPlaying = false;
  isPlaybackPaused = true;
  if (playbackFrameHandle !== null) {
    window.cancelAnimationFrame(playbackFrameHandle);
    playbackFrameHandle = null;
  }
  updateTrajectoryUi();
}
function playbackLoop(): void {
  if (!isPlaying) {
    return;
  }

  const durationMs = getTrajectoryDurationMs();
  if (durationMs <= 0) {
    stopPlayback();
    return;
  }

  const elapsed = performance.now() - playbackStartTime;
  while (playbackIndex < trajectory.length - 1 && trajectory[playbackIndex + 1].t <= elapsed) {
    playbackIndex += 1;
  }

  const frame = trajectory[playbackIndex];
  applyJointSnapshot(frame.names, frame.positions);
  scheduleRightPanelUpdate();
  updateTrajectoryUi();

  if (elapsed >= durationMs) {
    stopPlayback();
    return;
  }

  playbackFrameHandle = window.requestAnimationFrame(playbackLoop);
}

function startPlayback(): void {
  if (trajectory.length === 0) {
    return;
  }

  if (playbackIndex >= trajectory.length - 1) {
    playbackIndex = 0;
  }

  isRecording = false;
  recordArmed = false;
  isPlaying = true;
  isPlaybackPaused = false;

  const endEffectorFrame = sceneManager.getDefaultEndEffectorFrame();
  if (endEffectorFrame) {
    cartesianFrame = endEffectorFrame;
    elements.cartesianFrame.value = cartesianFrame;
  }
  sceneManager.setEndEffectorFrame(cartesianFrame || endEffectorFrame);
  sceneManager.setShowOnlyEndEffector(true);

  playbackStartTime = performance.now() - trajectory[playbackIndex].t;
  updateTrajectoryUi();
  playbackLoop();
}
function clearTrajectory(): void {
  stopPlayback();
  isRecording = false;
  isPlaybackPaused = false;
  recordArmed = false;
  recordArmBaseline = null;
  recordLastMotionTime = 0;
  recordLastPositions = null;
  trajectory = [];
  playbackIndex = 0;
  recordStartTime = 0;
  lastRecordTime = 0;
  updateTrajectoryUi();
}

function recordTrajectorySnapshot(message: unknown): void {
  if (!isRecording || isPlaying) {
    return;
  }

  const payload = message as { name?: string[]; position?: number[] };
  if (!payload || !Array.isArray(payload.name) || !Array.isArray(payload.position)) {
    return;
  }

  const names = payload.name;
  const positions = payload.position;

  if (recordArmed) {
    if (!recordArmBaseline) {
      recordArmBaseline = new Map<string, number>();
      const count = Math.min(names.length, positions.length);
      for (let index = 0; index < count; index += 1) {
        recordArmBaseline.set(names[index], positions[index]);
      }
      return;
    }

    let moved = false;
    let compared = 0;
    const count = Math.min(names.length, positions.length);
    for (let index = 0; index < count; index += 1) {
      const baseline = recordArmBaseline.get(names[index]);
      if (baseline === undefined) {
        continue;
      }
      compared += 1;
      if (Math.abs(positions[index] - baseline) >= TRAJ_MOTION_THRESHOLD) {
        moved = true;
        break;
      }
    }

    if (!moved || compared === 0) {
      return;
    }

    recordArmed = false;
    recordStartTime = performance.now();
    lastRecordTime = -TRAJ_SAMPLE_INTERVAL_MS;
    trajectory = [];
    playbackIndex = 0;
  }

  const now = performance.now();
  if (trajectory.length === 0 && recordStartTime === 0) {
    recordStartTime = now;
  }

  const previousPositions = recordLastPositions;
  const nextPositions = new Map<string, number>();
  let moved = false;
  let compared = 0;
  const count = Math.min(names.length, positions.length);
  for (let index = 0; index < count; index += 1) {
    const name = names[index];
    const position = positions[index];
    nextPositions.set(name, position);
    const previous = previousPositions?.get(name);
    if (previous === undefined) {
      continue;
    }
    compared += 1;
    if (Math.abs(position - previous) >= TRAJ_MOTION_THRESHOLD) {
      moved = true;
    }
  }

  if (!previousPositions || compared === 0) {
    moved = true;
  }

  if (moved) {
    recordLastMotionTime = now;
  } else if (recordLastMotionTime > 0 && now - recordLastMotionTime >= TRAJ_IDLE_STOP_MS) {
    isRecording = false;
    recordArmed = false;
    recordArmBaseline = null;
    recordLastMotionTime = 0;
    recordLastPositions = null;
    updateTrajectoryUi();
    return;
  }

  recordLastPositions = nextPositions;

  const elapsed = now - recordStartTime;
  if (elapsed - lastRecordTime < TRAJ_SAMPLE_INTERVAL_MS && trajectory.length > 0) {
    return;
  }

  lastRecordTime = elapsed;
  trajectory.push({
    t: elapsed,
    names: names.slice(),
    positions: positions.slice()
  });
  updateTrajectoryUi();
}
function renderRightPanel(): void {
  renderJointValues();
  updateFrameOptions();
  renderCartesianValues();
  renderTfTree();
  updateTrajectoryUi();
}

function scheduleRightPanelUpdate(): void {
  if (pendingRightPanelUpdate) {
    return;
  }
  pendingRightPanelUpdate = true;
  window.requestAnimationFrame(() => {
    pendingRightPanelUpdate = false;
    renderRightPanel();
  });
}

function clearRightPanelState(): void {
  jointState = [];
  cartesianFrame = "";
  lastFrameOptions = [];
  elements.cartesianFrame.innerHTML = "";
  renderRightPanel();
}

function updateJointStateSnapshot(message: unknown): void {
  const payload = message as { name?: string[]; position?: number[] };
  if (!payload || !Array.isArray(payload.name) || !Array.isArray(payload.position)) {
    return;
  }

  const count = Math.min(payload.name.length, payload.position.length);
  const next: JointSnapshot[] = [];
  for (let index = 0; index < count; index += 1) {
    next.push({ name: payload.name[index], position: payload.position[index] });
  }
  jointState = next;
}

function renderConfigToUi(): void {
  elements.rosbridgeUrl.value = config.rosbridgeUrl;
  elements.rvizConfigPath.value = config.rvizConfigPath;
  elements.urdfFallbackPath.value = config.urdfFallbackPath;
  elements.packageRootUrl.value = config.packageRootUrl;
  elements.fixedFrameValue.textContent = config.fixedFrame;
}

function syncConfigFromUi(): void {
  config.rosbridgeUrl = elements.rosbridgeUrl.value.trim() || config.rosbridgeUrl;
  config.rvizConfigPath = elements.rvizConfigPath.value.trim() || config.rvizConfigPath;
  config.urdfFallbackPath = elements.urdfFallbackPath.value.trim() || config.urdfFallbackPath;
  config.packageRootUrl = elements.packageRootUrl.value.trim() || config.packageRootUrl;
  saveConfig(config);
}

function unsubscribe(topic?: any): void {
  if (!topic) {
    return;
  }

  try {
    topic.unsubscribe();
  } catch {
    // ignore unsubscribe errors
  }
}

function clearSubscriptions(): void {
  unsubscribe(subscriptions.jointStates);
  unsubscribe(subscriptions.tf);
  unsubscribe(subscriptions.tfStatic);
  unsubscribe(subscriptions.pointCloud);
  subscriptions = {};
}

function pointCloudTopicNames(sourceTopics: TopicInfo[]): string[] {
  return sourceTopics
    .filter((topic) => topic.type === POINTCLOUD2_TYPE)
    .map((topic) => topic.name)
    .sort((left, right) => left.localeCompare(right));
}

function choosePointCloudTopic(candidates: string[], availableTopics: string[]): string {
  for (const candidate of candidates) {
    if (candidate && availableTopics.includes(candidate)) {
      return candidate;
    }
  }

  if (availableTopics.length > 0) {
    return availableTopics[0];
  }

  return candidates[0] || "/pointcloud/output";
}

function updatePointCloudOptions(availableTopics: string[], preferredTopic?: string): void {
  const uniqueTopics = new Set<string>(availableTopics);
  if (preferredTopic) {
    uniqueTopics.add(preferredTopic);
  }
  uniqueTopics.add(config.defaultPointCloudTopic);

  elements.pointCloudTopic.innerHTML = "";
  for (const topicName of Array.from(uniqueTopics).sort((left, right) => left.localeCompare(right))) {
    const option = document.createElement("option");
    option.value = topicName;
    option.textContent = topicName;
    elements.pointCloudTopic.appendChild(option);
  }

  const selected = preferredTopic || config.defaultPointCloudTopic;
  elements.pointCloudTopic.value = selected;
}

async function refreshTopicCache(): Promise<void> {
  topics = await rosClient.listTopicsWithTypes();
  const cloudTopics = pointCloudTopicNames(topics);
  updatePointCloudOptions(cloudTopics, elements.pointCloudTopic.value || config.defaultPointCloudTopic);
  log(`topic discovery finished: ${topics.length} topics, ${cloudTopics.length} pointcloud topics`);
}

async function subscribeCoreTopics(): Promise<void> {
  unsubscribe(subscriptions.jointStates);
  unsubscribe(subscriptions.tf);
  unsubscribe(subscriptions.tfStatic);

  subscriptions.jointStates = rosClient.createTopic("/joint_states", "sensor_msgs/JointState");
  subscriptions.jointStates.subscribe((message: unknown) => {
    if (isPlaying || isPlaybackPaused) {
      return;
    }
    sceneManager.updateJointStates(message);
    updateJointStateSnapshot(message);
    recordTrajectorySnapshot(message);
    scheduleRightPanelUpdate();
  });

  subscriptions.tf = rosClient.createTopic("/tf", "tf2_msgs/TFMessage");
  subscriptions.tf.subscribe((message: unknown) => {
    sceneManager.upsertTfMessage(message);
    scheduleRightPanelUpdate();
  });

  subscriptions.tfStatic = rosClient.createTopic("/tf_static", "tf2_msgs/TFMessage");
  subscriptions.tfStatic.subscribe((message: unknown) => {
    sceneManager.upsertTfMessage(message);
    scheduleRightPanelUpdate();
  });

  log("subscribed core topics: /joint_states, /tf, /tf_static");
}

async function subscribePointCloud(topicName: string): Promise<void> {
  unsubscribe(subscriptions.pointCloud);

  if (!topicName) {
    sceneManager.setPointCloud(null);
    return;
  }

  const throttleRate = Math.floor(1000 / Math.max(1, config.targetFps));
  subscriptions.pointCloud = rosClient.createTopic(topicName, POINTCLOUD2_TYPE, throttleRate);

  subscriptions.pointCloud.subscribe((message: unknown) => {
    const now = Date.now();
    if (now - lastPointCloudTimestamp < throttleRate) {
      return;
    }
    lastPointCloudTimestamp = now;

    const parsed = decodePointCloud2(message, config.maxPoints);
    sceneManager.setPointCloud(parsed);
  });

  log(`subscribed pointcloud topic: ${topicName}`);
}

async function loadRobotIntoScene(): Promise<void> {
  const loaded = await loadRobotModel(rosClient, config.urdfFallbackPath, config.packageRootUrl);
  sceneManager.setRobot(loaded.robot, loaded.rootLink);
  log(`robot model loaded (root=${loaded.rootLink})`);
}

async function connect(): Promise<void> {
  syncConfigFromUi();
  log(`connecting ROS bridge: ${config.rosbridgeUrl}`);

  await rosClient.connect(config.rosbridgeUrl);
  log("rosbridge connected");

  try {
    await loadRobotIntoScene();
  } catch (error) {
    log(`robot load warning: ${formatError(error)}`);
  }

  await refreshTopicCache();
  await subscribeCoreTopics();

  const cloudTopics = pointCloudTopicNames(topics);
  const defaultTopic = choosePointCloudTopic([config.defaultPointCloudTopic], cloudTopics);
  config.defaultPointCloudTopic = defaultTopic;
  saveConfig(config);
  updatePointCloudOptions(cloudTopics, defaultTopic);

  try {
    await subscribePointCloud(defaultTopic);
  } catch (error) {
    log(`pointcloud subscribe warning: ${formatError(error)}`);
  }

  log("initial sync complete");
}

function disconnect(): void {
  clearSubscriptions();
  rosClient.disconnect();
  sceneManager.setPointCloud(null);
  sceneManager.setRobot(null);
  sceneManager.clearTfRecords();
  stopPlayback();
  isRecording = false;
  isPlaybackPaused = false;
  recordArmed = false;
  recordArmBaseline = null;
  recordLastMotionTime = 0;
  recordLastPositions = null;
  clearTrajectory();
  clearRightPanelState();
  log("disconnected");
}

async function syncFromRviz(): Promise<void> {
  if (!rosClient.isConnected()) {
    throw new Error("ROS is not connected");
  }

  syncConfigFromUi();

  let fixedFrame = config.fixedFrame;
  let rvizPointCloudTopics: string[] = [];

  try {
    const rvizHints = await loadRvizSyncHints(config.rvizConfigPath);
    if (rvizHints.fixedFrame) {
      fixedFrame = rvizHints.fixedFrame;
    }
    rvizPointCloudTopics = rvizHints.pointCloudTopics;
    log(`loaded RViz config: ${config.rvizConfigPath}`);
  } catch (error) {
    log(`RViz config load warning: ${formatError(error)}`);
  }

  await refreshTopicCache();

  const availableCloudTopics = pointCloudTopicNames(topics);
  const desiredTopic = choosePointCloudTopic(
    [...rvizPointCloudTopics, config.defaultPointCloudTopic, "/pointcloud/output"],
    availableCloudTopics
  );

  config.fixedFrame = fixedFrame;
  config.defaultPointCloudTopic = desiredTopic;
  saveConfig(config);

  sceneManager.setFixedFrame(config.fixedFrame);
  elements.fixedFrameValue.textContent = config.fixedFrame;

  updatePointCloudOptions(availableCloudTopics, desiredTopic);
  await subscribePointCloud(desiredTopic);

  scheduleRightPanelUpdate();
  log(`sync applied: fixed_frame=${config.fixedFrame}, pointcloud=${desiredTopic}`);
}

renderConfigToUi();
updatePointCloudOptions([], config.defaultPointCloudTopic);
updateStatusLabel("disconnected");
setSidebarCollapsed(sidebarCollapsed);
setRightSidebarCollapsed(rightSidebarCollapsed);
renderRightPanel();
updateTrajectoryUi();

rosClient.onStateChange((state, detail) => {
  updateStatusLabel(state, detail);
  elements.connectBtn.textContent = state === "connected" ? "Disconnect" : "Connect";
});

elements.connectBtn.addEventListener("click", async () => {
  try {
    if (rosClient.isConnected()) {
      disconnect();
      return;
    }

    await connect();
  } catch (error) {
    log(`connect failed: ${formatError(error)}`);
    if (rosClient.isConnected()) {
      disconnect();
    }
  }
});

elements.syncBtn.addEventListener("click", async () => {
  try {
    await syncFromRviz();
  } catch (error) {
    log(`sync failed: ${formatError(error)}`);
  }
});

elements.sidebarToggleBtn.addEventListener("click", () => {
  setSidebarCollapsed(!sidebarCollapsed);
});

elements.rightSidebarToggleBtn.addEventListener("click", () => {
  setRightSidebarCollapsed(!rightSidebarCollapsed);
});

elements.cartesianFrame.addEventListener("change", () => {
  if (isPlaying) {
    elements.cartesianFrame.value = cartesianFrame;
    return;
  }
  cartesianFrame = elements.cartesianFrame.value;
  scheduleRightPanelUpdate();
});

elements.trajRecordBtn.addEventListener("click", () => {
  if (isRecording) {
    isRecording = false;
    recordArmed = false;
    recordArmBaseline = null;
    recordLastMotionTime = 0;
    recordLastPositions = null;
    updateTrajectoryUi();
    return;
  }

  stopPlayback();
  isRecording = true;
  recordArmed = true;
  recordLastMotionTime = 0;
  recordLastPositions = null;
  trajectory = [];
  playbackIndex = 0;
  recordStartTime = 0;
  lastRecordTime = 0;
  if (jointState.length > 0) {
    recordArmBaseline = new Map<string, number>();
    for (const joint of jointState) {
      recordArmBaseline.set(joint.name, joint.position);
    }
  } else {
    recordArmBaseline = null;
  }
  updateTrajectoryUi();
});
elements.trajPlayBtn.addEventListener("click", () => {
  if (isPlaying || trajectory.length === 0) {
    return;
  }
  startPlayback();
});

elements.trajPauseBtn.addEventListener("click", () => {
  pausePlayback();
});

elements.trajClearBtn.addEventListener("click", () => {
  clearTrajectory();
});

elements.trajProgress.addEventListener("input", () => {
  if (trajectory.length === 0) {
    elements.trajProgress.value = "0";
    return;
  }

  const durationMs = getTrajectoryDurationMs();
  const percent = Number(elements.trajProgress.value) / 100;
  const targetTime = durationMs * percent;

  let index = 0;
  while (index < trajectory.length - 1 && trajectory[index].t < targetTime) {
    index += 1;
  }

  pausePlayback();
  setPlaybackIndex(index);
});

elements.pointCloudTopic.addEventListener("change", async () => {
  const topicName = elements.pointCloudTopic.value;
  config.defaultPointCloudTopic = topicName;
  saveConfig(config);

  if (!rosClient.isConnected()) {
    return;
  }

  try {
    await subscribePointCloud(topicName);
  } catch (error) {
    log(`pointcloud subscribe failed: ${formatError(error)}`);
  }
});

window.addEventListener("beforeunload", () => {
  clearSubscriptions();
  rosClient.disconnect();
  sceneManager.dispose();
});

log("WebRviz ready. Click Connect to start.");
















