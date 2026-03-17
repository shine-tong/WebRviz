import "./style.css";
import { Vector3 } from "three";
import { RuntimeConfig, loadConfig, saveConfig } from "./config";
import { decodePointCloud2 } from "./ros/pointcloud";
import { ConnectionState, MessageDetailsResponse, MessageTypeDef, RosClient, ServiceInfo, TopicInfo } from "./ros/rosClient";
import { loadRvizSyncHints } from "./rviz/rvizConfig";
import { loadRobotModel } from "./visualization/robotLoader";
import { RobotJointConnection, RobotJointDetail, RobotLinkDetail, SceneManager } from "./visualization/sceneManager";
import { applyStaticTranslations, applyTheme, Language, loadLanguage, loadTheme, saveLanguage, saveTheme, t, ThemeMode } from "./uiSettings";
import { enhanceSelect, refreshEnhancedSelect } from "./customSelect";

interface AppElements {
  appRoot: HTMLElement;
  rosbridgeUrl: HTMLInputElement;
  rvizConfigPath: HTMLInputElement;
  urdfFallbackPath: HTMLInputElement;
  packageRootUrl: HTMLInputElement;
  pointCloudTopic: HTMLSelectElement;
  languageToggleBtn: HTMLButtonElement;
  themeToggleBtn: HTMLButtonElement;
  connectBtn: HTMLButtonElement;
  syncBtn: HTMLButtonElement;
  sidebarToggleBtn: HTMLButtonElement;
  plannedTrajectoryToggleBtn: HTMLButtonElement;
  rightSidebarToggleBtn: HTMLButtonElement;
  fixedFrameValue: HTMLElement;
  connectionStatus: HTMLElement;
  logBox: HTMLElement;
  viewport: HTMLElement;
  jointValues: HTMLElement;
  jointAngleUnit: HTMLSelectElement;
  cartesianFrame: HTMLSelectElement;
  cartesianLengthUnit: HTMLSelectElement;
  cartesianAngleUnit: HTMLSelectElement;
  cartesianValues: HTMLElement;
  tfToggleAllBtn: HTMLButtonElement;
  tfSelectBtn: HTMLButtonElement;
  tfFilterMenu: HTMLElement;
  tfTree: HTMLElement;
  trajRecordBtn: HTMLButtonElement;
  trajPlayBtn: HTMLButtonElement;
  trajClearBtn: HTMLButtonElement;
  trajProgress: HTMLInputElement;
  trajTime: HTMLElement;
  rightTabRobot: HTMLButtonElement;
  rightTabRosInfo: HTMLButtonElement;
  robotStateView: HTMLElement;
  rosInfoView: HTMLElement;
  rosInfoTopics: HTMLElement;
  rosInfoServices: HTMLElement;
  rosInfoParams: HTMLElement;
  loadAllParamsBtn: HTMLButtonElement;
  detailModal: HTMLElement;
  detailModalBackdrop: HTMLElement;
  detailModalContent: HTMLElement;
  detailModalClose: HTMLButtonElement;
}

interface TopicSubscriptions {
  jointStates?: any;
  tf?: any;
  tfStatic?: any;
  pointCloud?: any;
  moveGroupGoal?: any;
  moveGroupResult?: any;
  displayTrajectory?: any;
  executeTrajectoryGoal?: any;
  executeTrajectoryResult?: any;
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

interface LogEntry {
  time: string;
  message: string;
  kind: "default" | "moveit";
}

interface PlannedTrajectoryRestoreState {
  names: string[];
  positions: number[];
}

interface PlannedTrajectoryPayload {
  jointNames: string[];
  points: Array<{ positions?: number[] }>;
  source: "result" | "display";
  targetFrame: string;
  restoreState?: PlannedTrajectoryRestoreState;
}


interface TfTreeGraph {
  roots: string[];
  childrenByParent: Map<string, string[]>;
}

interface DetailField {
  label: string;
  value: string;
}

interface DetailSection {
  title: string;
  fields: DetailField[];
}

const POINTCLOUD2_TYPE = "sensor_msgs/PointCloud2";
const MOVE_GROUP_GOAL_TOPIC = "/move_group/goal";
const MOVE_GROUP_RESULT_TOPIC = "/move_group/result";
const DISPLAY_TRAJECTORY_TOPICS = ["/move_group/display_planned_path", "/display_planned_path"] as const;
const EXECUTE_TRAJECTORY_GOAL_TOPIC = "/execute_trajectory/goal";
const EXECUTE_TRAJECTORY_RESULT_TOPIC = "/execute_trajectory/result";
const SIDEBAR_STATE_KEY = "webrviz-sidebar-collapsed";
const RIGHT_SIDEBAR_STATE_KEY = "webrviz-right-sidebar-collapsed";
const TRAJ_SAMPLE_INTERVAL_MS = 50;
const TRAJ_MOTION_THRESHOLD = 0.001;
const TRAJ_IDLE_STOP_MS = 800;
const MAX_PLANNED_TRAJECTORY_SAMPLES = 180;
const TRAJ_ICON_RECORD = '<svg viewBox="0 0 24 24" aria-hidden="true"><circle cx="12" cy="12" r="7"/></svg>';
const TRAJ_ICON_STOP = '<svg viewBox="0 0 24 24" aria-hidden="true"><rect x="7" y="7" width="10" height="10"/></svg>';
const TRAJ_ICON_PLAY = '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M8 5v14l11-7z"/></svg>';
const TRAJ_ICON_PAUSE = '<svg viewBox="0 0 24 24" aria-hidden="true"><rect x="6" y="5" width="4" height="14"/><rect x="14" y="5" width="4" height="14"/></svg>';
const TRAJ_ICON_CLEAR = '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M6 6l12 12M18 6L6 18" fill="none" stroke-width="2" stroke-linecap="round"/></svg>';
const TRAJ_ICON_PLAN_PATH = '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M6 17a2 2 0 1 0 0.001 0zM12 7a2 2 0 1 0 0.001 0zM18 15a2 2 0 1 0 0.001 0z"/><path d="M7.8 15.9l2.4-6.2M13.7 8.4l2.7 5.1" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"/></svg>';

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
    languageToggleBtn: byId<HTMLButtonElement>("languageToggleBtn"),
    themeToggleBtn: byId<HTMLButtonElement>("themeToggleBtn"),
    connectBtn: byId<HTMLButtonElement>("connectBtn"),
    syncBtn: byId<HTMLButtonElement>("syncBtn"),
    sidebarToggleBtn: byId<HTMLButtonElement>("sidebarToggleBtn"),
    plannedTrajectoryToggleBtn: byId<HTMLButtonElement>("plannedTrajectoryToggleBtn"),
    rightSidebarToggleBtn: byId<HTMLButtonElement>("rightSidebarToggleBtn"),
    fixedFrameValue: byId<HTMLElement>("fixedFrameValue"),
    connectionStatus: byId<HTMLElement>("connectionStatus"),
    logBox: byId<HTMLElement>("logBox"),
    viewport: byId<HTMLElement>("viewport"),
    jointValues: byId<HTMLElement>("jointValues"),
    jointAngleUnit: byId<HTMLSelectElement>("jointAngleUnit"),
    cartesianFrame: byId<HTMLSelectElement>("cartesianFrame"),
    cartesianLengthUnit: byId<HTMLSelectElement>("cartesianLengthUnit"),
    cartesianAngleUnit: byId<HTMLSelectElement>("cartesianAngleUnit"),
    cartesianValues: byId<HTMLElement>("cartesianValues"),
    tfToggleAllBtn: byId<HTMLButtonElement>("tfToggleAllBtn"),
    tfSelectBtn: byId<HTMLButtonElement>("tfSelectBtn"),
    tfFilterMenu: byId<HTMLElement>("tfFilterMenu"),
    tfTree: byId<HTMLElement>("tfTree"),
    trajRecordBtn: byId<HTMLButtonElement>("trajRecordBtn"),
    trajPlayBtn: byId<HTMLButtonElement>("trajPlayBtn"),
    trajClearBtn: byId<HTMLButtonElement>("trajClearBtn"),
    trajProgress: byId<HTMLInputElement>("trajProgress"),
    trajTime: byId<HTMLElement>("trajTime"),
    rightTabRobot: byId<HTMLButtonElement>("rightTabRobot"),
    rightTabRosInfo: byId<HTMLButtonElement>("rightTabRosInfo"),
    robotStateView: byId<HTMLElement>("robotStateView"),
    rosInfoView: byId<HTMLElement>("rosInfoView"),
    rosInfoTopics: byId<HTMLElement>("rosInfoTopics"),
    rosInfoServices: byId<HTMLElement>("rosInfoServices"),
    rosInfoParams: byId<HTMLElement>("rosInfoParams"),
    loadAllParamsBtn: byId<HTMLButtonElement>("loadAllParamsBtn"),
    detailModal: byId<HTMLElement>("detailModal"),
    detailModalBackdrop: byId<HTMLElement>("detailModalBackdrop"),
    detailModalContent: byId<HTMLElement>("detailModalContent"),
    detailModalClose: byId<HTMLButtonElement>("detailModalClose")
  };
}

const elements = getElements();
let config: RuntimeConfig = loadConfig();

const rosClient = new RosClient();
const sceneManager = new SceneManager(elements.viewport, config.targetFps);
sceneManager.setFixedFrame(config.fixedFrame);

let topics: TopicInfo[] = [];
let logEntries: LogEntry[] = [];
let subscriptions: TopicSubscriptions = {};
let lastPointCloudTimestamp = 0;
let plannedTrajectoryVisible = true;
let latestPlannedTrajectory: PlannedTrajectoryPayload | null = null;
let pendingPlannedTrajectory: PlannedTrajectoryPayload | null = null;
let moveItPreviewArmed = false;
let sidebarCollapsed = window.localStorage.getItem(SIDEBAR_STATE_KEY) === "1";
let rightSidebarCollapsed = window.localStorage.getItem(RIGHT_SIDEBAR_STATE_KEY) !== "0";
let jointState: JointSnapshot[] = [];
let cartesianFrame = "";
let lastFrameOptions: string[] = [];
let tfVisibilityMode: TfVisibilityMode = "all";
let lastNonSelectionTfVisibilityMode: Exclude<TfVisibilityMode, "selection"> = "all";
let selectedTfFrames = new Set<string>();
let tfFilterMenuOpen = false;
let pendingRightPanelUpdate = false;
let lastTfTreeRenderSignature = "";
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
type RightPanelTab = "robot" | "rosInfo";
type AngleUnit = "deg" | "rad";
type LengthUnit = "mm" | "m";
type TfVisibilityMode = "all" | "none" | "selection";

const PARAM_EXCLUDE = new Set<string>([
  "/robot_description",
  "/robot_description_semantic",
  "/robot_description_kinematics"
]);
const MAX_PARAM_VALUE_LENGTH = 500;

let rightPanelTab: RightPanelTab = "robot";
let services: ServiceInfo[] = [];
let params: string[] = [];
let rosInfoBusy = false;
let jointAngleUnit: AngleUnit = "deg";
let cartesianLengthUnit: LengthUnit = "mm";
let cartesianAngleUnit: AngleUnit = "deg";
let currentLanguage: Language = loadLanguage();
let currentTheme: ThemeMode = loadTheme();
let lastConnectionState: ConnectionState = "disconnected";
let lastConnectionDetail: string | undefined;


function formatError(error: unknown): string {
  if (error instanceof Error) {
    return error.message;
  }
  return String(error);
}

function formatStatusDetail(detail?: string): string | undefined {
  if (!detail) {
    return undefined;
  }

  if (detail === "connected") {
    return t(currentLanguage, "statusDetail.connected");
  }

  if (detail === "disconnected") {
    return t(currentLanguage, "statusDetail.disconnected");
  }

  if (detail === "connection closed") {
    return t(currentLanguage, "statusDetail.connectionClosed");
  }

  if (detail.startsWith("connecting to ")) {
    const url = detail.slice("connecting to ".length);
    return currentLanguage === "zh" ? "连接到 " + url : "connecting to " + url;
  }

  return detail;
}

function updateLanguageToggleUi(): void {
  elements.languageToggleBtn.textContent = currentLanguage === "en" ? "中文" : "EN";
  elements.languageToggleBtn.title = t(currentLanguage, "button.switchLanguage");
  elements.languageToggleBtn.setAttribute("aria-label", t(currentLanguage, "button.switchLanguage"));
}

function updateThemeToggleUi(): void {
  const nextThemeKey = currentTheme === "dark" ? "button.themeLight" : "button.themeDark";
  elements.themeToggleBtn.textContent = t(currentLanguage, nextThemeKey as "button.themeLight" | "button.themeDark");
  elements.themeToggleBtn.title = t(currentLanguage, "button.switchTheme");
  elements.themeToggleBtn.setAttribute("aria-label", t(currentLanguage, "button.switchTheme"));
}

function updateConnectButtonText(): void {
  elements.connectBtn.textContent = lastConnectionState === "connected"
    ? t(currentLanguage, "button.disconnect")
    : t(currentLanguage, "button.connect");
}

function updateSidebarToggleUi(): void {
  elements.sidebarToggleBtn.textContent = sidebarCollapsed ? t(currentLanguage, "button.showSidebar") : t(currentLanguage, "button.hideSidebar");
  elements.sidebarToggleBtn.setAttribute("aria-label", elements.sidebarToggleBtn.textContent);
}

function updatePlannedTrajectoryToggleUi(): void {
  const toggleKey = plannedTrajectoryVisible ? "button.hidePlannedTrajectory" : "button.showPlannedTrajectory";
  const label = t(currentLanguage, toggleKey as "button.hidePlannedTrajectory" | "button.showPlannedTrajectory");
  elements.plannedTrajectoryToggleBtn.title = label;
  elements.plannedTrajectoryToggleBtn.setAttribute("aria-label", label);
  elements.plannedTrajectoryToggleBtn.setAttribute("aria-pressed", plannedTrajectoryVisible ? "true" : "false");
  elements.plannedTrajectoryToggleBtn.classList.toggle("is-active", plannedTrajectoryVisible);
}

function updateRightSidebarToggleUi(): void {
  elements.rightSidebarToggleBtn.textContent = rightSidebarCollapsed ? "<" : ">";
  elements.rightSidebarToggleBtn.setAttribute(
    "aria-label",
    rightSidebarCollapsed ? t(currentLanguage, "button.showRightSidebar") : t(currentLanguage, "button.hideRightSidebar")
  );
}

function updateTabLabels(): void {
  elements.rightTabRobot.textContent = t(currentLanguage, "panel.robotState");
  elements.rightTabRosInfo.textContent = t(currentLanguage, "panel.rosInfo");
}

function refreshUiText(): void {
  applyStaticTranslations(currentLanguage);
  updateLanguageToggleUi();
  updateThemeToggleUi();
  updateConnectButtonText();
  updateSidebarToggleUi();
  updatePlannedTrajectoryToggleUi();
  updateRightSidebarToggleUi();
  updateTabLabels();
  updateStatusLabel(lastConnectionState, lastConnectionDetail);
  renderRightPanel();
  updateTrajectoryUi();
}

function log(message: string): void {
  appendLogEntry(message, "default");
}

function logMoveIt(message: string): void {
  const normalized = message.startsWith("MoveIt ") ? message.slice("MoveIt ".length) : message;
  appendLogEntry(normalized, "moveit");
}

function appendLogEntry(message: string, kind: LogEntry["kind"]): void {
  logEntries.push({
    time: new Date().toLocaleTimeString(),
    message,
    kind
  });

  if (logEntries.length > 120) {
    logEntries = logEntries.slice(-120);
  }

  renderLogEntries();
}

function renderLogEntries(): void {
  elements.logBox.innerHTML = "";

  for (const entry of logEntries) {
    const line = document.createElement("div");
    line.className = "log-line";

    const time = document.createElement("span");
    time.className = "log-time";
    time.textContent = "[" + entry.time + "]";
    line.appendChild(time);

    if (entry.kind === "moveit") {
      const prefix = document.createElement("span");
      prefix.className = "log-prefix log-prefix-moveit";
      prefix.textContent = "[MoveIt]";
      line.appendChild(prefix);
    }

    const messageElement = document.createElement("span");
    messageElement.className = "log-message";
    messageElement.textContent = entry.message;
    line.appendChild(messageElement);

    elements.logBox.appendChild(line);
  }

  elements.logBox.scrollTop = elements.logBox.scrollHeight;
}

function findTopicInfo(name: string): TopicInfo | undefined {
  return topics.find((topic) => topic.name === name);
}

function actionStatusText(statusCode: number): string {
  switch (statusCode) {
    case 0:
      return "pending";
    case 1:
      return "active";
    case 2:
      return "preempted";
    case 3:
      return "succeeded";
    case 4:
      return "aborted";
    case 5:
      return "rejected";
    case 6:
      return "preempting";
    case 7:
      return "recalling";
    case 8:
      return "recalled";
    case 9:
      return "lost";
    default:
      return "unknown(" + String(statusCode) + ")";
  }
}

function formatSecondsValue(value: unknown, digits = 2): string | null {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    return null;
  }
  return value.toFixed(digits) + "s";
}

function isMoveItPlanSuccessful(statusCode: number, errorCode?: number): boolean {
  if (typeof errorCode === "number") {
    return errorCode === 1;
  }
  return statusCode === 3;
}

function toPlannedTrajectoryRestoreState(
  snapshot: { name?: unknown; position?: unknown } | undefined
): PlannedTrajectoryRestoreState | undefined {
  if (!snapshot || !Array.isArray(snapshot.name) || !Array.isArray(snapshot.position)) {
    return undefined;
  }

  const names: string[] = [];
  const positions: number[] = [];
  const count = Math.min(snapshot.name.length, snapshot.position.length);
  for (let index = 0; index < count; index += 1) {
    const name = snapshot.name[index];
    const position = snapshot.position[index];
    if (typeof name === "string" && name.length > 0 && typeof position === "number" && Number.isFinite(position)) {
      names.push(name);
      positions.push(position);
    }
  }

  if (names.length === 0 || positions.length === 0) {
    return undefined;
  }

  return { names, positions };
}

function clearPlannedTrajectoryState(): void {
  latestPlannedTrajectory = null;
  pendingPlannedTrajectory = null;
  sceneManager.clearPlannedTrajectory();
}

function buildPlannedTrajectoryPathForFrame(
  jointNames: string[],
  points: Array<{ positions?: number[] }>,
  targetFrame: string,
  fallbackRestoreState?: PlannedTrajectoryRestoreState
): Vector3[] {
  if (jointNames.length === 0 || points.length < 2 || !targetFrame) {
    return [];
  }

  const restoreState = jointState.length > 0
    ? {
        names: jointState.map((joint) => joint.name),
        positions: jointState.map((joint) => joint.position)
      }
    : fallbackRestoreState;

  if (!restoreState || restoreState.names.length === 0 || restoreState.positions.length === 0) {
    return [];
  }

  const sampledIndices = new Set<number>([0, points.length - 1]);
  const stride = Math.max(1, Math.ceil(points.length / MAX_PLANNED_TRAJECTORY_SAMPLES));
  for (let index = 0; index < points.length; index += stride) {
    sampledIndices.add(index);
  }

  const orderedIndices = Array.from(sampledIndices).sort((left, right) => left - right);
  const path: Vector3[] = [];

  try {
    for (const index of orderedIndices) {
      const positions = points[index]?.positions;
      if (!Array.isArray(positions) || positions.length < jointNames.length) {
        continue;
      }

      sceneManager.updateJointStates({
        name: jointNames,
        position: positions.slice(0, jointNames.length)
      });

      const pose = sceneManager.getRobotRelativeTransform(targetFrame);
      if (!pose) {
        continue;
      }

      const translation = pose.translation.clone();
      const lastPoint = path[path.length - 1];
      if (!lastPoint || lastPoint.distanceToSquared(translation) > 1e-10) {
        path.push(translation);
      }
    }
  } finally {
    sceneManager.updateJointStates({
      name: restoreState.names,
      position: restoreState.positions
    });
    scheduleRightPanelUpdate();
  }

  return path;
}

function getCurrentTcpLinkFrame(): string {
  const directValue = elements.cartesianFrame.value.trim();
  if (directValue) {
    return directValue;
  }

  const rememberedValue = cartesianFrame.trim();
  if (rememberedValue) {
    return rememberedValue;
  }

  const frames = sceneManager.getLinkList();
  if (frames.length === 0) {
    return "";
  }

  const preferred = sceneManager.getDefaultEndEffectorFrame();
  if (preferred && frames.includes(preferred)) {
    return preferred;
  }

  return frames[frames.length - 1];
}

function buildPlannedTrajectoryPreview(payload: PlannedTrajectoryPayload): { path: Vector3[]; frame: string } | null {
  if (!payload.targetFrame) {
    return null;
  }

  const path = buildPlannedTrajectoryPathForFrame(
    payload.jointNames,
    payload.points,
    payload.targetFrame,
    payload.restoreState
  );

  if (path.length < 2) {
    return null;
  }

  return { path, frame: payload.targetFrame };
}

function tryRenderPlannedTrajectory(payload: PlannedTrajectoryPayload, logSuccess = true): boolean {
  const preview = buildPlannedTrajectoryPreview(payload);
  if (!preview) {
    return false;
  }

  pendingPlannedTrajectory = null;
  sceneManager.setPlannedTrajectoryPath(preview.path);
  sceneManager.setPlannedTrajectoryVisible(plannedTrajectoryVisible);
  if (logSuccess) {
    logMoveIt(
      "planned trajectory visualized | source=" + payload.source +
      " | points=" + String(preview.path.length) +
      " | frame=" + preview.frame
    );
  }
  return true;
}

function retryPendingPlannedTrajectory(): void {
  if (!pendingPlannedTrajectory) {
    return;
  }
  tryRenderPlannedTrajectory(pendingPlannedTrajectory);
}

function refreshLatestPlannedTrajectoryPreview(): void {
  if (!latestPlannedTrajectory) {
    return;
  }
  if (!tryRenderPlannedTrajectory(latestPlannedTrajectory, false)) {
    pendingPlannedTrajectory = latestPlannedTrajectory;
  }
}

function queuePlannedTrajectoryFromJointTrajectory(
  jointTrajectory:
    | {
        joint_names?: string[];
        points?: Array<{ positions?: number[] }>;
      }
    | undefined,
  source: "result" | "display",
  restoreState?: PlannedTrajectoryRestoreState
): void {
  const jointNames = Array.isArray(jointTrajectory?.joint_names)
    ? jointTrajectory.joint_names.filter((name): name is string => typeof name === "string" && name.length > 0)
    : [];
  const points = Array.isArray(jointTrajectory?.points) ? jointTrajectory.points : [];

  if (jointNames.length === 0 || points.length < 2) {
    return;
  }

  const payload: PlannedTrajectoryPayload = {
    jointNames,
    points,
    source,
    targetFrame: getCurrentTcpLinkFrame(),
    restoreState
  };
  latestPlannedTrajectory = payload;
  pendingPlannedTrajectory = payload;

  if (!tryRenderPlannedTrajectory(payload)) {
    logMoveIt("planned trajectory preview pending scene sync | source=" + source);
  }
}

function updatePlannedTrajectoryFromMoveItResult(resultMessage: {
  status?: { status?: number };
  result?: {
    error_code?: { val?: number };
    planned_trajectory?: {
      joint_trajectory?: {
        joint_names?: string[];
        points?: Array<{ positions?: number[] }>;
      };
    };
  };
}): void {
  const statusCode = typeof resultMessage.status?.status === "number" ? resultMessage.status.status : -1;
  const errorCode = typeof resultMessage.result?.error_code?.val === "number" ? resultMessage.result.error_code.val : undefined;

  if (!isMoveItPlanSuccessful(statusCode, errorCode)) {
    clearPlannedTrajectoryState();
    return;
  }

  queuePlannedTrajectoryFromJointTrajectory(resultMessage.result?.planned_trajectory?.joint_trajectory, "result");
}

function updatePlannedTrajectoryFromDisplayTrajectory(message: {
  trajectory_start?: {
    joint_state?: {
      name?: string[];
      position?: number[];
    };
  };
  trajectory?: Array<{
    joint_trajectory?: {
      joint_names?: string[];
      points?: Array<{ positions?: number[] }>;
    };
  }>;
}): void {
  const restoreState = toPlannedTrajectoryRestoreState(message.trajectory_start?.joint_state);
  const trajectories = Array.isArray(message.trajectory) ? message.trajectory : [];
  for (let index = trajectories.length - 1; index >= 0; index -= 1) {
    const jointTrajectory = trajectories[index]?.joint_trajectory;
    const pointCount = Array.isArray(jointTrajectory?.points) ? jointTrajectory.points.length : 0;
    if (pointCount >= 2) {
      queuePlannedTrajectoryFromJointTrajectory(jointTrajectory, "display", restoreState);
      return;
    }
  }
}
function findDisplayTrajectoryTopicInfo(): TopicInfo | undefined {
  for (const topicName of DISPLAY_TRAJECTORY_TOPICS) {
    const info = findTopicInfo(topicName);
    if (info) {
      return info;
    }
  }

  return topics.find((topic) => /(^|\/)DisplayTrajectory$/.test(topic.type));
}

function subscribeMoveItTopics(): void {
  unsubscribe(subscriptions.moveGroupGoal);
  unsubscribe(subscriptions.moveGroupResult);
  unsubscribe(subscriptions.displayTrajectory);
  unsubscribe(subscriptions.executeTrajectoryGoal);
  unsubscribe(subscriptions.executeTrajectoryResult);
  subscriptions.moveGroupGoal = undefined;
  subscriptions.moveGroupResult = undefined;
  subscriptions.displayTrajectory = undefined;
  subscriptions.executeTrajectoryGoal = undefined;
  subscriptions.executeTrajectoryResult = undefined;

  const subscribed: string[] = [];

  const subscribeResolvedTopic = (
    info: TopicInfo | undefined,
    fallbackType: string,
    assign: (topic: any) => void,
    onMessage: (message: unknown) => void
  ): void => {
    if (!info) {
      return;
    }

    const topicName = info.name;
    const topicType = info.type || fallbackType;
    try {
      const topic = rosClient.createTopic(topicName, topicType);
      topic.subscribe(onMessage);
      assign(topic);
      subscribed.push(topicName);
    } catch (error) {
      logMoveIt("monitor skipped " + topicName + ": " + formatError(error));
    }
  };

  const subscribeIfAvailable = (
    topicName: string,
    fallbackType: string,
    assign: (topic: any) => void,
    onMessage: (message: unknown) => void
  ): void => {
    subscribeResolvedTopic(findTopicInfo(topicName), fallbackType, assign, onMessage);
  };

  subscribeIfAvailable(MOVE_GROUP_GOAL_TOPIC, "moveit_msgs/MoveGroupActionGoal", (topic) => {
    subscriptions.moveGroupGoal = topic;
  }, (message) => {
    const goal = message as {
      goal?: {
        request?: {
          planner_id?: string;
          group_name?: string;
          num_planning_attempts?: number;
          allowed_planning_time?: number;
        };
        planning_options?: {
          plan_only?: boolean;
        };
      };
    };

    const request = goal.goal?.request;
    const plannerId = request?.planner_id?.trim() || "default";
    const groupName = request?.group_name?.trim() || "unknown";
    const parts = [
      "MoveIt planning started",
      "planner=" + plannerId,
      "group=" + groupName
    ];

    if (typeof request?.num_planning_attempts === "number" && Number.isFinite(request.num_planning_attempts)) {
      parts.push("attempts=" + String(request.num_planning_attempts));
    }

    const allowedTime = formatSecondsValue(request?.allowed_planning_time);
    if (allowedTime) {
      parts.push("allowed_time=" + allowedTime);
    }

    if (goal.goal?.planning_options?.plan_only) {
      parts.push("plan_only=true");
    }

    moveItPreviewArmed = true;
    clearPlannedTrajectoryState();
    logMoveIt(parts.join(" | "));
  });

  subscribeIfAvailable(MOVE_GROUP_RESULT_TOPIC, "moveit_msgs/MoveGroupActionResult", (topic) => {
    subscriptions.moveGroupResult = topic;
  }, (message) => {
    if (!moveItPreviewArmed) {
      return;
    }
    const resultMessage = message as {
      status?: { status?: number };
      result?: {
        planning_time?: number;
        error_code?: { val?: number };
        planned_trajectory?: {
          joint_trajectory?: {
            joint_names?: string[];
            points?: Array<{ positions?: number[] }>;
          };
        };
      };
    };

    const statusCode = typeof resultMessage.status?.status === "number" ? resultMessage.status.status : -1;
    const parts = [
      "MoveIt planning finished",
      "status=" + actionStatusText(statusCode)
    ];

    const planningTime = formatSecondsValue(resultMessage.result?.planning_time, 3);
    if (planningTime) {
      parts.push("planning_time=" + planningTime);
    }

    if (typeof resultMessage.result?.error_code?.val === "number") {
      parts.push("error_code=" + String(resultMessage.result.error_code.val));
    }

    updatePlannedTrajectoryFromMoveItResult(resultMessage);
    logMoveIt(parts.join(" | "));
  });

  subscribeResolvedTopic(findDisplayTrajectoryTopicInfo(), "moveit_msgs/DisplayTrajectory", (topic) => {
    subscriptions.displayTrajectory = topic;
  }, (message) => {
    if (!moveItPreviewArmed) {
      return;
    }
    updatePlannedTrajectoryFromDisplayTrajectory(message as {
      trajectory?: Array<{
        joint_trajectory?: {
          joint_names?: string[];
          points?: Array<{ positions?: number[] }>;
        };
      }>;
    });
  });

  subscribeIfAvailable(EXECUTE_TRAJECTORY_GOAL_TOPIC, "moveit_msgs/ExecuteTrajectoryActionGoal", (topic) => {
    subscriptions.executeTrajectoryGoal = topic;
  }, (message) => {
    const goal = message as {
      goal?: {
        trajectory?: {
          joint_trajectory?: {
            joint_names?: string[];
            points?: unknown[];
          };
        };
      };
    };

    const jointCount = Array.isArray(goal.goal?.trajectory?.joint_trajectory?.joint_names)
      ? goal.goal?.trajectory?.joint_trajectory?.joint_names?.length ?? 0
      : 0;
    const pointCount = Array.isArray(goal.goal?.trajectory?.joint_trajectory?.points)
      ? goal.goal?.trajectory?.joint_trajectory?.points?.length ?? 0
      : 0;
    const parts = ["MoveIt execution started"];

    if (jointCount > 0) {
      parts.push("joints=" + String(jointCount));
    }
    if (pointCount > 0) {
      parts.push("points=" + String(pointCount));
    }

    logMoveIt(parts.join(" | "));
  });

  subscribeIfAvailable(EXECUTE_TRAJECTORY_RESULT_TOPIC, "moveit_msgs/ExecuteTrajectoryActionResult", (topic) => {
    subscriptions.executeTrajectoryResult = topic;
  }, (message) => {
    const resultMessage = message as {
      status?: { status?: number };
      result?: {
        error_code?: { val?: number };
      };
    };

    const statusCode = typeof resultMessage.status?.status === "number" ? resultMessage.status.status : -1;
    const parts = [
      "MoveIt execution finished",
      "status=" + actionStatusText(statusCode)
    ];

    if (typeof resultMessage.result?.error_code?.val === "number") {
      parts.push("error_code=" + String(resultMessage.result.error_code.val));
    }

    logMoveIt(parts.join(" | "));
  });

  if (subscribed.length > 0) {
    logMoveIt("monitor subscribed: " + subscribed.join(", "));
  }
}

function updateStatusLabel(state: ConnectionState, detail?: string): void {
  const stateKey = ("status." + state) as "status.disconnected" | "status.connecting" | "status.connected" | "status.error";
  const stateText = t(currentLanguage, stateKey);
  const detailText = formatStatusDetail(detail);
  elements.connectionStatus.textContent = detailText && detailText !== stateText ? stateText + " (" + detailText + ")" : stateText;
}

function setSidebarCollapsed(collapsed: boolean): void {
  sidebarCollapsed = collapsed;
  elements.appRoot.classList.toggle("sidebar-collapsed", sidebarCollapsed);
  updateSidebarToggleUi();
  window.localStorage.setItem(SIDEBAR_STATE_KEY, sidebarCollapsed ? "1" : "0");

  window.requestAnimationFrame(() => {
    window.dispatchEvent(new Event("resize"));
  });
}
function setRightSidebarCollapsed(collapsed: boolean): void {
  rightSidebarCollapsed = collapsed;
  elements.appRoot.classList.toggle("right-sidebar-collapsed", rightSidebarCollapsed);
  updateRightSidebarToggleUi();
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

function convertAngleFromRad(value: number, unit: AngleUnit): number {
  return unit === "deg" ? (value * 180) / Math.PI : value;
}

function convertLengthFromMeter(value: number, unit: LengthUnit): number {
  return unit === "mm" ? value * 1000 : value;
}

function angleUnitLabel(unit: AngleUnit): string {
  return unit === "deg" ? "deg" : "rad";
}

function lengthUnitLabel(unit: LengthUnit): string {
  return unit === "mm" ? "mm" : "m";
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

function setRightPanelTab(tab: RightPanelTab): void {
  rightPanelTab = tab;
  updateTabLabels();

  const robotActive = rightPanelTab === "robot";
  elements.rightTabRobot.classList.toggle("active", robotActive);
  elements.rightTabRosInfo.classList.toggle("active", !robotActive);
  elements.rightTabRobot.setAttribute("aria-selected", robotActive ? "true" : "false");
  elements.rightTabRosInfo.setAttribute("aria-selected", robotActive ? "false" : "true");

  elements.robotStateView.classList.toggle("active", robotActive);
  elements.rosInfoView.classList.toggle("active", !robotActive);

  if (!robotActive) {
    tfFilterMenuOpen = false;
    elements.tfFilterMenu.hidden = true;
  }

  renderRightPanel();
}

function openDetailModalContent(content: string | Node, contentClass?: string): void {
  elements.detailModalContent.classList.remove("param-detail-content", "structured-detail-content");
  clearElement(elements.detailModalContent);

  if (contentClass) {
    elements.detailModalContent.classList.add(contentClass);
  }

  if (typeof content === "string") {
    elements.detailModalContent.textContent = content;
  } else {
    elements.detailModalContent.appendChild(content);
  }

  elements.detailModal.classList.add("active");
  elements.detailModal.setAttribute("aria-hidden", "false");
}

function showDetailModal(text: string): void {
  openDetailModalContent(text);
}

function showStructuredDetailModal(content: Node): void {
  openDetailModalContent(content, "structured-detail-content");
}

function showParamDetailModal(paramName: string, value: string): void {
  elements.detailModalContent.classList.remove("structured-detail-content");
  elements.detailModalContent.classList.add("param-detail-content");
  clearElement(elements.detailModalContent);

  const labelText = document.createTextNode(t(currentLanguage, "detail.paramValueLabel", { paramName }));
  elements.detailModalContent.appendChild(labelText);

  const valueSpan = document.createElement("span");
  valueSpan.className = "param-detail-value";
  valueSpan.textContent = ": " + value;
  valueSpan.style.color = "var(--param-value)";
  elements.detailModalContent.appendChild(valueSpan);

  elements.detailModal.classList.add("active");
  elements.detailModal.setAttribute("aria-hidden", "false");
}

function hideDetailModal(): void {
  elements.detailModal.classList.remove("active");
  elements.detailModal.setAttribute("aria-hidden", "true");
}

function detailText(zh: string, en: string): string {
  return currentLanguage === "zh" ? zh : en;
}

function detailFallback(value?: string | null): string {
  const text = (value || "").trim();
  return text ? text : detailText("\u65e0", "None");
}

function formatDetailList(values: string[]): string {
  return values.length > 0 ? values.join(", ") : detailText("\u65e0", "None");
}

function formatDetailVector(values: [number, number, number] | null, unit = ""): string {
  if (!values) {
    return detailText("\u65e0", "None");
  }
  const joined = values.map((value) => formatNumber(value)).join(", ");
  return unit ? joined + " " + unit : joined;
}

function formatDetailAngles(values: [number, number, number] | null): string {
  if (!values) {
    return detailText("\u65e0", "None");
  }
  return values.map((value) => formatNumber(convertAngleFromRad(value, "deg"), 1)).join(", ") + " deg";
}

function formatDetailMeters(value: number | null, digits = 4, unit = "m"): string {
  if (value == null || !Number.isFinite(value)) {
    return detailText("\u65e0", "None");
  }
  return formatNumber(value, digits) + " " + unit;
}

function formatDetailRadians(value: number | null): string {
  if (value == null || !Number.isFinite(value)) {
    return detailText("\u65e0", "None");
  }
  return formatNumber(value, 3) + " rad / " + formatNumber(convertAngleFromRad(value, "deg"), 1) + " deg";
}

function formatJointLimitValue(value: number | null, jointType: string): string {
  if (jointType === "revolute" || jointType === "continuous") {
    return formatDetailRadians(value);
  }
  if (jointType === "prismatic") {
    return formatDetailMeters(value, 4, "m");
  }
  if (value == null || !Number.isFinite(value)) {
    return detailText("\u65e0", "None");
  }
  return formatNumber(value);
}

function formatJointCurrentValue(detail: RobotJointDetail): string {
  if (detail.currentValue.length === 0) {
    return detailText("\u56fa\u5b9a / \u65e0\u81ea\u7531\u5ea6", "Fixed / no DoF");
  }

  if (detail.type === "revolute" || detail.type === "continuous") {
    return formatDetailRadians(detail.currentValue[0] ?? null);
  }

  if (detail.type === "prismatic") {
    return formatDetailMeters(detail.currentValue[0] ?? null, 4, "m");
  }

  return detail.currentValue.map((value) => formatNumber(value)).join(", ");
}

function createStructuredDetailContent(kicker: string, title: string, sections: DetailSection[]): HTMLDivElement {
  const root = document.createElement("div");
  root.className = "structured-detail";

  const header = document.createElement("div");
  header.className = "structured-detail-header";

  const kickerElement = document.createElement("div");
  kickerElement.className = "structured-detail-kicker";
  kickerElement.textContent = kicker;
  header.appendChild(kickerElement);

  const titleElement = document.createElement("div");
  titleElement.className = "structured-detail-title";
  titleElement.textContent = title;
  header.appendChild(titleElement);

  root.appendChild(header);

  const sectionList = document.createElement("div");
  sectionList.className = "structured-detail-sections";

  for (const section of sections) {
    if (section.fields.length === 0) {
      continue;
    }

    const card = document.createElement("section");
    card.className = "structured-detail-card";

    const sectionTitle = document.createElement("h4");
    sectionTitle.className = "structured-detail-section-title";
    sectionTitle.textContent = section.title;
    card.appendChild(sectionTitle);

    const grid = document.createElement("div");
    grid.className = "structured-detail-grid";

    for (const field of section.fields) {
      const label = document.createElement("div");
      label.className = "structured-detail-label";
      label.textContent = field.label;
      grid.appendChild(label);

      const value = document.createElement("div");
      value.className = "structured-detail-value";
      value.textContent = field.value;
      grid.appendChild(value);
    }

    card.appendChild(grid);
    sectionList.appendChild(card);
  }

  root.appendChild(sectionList);
  return root;
}

function buildLinkDetailSections(detail: RobotLinkDetail): DetailSection[] {
  const sections: DetailSection[] = [
    {
      title: detailText("\u6982\u89c8", "Overview"),
      fields: [
        { label: detailText("\u540d\u79f0", "Name"), value: detail.name },
        { label: detailText("\u7236\u5173\u8282", "Parent joint"), value: detailFallback(detail.parentJoint) },
        { label: detailText("\u5b50\u5173\u8282", "Child joints"), value: formatDetailList(detail.childJoints) }
      ]
    },
    {
      title: detailText("\u51e0\u4f55\u4fe1\u606f", "Geometry"),
      fields: [
        { label: detailText("\u53ef\u89c6\u4f53\u6570\u91cf", "Visual count"), value: String(detail.visualCount) },
        { label: detailText("\u78b0\u649e\u4f53\u6570\u91cf", "Collision count"), value: String(detail.collisionCount) },
        { label: detailText("\u6750\u8d28", "Materials"), value: formatDetailList(detail.materialNames) }
      ]
    }
  ];

  if (detail.pose) {
    sections.push({
      title: detailText("\u5f53\u524d\u4f4d\u59ff", "Current pose"),
      fields: [
        { label: detailText("\u4f4d\u7f6e xyz", "Position xyz"), value: formatDetailVector(detail.pose.xyz, "m") },
        { label: detailText("\u59ff\u6001 rpy", "Orientation rpy"), value: formatDetailAngles(detail.pose.rpy) }
      ]
    });
  }

  if (detail.mass !== null || detail.inertialOrigin || detail.inertia) {
    sections.push({
      title: detailText("\u60ef\u6027\u4fe1\u606f", "Inertial"),
      fields: [
        { label: detailText("\u8d28\u91cf", "Mass"), value: formatDetailMeters(detail.mass, 4, "kg") },
        { label: detailText("\u8d28\u5fc3 xyz", "COM xyz"), value: detail.inertialOrigin ? formatDetailVector(detail.inertialOrigin.xyz, "m") : detailText("\u65e0", "None") },
        { label: detailText("\u8d28\u5fc3 rpy", "COM rpy"), value: detail.inertialOrigin ? formatDetailAngles(detail.inertialOrigin.rpy) : detailText("\u65e0", "None") },
        { label: "Ixx / Iyy / Izz", value: detail.inertia ? [detail.inertia.ixx, detail.inertia.iyy, detail.inertia.izz].map((value) => value == null ? "--" : formatNumber(value, 4)).join(" / ") : detailText("\u65e0", "None") },
        { label: "Ixy / Ixz / Iyz", value: detail.inertia ? [detail.inertia.ixy, detail.inertia.ixz, detail.inertia.iyz].map((value) => value == null ? "--" : formatNumber(value, 4)).join(" / ") : detailText("\u65e0", "None") }
      ]
    });
  }

  return sections;
}

function buildJointDetailSections(detail: RobotJointDetail): DetailSection[] {
  const sections: DetailSection[] = [
    {
      title: detailText("\u6982\u89c8", "Overview"),
      fields: [
        { label: detailText("\u540d\u79f0", "Name"), value: detail.name },
        { label: detailText("\u7c7b\u578b", "Type"), value: detailFallback(detail.type) },
        { label: detailText("\u7236 Link", "Parent link"), value: detailFallback(detail.parentLink) },
        { label: detailText("\u5b50 Link", "Child link"), value: detailFallback(detail.childLink) }
      ]
    },
    {
      title: detailText("\u8fd0\u52a8\u4fe1\u606f", "Motion"),
      fields: [
        { label: detailText("\u5f53\u524d\u503c", "Current value"), value: formatJointCurrentValue(detail) },
        { label: detailText("\u8f74\u5411", "Axis"), value: formatDetailVector(detail.axis) },
        { label: detailText("\u539f\u70b9 xyz", "Origin xyz"), value: detail.origin ? formatDetailVector(detail.origin.xyz, "m") : detailText("\u65e0", "None") },
        { label: detailText("\u539f\u70b9 rpy", "Origin rpy"), value: detail.origin ? formatDetailAngles(detail.origin.rpy) : detailText("\u65e0", "None") }
      ]
    }
  ];

  if (detail.limit) {
    sections.push({
      title: detailText("\u9650\u4f4d\u53c2\u6570", "Limits"),
      fields: [
        { label: detailText("\u4e0b\u9650", "Lower"), value: formatJointLimitValue(detail.limit.lower, detail.type) },
        { label: detailText("\u4e0a\u9650", "Upper"), value: formatJointLimitValue(detail.limit.upper, detail.type) },
        { label: detailText("\u529b/\u529b\u77e9", "Effort"), value: detail.limit.effort == null ? detailText("\u65e0", "None") : formatNumber(detail.limit.effort, 3) },
        { label: detailText("\u901f\u5ea6", "Velocity"), value: detail.limit.velocity == null ? detailText("\u65e0", "None") : formatNumber(detail.limit.velocity, 3) }
      ]
    });
  }

  if (detail.dynamics || detail.mimic) {
    sections.push({
      title: detailText("\u9644\u52a0\u4fe1\u606f", "Additional"),
      fields: [
        { label: detailText("\u963b\u5c3c", "Damping"), value: detail.dynamics?.damping == null ? detailText("\u65e0", "None") : formatNumber(detail.dynamics.damping, 4) },
        { label: detailText("\u6469\u64e6", "Friction"), value: detail.dynamics?.friction == null ? detailText("\u65e0", "None") : formatNumber(detail.dynamics.friction, 4) },
        { label: detailText("\u4ece\u52a8\u5173\u8282", "Mimic joint"), value: detail.mimic ? detailFallback(detail.mimic.joint) : detailText("\u65e0", "None") },
        { label: detailText("\u4ece\u52a8\u53c2\u6570", "Mimic params"), value: detail.mimic ? "multiplier=" + (detail.mimic.multiplier == null ? "--" : formatNumber(detail.mimic.multiplier, 3)) + ", offset=" + (detail.mimic.offset == null ? "--" : formatNumber(detail.mimic.offset, 3)) : detailText("\u65e0", "None") }
      ]
    });
  }

  return sections;
}

function showLinkDetailModal(linkName: string): void {
  const detail = sceneManager.getRobotLinkDetail(linkName);
  if (!detail) {
    const snapshot = sceneManager.getTfSnapshot();
    const parent = snapshot.find((edge) => edge.child === linkName)?.parent ?? null;
    const children = snapshot.filter((edge) => edge.parent === linkName).map((edge) => edge.child).sort((left, right) => left.localeCompare(right));
    const transform = sceneManager.getRelativeTransform(linkName);
    const euler = transform ? quatToEuler(transform.rotation) : null;
    const sections: DetailSection[] = [
      {
        title: detailText("\u6982\u89c8", "Overview"),
        fields: [
          { label: detailText("\u540d\u79f0", "Name"), value: linkName },
          { label: detailText("\u7236\u5750\u6807\u7cfb", "Parent frame"), value: detailFallback(parent) },
          { label: detailText("\u5b50\u5750\u6807\u7cfb", "Child frames"), value: formatDetailList(children) }
        ]
      }
    ];
    if (transform && euler) {
      sections.push({
        title: detailText("\u5f53\u524d\u4f4d\u59ff", "Current pose"),
        fields: [
          { label: detailText("\u4f4d\u7f6e xyz", "Position xyz"), value: formatDetailVector([transform.translation.x, transform.translation.y, transform.translation.z], "m") },
          { label: detailText("\u59ff\u6001 rpy", "Orientation rpy"), value: formatDetailAngles([euler.roll, euler.pitch, euler.yaw]) }
        ]
      });
    }
    showStructuredDetailModal(createStructuredDetailContent(detailText("\u5750\u6807\u7cfb\u8be6\u60c5", "Frame Detail"), linkName, sections));
    return;
  }

  showStructuredDetailModal(createStructuredDetailContent(detailText("Link \u8be6\u60c5", "Link Detail"), detail.name, buildLinkDetailSections(detail)));
}

function showJointDetailModal(jointName: string): void {
  const detail = sceneManager.getRobotJointDetail(jointName);
  if (!detail) {
    return;
  }

  showStructuredDetailModal(createStructuredDetailContent(detailText("\u5173\u8282\u8be6\u60c5", "Joint Detail"), detail.name, buildJointDetailSections(detail)));
}

function makeTfTreeRowInteractive(row: HTMLDivElement, onActivate: () => void): HTMLDivElement {
  row.classList.add("tf-tree-row-clickable");
  row.tabIndex = 0;
  row.setAttribute("role", "button");
  row.addEventListener("click", onActivate);
  row.addEventListener("keydown", (event) => {
    if (event.key === "Enter" || event.key === " ") {
      event.preventDefault();
      onActivate();
    }
  });
  return row;
}

function formatUnknownValue(value: unknown): string {
  if (typeof value === "string") {
    return value;
  }

  try {
    return JSON.stringify(value, null, 2);
  } catch {
    return String(value);
  }
}

function truncateValue(text: string, maxLength = MAX_PARAM_VALUE_LENGTH): string {
  if (text.length <= maxLength) {
    return text;
  }
  return `${text.slice(0, maxLength)} ... (truncated)`;
}

function parseParamValue(raw: string): unknown {
  try {
    return JSON.parse(raw);
  } catch {
    return raw;
  }
}

function formatMessageDetail(type: string, details: MessageDetailsResponse): string {
  const typedefs = details.typedefs ?? [];
  if (typedefs.length === 0) {
    return t(currentLanguage, "detail.noMessageInfo", { type });
  }

  const typedefMap = new Map<string, MessageTypeDef>();
  for (const def of typedefs) {
    typedefMap.set(def.type, def);
  }

  if (!typedefMap.has(type)) {
    return t(currentLanguage, "detail.noMessageInfo", { type });
  }

  const printed = new Set<string>();
  const output: string[] = [];

  const printMessage = (messageType: string, isRoot: boolean): void => {
    if (printed.has(messageType)) {
      return;
    }

    const msg = typedefMap.get(messageType);
    if (!msg) {
      return;
    }

    printed.add(messageType);

    if (isRoot) {
      output.push(`# ${messageType}`);
      output.push("----------------------------------");
    } else {
      output.push("");
      output.push("=".repeat(80));
      output.push(`MSG: ${messageType}`);
    }

    const count = Math.min(msg.fieldnames.length, msg.fieldtypes.length, msg.fieldarraylen.length);
    for (let index = 0; index < count; index += 1) {
      const name = msg.fieldnames[index];
      const fieldType = msg.fieldtypes[index];
      const arrLen = msg.fieldarraylen[index];

      if (arrLen === -1) {
        output.push(`${fieldType} ${name}`);
      } else if (arrLen === 0) {
        output.push(`${fieldType}[] ${name}`);
      } else {
        output.push(`${fieldType}[${arrLen}] ${name}`);
      }
    }

    for (const fieldType of msg.fieldtypes) {
      if (typedefMap.has(fieldType) && !printed.has(fieldType)) {
        printMessage(fieldType, false);
      }
    }
  };

  printMessage(type, true);
  return output.join("\n");
}

function formatMessageDetailAutoRoot(preferredType: string, details: MessageDetailsResponse): string {
  const typedefs = details.typedefs ?? [];
  if (typedefs.length === 0) {
    return t(currentLanguage, "detail.noMessageInfo", { type: preferredType });
  }

  const root = typedefs.some((item) => item.type === preferredType) ? preferredType : typedefs[0].type;
  return formatMessageDetail(root, details);
}

async function showServiceTypeDetails(type: string): Promise<void> {
  if (!type) {
    return;
  }

  try {
    showDetailModal(t(currentLanguage, "detail.loadingService", { type }));
    const details = await rosClient.getServiceDetails(type);

    const requestText = formatMessageDetailAutoRoot(`${type}Request`, details.request);
    const responseText = formatMessageDetailAutoRoot(`${type}Response`, details.response);

    const output = [
      `# ${type}`,
      "----------------------------------",
      "",
      t(currentLanguage, "detail.request"),
      requestText,
      "",
      t(currentLanguage, "detail.response"),
      responseText
    ].join("\n");

    showDetailModal(output);
  } catch (error) {
    const detail = formatError(error);
    showDetailModal(t(currentLanguage, "detail.serviceUnavailable", { type, detail }));
    log(`service detail unavailable: ${detail}`);
  }
}
function clearElement(container: HTMLElement): void {
  container.innerHTML = "";
}

function renderInfoEmpty(container: HTMLElement, text: string): void {
  clearElement(container);
  const row = document.createElement("div");
  row.className = "data-row";
  row.textContent = text;
  container.appendChild(row);
}

function addInfoItem(container: HTMLElement, name: string, actionText: string, actionClass: string, onClick?: () => void): void {
  const item = document.createElement("div");
  item.className = "info-item";

  const nameSpan = document.createElement("span");
  nameSpan.className = "info-item-name";
  nameSpan.title = name;
  nameSpan.textContent = name;

  const action = document.createElement("button");
  action.type = "button";
  action.className = actionClass;
  action.textContent = actionText || t(currentLanguage, "common.unknown");
  action.disabled = !onClick;
  if (onClick) {
    action.addEventListener("click", onClick);
  }

  item.appendChild(nameSpan);
  item.appendChild(action);
  container.appendChild(item);
}

async function showTopicTypeDetails(type: string): Promise<void> {
  if (!type) {
    showDetailModal(t(currentLanguage, "detail.noTypeForTopic"));
    return;
  }

  try {
    showDetailModal(t(currentLanguage, "detail.loadingMessage", { type }));
    const details = await rosClient.getMessageDetails(type);
    showDetailModal(formatMessageDetail(type, details));
  } catch (error) {
    const detail = formatError(error);
    showDetailModal(t(currentLanguage, "detail.messageError", { type, detail }));
    log(`message detail failed: ${detail}`);
  }
}

async function showParamValue(paramName: string): Promise<void> {
  try {
    showDetailModal(t(currentLanguage, "detail.loadingParam", { paramName }));
    const raw = await rosClient.getParam(paramName);
    const parsed = parseParamValue(raw);
    const text = formatUnknownValue(parsed);
    showParamDetailModal(paramName, truncateValue(text));
  } catch (error) {
    const detail = formatError(error);
    showDetailModal(t(currentLanguage, "detail.paramError", { paramName, detail }));
    log(`param load failed: ${detail}`);
  }
}

async function showAllParamValues(): Promise<void> {
  if (rosInfoBusy) {
    return;
  }

  rosInfoBusy = true;
  elements.loadAllParamsBtn.disabled = true;

  try {
    const lines: string[] = [];
    lines.push(t(currentLanguage, "detail.parametersTitle"));
    lines.push("----------------------------------");

    const sorted = params.filter((name) => !PARAM_EXCLUDE.has(name)).sort((left, right) => left.localeCompare(right));

    for (const param of sorted) {
      try {
        const raw = await rosClient.getParam(param);
        const parsed = parseParamValue(raw);
        const text = truncateValue(formatUnknownValue(parsed));
        lines.push(`${param}: ${text}`);
      } catch (error) {
        lines.push(`${param}: ${t(currentLanguage, "detail.errorValue", { detail: formatError(error) })}`);
      }
    }

    showDetailModal(lines.join("\n"));
  } finally {
    rosInfoBusy = false;
    elements.loadAllParamsBtn.disabled = false;
  }
}

function renderRosInfoPanel(): void {
  if (!rosClient.isConnected()) {
    renderInfoEmpty(elements.rosInfoTopics, t(currentLanguage, "state.notConnected"));
    renderInfoEmpty(elements.rosInfoServices, t(currentLanguage, "state.notConnected"));
    renderInfoEmpty(elements.rosInfoParams, t(currentLanguage, "state.notConnected"));
    elements.loadAllParamsBtn.disabled = true;
    return;
  }

  elements.loadAllParamsBtn.disabled = rosInfoBusy;

  clearElement(elements.rosInfoTopics);
  if (topics.length === 0) {
    renderInfoEmpty(elements.rosInfoTopics, t(currentLanguage, "state.noTopics"));
  } else {
    const sortedTopics = [...topics].sort((left, right) => left.name.localeCompare(right.name));
    for (const topic of sortedTopics) {
      addInfoItem(elements.rosInfoTopics, topic.name, topic.type || t(currentLanguage, "common.unknown"), "info-item-type", () => {
        void showTopicTypeDetails(topic.type);
      });
    }
  }

  clearElement(elements.rosInfoServices);
  if (services.length === 0) {
    renderInfoEmpty(elements.rosInfoServices, t(currentLanguage, "state.noServices"));
  } else {
    const sortedServices = [...services].sort((left, right) => left.name.localeCompare(right.name));
    for (const service of sortedServices) {
      addInfoItem(elements.rosInfoServices, service.name, service.type || t(currentLanguage, "common.unknown"), "info-item-type", () => {
        void showServiceTypeDetails(service.type);
      });
    }
  }

  clearElement(elements.rosInfoParams);
  if (params.length === 0) {
    renderInfoEmpty(elements.rosInfoParams, t(currentLanguage, "state.noParams"));
  } else {
    const sortedParams = [...params].sort((left, right) => left.localeCompare(right));
    for (const param of sortedParams) {
      addInfoItem(elements.rosInfoParams, param, t(currentLanguage, "button.view"), "info-item-action", () => {
        void showParamValue(param);
      });
    }
  }
}

function clearRosInfoState(): void {
  services = [];
  params = [];
  rosInfoBusy = false;
  elements.loadAllParamsBtn.disabled = true;
  hideDetailModal();
  renderRosInfoPanel();
}

async function refreshRosInfoCache(): Promise<void> {
  if (!rosClient.isConnected()) {
    clearRosInfoState();
    return;
  }

  try {
    const [nextServices, nextParams] = await Promise.all([
      rosClient.listServicesWithTypes(),
      rosClient.listParams()
    ]);

    services = nextServices;
    params = nextParams;
    renderRosInfoPanel();
    log(`ROS info discovery finished: ${services.length} services, ${params.length} params`);
  } catch (error) {
    const detail = formatError(error);
    log(`ROS info discovery warning: ${detail}`);
    renderRosInfoPanel();
  }
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
    renderPlaceholder(elements.jointValues, t(currentLanguage, "state.noJointData"));
    return;
  }

  elements.jointValues.innerHTML = "";
  const unitText = angleUnitLabel(jointAngleUnit);
  for (const joint of jointState) {
    const value = convertAngleFromRad(joint.position, jointAngleUnit);
    addDataRow(elements.jointValues, `${joint.name} (${unitText})`, formatNumber(value));
  }
}

function updateFrameOptions(): void {
  const frames = sceneManager.getLinkList();
  if (frames.length === 0) {
    elements.cartesianFrame.innerHTML = "";
    cartesianFrame = "";
    lastFrameOptions = [];
    sceneManager.setEndEffectorFrame("");
    refreshEnhancedSelect(elements.cartesianFrame);
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
  sceneManager.setEndEffectorFrame(cartesianFrame);
  refreshEnhancedSelect(elements.cartesianFrame);
}
function renderCartesianValues(): void {
  if (!cartesianFrame) {
    renderPlaceholder(elements.cartesianValues, t(currentLanguage, "state.noFrame"));
    return;
  }

  const transform = sceneManager.getRelativeTransform(cartesianFrame);
  if (!transform) {
    renderPlaceholder(elements.cartesianValues, t(currentLanguage, "state.unavailable"));
    return;
  }

  const position = transform.translation;
  const rotation = transform.rotation;
  const euler = quatToEuler(rotation);

  const lengthUnit = lengthUnitLabel(cartesianLengthUnit);
  const angleUnit = angleUnitLabel(cartesianAngleUnit);

  elements.cartesianValues.innerHTML = "";
  addDataRow(elements.cartesianValues, `X (${lengthUnit})`, formatNumber(convertLengthFromMeter(position.x, cartesianLengthUnit)));
  addDataRow(elements.cartesianValues, `Y (${lengthUnit})`, formatNumber(convertLengthFromMeter(position.y, cartesianLengthUnit)));
  addDataRow(elements.cartesianValues, `Z (${lengthUnit})`, formatNumber(convertLengthFromMeter(position.z, cartesianLengthUnit)));
  addDataRow(elements.cartesianValues, `Roll (${angleUnit})`, formatNumber(convertAngleFromRad(euler.roll, cartesianAngleUnit)));
  addDataRow(elements.cartesianValues, `Pitch (${angleUnit})`, formatNumber(convertAngleFromRad(euler.pitch, cartesianAngleUnit)));
  addDataRow(elements.cartesianValues, `Yaw (${angleUnit})`, formatNumber(convertAngleFromRad(euler.yaw, cartesianAngleUnit)));
}

function buildTfTreeGraph(snapshot: Array<{ parent: string; child: string }>, visibleNodes: Set<string> | null = null): TfTreeGraph | null {
  if (snapshot.length === 0) {
    return null;
  }

  if (visibleNodes && visibleNodes.size === 0) {
    return null;
  }

  const childrenByParent = new Map<string, string[]>();
  const nodes = new Set<string>();
  const childrenSet = new Set<string>();

  for (const edge of snapshot) {
    if (!edge.parent || !edge.child) {
      continue;
    }

    if (visibleNodes && (!visibleNodes.has(edge.parent) || !visibleNodes.has(edge.child))) {
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
    return null;
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

  return { roots, childrenByParent };
}

function buildTfJointLookup(connections: RobotJointConnection[] = sceneManager.getRobotJointConnections()): Map<string, RobotJointConnection> {
  const lookup = new Map<string, RobotJointConnection>();
  for (const joint of connections) {
    lookup.set(joint.parent + "::" + joint.child, joint);
  }
  return lookup;
}

function createTfTreeRow(
  kind: "frame" | "joint" | "note",
  label: string,
  depth: number,
  badges: string[] = []
): HTMLDivElement {
  const row = document.createElement("div");
  row.className = "tf-tree-row tf-tree-row-" + kind;
  row.style.paddingLeft = String(depth * 10) + "px";

  const name = document.createElement("span");
  name.className = "tf-tree-name";
  name.textContent = label;
  row.appendChild(name);

  for (const badgeText of badges) {
    const badge = document.createElement("span");
    badge.className = "tf-tree-badge";
    badge.textContent = "[" + badgeText + "]";
    row.appendChild(badge);
  }

  return row;
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

function pruneSelectedTfFrames(availableFrames: string[]): void {
  const availableSet = new Set(availableFrames);
  const nextSelected = new Set<string>();
  for (const frame of selectedTfFrames) {
    if (availableSet.has(frame)) {
      nextSelected.add(frame);
    }
  }
  selectedTfFrames = nextSelected;
  if (selectedTfFrames.size === 0 && tfVisibilityMode === "selection") {
    tfVisibilityMode = lastNonSelectionTfVisibilityMode;
  }
}

function getVisibleTfNodes(snapshot: Array<{ parent: string; child: string }>): Set<string> | null {
  if (tfVisibilityMode !== "selection") {
    return tfVisibilityMode === "none" ? new Set<string>() : null;
  }

  const allFrames = new Set<string>();
  for (const edge of snapshot) {
    if (edge.parent) {
      allFrames.add(edge.parent);
    }
    if (edge.child) {
      allFrames.add(edge.child);
    }
  }

  const visible = new Set<string>();
  for (const frame of selectedTfFrames) {
    if (allFrames.has(frame)) {
      visible.add(frame);
    }
  }

  return visible;
}

function updateTfToolbarUi(availableFrames: string[]): void {
  const hasFrames = availableFrames.length > 0;
  const baseVisibilityMode = tfVisibilityMode === "selection" ? lastNonSelectionTfVisibilityMode : tfVisibilityMode;
  const toggleKey = baseVisibilityMode === "all" ? "button.hideAllTf" : "button.showAllTf";
  elements.tfToggleAllBtn.textContent = t(currentLanguage, toggleKey as "button.hideAllTf" | "button.showAllTf");
  elements.tfToggleAllBtn.disabled = !hasFrames;

  const selectedCount = selectedTfFrames.size;
  const selectLabel = selectedCount > 0
    ? t(currentLanguage, "button.selectTf") + ` (${selectedCount})`
    : t(currentLanguage, "button.selectTf");
  elements.tfSelectBtn.textContent = selectLabel;
  elements.tfSelectBtn.disabled = !hasFrames;
  elements.tfSelectBtn.setAttribute("aria-expanded", tfFilterMenuOpen ? "true" : "false");
}

function renderTfFilterMenu(availableFrames: string[]): void {
  elements.tfFilterMenu.innerHTML = "";
  elements.tfFilterMenu.hidden = !tfFilterMenuOpen;

  if (!tfFilterMenuOpen) {
    return;
  }

  if (availableFrames.length === 0) {
    const empty = document.createElement("div");
    empty.className = "tf-filter-empty";
    empty.textContent = t(currentLanguage, "state.noTfData");
    elements.tfFilterMenu.appendChild(empty);
    return;
  }

  for (const frame of availableFrames) {
    const selected = selectedTfFrames.has(frame);
    const option = document.createElement("button");
    option.type = "button";
    option.className = selected ? "tf-filter-item selected" : "tf-filter-item";
    option.setAttribute("aria-pressed", selected ? "true" : "false");
    option.addEventListener("click", () => {
      if (selectedTfFrames.has(frame)) {
        selectedTfFrames.delete(frame);
      } else {
        selectedTfFrames.add(frame);
      }
      tfVisibilityMode = selectedTfFrames.size > 0 ? "selection" : lastNonSelectionTfVisibilityMode;
      renderTfTree();
    });

    const mark = document.createElement("input");
    mark.type = "checkbox";
    mark.className = "tf-filter-check";
    mark.checked = selected;
    mark.tabIndex = -1;
    mark.setAttribute("aria-hidden", "true");

    const text = document.createElement("span");
    text.className = "tf-filter-label";
    text.textContent = frame;

    option.appendChild(mark);
    option.appendChild(text);
    elements.tfFilterMenu.appendChild(option);
  }
}



function applyTfVisualizationFilter(snapshot: Array<{ parent: string; child: string }>): void {
  sceneManager.setVisibleTfFrames(getVisibleTfNodes(snapshot));
}

function getTfTreeStructureSignature(snapshot: Array<{ parent: string; child: string }>, joints: RobotJointConnection[]): string {
  const edgeSignature = [...snapshot]
    .map((edge) => edge.parent + ">" + edge.child)
    .sort((left, right) => left.localeCompare(right))
    .join("|");
  const jointSignature = joints
    .map((joint) => joint.parent + ">" + joint.name + ">" + joint.child + ">" + joint.type)
    .sort((left, right) => left.localeCompare(right))
    .join("|");

  return [
    currentLanguage,
    sceneManager.getFixedFrame(),
    cartesianFrame,
    edgeSignature,
    jointSignature
  ].join("||");
}

function renderTfTree(): void {
  const snapshot = sceneManager.getTfSnapshot();
  const availableFrames = getTfTreeOrder(snapshot);
  pruneSelectedTfFrames(availableFrames);
  applyTfVisualizationFilter(snapshot);
  updateTfToolbarUi(availableFrames);
  renderTfFilterMenu(availableFrames);

  const graph = buildTfTreeGraph(snapshot, null);
  const jointConnections = sceneManager.getRobotJointConnections();
  const renderSignature = graph
    ? getTfTreeStructureSignature(snapshot, jointConnections)
    : ["empty", currentLanguage, sceneManager.getFixedFrame(), cartesianFrame].join("||");

  if (renderSignature === lastTfTreeRenderSignature) {
    return;
  }

  lastTfTreeRenderSignature = renderSignature;
  clearElement(elements.tfTree);

  if (!graph) {
    elements.tfTree.textContent = t(currentLanguage, "state.noTfData");
    return;
  }

  const fixedFrame = sceneManager.getFixedFrame();
  const selectedFrame = cartesianFrame;
  const jointLookup = buildTfJointLookup(jointConnections);
  const content = document.createElement("div");
  content.className = "tf-tree-content";
  elements.tfTree.appendChild(content);

  const stack = new Set<string>();
  let rowCount = 0;
  const maxRows = 600;

  const appendFrame = (frame: string, depth: number): void => {
    if (rowCount >= maxRows) {
      return;
    }

    const badges: string[] = [];
    if (frame === fixedFrame) {
      badges.push(t(currentLanguage, "tf.fixed"));
    }
    if (frame === selectedFrame) {
      badges.push(t(currentLanguage, "tf.selected"));
    }
    const frameRow = makeTfTreeRowInteractive(createTfTreeRow("frame", frame, depth, badges), () => {
      showLinkDetailModal(frame);
    });
    content.appendChild(frameRow);
    rowCount += 1;

    if (stack.has(frame)) {
      if (rowCount < maxRows) {
        content.appendChild(createTfTreeRow("note", "(" + t(currentLanguage, "tf.loop") + ")", depth + 1));
        rowCount += 1;
      }
      return;
    }

    const children = graph.childrenByParent.get(frame);
    if (!children || children.length === 0) {
      return;
    }

    stack.add(frame);
    for (const child of children) {
      if (rowCount >= maxRows) {
        break;
      }

      const joint = jointLookup.get(frame + "::" + child);
      if (joint) {
        const jointRow = makeTfTreeRowInteractive(createTfTreeRow("joint", joint.name, depth + 1), () => {
          showJointDetailModal(joint.name);
        });
        content.appendChild(jointRow);
        rowCount += 1;
        if (rowCount >= maxRows) {
          break;
        }
      }

      appendFrame(child, depth + (joint ? 2 : 1));
    }
    stack.delete(frame);
  };

  for (const root of graph.roots) {
    if (rowCount >= maxRows) {
      break;
    }
    appendFrame(root, 0);
  }

  if (rowCount >= maxRows) {
    content.appendChild(createTfTreeRow("note", "...", 0));
  }
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

function getDisplayedTrajectoryProgressMs(): number {
  const durationMs = getTrajectoryDurationMs();
  if (durationMs <= 0) {
    return 0;
  }

  if (isPlaying) {
    return Math.min(durationMs, Math.max(0, performance.now() - playbackStartTime));
  }

  return getTrajectoryProgressMs();
}
function formatTrajectoryTime(ms: number): string {
  const totalSeconds = Math.max(0, Math.floor(ms / 1000));
  const minutes = Math.floor(totalSeconds / 60);
  const seconds = totalSeconds % 60;
  return String(minutes).padStart(2, "0") + ":" + String(seconds).padStart(2, "0");
}

function updateTrajectoryUi(): void {
  const durationMs = getTrajectoryDurationMs();
  const progressMs = getDisplayedTrajectoryProgressMs();
  const progress = durationMs > 0 ? Math.min(100, Math.max(0, Math.round((progressMs / durationMs) * 100))) : 0;
  elements.trajProgress.value = String(progress);

  elements.trajTime.textContent = formatTrajectoryTime(progressMs) + " / " + formatTrajectoryTime(durationMs);

  const canPlay = trajectory.length > 0;
  elements.trajPlayBtn.disabled = !canPlay;
  elements.trajClearBtn.disabled = trajectory.length === 0 && !isRecording && !isPlaying;
  elements.trajRecordBtn.disabled = isPlaying;

  elements.trajRecordBtn.innerHTML = isRecording ? TRAJ_ICON_STOP : TRAJ_ICON_RECORD;
  elements.trajPlayBtn.innerHTML = isPlaying ? TRAJ_ICON_PAUSE : TRAJ_ICON_PLAY;
  elements.trajClearBtn.innerHTML = TRAJ_ICON_CLEAR;

  const recordLabel = isRecording ? t(currentLanguage, "button.stop") : t(currentLanguage, "button.record");
  const playLabel = isPlaying ? t(currentLanguage, "button.pause") : t(currentLanguage, "button.play");
  const clearLabel = t(currentLanguage, "button.clear");
  elements.trajRecordBtn.title = recordLabel;
  elements.trajRecordBtn.setAttribute("aria-label", recordLabel);
  elements.trajPlayBtn.title = playLabel;
  elements.trajPlayBtn.setAttribute("aria-label", playLabel);
  elements.trajClearBtn.title = clearLabel;
  elements.trajClearBtn.setAttribute("aria-label", clearLabel);

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

function enableEndEffectorPreview(): void {
  const endEffectorFrame = sceneManager.getDefaultEndEffectorFrame();
  if (endEffectorFrame) {
    cartesianFrame = endEffectorFrame;
    elements.cartesianFrame.value = cartesianFrame;
  }
  sceneManager.setEndEffectorFrame(cartesianFrame || endEffectorFrame);
  sceneManager.setShowOnlyEndEffector(true);
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

  enableEndEffectorPreview();

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

function clearAllTrajectoryDisplays(): void {
  moveItPreviewArmed = false;
  clearPlannedTrajectoryState();
  clearTrajectory();
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
  if (rightPanelTab === "robot") {
    renderJointValues();
    updateFrameOptions();
    renderCartesianValues();
    if (tfFilterMenuOpen) {
      const snapshot = sceneManager.getTfSnapshot();
      const availableFrames = getTfTreeOrder(snapshot);
      pruneSelectedTfFrames(availableFrames);
      applyTfVisualizationFilter(snapshot);
      updateTfToolbarUi(availableFrames);
    } else {
      renderTfTree();
    }
    updateTrajectoryUi();
    return;
  }

  renderRosInfoPanel();
}

function scheduleRightPanelUpdate(): void {
  if (rightPanelTab !== "robot") {
    return;
  }

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
  lastTfTreeRenderSignature = "";
  lastFrameOptions = [];
  elements.cartesianFrame.innerHTML = "";
  clearRosInfoState();
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
  unsubscribe(subscriptions.moveGroupGoal);
  unsubscribe(subscriptions.moveGroupResult);
  unsubscribe(subscriptions.displayTrajectory);
  unsubscribe(subscriptions.executeTrajectoryGoal);
  unsubscribe(subscriptions.executeTrajectoryResult);
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
  refreshEnhancedSelect(elements.pointCloudTopic);
}

async function refreshTopicCache(): Promise<void> {
  topics = await rosClient.listTopicsWithTypes();
  const cloudTopics = pointCloudTopicNames(topics);
  updatePointCloudOptions(cloudTopics, elements.pointCloudTopic.value || config.defaultPointCloudTopic);
  renderRosInfoPanel();
  subscribeMoveItTopics();
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
    retryPendingPlannedTrajectory();
    recordTrajectorySnapshot(message);
    scheduleRightPanelUpdate();
  });

  subscriptions.tf = rosClient.createTopic("/tf", "tf2_msgs/TFMessage");
  subscriptions.tf.subscribe((message: unknown) => {
    sceneManager.upsertTfMessage(message);
    retryPendingPlannedTrajectory();
    scheduleRightPanelUpdate();
  });

  subscriptions.tfStatic = rosClient.createTopic("/tf_static", "tf2_msgs/TFMessage");
  subscriptions.tfStatic.subscribe((message: unknown) => {
    sceneManager.upsertTfMessage(message);
    retryPendingPlannedTrajectory();
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
  refreshLatestPlannedTrajectoryPreview();
  log(`robot model loaded (root=${loaded.rootLink})`);
}

async function connect(): Promise<void> {
  clearAllTrajectoryDisplays();
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
  await refreshRosInfoCache();
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
  topics = [];
  rosClient.disconnect();
  sceneManager.setPointCloud(null);
  sceneManager.setRobot(null);
  sceneManager.clearTfRecords();
  clearAllTrajectoryDisplays();
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
  await refreshRosInfoCache();

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

applyTheme(currentTheme);
sceneManager.setTheme(currentTheme);
sceneManager.setPlannedTrajectoryVisible(plannedTrajectoryVisible);
elements.plannedTrajectoryToggleBtn.innerHTML = TRAJ_ICON_PLAN_PATH;
updatePlannedTrajectoryToggleUi();
renderConfigToUi();
updatePointCloudOptions([], config.defaultPointCloudTopic);
enhanceSelect(elements.pointCloudTopic);
enhanceSelect(elements.jointAngleUnit);
enhanceSelect(elements.cartesianFrame);
enhanceSelect(elements.cartesianLengthUnit);
enhanceSelect(elements.cartesianAngleUnit);
jointAngleUnit = elements.jointAngleUnit.value as AngleUnit;
cartesianLengthUnit = elements.cartesianLengthUnit.value as LengthUnit;
cartesianAngleUnit = elements.cartesianAngleUnit.value as AngleUnit;
setSidebarCollapsed(sidebarCollapsed);
setRightSidebarCollapsed(rightSidebarCollapsed);
setRightPanelTab("robot");
clearRosInfoState();
refreshUiText();

rosClient.onStateChange((state, detail) => {
  lastConnectionState = state;
  lastConnectionDetail = detail;
  updateStatusLabel(state, detail);
  updateConnectButtonText();
});

elements.languageToggleBtn.addEventListener("click", () => {
  currentLanguage = currentLanguage === "en" ? "zh" : "en";
  saveLanguage(currentLanguage);
  refreshUiText();
});

elements.themeToggleBtn.addEventListener("click", () => {
  currentTheme = currentTheme === "dark" ? "light" : "dark";
  saveTheme(currentTheme);
  applyTheme(currentTheme);
  sceneManager.setTheme(currentTheme);
  updateThemeToggleUi();
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

elements.rightTabRobot.addEventListener("click", () => {
  setRightPanelTab("robot");
});

elements.rightTabRosInfo.addEventListener("click", () => {
  setRightPanelTab("rosInfo");
});

elements.plannedTrajectoryToggleBtn.addEventListener("click", () => {
  plannedTrajectoryVisible = !plannedTrajectoryVisible;
  sceneManager.setPlannedTrajectoryVisible(plannedTrajectoryVisible);
  updatePlannedTrajectoryToggleUi();
});

elements.tfToggleAllBtn.addEventListener("click", () => {
  lastNonSelectionTfVisibilityMode = lastNonSelectionTfVisibilityMode === "all" ? "none" : "all";
  if (selectedTfFrames.size === 0) {
    tfVisibilityMode = lastNonSelectionTfVisibilityMode;
  }
  tfFilterMenuOpen = false;
  renderTfTree();
});

elements.tfSelectBtn.addEventListener("click", (event) => {
  event.stopPropagation();
  if (elements.tfSelectBtn.disabled) {
    return;
  }
  tfFilterMenuOpen = !tfFilterMenuOpen;
  renderTfTree();
});

elements.tfFilterMenu.addEventListener("click", (event) => {
  event.stopPropagation();
});

document.addEventListener("click", (event) => {
  const target = event.target;
  if (!(target instanceof HTMLElement)) {
    return;
  }
  if (!tfFilterMenuOpen) {
    return;
  }
  if (target.closest("#tfFilterMenu") || target.closest("#tfSelectBtn")) {
    return;
  }
  tfFilterMenuOpen = false;
  renderTfTree();
});

elements.loadAllParamsBtn.addEventListener("click", () => {
  void showAllParamValues();
});

elements.detailModalClose.addEventListener("click", () => {
  hideDetailModal();
});

elements.detailModalBackdrop.addEventListener("click", () => {
  hideDetailModal();
});

window.addEventListener("keydown", (event) => {
  if (event.key === "Escape") {
    hideDetailModal();
  }
});

elements.jointAngleUnit.addEventListener("change", () => {
  jointAngleUnit = elements.jointAngleUnit.value as AngleUnit;
  scheduleRightPanelUpdate();
});

elements.cartesianLengthUnit.addEventListener("change", () => {
  cartesianLengthUnit = elements.cartesianLengthUnit.value as LengthUnit;
  scheduleRightPanelUpdate();
});

elements.cartesianAngleUnit.addEventListener("change", () => {
  cartesianAngleUnit = elements.cartesianAngleUnit.value as AngleUnit;
  scheduleRightPanelUpdate();
});

elements.cartesianFrame.addEventListener("change", () => {
  if (isPlaying) {
    elements.cartesianFrame.value = cartesianFrame;
    return;
  }
  cartesianFrame = elements.cartesianFrame.value;
  sceneManager.setEndEffectorFrame(cartesianFrame);
  refreshLatestPlannedTrajectoryPreview();
  log(`cartesian frame changed: ${cartesianFrame || "(none)"}`);
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
    log(`trajectory record stopped (${trajectory.length} frame${trajectory.length === 1 ? "" : "s"})`);
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
  log("trajectory record armed");
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
  if (isPlaying) {
    pausePlayback();
    log("trajectory paused");
    return;
  }

  if (trajectory.length === 0) {
    return;
  }

  const resume = isPlaybackPaused;
  startPlayback();
  log(resume ? "trajectory resumed" : "trajectory play");
});

elements.trajClearBtn.addEventListener("click", () => {
  clearTrajectory();
  log("trajectory cleared");
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

  enableEndEffectorPreview();
  pausePlayback();
  setPlaybackIndex(index);
  log(`trajectory scrubbed: ${Math.round(percent * 100)}%`);
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

log(t(currentLanguage, "log.ready"));







































































