import "./style.css";
import { Vector3 } from "three";
import { RuntimeConfig, defaultConfig, loadConfig, saveConfig } from "./config";
import { decodePointCloud2 } from "./ros/pointcloud";
import { ConnectionState, MessageDetailsResponse, MessageTypeDef, NodeDetails, RosClient, ServiceInfo, TopicInfo } from "./ros/rosClient";
import { loadRvizSyncHints } from "./rviz/rvizConfig";
import { loadRobotModel } from "./visualization/robotLoader";
import { RobotJointConnection, RobotJointDetail, RobotLinkDetail, SceneManager } from "./visualization/sceneManager";
import { applyStaticTranslations, applyTheme, Language, loadLanguage, loadTheme, saveLanguage, saveTheme, t, ThemeMode } from "./uiSettings";
import { enhanceSelect, refreshEnhancedSelect } from "./customSelect";
import { TrajectoryChartSeries, TrajectoryChartTheme, renderTrajectoryLineChart } from "./trajectoryCharts";

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
  rosGraphToggleBtn: HTMLButtonElement;
  trajectoryChartsToggleBtn: HTMLButtonElement;
  resetViewBtn: HTMLButtonElement;
  rightSidebarToggleBtn: HTMLButtonElement;
  fixedFrameValue: HTMLElement;
  connectionStatus: HTMLElement;
  logBox: HTMLElement;
  viewport: HTMLElement;
  jointValues: HTMLElement;
  jointAngleUnit: HTMLSelectElement;
  jointAngleDegBtn: HTMLButtonElement;
  jointAngleRadBtn: HTMLButtonElement;
  cartesianFrame: HTMLSelectElement;
  cartesianLengthUnit: HTMLSelectElement;
  cartesianLengthMmBtn: HTMLButtonElement;
  cartesianLengthMBtn: HTMLButtonElement;
  cartesianAngleUnit: HTMLSelectElement;
  cartesianAngleDegBtn: HTMLButtonElement;
  cartesianAngleRadBtn: HTMLButtonElement;
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
  rosGraphModal: HTMLElement;
  rosGraphModalBackdrop: HTMLElement;
  rosGraphSearch: HTMLInputElement;
  rosGraphRefreshBtn: HTMLButtonElement;
  rosGraphFitBtn: HTMLButtonElement;
  rosGraphModeBtn: HTMLButtonElement;
  rosGraphCloseBtn: HTMLButtonElement;
  rosGraphMeta: HTMLElement;
  rosGraphBanner: HTMLElement;
  rosGraphSvg: SVGSVGElement;
  rosGraphViewportGroup: SVGGElement;
  rosGraphEmpty: HTMLElement;
  rosGraphDetail: HTMLElement;
  trajectoryChartsModal: HTMLElement;
  trajectoryChartsModalBackdrop: HTMLElement;
  trajectoryChartsCloseBtn: HTMLButtonElement;
  trajectoryChartsMeta: HTMLElement;
  trajectoryChartsStatusValue: HTMLElement;
  trajectoryChartsDurationValue: HTMLElement;
  trajectoryChartsSamplesValue: HTMLElement;
  trajectoryChartsTcpFrameValue: HTMLElement;
  trajectoryChartsJointChips: HTMLElement;
  trajectoryChartsShowAllBtn: HTMLButtonElement;
  trajectoryChartsResetBtn: HTMLButtonElement;
  trajectoryChartsPositionDegBtn: HTMLButtonElement;
  trajectoryChartsPositionRadBtn: HTMLButtonElement;
  trajectoryChartsTcpMmBtn: HTMLButtonElement;
  trajectoryChartsTcpMBtn: HTMLButtonElement;
  trajectoryChartsVelocityDegBtn: HTMLButtonElement;
  trajectoryChartsVelocityRadBtn: HTMLButtonElement;
  trajectoryChartsAccelerationDegBtn: HTMLButtonElement;
  trajectoryChartsAccelerationRadBtn: HTMLButtonElement;
  trajectoryChartsPositionCanvas: HTMLCanvasElement;
  trajectoryChartsTcpCanvas: HTMLCanvasElement;
  trajectoryChartsVelocityCanvas: HTMLCanvasElement;
  trajectoryChartsAccelerationCanvas: HTMLCanvasElement;
  trajectoryChartsTorqueCanvas: HTMLCanvasElement;
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

interface JointStatePayload {
  names: string[];
  positions: number[];
  velocities?: number[];
  efforts?: number[];
}

interface TrajectoryPointPayload {
  positions?: number[];
  velocities?: number[];
  accelerations?: number[];
  effort?: number[];
  time_from_start?: {
    secs?: number;
    nsecs?: number;
    sec?: number;
    nanosec?: number;
  };
}

interface MotionSample {
  t: number;
  positions: number[];
  velocities?: number[];
  accelerations?: number[];
  efforts?: number[];
  tcpPosition: { x: number; y: number; z: number } | null;
}

interface ActiveMotionSegment {
  jointNames: string[];
  tcpFrame: string;
  startedAt: number;
  samples: MotionSample[];
  lastMovementAt: number;
  lastSampleAt: number;
}

interface MotionSegmentSnapshot {
  jointNames: string[];
  tcpFrame: string;
  startedAt: number;
  endedAt: number;
  samples: MotionSample[];
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

interface PlannedTrajectoryPose {
  translation: Vector3;
  rotation: { x: number; y: number; z: number; w: number };
}

interface PlannedTrajectoryPreview {
  path: Vector3[];
  frame: string;
  startPose: PlannedTrajectoryPose | null;
  endPose: PlannedTrajectoryPose | null;
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

interface RosGraphNodeEntity {
  id: string;
  kind: "node";
  name: string;
  publishing: string[];
  subscribing: string[];
  services: string[];
  x: number;
  y: number;
  width: number;
  height: number;
}

interface RosGraphTopicEntity {
  id: string;
  kind: "topic";
  name: string;
  type: string;
  publishers: string[];
  subscribers: string[];
  x: number;
  y: number;
  width: number;
  height: number;
}

type RosGraphEntity = RosGraphNodeEntity | RosGraphTopicEntity;

type RosGraphViewMode = "detail" | "nodeCommunication";

interface RosGraphEdge {
  id: string;
  sourceId: string;
  targetId: string;
  kind: "publish" | "subscribe" | "nodeCommunication";
  labels?: string[];
}

interface RosGraphBounds {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
}

interface RosGraphViewSnapshot {
  entities: RosGraphEntity[];
  entityById: Map<string, RosGraphEntity>;
  edges: RosGraphEdge[];
  adjacency: Map<string, Set<string>>;
}

interface RosGraphSnapshot {
  detailView: RosGraphViewSnapshot;
  nodeCommunicationView: RosGraphViewSnapshot;
  updatedAt: number;
  partialFailures: string[];
}

interface RosGraphVisibleData {
  entities: RosGraphEntity[];
  entityIds: Set<string>;
  edges: RosGraphEdge[];
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
const VIEW_ICON_RESET = '<svg viewBox="0 0 24 24" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><path d="M12 3.8 7.2 6.8 12 9.8l4.8-3-4.8-3Z"/><path d="M7.2 6.8V13L12 16V9.8"/><path d="M16.8 6.8V13L12 16"/><path d="M6.4 16.3a7.2 7.2 0 0 0 11 .5"/><path d="M18 16.8v2.7h-2.7"/></svg>';
const SIDEBAR_ICON_HIDE = '<svg viewBox="0 0 24 24" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><rect x="3" y="3" width="18" height="18" rx="3.8"/><path d="M8.4 4.8v14.4"/><path d="M14.7 12h-5"/><path d="m12.2 9.5-2.5 2.5 2.5 2.5"/></svg>';
const SIDEBAR_ICON_SHOW = '<svg viewBox="0 0 24 24" aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"><rect x="3" y="3" width="18" height="18" rx="3.8"/><path d="M8.4 4.8v14.4"/><path d="M9.3 12h5"/><path d="m11.8 9.5 2.5 2.5-2.5 2.5"/></svg>';
const ROS_GRAPH_ICON = '<svg viewBox="0 0 1024 1024" aria-hidden="true"><path d="M409.088 622.08l-68.608 51.712c18.432 49.664-7.168 104.96-56.832 123.392s-104.96-7.168-123.392-56.832 7.168-104.96 56.832-123.392c32.768-12.288 69.632-5.12 95.744 17.408l70.144-53.248c-37.376-84.992 1.024-184.32 86.016-221.696 42.496-18.944 90.624-18.944 133.632-1.024l24.064-53.76c-43.52-30.72-53.76-90.624-23.04-133.632 30.72-43.52 90.624-53.76 133.632-23.04 43.52 30.72 53.76 90.624 23.04 133.632-19.968 28.672-54.272 44.032-88.576 39.936L644.608 384c38.4 31.744 60.928 79.36 60.928 129.536 0 41.984-15.36 79.872-40.448 109.568l74.24 92.16c48.64-21.504 104.96 0 126.976 48.64 21.504 48.64 0 104.96-48.64 126.976-48.64 21.504-104.96 0-126.976-48.64-14.336-31.744-10.24-69.12 10.752-96.768L628.224 655.36c-26.112 16.896-57.856 27.136-91.136 27.136-49.152-1.024-95.744-23.04-128-60.416z" fill="currentColor"/></svg>';
const ROS_GRAPH_REFRESH_ICON = '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M20 11a8 8 0 1 1-2.34-5.66" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"/><path d="M20 4v5h-5" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"/></svg>';
const ROS_GRAPH_FIT_ICON = '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M8 4H4v4M16 4h4v4M20 16v4h-4M8 20H4v-4" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"/><path d="M9 9l-5-5M15 9l5-5M15 15l5 5M9 15l-5 5" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"/></svg>';
const ROS_GRAPH_MODE_ICON = '<svg viewBox="0 0 24 24" aria-hidden="true"><circle cx="6.5" cy="12" r="2"/><circle cx="17.5" cy="6.5" r="2"/><circle cx="17.5" cy="17.5" r="2"/><path d="M8.6 11.2 14.9 7.4M8.6 12.8 14.9 16.6M10.6 12h2.8" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"/></svg>';
const ROS_GRAPH_CLOSE_ICON = '<svg viewBox="0 0 24 24" aria-hidden="true"><path d="M6 6 18 18M18 6 6 18" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"/></svg>';
const TRAJECTORY_CHARTS_ICON = '<svg viewBox="0 0 1024 1024" aria-hidden="true"><path d="M527.8 487.7l-15.3 16.6c-6.2-3.5-13.3-5.7-21-5.7-5.4 0-10.5 1.1-15.3 3L416.1 413c-0.1-0.2-0.4-0.3-0.5-0.5 6.7-6.5 10.9-15.6 10.9-25.6 0-19.8-16.1-35.8-35.8-35.8-19.8 0-35.8 16.1-35.8 35.8 0 7.1 2.2 13.8 5.8 19.3L291.8 475c-6-4.4-13.2-7.1-21.2-7.1-19.8 0-35.8 16.1-35.8 35.8 0 19.8 16.1 35.8 35.8 35.8 19.8 0 35.8-16.1 35.8-35.8 0-4.7-1-9.2-2.6-13.3l71.4-71.4c4.7 2.3 9.9 3.6 15.4 3.6 2.9 0 5.7-0.4 8.4-1.1 0.3 0.8 0.5 1.6 1 2.3l60.1 88.5c-7 7.6-11.4 17.7-11.4 28.9 0 23.6 19.2 42.8 42.8 42.8 23.6 0 42.8-19.2 42.8-42.8 0-8.9-2.7-17.1-7.4-24l15.2-16.5-14.3-13z m-257.2 32.5c-9 0-16.4-7.3-16.4-16.4 0-9 7.3-16.4 16.4-16.4s16.4 7.3 16.4 16.4c0 9-7.4 16.4-16.4 16.4z m120.1-116.9c-9 0-16.4-7.3-16.4-16.4 0-9 7.3-16.4 16.4-16.4 9 0 16.4 7.3 16.4 16.4-0.1 9-7.4 16.4-16.4 16.4zM664 302.5c-25.5 0-46.3 20.8-46.3 46.3 0 8.4 2.4 16.2 6.4 23l-55.3 59.9 21.5 19.8 55.5-60.2c5.6 2.4 11.8 3.8 18.2 3.8 25.5 0 46.3-20.8 46.3-46.3 0.1-25.6-20.7-46.3-46.3-46.3z m0 63.4c-9.4 0-17.1-7.7-17.1-17.1 0-9.4 7.7-17.1 17.1-17.1 9.4 0 17.1 7.7 17.1 17.1 0.1 9.4-7.6 17.1-17.1 17.1z" fill="currentColor"/><path d="M854.8 578.5l-67.3-97.6c35.3-33 57.6-79.9 57.6-132.1 0-99.9-81-180.9-180.9-180.9s-180.9 81-180.9 180.9 81 180.9 180.9 180.9c26.8 0 52.2-6 75-16.4l67.8 98.2c8.8 12.7 26.4 15.9 39.1 7.2l1.6-1.1c12.6-8.8 15.8-26.4 7.1-39.1zM664 494.3c-80.4 0-145.5-65.2-145.5-145.5S583.7 203.2 664 203.2s145.5 65.2 145.5 145.5S744.4 494.3 664 494.3z" fill="currentColor"/><path d="M716.6 649.1c0 10.9-8.9 19.7-19.7 19.7H237.3c-10.9 0-19.7-8.9-19.7-19.7V265.8c0-10.9 8.9-19.7 19.7-19.7h221.4c5.5-11.3 11.9-22.2 19.1-32.4H220.4c-23 0-41.8 18.8-41.8 41.8v412.4c0 23 18.8 41.8 41.8 41.8h216.9v51.8H319.2c-12.7 0-23.1 10.4-23.1 23.1v1.6c0 12.7 10.4 23.1 23.1 23.1h295.7c12.7 0 23.1-10.4 23.1-23.1v-1.6c0-12.7-10.4-23.1-23.1-23.1H496.8v-51.8h216.9c23 0 41.8-18.8 41.8-41.8v-84.7c-12.5 4.2-25.5 7.5-38.9 9.6v56.3z" fill="currentColor"/></svg>';
const ROS_GRAPH_MIN_SCALE = 0.3;
const ROS_GRAPH_MAX_SCALE = 2.4;
const ROS_GRAPH_LAYER_GAP = 280;
const ROS_GRAPH_ROW_GAP = 30;
const ROS_GRAPH_COMPONENT_GAP = 110;
const ROS_GRAPH_FIT_PADDING = 56;
const ROS_GRAPH_NODE_HEIGHT = 42;
const ROS_GRAPH_TOPIC_HEIGHT = 36;
const DEFAULT_ASSET_SERVER_PORT = "8081";
const SVG_NS = "http://www.w3.org/2000/svg";

function getElements(): AppElements {
  const byId = <T extends Element>(id: string): T => {
    const element = document.getElementById(id);
    if (!element) {
      throw new Error(`missing element: ${id}`);
    }
    return element as unknown as T;
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
    rosGraphToggleBtn: byId<HTMLButtonElement>("rosGraphToggleBtn"),
    trajectoryChartsToggleBtn: byId<HTMLButtonElement>("trajectoryChartsToggleBtn"),
    resetViewBtn: byId<HTMLButtonElement>("resetViewBtn"),
    rightSidebarToggleBtn: byId<HTMLButtonElement>("rightSidebarToggleBtn"),
    fixedFrameValue: byId<HTMLElement>("fixedFrameValue"),
    connectionStatus: byId<HTMLElement>("connectionStatus"),
    logBox: byId<HTMLElement>("logBox"),
    viewport: byId<HTMLElement>("viewport"),
    jointValues: byId<HTMLElement>("jointValues"),
    jointAngleUnit: byId<HTMLSelectElement>("jointAngleUnit"),
    jointAngleDegBtn: byId<HTMLButtonElement>("jointAngleDegBtn"),
    jointAngleRadBtn: byId<HTMLButtonElement>("jointAngleRadBtn"),
    cartesianFrame: byId<HTMLSelectElement>("cartesianFrame"),
    cartesianLengthUnit: byId<HTMLSelectElement>("cartesianLengthUnit"),
    cartesianLengthMmBtn: byId<HTMLButtonElement>("cartesianLengthMmBtn"),
    cartesianLengthMBtn: byId<HTMLButtonElement>("cartesianLengthMBtn"),
    cartesianAngleUnit: byId<HTMLSelectElement>("cartesianAngleUnit"),
    cartesianAngleDegBtn: byId<HTMLButtonElement>("cartesianAngleDegBtn"),
    cartesianAngleRadBtn: byId<HTMLButtonElement>("cartesianAngleRadBtn"),
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
    detailModalClose: byId<HTMLButtonElement>("detailModalClose"),
    rosGraphModal: byId<HTMLElement>("rosGraphModal"),
    rosGraphModalBackdrop: byId<HTMLElement>("rosGraphModalBackdrop"),
    rosGraphSearch: byId<HTMLInputElement>("rosGraphSearch"),
    rosGraphRefreshBtn: byId<HTMLButtonElement>("rosGraphRefreshBtn"),
    rosGraphFitBtn: byId<HTMLButtonElement>("rosGraphFitBtn"),
    rosGraphModeBtn: byId<HTMLButtonElement>("rosGraphModeBtn"),
    rosGraphCloseBtn: byId<HTMLButtonElement>("rosGraphCloseBtn"),
    rosGraphMeta: byId<HTMLElement>("rosGraphMeta"),
    rosGraphBanner: byId<HTMLElement>("rosGraphBanner"),
    rosGraphSvg: byId<SVGSVGElement>("rosGraphSvg"),
    rosGraphViewportGroup: byId<SVGGElement>("rosGraphViewportGroup"),
    rosGraphEmpty: byId<HTMLElement>("rosGraphEmpty"),
    rosGraphDetail: byId<HTMLElement>("rosGraphDetail"),
    trajectoryChartsModal: byId<HTMLElement>("trajectoryChartsModal"),
    trajectoryChartsModalBackdrop: byId<HTMLElement>("trajectoryChartsModalBackdrop"),
    trajectoryChartsCloseBtn: byId<HTMLButtonElement>("trajectoryChartsCloseBtn"),
    trajectoryChartsMeta: byId<HTMLElement>("trajectoryChartsMeta"),
    trajectoryChartsStatusValue: byId<HTMLElement>("trajectoryChartsStatusValue"),
    trajectoryChartsDurationValue: byId<HTMLElement>("trajectoryChartsDurationValue"),
    trajectoryChartsSamplesValue: byId<HTMLElement>("trajectoryChartsSamplesValue"),
    trajectoryChartsTcpFrameValue: byId<HTMLElement>("trajectoryChartsTcpFrameValue"),
    trajectoryChartsJointChips: byId<HTMLElement>("trajectoryChartsJointChips"),
    trajectoryChartsShowAllBtn: byId<HTMLButtonElement>("trajectoryChartsShowAllBtn"),
    trajectoryChartsResetBtn: byId<HTMLButtonElement>("trajectoryChartsResetBtn"),
    trajectoryChartsPositionDegBtn: byId<HTMLButtonElement>("trajectoryChartsPositionDegBtn"),
    trajectoryChartsPositionRadBtn: byId<HTMLButtonElement>("trajectoryChartsPositionRadBtn"),
    trajectoryChartsTcpMmBtn: byId<HTMLButtonElement>("trajectoryChartsTcpMmBtn"),
    trajectoryChartsTcpMBtn: byId<HTMLButtonElement>("trajectoryChartsTcpMBtn"),
    trajectoryChartsVelocityDegBtn: byId<HTMLButtonElement>("trajectoryChartsVelocityDegBtn"),
    trajectoryChartsVelocityRadBtn: byId<HTMLButtonElement>("trajectoryChartsVelocityRadBtn"),
    trajectoryChartsAccelerationDegBtn: byId<HTMLButtonElement>("trajectoryChartsAccelerationDegBtn"),
    trajectoryChartsAccelerationRadBtn: byId<HTMLButtonElement>("trajectoryChartsAccelerationRadBtn"),
    trajectoryChartsPositionCanvas: byId<HTMLCanvasElement>("trajectoryChartsPositionCanvas"),
    trajectoryChartsTcpCanvas: byId<HTMLCanvasElement>("trajectoryChartsTcpCanvas"),
    trajectoryChartsVelocityCanvas: byId<HTMLCanvasElement>("trajectoryChartsVelocityCanvas"),
    trajectoryChartsAccelerationCanvas: byId<HTMLCanvasElement>("trajectoryChartsAccelerationCanvas"),
    trajectoryChartsTorqueCanvas: byId<HTMLCanvasElement>("trajectoryChartsTorqueCanvas")
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
let rosGraphOpen = false;
let rosGraphBusy = false;
let rosGraphSnapshot: RosGraphSnapshot | null = null;
let rosGraphError: string | null = null;
let rosGraphSelectedId: string | null = null;
let rosGraphSearchQuery = "";
let rosGraphViewMode: RosGraphViewMode = "detail";
let rosGraphPanX = 0;
let rosGraphPanY = 0;
let rosGraphScale = 1;
let rosGraphContentBounds: RosGraphBounds | null = null;
let rosGraphPointerState: { pointerId: number; startX: number; startY: number; panX: number; panY: number } | null = null;
let trajectoryChartsOpen = false;
let trajectoryChartsPositionUnit: AngleUnit = "deg";
let trajectoryChartsTcpUnit: LengthUnit = "mm";
let trajectoryChartsVelocityUnit: AngleUnit = "deg";
let trajectoryChartsAccelerationUnit: AngleUnit = "deg";
let activeMotionSegment: ActiveMotionSegment | null = null;
let latestMotionSegment: MotionSegmentSnapshot | null = null;
let motionLastPositions: Map<string, number> | null = null;
let trajectoryChartsVisibleJoints = new Set<string>();
let trajectoryChartsJointSignature = "";
let pendingTrajectoryChartsRender = false;


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
  const label = sidebarCollapsed ? t(currentLanguage, "button.showSidebar") : t(currentLanguage, "button.hideSidebar");
  elements.sidebarToggleBtn.innerHTML = sidebarCollapsed ? SIDEBAR_ICON_SHOW : SIDEBAR_ICON_HIDE;
  elements.sidebarToggleBtn.title = label;
  elements.sidebarToggleBtn.setAttribute("aria-label", label);
  elements.sidebarToggleBtn.setAttribute("aria-expanded", sidebarCollapsed ? "false" : "true");
}

function updatePlannedTrajectoryToggleUi(): void {
  const toggleKey = plannedTrajectoryVisible ? "button.hidePlannedTrajectory" : "button.showPlannedTrajectory";
  const label = t(currentLanguage, toggleKey as "button.hidePlannedTrajectory" | "button.showPlannedTrajectory");
  elements.plannedTrajectoryToggleBtn.title = label;
  elements.plannedTrajectoryToggleBtn.setAttribute("aria-label", label);
  elements.plannedTrajectoryToggleBtn.setAttribute("aria-pressed", plannedTrajectoryVisible ? "true" : "false");
  elements.plannedTrajectoryToggleBtn.classList.toggle("is-active", plannedTrajectoryVisible);
}

function updateRosGraphToggleUi(): void {
  const toggleKey = rosGraphOpen ? "button.hideRosGraph" : "button.showRosGraph";
  const label = t(currentLanguage, toggleKey as "button.hideRosGraph" | "button.showRosGraph");
  elements.rosGraphToggleBtn.title = label;
  elements.rosGraphToggleBtn.setAttribute("aria-label", label);
  elements.rosGraphToggleBtn.setAttribute("aria-pressed", rosGraphOpen ? "true" : "false");
  elements.rosGraphToggleBtn.classList.toggle("is-active", rosGraphOpen);
}

function updateTrajectoryChartsToggleUi(): void {
  const toggleKey = trajectoryChartsOpen ? "button.hideTrajectoryCharts" : "button.showTrajectoryCharts";
  const label = t(currentLanguage, toggleKey as "button.hideTrajectoryCharts" | "button.showTrajectoryCharts");
  elements.trajectoryChartsToggleBtn.title = label;
  elements.trajectoryChartsToggleBtn.setAttribute("aria-label", label);
  elements.trajectoryChartsToggleBtn.setAttribute("aria-pressed", trajectoryChartsOpen ? "true" : "false");
  elements.trajectoryChartsToggleBtn.classList.toggle("is-active", trajectoryChartsOpen);
}

function updateResetViewButtonUi(): void {
  const label = t(currentLanguage, "button.resetView");
  elements.resetViewBtn.title = label;
  elements.resetViewBtn.setAttribute("aria-label", label);
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
  updateRosGraphToggleUi();
  updateTrajectoryChartsToggleUi();
  updateResetViewButtonUi();
  updateRightSidebarToggleUi();
  updateTabLabels();
  updateStatusLabel(lastConnectionState, lastConnectionDetail);
  renderRosGraphModal();
  renderTrajectoryChartsModal();
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
): { path: Vector3[]; startPose: PlannedTrajectoryPose | null; endPose: PlannedTrajectoryPose | null } {
  if (jointNames.length === 0 || points.length < 2 || !targetFrame) {
    return { path: [], startPose: null, endPose: null };
  }

  const restoreState = jointState.length > 0
    ? {
        names: jointState.map((joint) => joint.name),
        positions: jointState.map((joint) => joint.position)
      }
    : fallbackRestoreState;

  if (!restoreState || restoreState.names.length === 0 || restoreState.positions.length === 0) {
    return { path: [], startPose: null, endPose: null };
  }

  const sampledIndices = new Set<number>([0, points.length - 1]);
  const stride = Math.max(1, Math.ceil(points.length / MAX_PLANNED_TRAJECTORY_SAMPLES));
  for (let index = 0; index < points.length; index += stride) {
    sampledIndices.add(index);
  }

  const orderedIndices = Array.from(sampledIndices).sort((left, right) => left - right);
  const path: Vector3[] = [];
  let startPose: PlannedTrajectoryPose | null = null;
  let endPose: PlannedTrajectoryPose | null = null;

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
      const previewPose: PlannedTrajectoryPose = {
        translation: translation.clone(),
        rotation: {
          x: pose.rotation.x,
          y: pose.rotation.y,
          z: pose.rotation.z,
          w: pose.rotation.w
        }
      };
      if (!startPose) {
        startPose = previewPose;
      }
      endPose = previewPose;

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

  return { path, startPose, endPose };
}

function getCurrentTcpLinkFrame(): string {
  const rememberedValue = cartesianFrame.trim();
  if (rememberedValue) {
    return rememberedValue;
  }

  const preferred = sceneManager.getDefaultEndEffectorFrame().trim();
  if (preferred) {
    return preferred;
  }

  const directValue = elements.cartesianFrame.value.trim();
  if (directValue) {
    return directValue;
  }

  const frames = sceneManager.getLinkList();
  if (frames.length === 0) {
    return "";
  }

  return frames[frames.length - 1];
}

function buildPlannedTrajectoryPreview(payload: PlannedTrajectoryPayload): PlannedTrajectoryPreview | null {
  if (!payload.targetFrame) {
    return null;
  }

  const preview = buildPlannedTrajectoryPathForFrame(
    payload.jointNames,
    payload.points,
    payload.targetFrame,
    payload.restoreState
  );

  if (preview.path.length < 2) {
    return null;
  }

  return {
    path: preview.path,
    frame: payload.targetFrame,
    startPose: preview.startPose,
    endPose: preview.endPose
  };
}

function tryRenderPlannedTrajectory(payload: PlannedTrajectoryPayload, logSuccess = true): boolean {
  const preview = buildPlannedTrajectoryPreview(payload);
  if (!preview) {
    return false;
  }

  pendingPlannedTrajectory = null;
  sceneManager.setPlannedTrajectoryPath(preview.path, preview.startPose, preview.endPose);
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
        points?: TrajectoryPointPayload[];
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

function trajectoryPointTimeToMs(point: TrajectoryPointPayload, fallbackMs: number): number {
  const secs = typeof point.time_from_start?.secs === "number"
    ? point.time_from_start.secs
    : typeof point.time_from_start?.sec === "number"
      ? point.time_from_start.sec
      : 0;
  const nsecs = typeof point.time_from_start?.nsecs === "number"
    ? point.time_from_start.nsecs
    : typeof point.time_from_start?.nanosec === "number"
      ? point.time_from_start.nanosec
      : 0;
  const totalMs = secs * 1000 + nsecs / 1e6;
  return Number.isFinite(totalMs) && totalMs >= 0 ? totalMs : fallbackMs;
}

function normalizeTrajectoryPointValues(values: number[] | undefined, jointCount: number): number[] | undefined {
  if (!Array.isArray(values) || values.length === 0) {
    return undefined;
  }

  const normalized = new Array<number>(jointCount).fill(Number.NaN);
  let hasFinite = false;
  const count = Math.min(jointCount, values.length);
  for (let index = 0; index < count; index += 1) {
    const value = values[index];
    if (typeof value !== "number" || !Number.isFinite(value)) {
      continue;
    }
    normalized[index] = value;
    hasFinite = true;
  }
  return hasFinite ? normalized : undefined;
}

function sampleTcpPositionsForTrajectory(
  jointNames: string[],
  points: TrajectoryPointPayload[],
  tcpFrame: string
): Array<{ x: number; y: number; z: number } | null> {
  if (!tcpFrame || jointNames.length === 0 || points.length === 0) {
    return points.map(() => null);
  }

  const restoreNames = jointState.map((joint) => joint.name);
  const restorePositions = jointState.map((joint) => joint.position);
  const tcpPositions: Array<{ x: number; y: number; z: number } | null> = [];

  try {
    for (const point of points) {
      const rawPositions = Array.isArray(point.positions) ? point.positions : [];
      const names: string[] = [];
      const positions: number[] = [];
      const count = Math.min(jointNames.length, rawPositions.length);
      for (let index = 0; index < count; index += 1) {
        const value = rawPositions[index];
        if (typeof value !== "number" || !Number.isFinite(value)) {
          continue;
        }
        names.push(jointNames[index]);
        positions.push(value);
      }

      if (names.length === 0) {
        tcpPositions.push(null);
        continue;
      }

      sceneManager.updateJointStates({ name: names, position: positions });
      const tcpTransform = sceneManager.getRobotRelativeTransform(tcpFrame) ?? sceneManager.getRelativeTransform(tcpFrame);
      tcpPositions.push(tcpTransform ? {
        x: tcpTransform.translation.x,
        y: tcpTransform.translation.y,
        z: tcpTransform.translation.z
      } : null);
    }
  } finally {
    if (restoreNames.length > 0) {
      sceneManager.updateJointStates({ name: restoreNames, position: restorePositions });
    }
  }

  return tcpPositions;
}

function setTrajectoryChartsFromJointTrajectory(jointNames: string[], points: TrajectoryPointPayload[]): void {
  const normalizedJointNames = orderedUniqueStrings(jointNames);
  if (normalizedJointNames.length === 0 || points.length === 0) {
    clearTrajectoryChartsState();
    if (trajectoryChartsOpen) {
      renderTrajectoryChartsModal();
    }
    return;
  }

  const tcpFrame = resolveTrajectoryChartsTcpFrame();
  const tcpPositions = sampleTcpPositionsForTrajectory(normalizedJointNames, points, tcpFrame);
  const samples: MotionSample[] = [];
  let lastTimeMs = 0;

  for (let index = 0; index < points.length; index += 1) {
    const point = points[index];
    const fallbackMs = index === 0 ? 0 : lastTimeMs + TRAJ_SAMPLE_INTERVAL_MS;
    const timeMs = Math.max(lastTimeMs, trajectoryPointTimeToMs(point, fallbackMs));
    lastTimeMs = timeMs;
    samples.push({
      t: timeMs,
      positions: normalizeTrajectoryPointValues(point.positions, normalizedJointNames.length) ?? new Array<number>(normalizedJointNames.length).fill(Number.NaN),
      velocities: normalizeTrajectoryPointValues(point.velocities, normalizedJointNames.length),
      accelerations: normalizeTrajectoryPointValues(point.accelerations, normalizedJointNames.length),
      efforts: normalizeTrajectoryPointValues(point.effort, normalizedJointNames.length),
      tcpPosition: tcpPositions[index] ?? null
    });
  }

  activeMotionSegment = null;
  motionLastPositions = null;
  latestMotionSegment = {
    jointNames: normalizedJointNames,
    tcpFrame,
    startedAt: 0,
    endedAt: lastTimeMs,
    samples
  };
  ensureTrajectoryChartsVisibleJoints(latestMotionSegment);
  if (trajectoryChartsOpen) {
    renderTrajectoryChartsModal();
  }
}

function updateTrajectoryChartsFromMoveItResult(resultMessage: {
  status?: { status?: number };
  result?: {
    error_code?: { val?: number };
    planned_trajectory?: {
      joint_trajectory?: {
        joint_names?: string[];
        points?: TrajectoryPointPayload[];
      };
    };
  };
}): void {
  const statusCode = typeof resultMessage.status?.status === "number" ? resultMessage.status.status : -1;
  const errorCode = typeof resultMessage.result?.error_code?.val === "number" ? resultMessage.result.error_code.val : undefined;
  if (!isMoveItPlanSuccessful(statusCode, errorCode)) {
    clearTrajectoryChartsState();
    if (trajectoryChartsOpen) {
      renderTrajectoryChartsModal();
    }
    return;
  }

  const jointTrajectory = resultMessage.result?.planned_trajectory?.joint_trajectory;
  const jointNames = Array.isArray(jointTrajectory?.joint_names)
    ? jointTrajectory.joint_names.filter((name): name is string => typeof name === "string" && name.length > 0)
    : [];
  const points = Array.isArray(jointTrajectory?.points) ? jointTrajectory.points : [];
  setTrajectoryChartsFromJointTrajectory(jointNames, points);
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
    const resultMessage = message as {
      status?: { status?: number };
      result?: {
        planning_time?: number;
        error_code?: { val?: number };
        planned_trajectory?: {
          joint_trajectory?: {
            joint_names?: string[];
            points?: TrajectoryPointPayload[];
          };
        };
      };
    };

    updateTrajectoryChartsFromMoveItResult(resultMessage);
    if (!moveItPreviewArmed) {
      return;
    }

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
  let shouldRefresh = false;

  if (frames.length === 0) {
    const hadOptions = elements.cartesianFrame.options.length > 0 || lastFrameOptions.length > 0;
    if (hadOptions) {
      elements.cartesianFrame.innerHTML = "";
      lastFrameOptions = [];
      shouldRefresh = true;
    }
    if (cartesianFrame) {
      cartesianFrame = "";
      sceneManager.setEndEffectorFrame("");
    }
    if (shouldRefresh) {
      refreshEnhancedSelect(elements.cartesianFrame);
    }
    return;
  }

  const optionsChanged =
    frames.length !== lastFrameOptions.length ||
    frames.some((frame, index) => frame !== lastFrameOptions[index]);

  if (optionsChanged) {
    elements.cartesianFrame.innerHTML = "";
    for (const frame of frames) {
      const option = document.createElement("option");
      option.value = frame;
      option.textContent = frame;
      elements.cartesianFrame.appendChild(option);
    }
    lastFrameOptions = [...frames];
    shouldRefresh = true;
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

  const defaultEndEffectorFrame = sceneManager.getDefaultEndEffectorFrame();
  const nextFrame =
    (cartesianFrame && frames.includes(cartesianFrame) ? cartesianFrame : "") ||
    (defaultEndEffectorFrame && frames.includes(defaultEndEffectorFrame) ? defaultEndEffectorFrame : "") ||
    (lastTfLink && frames.includes(lastTfLink) ? lastTfLink : "") ||
    (sceneManager.getRobotBaseFrame() && frames.includes(sceneManager.getRobotBaseFrame())
      ? sceneManager.getRobotBaseFrame()
      : "") ||
    (sceneManager.getFixedFrame() && frames.includes(sceneManager.getFixedFrame())
      ? sceneManager.getFixedFrame()
      : "") ||
    (elements.cartesianFrame.value && frames.includes(elements.cartesianFrame.value)
      ? elements.cartesianFrame.value
      : "") ||
    frames[frames.length - 1] ||
    frames[0];

  if (cartesianFrame !== nextFrame) {
    cartesianFrame = nextFrame;
    sceneManager.setEndEffectorFrame(cartesianFrame);
    shouldRefresh = true;
  }

  if (elements.cartesianFrame.value !== cartesianFrame) {
    elements.cartesianFrame.value = cartesianFrame;
    shouldRefresh = true;
  }

  if (shouldRefresh) {
    refreshEnhancedSelect(elements.cartesianFrame);
  }
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

  if (elements.cartesianFrame.disabled !== isPlaying) {
    elements.cartesianFrame.disabled = isPlaying;
    refreshEnhancedSelect(elements.cartesianFrame);
  }
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

  motionLastPositions = null;
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
  motionLastPositions = null;
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
    updateRightPanelUnitButtons();
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

function orderedUniqueStrings(values: string[]): string[] {
  const seen = new Set<string>();
  const result: string[] = [];
  for (const value of values) {
    if (!value || seen.has(value)) {
      continue;
    }
    seen.add(value);
    result.push(value);
  }
  return result;
}

function rateUnitLabel(unit: AngleUnit): string {
  return unit === "deg" ? "deg/s" : "rad/s";
}

function accelerationUnitLabel(unit: AngleUnit): string {
  return unit === "deg" ? "deg/s^2" : "rad/s^2";
}

function parseJointStatePayload(message: unknown): JointStatePayload | null {
  const payload = message as { name?: unknown; position?: unknown; velocity?: unknown; effort?: unknown };
  if (!payload || !Array.isArray(payload.name) || !Array.isArray(payload.position)) {
    return null;
  }

  const names: string[] = [];
  const positions: number[] = [];
  const velocities: number[] = [];
  const efforts: number[] = [];
  const velocityArray = Array.isArray(payload.velocity) ? payload.velocity : null;
  const effortArray = Array.isArray(payload.effort) ? payload.effort : null;
  const count = Math.min(payload.name.length, payload.position.length);
  for (let index = 0; index < count; index += 1) {
    const name = payload.name[index];
    const position = payload.position[index];
    if (typeof name !== "string" || !name || typeof position !== "number" || !Number.isFinite(position)) {
      continue;
    }
    names.push(name);
    positions.push(position);
    const velocity = velocityArray && typeof velocityArray[index] === "number" && Number.isFinite(velocityArray[index])
      ? velocityArray[index]
      : Number.NaN;
    velocities.push(velocity);
    const effort = effortArray && typeof effortArray[index] === "number" && Number.isFinite(effortArray[index])
      ? effortArray[index]
      : Number.NaN;
    efforts.push(effort);
  }

  if (names.length === 0) {
    return null;
  }

  return {
    names,
    positions,
    velocities: velocities.some((value) => Number.isFinite(value)) ? velocities : undefined,
    efforts: efforts.some((value) => Number.isFinite(value)) ? efforts : undefined
  };
}

function resolveTrajectoryChartsTcpFrame(): string {
  if (cartesianFrame) {
    return cartesianFrame;
  }
  return sceneManager.getDefaultEndEffectorFrame() || "";
}

function trajectoryChartsSegmentSignature(segment: { jointNames: string[] } | null): string {
  return segment ? segment.jointNames.join("|") : "";
}

function defaultTrajectoryChartsVisibleJoints(jointNames: string[]): Set<string> {
  return new Set(jointNames.length > 6 ? jointNames.slice(0, 6) : jointNames);
}

function ensureTrajectoryChartsVisibleJoints(segment: { jointNames: string[] } | null): void {
  if (!segment) {
    trajectoryChartsJointSignature = "";
    trajectoryChartsVisibleJoints.clear();
    return;
  }

  const signature = trajectoryChartsSegmentSignature(segment);
  if (signature !== trajectoryChartsJointSignature) {
    trajectoryChartsJointSignature = signature;
    trajectoryChartsVisibleJoints = defaultTrajectoryChartsVisibleJoints(segment.jointNames);
    return;
  }

  const validNames = new Set(segment.jointNames);
  for (const name of Array.from(trajectoryChartsVisibleJoints)) {
    if (!validNames.has(name)) {
      trajectoryChartsVisibleJoints.delete(name);
    }
  }

  if (trajectoryChartsVisibleJoints.size === 0 && segment.jointNames.length > 0) {
    trajectoryChartsVisibleJoints = defaultTrajectoryChartsVisibleJoints(segment.jointNames);
  }
}

function alignNumericValues(sourceNames: string[], sourceValues: number[], jointNames: string[]): number[] {
  const valueByName = new Map<string, number>();
  const count = Math.min(sourceNames.length, sourceValues.length);
  for (let index = 0; index < count; index += 1) {
    valueByName.set(sourceNames[index], sourceValues[index]);
  }
  return jointNames.map((name) => {
    const value = valueByName.get(name);
    return value === undefined ? Number.NaN : value;
  });
}

function createActiveMotionSegment(payload: JointStatePayload, now: number): ActiveMotionSegment {
  const jointNames = orderedUniqueStrings(payload.names);
  return {
    jointNames,
    tcpFrame: resolveTrajectoryChartsTcpFrame(),
    startedAt: now,
    samples: [],
    lastMovementAt: now,
    lastSampleAt: -TRAJ_SAMPLE_INTERVAL_MS
  };
}

function buildMotionSample(segment: ActiveMotionSegment, payload: JointStatePayload, now: number): MotionSample {
  const tcpTransform = segment.tcpFrame ? sceneManager.getRelativeTransform(segment.tcpFrame) : null;
  return {
    t: Math.max(0, now - segment.startedAt),
    positions: alignNumericValues(payload.names, payload.positions, segment.jointNames),
    velocities: payload.velocities ? alignNumericValues(payload.names, payload.velocities, segment.jointNames) : undefined,
    accelerations: undefined,
    efforts: payload.efforts ? alignNumericValues(payload.names, payload.efforts, segment.jointNames) : undefined,
    tcpPosition: tcpTransform ? {
      x: tcpTransform.translation.x,
      y: tcpTransform.translation.y,
      z: tcpTransform.translation.z
    } : null
  };
}

function appendMotionSample(segment: ActiveMotionSegment, payload: JointStatePayload, now: number, force = false): void {
  if (!force && segment.samples.length > 0 && now - segment.lastSampleAt < TRAJ_SAMPLE_INTERVAL_MS) {
    return;
  }
  segment.samples.push(buildMotionSample(segment, payload, now));
  segment.lastSampleAt = now;
}

function finalizeMotionSegment(segment: ActiveMotionSegment, endedAt: number): MotionSegmentSnapshot {
  return {
    jointNames: segment.jointNames.slice(),
    tcpFrame: segment.tcpFrame,
    startedAt: segment.startedAt,
    endedAt,
    samples: segment.samples.map((sample) => ({
      t: sample.t,
      positions: sample.positions.slice(),
      velocities: sample.velocities ? sample.velocities.slice() : undefined,
      accelerations: sample.accelerations ? sample.accelerations.slice() : undefined,
      efforts: sample.efforts ? sample.efforts.slice() : undefined,
      tcpPosition: sample.tcpPosition ? { ...sample.tcpPosition } : null
    }))
  };
}

function clearTrajectoryChartsState(): void {
  activeMotionSegment = null;
  latestMotionSegment = null;
  motionLastPositions = null;
  trajectoryChartsVisibleJoints.clear();
  trajectoryChartsJointSignature = "";
  pendingTrajectoryChartsRender = false;
  clearElement(elements.trajectoryChartsJointChips);
  elements.trajectoryChartsJointChips.removeAttribute("data-signature");
  const jointToolbar = elements.trajectoryChartsJointChips.parentElement;
  if (jointToolbar instanceof HTMLElement) {
    jointToolbar.dataset.compactJoints = "false";
  }
}

function getTrajectoryChartsSegment(): { jointNames: string[]; tcpFrame: string; startedAt: number; samples: MotionSample[] } | null {
  return activeMotionSegment ?? latestMotionSegment;
}

function trackTrajectoryChartsMotion(message: unknown): void {
  const payload = parseJointStatePayload(message);
  if (!payload) {
    return;
  }

  const now = performance.now();
  const currentPositions = new Map<string, number>();
  let moved = false;
  let compared = 0;
  for (let index = 0; index < payload.names.length; index += 1) {
    const name = payload.names[index];
    const position = payload.positions[index];
    currentPositions.set(name, position);
    const previous = motionLastPositions?.get(name);
    if (previous === undefined) {
      continue;
    }
    compared += 1;
    if (Math.abs(position - previous) >= TRAJ_MOTION_THRESHOLD) {
      moved = true;
    }
  }

  if (!activeMotionSegment) {
    if (moved && compared > 0) {
      activeMotionSegment = createActiveMotionSegment(payload, now);
      appendMotionSample(activeMotionSegment, payload, now, true);
      ensureTrajectoryChartsVisibleJoints(activeMotionSegment);
      scheduleTrajectoryChartsRender();
    }
    motionLastPositions = currentPositions;
    return;
  }

  if (moved) {
    activeMotionSegment.lastMovementAt = now;
  }

  if (moved || activeMotionSegment.samples.length === 0 || now - activeMotionSegment.lastSampleAt >= TRAJ_SAMPLE_INTERVAL_MS) {
    appendMotionSample(activeMotionSegment, payload, now, false);
  }

  if (!moved && now - activeMotionSegment.lastMovementAt >= TRAJ_IDLE_STOP_MS) {
    if (activeMotionSegment.samples.length === 0 || now - activeMotionSegment.lastSampleAt > 1) {
      appendMotionSample(activeMotionSegment, payload, now, true);
    }
    latestMotionSegment = finalizeMotionSegment(activeMotionSegment, now);
    activeMotionSegment = null;
    ensureTrajectoryChartsVisibleJoints(latestMotionSegment);
  }

  motionLastPositions = currentPositions;
  scheduleTrajectoryChartsRender();
}

function trajectoryChartTheme(): TrajectoryChartTheme {
  const styles = getComputedStyle(document.documentElement);
  return {
    background: styles.getPropertyValue("--surface").trim() || "#ffffff",
    border: styles.getPropertyValue("--modal-header-border").trim() || "#d7dfe8",
    grid: styles.getPropertyValue("--border-soft").trim() || "#dfe5ec",
    text: styles.getPropertyValue("--text").trim() || "#1f2630",
    muted: styles.getPropertyValue("--muted").trim() || "#687385",
    accent: styles.getPropertyValue("--accent").trim() || "#4e84c4"
  };
}

function trajectoryChartColor(index: number): string {
  const colors = ["#3f7de0", "#e05f3f", "#29a383", "#8a63d2", "#b8822b", "#d14c8f", "#2d8aa6", "#7b8c30"];
  return colors[index % colors.length];
}

function buildJointChartSeries(
  segment: { jointNames: string[]; samples: MotionSample[] },
  readValue: (sample: MotionSample, jointIndex: number) => number,
  convertValue: (value: number) => number
): TrajectoryChartSeries[] {
  return segment.jointNames.map((jointName, jointIndex) => ({
    label: jointName,
    color: trajectoryChartColor(jointIndex),
    visible: trajectoryChartsVisibleJoints.has(jointName),
    values: segment.samples.map((sample) => {
      const value = readValue(sample, jointIndex);
      return Number.isFinite(value) ? convertValue(value) : Number.NaN;
    })
  }));
}

function buildAccelerationValues(samples: MotionSample[], jointIndex: number): number[] {
  const directValues = samples.map((sample) => sample.accelerations?.[jointIndex] ?? Number.NaN);
  if (directValues.some((value) => Number.isFinite(value))) {
    return directValues;
  }

  const result = new Array<number>(samples.length).fill(Number.NaN);
  let previousVelocity = Number.NaN;
  let previousTime = 0;
  for (let index = 0; index < samples.length; index += 1) {
    const value = samples[index].velocities?.[jointIndex] ?? Number.NaN;
    if (!Number.isFinite(value)) {
      continue;
    }
    if (Number.isFinite(previousVelocity)) {
      const deltaSeconds = (samples[index].t - previousTime) / 1000;
      if (deltaSeconds > 0) {
        result[index] = (value - previousVelocity) / deltaSeconds;
      }
    }
    previousVelocity = value;
    previousTime = samples[index].t;
  }
  return result;
}

function buildAccelerationSeries(segment: { jointNames: string[]; samples: MotionSample[] }): TrajectoryChartSeries[] {
  return segment.jointNames.map((jointName, jointIndex) => ({
    label: jointName,
    color: trajectoryChartColor(jointIndex),
    visible: trajectoryChartsVisibleJoints.has(jointName),
    values: buildAccelerationValues(segment.samples, jointIndex).map((value) => Number.isFinite(value) ? convertAngleFromRad(value, trajectoryChartsAccelerationUnit) : Number.NaN)
  }));
}

function buildTcpSeries(segment: { samples: MotionSample[] }): TrajectoryChartSeries[] {
  const axisColors = ["#d85b52", "#2f9d72", "#2d7fe3"];
  const axisNames = ["X", "Y", "Z"] as const;
  return axisNames.map((axis, axisIndex) => ({
    label: axis,
    color: axisColors[axisIndex],
    values: segment.samples.map((sample) => {
      if (!sample.tcpPosition) {
        return Number.NaN;
      }
      if (axis === "X") {
        return convertLengthFromMeter(sample.tcpPosition.x, trajectoryChartsTcpUnit);
      }
      if (axis === "Y") {
        return convertLengthFromMeter(sample.tcpPosition.y, trajectoryChartsTcpUnit);
      }
      return convertLengthFromMeter(sample.tcpPosition.z, trajectoryChartsTcpUnit);
    })
  }));
}

function setTrajectoryChartUnitToggleState(primaryButton: HTMLButtonElement, secondaryButton: HTMLButtonElement, primaryActive: boolean): void {
  const toggle = primaryButton.parentElement;
  if (toggle instanceof HTMLElement) {
    toggle.dataset.activeIndex = primaryActive ? "0" : "1";
  }
  primaryButton.setAttribute("aria-pressed", primaryActive ? "true" : "false");
  secondaryButton.setAttribute("aria-pressed", primaryActive ? "false" : "true");
}

function updateTrajectoryChartsUnitButtons(): void {
  setTrajectoryChartUnitToggleState(elements.trajectoryChartsPositionDegBtn, elements.trajectoryChartsPositionRadBtn, trajectoryChartsPositionUnit === "deg");
  setTrajectoryChartUnitToggleState(elements.trajectoryChartsTcpMmBtn, elements.trajectoryChartsTcpMBtn, trajectoryChartsTcpUnit === "mm");
  setTrajectoryChartUnitToggleState(elements.trajectoryChartsVelocityDegBtn, elements.trajectoryChartsVelocityRadBtn, trajectoryChartsVelocityUnit === "deg");
  setTrajectoryChartUnitToggleState(elements.trajectoryChartsAccelerationDegBtn, elements.trajectoryChartsAccelerationRadBtn, trajectoryChartsAccelerationUnit === "deg");
}

function updateRightPanelUnitButtons(): void {
  setTrajectoryChartUnitToggleState(elements.jointAngleDegBtn, elements.jointAngleRadBtn, jointAngleUnit === "deg");
  setTrajectoryChartUnitToggleState(elements.cartesianLengthMmBtn, elements.cartesianLengthMBtn, cartesianLengthUnit === "mm");
  setTrajectoryChartUnitToggleState(elements.cartesianAngleDegBtn, elements.cartesianAngleRadBtn, cartesianAngleUnit === "deg");
}

function updateTrajectoryChartsJointChipState(button: HTMLButtonElement, jointName: string): void {
  const active = trajectoryChartsVisibleJoints.has(jointName);
  button.classList.toggle("is-inactive", !active);
  button.setAttribute("aria-pressed", active ? "true" : "false");
}

function renderTrajectoryChartsJointChips(segment: { jointNames: string[] } | null): void {
  const jointToolbar = elements.trajectoryChartsJointChips.parentElement;
  if (!segment || segment.jointNames.length === 0) {
    clearElement(elements.trajectoryChartsJointChips);
    elements.trajectoryChartsJointChips.removeAttribute("data-signature");
    if (jointToolbar instanceof HTMLElement) {
      jointToolbar.dataset.compactJoints = "false";
    }
    elements.trajectoryChartsShowAllBtn.disabled = true;
    elements.trajectoryChartsResetBtn.disabled = true;
    return;
  }

  if (jointToolbar instanceof HTMLElement) {
    jointToolbar.dataset.compactJoints = segment.jointNames.length <= 6 ? "true" : "false";
  }

  const signature = trajectoryChartsSegmentSignature(segment);
  const needsBuild = elements.trajectoryChartsJointChips.getAttribute("data-signature") !== signature;
  if (needsBuild) {
    clearElement(elements.trajectoryChartsJointChips);
    elements.trajectoryChartsJointChips.setAttribute("data-signature", signature);
    for (let index = 0; index < segment.jointNames.length; index += 1) {
      const jointName = segment.jointNames[index];
      const button = document.createElement("button");
      button.type = "button";
      button.className = "trajectory-charts-joint-chip";
      button.dataset.jointName = jointName;
      const swatch = document.createElement("span");
      swatch.className = "trajectory-charts-joint-chip-swatch";
      swatch.style.backgroundColor = trajectoryChartColor(index);
      const label = document.createElement("span");
      label.textContent = jointName;
      button.appendChild(swatch);
      button.appendChild(label);
      button.addEventListener("click", () => {
        if (trajectoryChartsVisibleJoints.has(jointName)) {
          trajectoryChartsVisibleJoints.delete(jointName);
        } else {
          trajectoryChartsVisibleJoints.add(jointName);
        }
        if (trajectoryChartsVisibleJoints.size === 0) {
          trajectoryChartsVisibleJoints = defaultTrajectoryChartsVisibleJoints(segment.jointNames);
        }
        renderTrajectoryChartsJointChips(getTrajectoryChartsSegment());
        scheduleTrajectoryChartsRender();
      });
      elements.trajectoryChartsJointChips.appendChild(button);
    }
  }

  for (const child of Array.from(elements.trajectoryChartsJointChips.children)) {
    if (!(child instanceof HTMLButtonElement)) {
      continue;
    }
    const jointName = child.dataset.jointName || "";
    updateTrajectoryChartsJointChipState(child, jointName);
  }

  elements.trajectoryChartsShowAllBtn.disabled = false;
  elements.trajectoryChartsResetBtn.disabled = false;
}

function scheduleTrajectoryChartsRender(): void {
  if (!trajectoryChartsOpen) {
    return;
  }
  if (pendingTrajectoryChartsRender) {
    return;
  }
  pendingTrajectoryChartsRender = true;
  window.requestAnimationFrame(() => {
    pendingTrajectoryChartsRender = false;
    renderTrajectoryChartsModal();
  });
}

function setTrajectoryChartsOpen(open: boolean): void {
  if (open && rosGraphOpen) {
    setRosGraphOpen(false);
  }
  trajectoryChartsOpen = open;
  elements.trajectoryChartsModal.classList.toggle("active", trajectoryChartsOpen);
  elements.trajectoryChartsModal.setAttribute("aria-hidden", trajectoryChartsOpen ? "false" : "true");
  updateTrajectoryChartsToggleUi();
  if (!trajectoryChartsOpen) {
    return;
  }
  renderTrajectoryChartsModal();
  window.requestAnimationFrame(() => {
    if (trajectoryChartsOpen) {
      renderTrajectoryChartsModal();
    }
  });
}

function renderTrajectoryChartsModal(): void {
  const closeLabel = t(currentLanguage, "button.close");
  elements.trajectoryChartsCloseBtn.title = closeLabel;
  elements.trajectoryChartsCloseBtn.setAttribute("aria-label", closeLabel);
  updateTrajectoryChartsUnitButtons();

  const segment = getTrajectoryChartsSegment();
  ensureTrajectoryChartsVisibleJoints(segment);
  renderTrajectoryChartsJointChips(segment);

  let statusText = t(currentLanguage, "state.trajectoryChartsEmpty");
  let metaText = t(currentLanguage, "state.trajectoryChartsEmpty");
  if (!rosClient.isConnected() && !segment) {
    statusText = t(currentLanguage, "state.notConnected");
    metaText = t(currentLanguage, "state.trajectoryChartsDisconnected");
  } else if (activeMotionSegment) {
    statusText = t(currentLanguage, "state.trajectoryChartsCapturing");
    metaText = t(currentLanguage, "state.trajectoryChartsMetaCapturing");
  } else if (latestMotionSegment) {
    statusText = t(currentLanguage, "state.trajectoryChartsLatest");
    metaText = t(currentLanguage, "state.trajectoryChartsMetaLatest");
  }

  const sampleCount = segment ? segment.samples.length : 0;
  const durationMs = !segment || sampleCount === 0
    ? 0
    : activeMotionSegment
      ? activeMotionSegment.samples[activeMotionSegment.samples.length - 1]?.t ?? 0
      : latestMotionSegment
        ? latestMotionSegment.endedAt - latestMotionSegment.startedAt
        : segment.samples[sampleCount - 1]?.t ?? 0;

  elements.trajectoryChartsMeta.textContent = metaText;
  elements.trajectoryChartsStatusValue.textContent = statusText;
  elements.trajectoryChartsDurationValue.textContent = sampleCount > 0 ? formatTrajectoryTime(durationMs) : "--";
  elements.trajectoryChartsSamplesValue.textContent = String(sampleCount);
  elements.trajectoryChartsTcpFrameValue.textContent = segment && segment.tcpFrame ? segment.tcpFrame : t(currentLanguage, "state.trajectoryChartsNoTcpFrame");

  if (!trajectoryChartsOpen) {
    return;
  }

  const theme = trajectoryChartTheme();
  const times = segment ? segment.samples.map((sample) => sample.t) : [];
  const emptyLabel = !rosClient.isConnected() && !segment
    ? t(currentLanguage, "state.trajectoryChartsDisconnected")
    : t(currentLanguage, "state.trajectoryChartsEmpty");
  const playheadTime = activeMotionSegment && activeMotionSegment.samples.length > 0
    ? activeMotionSegment.samples[activeMotionSegment.samples.length - 1].t
    : undefined;

  const positionSeries = segment
    ? buildJointChartSeries(segment, (sample, jointIndex) => sample.positions[jointIndex] ?? Number.NaN, (value) => convertAngleFromRad(value, trajectoryChartsPositionUnit))
    : [];
  const tcpSeries = segment ? buildTcpSeries(segment) : [];
  const velocitySeries = segment
    ? buildJointChartSeries(segment, (sample, jointIndex) => sample.velocities?.[jointIndex] ?? Number.NaN, (value) => convertAngleFromRad(value, trajectoryChartsVelocityUnit))
    : [];
  const accelerationSeries = segment ? buildAccelerationSeries(segment) : [];
  const effortSeries = segment
    ? buildJointChartSeries(segment, (sample, jointIndex) => sample.efforts?.[jointIndex] ?? Number.NaN, (value) => value)
    : [];

  renderTrajectoryLineChart({
    canvas: elements.trajectoryChartsPositionCanvas,
    times,
    series: positionSeries,
    unitLabel: angleUnitLabel(trajectoryChartsPositionUnit),
    emptyLabel,
    theme,
    playheadTime
  });
  renderTrajectoryLineChart({
    canvas: elements.trajectoryChartsTcpCanvas,
    times,
    series: tcpSeries,
    unitLabel: lengthUnitLabel(trajectoryChartsTcpUnit),
    emptyLabel,
    theme,
    playheadTime
  });
  renderTrajectoryLineChart({
    canvas: elements.trajectoryChartsVelocityCanvas,
    times,
    series: velocitySeries,
    unitLabel: rateUnitLabel(trajectoryChartsVelocityUnit),
    emptyLabel,
    theme,
    playheadTime
  });
  renderTrajectoryLineChart({
    canvas: elements.trajectoryChartsAccelerationCanvas,
    times,
    series: accelerationSeries,
    unitLabel: accelerationUnitLabel(trajectoryChartsAccelerationUnit),
    emptyLabel,
    theme,
    playheadTime
  });
  renderTrajectoryLineChart({
    canvas: elements.trajectoryChartsTorqueCanvas,
    times,
    series: effortSeries,
    unitLabel: "effort",
    emptyLabel,
    theme,
    playheadTime
  });
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

function deriveAssetPackageRootUrl(rosbridgeUrl: string): string | null {
  try {
    const url = new URL(rosbridgeUrl);
    if (url.protocol !== "ws:" && url.protocol !== "wss:") {
      return null;
    }

    url.protocol = url.protocol === "wss:" ? "https:" : "http:";
    url.port = DEFAULT_ASSET_SERVER_PORT;
    url.pathname = "/ros_pkgs";
    url.search = "";
    url.hash = "";
    return url.toString().replace(/\/+$/, "");
  } catch {
    return null;
  }
}

function resolveRuntimePackageRootUrl(): { url: string; autoDerived: boolean } {
  const configured = (config.packageRootUrl || "").trim();
  if (configured && configured !== defaultConfig.packageRootUrl) {
    return { url: configured, autoDerived: false };
  }

  const derived = deriveAssetPackageRootUrl(config.rosbridgeUrl);
  if (derived) {
    return { url: derived, autoDerived: true };
  }

  return { url: configured || defaultConfig.packageRootUrl, autoDerived: false };
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
    const parsed = decodePointCloud2(message, config.maxPoints);
    sceneManager.setPointCloud(parsed);
  });

  log(`subscribed pointcloud topic: ${topicName}`);
}

async function loadRobotIntoScene(packageRootUrl: string): Promise<void> {
  const loaded = await loadRobotModel(rosClient, config.urdfFallbackPath, packageRootUrl);
  sceneManager.setRobot(loaded.robot, loaded.rootLink);
  refreshLatestPlannedTrajectoryPreview();
  log(`robot model loaded (root=${loaded.rootLink})`);
}

async function connect(): Promise<void> {
  clearAllTrajectoryDisplays();
  clearTrajectoryChartsState();
  syncConfigFromUi();
  log(`connecting ROS bridge: ${config.rosbridgeUrl}`);

  await rosClient.connect(config.rosbridgeUrl);
  log("rosbridge connected");

  const runtimePackageRoot = resolveRuntimePackageRootUrl();
  if (runtimePackageRoot.autoDerived) {
    log(`package root auto resolved: ${runtimePackageRoot.url}`);
  }

  try {
    await loadRobotIntoScene(runtimePackageRoot.url);
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

  if (rosGraphOpen) {
    void refreshRosGraphSnapshot();
  }
  if (trajectoryChartsOpen) {
    renderTrajectoryChartsModal();
  }
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
  clearRosGraphState();
  clearTrajectoryChartsState();
  renderRosGraphModal();
  renderTrajectoryChartsModal();
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


function clampNumber(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

function rosGraphEntityId(kind: "node" | "topic", name: string): string {
  return kind + ":" + name;
}

function dedupeSortedStrings(values: string[]): string[] {
  const unique = new Set<string>();
  for (const value of values) {
    if (typeof value !== "string") {
      continue;
    }
    const trimmed = value.trim();
    if (trimmed) {
      unique.add(trimmed);
    }
  }
  return Array.from(unique).sort((left, right) => left.localeCompare(right));
}

function rosGraphLabel(name: string, maxLength: number): string {
  if (name.length <= maxLength) {
    return name;
  }
  const keep = Math.max(4, Math.floor((maxLength - 3) / 2));
  return name.slice(0, keep) + "..." + name.slice(name.length - keep);
}

function rosGraphEntityWidth(kind: "node" | "topic", name: string): number {
  const base = kind === "node" ? 84 : 108;
  const perChar = kind === "node" ? 7.8 : 6.6;
  const min = kind === "node" ? 116 : 148;
  const max = kind === "node" ? 240 : 320;
  return clampNumber(base + Math.round(name.length * perChar), min, max);
}

function compareRosGraphEntities(left: RosGraphEntity, right: RosGraphEntity): number {
  if (left.kind !== right.kind) {
    return left.kind === "node" ? -1 : 1;
  }
  return left.name.localeCompare(right.name);
}

function createRosGraphBounds(entities: RosGraphEntity[]): RosGraphBounds | null {
  if (entities.length === 0) {
    return null;
  }

  let minX = Number.POSITIVE_INFINITY;
  let minY = Number.POSITIVE_INFINITY;
  let maxX = Number.NEGATIVE_INFINITY;
  let maxY = Number.NEGATIVE_INFINITY;

  for (const entity of entities) {
    minX = Math.min(minX, entity.x - entity.width / 2);
    minY = Math.min(minY, entity.y - entity.height / 2);
    maxX = Math.max(maxX, entity.x + entity.width / 2);
    maxY = Math.max(maxY, entity.y + entity.height / 2);
  }

  return { minX, minY, maxX, maxY };
}

function layoutRosGraphEntities(entities: RosGraphEntity[], adjacency: Map<string, Set<string>>): void {
  const entityById = new Map<string, RosGraphEntity>();
  for (const entity of entities) {
    entityById.set(entity.id, entity);
  }

  const unvisited = new Set<string>(entities.map((entity) => entity.id));
  let currentTop = 0;

  const compareRootIds = (leftId: string, rightId: string): number => {
    const left = entityById.get(leftId);
    const right = entityById.get(rightId);
    if (!left || !right) {
      return 0;
    }
    const degreeDelta = (adjacency.get(rightId)?.size ?? 0) - (adjacency.get(leftId)?.size ?? 0);
    if (degreeDelta !== 0) {
      return degreeDelta;
    }
    return compareRosGraphEntities(left, right);
  };

  while (unvisited.size > 0) {
    const startId = unvisited.values().next().value as string;
    const componentIds: string[] = [];
    const queue: string[] = [startId];
    unvisited.delete(startId);

    while (queue.length > 0) {
      const currentId = queue.shift();
      if (!currentId) {
        continue;
      }
      componentIds.push(currentId);
      const neighbors = Array.from(adjacency.get(currentId) ?? []).sort(compareRootIds);
      for (const neighbor of neighbors) {
        if (unvisited.has(neighbor)) {
          unvisited.delete(neighbor);
          queue.push(neighbor);
        }
      }
    }

    componentIds.sort(compareRootIds);
    const rootId = componentIds[0];
    const levelById = new Map<string, number>([[rootId, 0]]);
    const bfsQueue: string[] = [rootId];

    while (bfsQueue.length > 0) {
      const currentId = bfsQueue.shift();
      if (!currentId) {
        continue;
      }
      const nextLevel = (levelById.get(currentId) ?? 0) + 1;
      const neighbors = Array.from(adjacency.get(currentId) ?? []).sort(compareRootIds);
      for (const neighbor of neighbors) {
        if (!levelById.has(neighbor)) {
          levelById.set(neighbor, nextLevel);
          bfsQueue.push(neighbor);
        }
      }
    }

    const layers: string[][] = [];
    for (const entityId of componentIds) {
      const level = levelById.get(entityId) ?? 0;
      if (!layers[level]) {
        layers[level] = [];
      }
      layers[level].push(entityId);
    }

    for (const layer of layers) {
      layer.sort((leftId, rightId) => compareRosGraphEntities(entityById.get(leftId)!, entityById.get(rightId)!));
    }

    for (let layerIndex = 1; layerIndex < layers.length; layerIndex += 1) {
      const previousOrder = new Map<string, number>();
      for (let index = 0; index < layers[layerIndex - 1].length; index += 1) {
        previousOrder.set(layers[layerIndex - 1][index], index);
      }

      const barycenter = (entityId: string): number => {
        const values: number[] = [];
        for (const neighbor of adjacency.get(entityId) ?? []) {
          const order = previousOrder.get(neighbor);
          if (order != null) {
            values.push(order);
          }
        }
        if (values.length === 0) {
          return Number.POSITIVE_INFINITY;
        }
        return values.reduce((sum, value) => sum + value, 0) / values.length;
      };

      layers[layerIndex].sort((leftId, rightId) => {
        const leftScore = barycenter(leftId);
        const rightScore = barycenter(rightId);
        if (leftScore !== rightScore) {
          return leftScore - rightScore;
        }
        return compareRosGraphEntities(entityById.get(leftId)!, entityById.get(rightId)!);
      });
    }

    const layerHeights = layers.map((layer) => {
      let height = 0;
      for (let index = 0; index < layer.length; index += 1) {
        const entity = entityById.get(layer[index]);
        if (!entity) {
          continue;
        }
        height += entity.height;
        if (index > 0) {
          height += ROS_GRAPH_ROW_GAP;
        }
      }
      return height;
    });

    const componentHeight = layerHeights.reduce((maxHeight, value) => Math.max(maxHeight, value), 0);

    for (let layerIndex = 0; layerIndex < layers.length; layerIndex += 1) {
      const layer = layers[layerIndex];
      const layerHeight = layerHeights[layerIndex] ?? 0;
      let yCursor = currentTop + (componentHeight - layerHeight) / 2;

      for (const entityId of layer) {
        const entity = entityById.get(entityId);
        if (!entity) {
          continue;
        }
        yCursor += entity.height / 2;
        entity.x = layerIndex * ROS_GRAPH_LAYER_GAP;
        entity.y = yCursor;
        yCursor += entity.height / 2 + ROS_GRAPH_ROW_GAP;
      }
    }

    currentTop += componentHeight + ROS_GRAPH_COMPONENT_GAP;
  }
}

function buildRosGraphViewSnapshot(entities: RosGraphEntity[], edges: RosGraphEdge[]): RosGraphViewSnapshot {
  const entityById = new Map<string, RosGraphEntity>();
  const adjacency = new Map<string, Set<string>>();

  for (const entity of entities) {
    entityById.set(entity.id, entity);
    adjacency.set(entity.id, new Set<string>());
  }

  for (const edge of edges) {
    const sourceNeighbors = adjacency.get(edge.sourceId);
    const targetNeighbors = adjacency.get(edge.targetId);
    if (!sourceNeighbors || !targetNeighbors) {
      continue;
    }
    sourceNeighbors.add(edge.targetId);
    targetNeighbors.add(edge.sourceId);
  }

  layoutRosGraphEntities(entities, adjacency);

  return {
    entities,
    entityById,
    edges,
    adjacency
  };
}

function cloneRosGraphNodeEntity(node: RosGraphNodeEntity): RosGraphNodeEntity {
  return {
    ...node,
    publishing: node.publishing.slice(),
    subscribing: node.subscribing.slice(),
    services: node.services.slice()
  };
}

function buildRosGraphNodeCommunicationView(nodeEntities: RosGraphNodeEntity[], topicEntities: RosGraphTopicEntity[]): RosGraphViewSnapshot {
  const communicationNodes = nodeEntities.map((node) => cloneRosGraphNodeEntity(node));
  const availableIds = new Set<string>(communicationNodes.map((node) => node.id));
  const edgeAccumulator = new Map<string, { sourceId: string; targetId: string; topics: Set<string> }>();

  for (const topic of topicEntities) {
    for (const publisher of topic.publishers) {
      const sourceId = rosGraphEntityId("node", publisher);
      if (!availableIds.has(sourceId)) {
        continue;
      }

      for (const subscriber of topic.subscribers) {
        const targetId = rosGraphEntityId("node", subscriber);
        if (!availableIds.has(targetId) || sourceId === targetId) {
          continue;
        }

        const edgeKey = sourceId + "->" + targetId;
        let record = edgeAccumulator.get(edgeKey);
        if (!record) {
          record = { sourceId, targetId, topics: new Set<string>() };
          edgeAccumulator.set(edgeKey, record);
        }
        record.topics.add(topic.name);
      }
    }
  }

  const edges: RosGraphEdge[] = Array.from(edgeAccumulator.values())
    .map((record) => ({
      id: record.sourceId + "->" + record.targetId + ":nodeCommunication",
      sourceId: record.sourceId,
      targetId: record.targetId,
      kind: "nodeCommunication" as const,
      labels: Array.from(record.topics).sort((left, right) => left.localeCompare(right))
    }))
    .sort((left, right) => left.sourceId.localeCompare(right.sourceId) || left.targetId.localeCompare(right.targetId));

  return buildRosGraphViewSnapshot(communicationNodes, edges);
}

function buildRosGraphSnapshot(nodeEntries: Array<{ name: string; details: NodeDetails }>, partialFailures: string[]): RosGraphSnapshot {
  const topicTypeByName = new Map<string, string>();
  for (const topic of topics) {
    topicTypeByName.set(topic.name, topic.type);
  }

  const topicAccumulator = new Map<string, { name: string; type: string; publishers: Set<string>; subscribers: Set<string> }>();
  const nodeEntities = nodeEntries
    .slice()
    .sort((left, right) => left.name.localeCompare(right.name))
    .map((entry) => {
      const publishing = dedupeSortedStrings(entry.details.publishing);
      const subscribing = dedupeSortedStrings(entry.details.subscribing);
      const servicesList = dedupeSortedStrings(entry.details.services);

      for (const topicName of publishing) {
        let topicRecord = topicAccumulator.get(topicName);
        if (!topicRecord) {
          topicRecord = {
            name: topicName,
            type: topicTypeByName.get(topicName) ?? "",
            publishers: new Set<string>(),
            subscribers: new Set<string>()
          };
          topicAccumulator.set(topicName, topicRecord);
        }
        topicRecord.publishers.add(entry.name);
      }

      for (const topicName of subscribing) {
        let topicRecord = topicAccumulator.get(topicName);
        if (!topicRecord) {
          topicRecord = {
            name: topicName,
            type: topicTypeByName.get(topicName) ?? "",
            publishers: new Set<string>(),
            subscribers: new Set<string>()
          };
          topicAccumulator.set(topicName, topicRecord);
        }
        topicRecord.subscribers.add(entry.name);
      }

      return {
        id: rosGraphEntityId("node", entry.name),
        kind: "node" as const,
        name: entry.name,
        publishing,
        subscribing,
        services: servicesList,
        x: 0,
        y: 0,
        width: rosGraphEntityWidth("node", entry.name),
        height: ROS_GRAPH_NODE_HEIGHT
      };
    });

  const topicEntities: RosGraphTopicEntity[] = Array.from(topicAccumulator.values())
    .sort((left, right) => left.name.localeCompare(right.name))
    .map((record) => ({
      id: rosGraphEntityId("topic", record.name),
      kind: "topic" as const,
      name: record.name,
      type: record.type,
      publishers: Array.from(record.publishers).sort((left, right) => left.localeCompare(right)),
      subscribers: Array.from(record.subscribers).sort((left, right) => left.localeCompare(right)),
      x: 0,
      y: 0,
      width: rosGraphEntityWidth("topic", record.name),
      height: ROS_GRAPH_TOPIC_HEIGHT
    }));

  const detailEdges: RosGraphEdge[] = [];
  for (const node of nodeEntities) {
    for (const topicName of node.publishing) {
      const topicId = rosGraphEntityId("topic", topicName);
      detailEdges.push({
        id: node.id + "->" + topicId + ":publish",
        sourceId: node.id,
        targetId: topicId,
        kind: "publish"
      });
    }

    for (const topicName of node.subscribing) {
      const topicId = rosGraphEntityId("topic", topicName);
      detailEdges.push({
        id: topicId + "->" + node.id + ":subscribe",
        sourceId: topicId,
        targetId: node.id,
        kind: "subscribe"
      });
    }
  }

  const detailView = buildRosGraphViewSnapshot([...nodeEntities, ...topicEntities], detailEdges);
  const nodeCommunicationView = buildRosGraphNodeCommunicationView(nodeEntities, topicEntities);

  return {
    detailView,
    nodeCommunicationView,
    updatedAt: Date.now(),
    partialFailures: partialFailures.slice()
  };
}

function clearRosGraphState(): void {
  rosGraphBusy = false;
  rosGraphError = null;
  rosGraphSnapshot = null;
  rosGraphSelectedId = null;
  rosGraphPanX = 0;
  rosGraphPanY = 0;
  rosGraphScale = 1;
  rosGraphContentBounds = null;
  rosGraphPointerState = null;
  elements.rosGraphSvg.classList.remove("is-panning");
}

function applyRosGraphTransform(): void {
  elements.rosGraphViewportGroup.setAttribute(
    "transform",
    "translate(" + rosGraphPanX.toFixed(2) + " " + rosGraphPanY.toFixed(2) + ") scale(" + rosGraphScale.toFixed(4) + ")"
  );
}

function getRosGraphVisibleData(): RosGraphVisibleData {
  const activeView = getActiveRosGraphView();
  if (!activeView) {
    return { entities: [], entityIds: new Set<string>(), edges: [] };
  }

  const query = rosGraphSearchQuery.trim().toLowerCase();
  if (!query) {
    return {
      entities: activeView.entities.slice(),
      entityIds: new Set<string>(activeView.entities.map((entity) => entity.id)),
      edges: activeView.edges.slice()
    };
  }

  const matchedIds = activeView.entities
    .filter((entity) => entity.name.toLowerCase().includes(query))
    .map((entity) => entity.id);
  const visibleIds = new Set<string>(matchedIds);
  for (const entityId of matchedIds) {
    for (const neighbor of activeView.adjacency.get(entityId) ?? []) {
      visibleIds.add(neighbor);
    }
  }

  return {
    entities: activeView.entities.filter((entity) => visibleIds.has(entity.id)),
    entityIds: visibleIds,
    edges: activeView.edges.filter((edge) => visibleIds.has(edge.sourceId) && visibleIds.has(edge.targetId))
  };
}

function syncRosGraphSelection(visibleIds: Set<string>): void {
  const activeView = getActiveRosGraphView();
  if (!activeView) {
    rosGraphSelectedId = null;
    return;
  }

  if (rosGraphSelectedId && !activeView.entityById.has(rosGraphSelectedId)) {
    rosGraphSelectedId = null;
  }

  if (rosGraphSelectedId && visibleIds.size > 0 && !visibleIds.has(rosGraphSelectedId)) {
    rosGraphSelectedId = null;
  }
}

function getRosGraphFocusIds(): Set<string> {
  const focusIds = new Set<string>();
  const activeView = getActiveRosGraphView();
  if (!activeView || !rosGraphSelectedId) {
    return focusIds;
  }
  focusIds.add(rosGraphSelectedId);
  for (const neighbor of activeView.adjacency.get(rosGraphSelectedId) ?? []) {
    focusIds.add(neighbor);
  }
  return focusIds;
}

function fitRosGraphToView(): void {
  const visible = getRosGraphVisibleData();
  rosGraphContentBounds = createRosGraphBounds(visible.entities);
  if (!rosGraphContentBounds) {
    applyRosGraphTransform();
    return;
  }

  const rect = elements.rosGraphSvg.getBoundingClientRect();
  if (rect.width <= 0 || rect.height <= 0) {
    return;
  }

  const width = Math.max(1, rosGraphContentBounds.maxX - rosGraphContentBounds.minX);
  const height = Math.max(1, rosGraphContentBounds.maxY - rosGraphContentBounds.minY);
  const scaleX = (rect.width - ROS_GRAPH_FIT_PADDING * 2) / width;
  const scaleY = (rect.height - ROS_GRAPH_FIT_PADDING * 2) / height;
  rosGraphScale = clampNumber(Math.min(scaleX, scaleY, 1.8), ROS_GRAPH_MIN_SCALE, ROS_GRAPH_MAX_SCALE);
  const centerX = (rosGraphContentBounds.minX + rosGraphContentBounds.maxX) / 2;
  const centerY = (rosGraphContentBounds.minY + rosGraphContentBounds.maxY) / 2;
  rosGraphPanX = rect.width / 2 - centerX * rosGraphScale;
  rosGraphPanY = rect.height / 2 - centerY * rosGraphScale;
  applyRosGraphTransform();
}

function centerRosGraphOnEntity(entityId: string): void {
  const activeView = getActiveRosGraphView();
  if (!activeView) {
    return;
  }
  const entity = activeView.entityById.get(entityId);
  if (!entity) {
    return;
  }

  const rect = elements.rosGraphSvg.getBoundingClientRect();
  if (rect.width <= 0 || rect.height <= 0) {
    return;
  }

  rosGraphPanX = rect.width / 2 - entity.x * rosGraphScale;
  rosGraphPanY = rect.height / 2 - entity.y * rosGraphScale;
  applyRosGraphTransform();
}

function setRosGraphOpen(open: boolean): void {
  if (open && trajectoryChartsOpen) {
    setTrajectoryChartsOpen(false);
  }
  rosGraphOpen = open;
  elements.rosGraphModal.classList.toggle("active", rosGraphOpen);
  elements.rosGraphModal.setAttribute("aria-hidden", rosGraphOpen ? "false" : "true");
  updateRosGraphToggleUi();

  if (!rosGraphOpen) {
    rosGraphPointerState = null;
    elements.rosGraphSvg.classList.remove("is-panning");
    return;
  }

  renderRosGraphModal();
  window.requestAnimationFrame(() => {
    if (!rosGraphOpen) {
      return;
    }

    if (!rosGraphSnapshot && rosClient.isConnected() && !rosGraphBusy) {
      void refreshRosGraphSnapshot();
      return;
    }

    if (rosGraphSnapshot) {
      fitRosGraphToView();
    }

    if (!elements.rosGraphSearch.disabled) {
      elements.rosGraphSearch.focus();
      elements.rosGraphSearch.select();
    }
  });
}

async function refreshRosGraphSnapshot(): Promise<void> {
  if (!rosClient.isConnected()) {
    clearRosGraphState();
    renderRosGraphModal();
    return;
  }

  rosGraphBusy = true;
  rosGraphError = null;
  renderRosGraphModal();

  try {
    const nodeNames = dedupeSortedStrings(await rosClient.listNodes());
    const results = await Promise.allSettled(nodeNames.map((nodeName) => rosClient.getNodeDetails(nodeName)));
    const successfulNodes: Array<{ name: string; details: NodeDetails }> = [];
    const partialFailures: string[] = [];

    for (let index = 0; index < results.length; index += 1) {
      const result = results[index];
      const nodeName = nodeNames[index];
      if (result.status === "fulfilled") {
        successfulNodes.push({ name: nodeName, details: result.value });
      } else {
        partialFailures.push(nodeName);
      }
    }

    rosGraphSnapshot = buildRosGraphSnapshot(successfulNodes, partialFailures);
    const activeView = getActiveRosGraphView();
    if (rosGraphSelectedId && activeView && !activeView.entityById.has(rosGraphSelectedId)) {
      rosGraphSelectedId = null;
    }
    log(
      "ROS graph refreshed: " +
      String(successfulNodes.length) +
      " nodes, " +
      String(rosGraphSnapshot.detailView.entities.filter((entity) => entity.kind === "topic").length) +
      " topics"
    );
    if (partialFailures.length > 0) {
      log("ROS graph warning: " + String(partialFailures.length) + " nodes failed to load");
    }
  } catch (error) {
    rosGraphSnapshot = null;
    rosGraphSelectedId = null;
    rosGraphError = formatError(error);
    log("ROS graph refresh failed: " + rosGraphError);
  } finally {
    rosGraphBusy = false;
    renderRosGraphModal();
    if (rosGraphOpen && rosGraphSnapshot) {
      window.requestAnimationFrame(() => {
        fitRosGraphToView();
      });
    }
  }
}

function rosGraphMetaText(): string {
  if (rosGraphBusy && !rosGraphSnapshot) {
    return t(currentLanguage, "state.rosGraphLoading");
  }
  if (!rosGraphSnapshot) {
    return t(currentLanguage, "state.rosGraphNeverUpdated");
  }
  return t(currentLanguage, "state.rosGraphUpdatedAt", {
    time: new Date(rosGraphSnapshot.updatedAt).toLocaleString(currentLanguage === "zh" ? "zh-CN" : "en-US")
  });
}

function getActiveRosGraphView(): RosGraphViewSnapshot | null {
  if (!rosGraphSnapshot) {
    return null;
  }
  return rosGraphViewMode === "nodeCommunication" ? rosGraphSnapshot.nodeCommunicationView : rosGraphSnapshot.detailView;
}

function rosGraphHintText(): string {
  return t(currentLanguage, rosGraphViewMode === "nodeCommunication" ? "state.rosGraphNodeHint" : "state.rosGraphHint");
}

function rosGraphEdgeLabelText(labels: string[]): string {
  if (labels.length === 0) {
    return "";
  }
  if (labels.length === 1) {
    return rosGraphLabel(labels[0], 24);
  }
  const preview = rosGraphLabel(labels[0], 14);
  return preview + " +" + String(labels.length - 1);
}

function shouldRenderRosGraphEdgeLabel(edge: RosGraphEdge, hasSelection: boolean): boolean {
  if (edge.kind !== "nodeCommunication" || !edge.labels || edge.labels.length === 0) {
    return false;
  }
  if (!hasSelection || !rosGraphSelectedId) {
    return false;
  }
  return edge.sourceId === rosGraphSelectedId || edge.targetId === rosGraphSelectedId;
}

function rosGraphEmptyText(visible: RosGraphVisibleData): string {
  const activeView = getActiveRosGraphView();
  if (!rosClient.isConnected()) {
    return t(currentLanguage, "state.notConnected");
  }
  if (rosGraphBusy && !rosGraphSnapshot) {
    return t(currentLanguage, "state.rosGraphLoading");
  }
  if (rosGraphError && !rosGraphSnapshot) {
    return t(currentLanguage, "state.rosGraphRefreshFailed", { detail: rosGraphError });
  }
  if (!activeView || activeView.entities.length === 0) {
    return t(currentLanguage, "state.rosGraphEmpty");
  }
  if (visible.entities.length === 0) {
    return t(currentLanguage, "state.rosGraphNoMatch");
  }
  return "";
}

function getRosGraphConnectionPoint(entity: RosGraphEntity, towardX: number, towardY: number): { x: number; y: number } {
  const dx = towardX - entity.x;
  const dy = towardY - entity.y;
  if (dx === 0 && dy === 0) {
    return { x: entity.x, y: entity.y };
  }

  if (entity.kind === "node") {
    const rx = entity.width / 2;
    const ry = entity.height / 2;
    const scale = 1 / Math.sqrt((dx * dx) / (rx * rx) + (dy * dy) / (ry * ry));
    return {
      x: entity.x + dx * scale,
      y: entity.y + dy * scale
    };
  }

  const halfWidth = entity.width / 2;
  const halfHeight = entity.height / 2;
  const absDx = Math.abs(dx);
  const absDy = Math.abs(dy);
  if (absDx * halfHeight > absDy * halfWidth) {
    const sign = dx >= 0 ? 1 : -1;
    return {
      x: entity.x + sign * halfWidth,
      y: entity.y + (dx === 0 ? 0 : dy * (halfWidth / absDx))
    };
  }
  const sign = dy >= 0 ? 1 : -1;
  return {
    x: entity.x + (dy === 0 ? 0 : dx * (halfHeight / absDy)),
    y: entity.y + sign * halfHeight
  };
}

function selectRosGraphEntity(entityId: string | null, center = false): void {
  rosGraphSelectedId = entityId;
  renderRosGraphModal();
  if (entityId && center) {
    window.requestAnimationFrame(() => {
      centerRosGraphOnEntity(entityId);
    });
  }
}

function selectRosGraphNamedEntity(kind: "node" | "topic", name: string): void {
  if (kind === "topic" && rosGraphViewMode === "nodeCommunication") {
    rosGraphViewMode = "detail";
  }
  selectRosGraphEntity(rosGraphEntityId(kind, name), true);
}

function toggleRosGraphViewMode(): void {
  if (!rosGraphSnapshot) {
    return;
  }
  rosGraphViewMode = rosGraphViewMode === "detail" ? "nodeCommunication" : "detail";
  renderRosGraphModal();
  window.requestAnimationFrame(() => {
    if (rosGraphOpen) {
      fitRosGraphToView();
    }
  });
}

function createRosGraphDetailEmpty(message: string): void {
  clearElement(elements.rosGraphDetail);
  const empty = document.createElement("div");
  empty.className = "ros-graph-detail-empty";
  empty.textContent = message;
  elements.rosGraphDetail.appendChild(empty);
}

function appendRosGraphDetailSummary(entity: RosGraphEntity): void {
  const card = document.createElement("section");
  card.className = "ros-graph-detail-card";

  const kind = document.createElement("div");
  kind.className = "ros-graph-detail-kind";
  kind.textContent = entity.kind === "node" ? t(currentLanguage, "detail.rosGraphNode") : t(currentLanguage, "detail.rosGraphTopic");
  card.appendChild(kind);

  const name = document.createElement("div");
  name.className = "ros-graph-detail-name";
  name.textContent = entity.name;
  card.appendChild(name);

  const grid = document.createElement("div");
  grid.className = "ros-graph-detail-grid";

  const addRow = (label: string, value: string): void => {
    const labelElement = document.createElement("div");
    labelElement.className = "ros-graph-detail-label";
    labelElement.textContent = label;
    grid.appendChild(labelElement);

    const valueElement = document.createElement("div");
    valueElement.className = "ros-graph-detail-value";
    valueElement.textContent = value;
    grid.appendChild(valueElement);
  };

  if (entity.kind === "topic") {
    addRow(
      t(currentLanguage, "detail.rosGraphType"),
      entity.type || t(currentLanguage, "common.unknown")
    );
    addRow(t(currentLanguage, "detail.rosGraphPublishers"), String(entity.publishers.length));
    addRow(t(currentLanguage, "detail.rosGraphSubscribers"), String(entity.subscribers.length));
  } else {
    addRow(t(currentLanguage, "detail.rosGraphPublishing"), String(entity.publishing.length));
    addRow(t(currentLanguage, "detail.rosGraphSubscribing"), String(entity.subscribing.length));
    addRow(t(currentLanguage, "detail.rosGraphServices"), String(entity.services.length));
  }

  card.appendChild(grid);
  elements.rosGraphDetail.appendChild(card);
}

function appendRosGraphDetailSection(title: string, items: string[], kind: "node" | "topic" | null): void {
  const section = document.createElement("section");
  section.className = "ros-graph-detail-section";

  const header = document.createElement("div");
  header.className = "ros-graph-detail-section-header";

  const titleElement = document.createElement("h4");
  titleElement.className = "ros-graph-detail-section-title";
  titleElement.textContent = title;
  header.appendChild(titleElement);

  const count = document.createElement("div");
  count.className = "ros-graph-detail-count";
  count.textContent = String(items.length);
  header.appendChild(count);

  section.appendChild(header);

  const list = document.createElement("div");
  list.className = "ros-graph-detail-list";

  if (items.length === 0) {
    const empty = document.createElement("div");
    empty.className = "ros-graph-detail-pill";
    empty.textContent = detailText("?", "None");
    list.appendChild(empty);
  } else {
    for (const item of items) {
      if (kind) {
        const button = document.createElement("button");
        button.type = "button";
        button.className = "ros-graph-detail-link";
        button.textContent = item;
        button.addEventListener("click", () => {
          selectRosGraphNamedEntity(kind, item);
        });
        list.appendChild(button);
      } else {
        const value = document.createElement("div");
        value.className = "ros-graph-detail-pill";
        value.textContent = item;
        list.appendChild(value);
      }
    }
  }

  section.appendChild(list);
  elements.rosGraphDetail.appendChild(section);
}

function renderRosGraphDetail(visibleIds: Set<string>): void {
  if (!rosClient.isConnected()) {
    createRosGraphDetailEmpty(t(currentLanguage, "state.notConnected"));
    return;
  }

  if (rosGraphBusy && !rosGraphSnapshot) {
    createRosGraphDetailEmpty(t(currentLanguage, "state.rosGraphLoading"));
    return;
  }

  if (rosGraphError && !rosGraphSnapshot) {
    createRosGraphDetailEmpty(t(currentLanguage, "state.rosGraphRefreshFailed", { detail: rosGraphError }));
    return;
  }

  const activeView = getActiveRosGraphView();
  if (!activeView) {
    createRosGraphDetailEmpty(rosGraphHintText());
    return;
  }

  syncRosGraphSelection(visibleIds);
  const entity = rosGraphSelectedId ? activeView.entityById.get(rosGraphSelectedId) ?? null : null;
  if (!entity) {
    createRosGraphDetailEmpty(rosGraphHintText());
    return;
  }

  clearElement(elements.rosGraphDetail);
  appendRosGraphDetailSummary(entity);
  if (entity.kind === "node") {
    appendRosGraphDetailSection(t(currentLanguage, "detail.rosGraphPublishing"), entity.publishing, "topic");
    appendRosGraphDetailSection(t(currentLanguage, "detail.rosGraphSubscribing"), entity.subscribing, "topic");
    appendRosGraphDetailSection(t(currentLanguage, "detail.rosGraphServices"), entity.services, null);
  } else {
    appendRosGraphDetailSection(t(currentLanguage, "detail.rosGraphPublishers"), entity.publishers, "node");
    appendRosGraphDetailSection(t(currentLanguage, "detail.rosGraphSubscribers"), entity.subscribers, "node");
  }
}

function getRosGraphEdgeGeometry(
  source: RosGraphEntity,
  target: RosGraphEntity,
  bendOffset: number,
  labelProgress = 0.5,
  labelLift = 14
): { pathData: string; labelX: number; labelY: number } {
  const midpointX = (source.x + target.x) / 2;
  const midpointY = (source.y + target.y) / 2;
  const centerDx = target.x - source.x;
  const centerDy = target.y - source.y;
  const centerDistance = Math.hypot(centerDx, centerDy) || 1;
  const normalX = -centerDy / centerDistance;
  const normalY = centerDx / centerDistance;

  if (bendOffset === 0) {
    const start = getRosGraphConnectionPoint(source, target.x, target.y);
    const end = getRosGraphConnectionPoint(target, source.x, source.y);
    const labelX = start.x + (end.x - start.x) * labelProgress + normalX * labelLift;
    const labelY = start.y + (end.y - start.y) * labelProgress + normalY * labelLift;
    return {
      pathData: "M " + start.x.toFixed(1) + " " + start.y.toFixed(1) + " L " + end.x.toFixed(1) + " " + end.y.toFixed(1),
      labelX,
      labelY
    };
  }

  const controlX = midpointX + normalX * bendOffset;
  const controlY = midpointY + normalY * bendOffset;
  const start = getRosGraphConnectionPoint(source, controlX, controlY);
  const end = getRosGraphConnectionPoint(target, controlX, controlY);
  const inverseT = 1 - labelProgress;
  const curveX = inverseT * inverseT * start.x + 2 * inverseT * labelProgress * controlX + labelProgress * labelProgress * end.x;
  const curveY = inverseT * inverseT * start.y + 2 * inverseT * labelProgress * controlY + labelProgress * labelProgress * end.y;
  const side = Math.sign(bendOffset) || 1;
  return {
    pathData: "M " + start.x.toFixed(1) + " " + start.y.toFixed(1) + " Q " + controlX.toFixed(1) + " " + controlY.toFixed(1) + " " + end.x.toFixed(1) + " " + end.y.toFixed(1),
    labelX: curveX + normalX * labelLift * side,
    labelY: curveY + normalY * labelLift * side
  };
}

function renderRosGraphCanvas(visible: RosGraphVisibleData): void {
  elements.rosGraphViewportGroup.replaceChildren();
  rosGraphContentBounds = createRosGraphBounds(visible.entities);

  const emptyText = rosGraphEmptyText(visible);
  elements.rosGraphEmpty.textContent = emptyText;
  elements.rosGraphEmpty.hidden = emptyText.length === 0;

  const activeView = getActiveRosGraphView();
  if (!activeView || visible.entities.length === 0) {
    applyRosGraphTransform();
    return;
  }

  syncRosGraphSelection(visible.entityIds);
  const focusIds = getRosGraphFocusIds();
  const hasSelection = focusIds.size > 0;
  const nodeCommunicationKeys = new Set<string>(
    visible.edges
      .filter((edge) => edge.kind === "nodeCommunication")
      .map((edge) => edge.sourceId + "->" + edge.targetId)
  );

  for (const edge of visible.edges) {
    const source = activeView.entityById.get(edge.sourceId);
    const target = activeView.entityById.get(edge.targetId);
    if (!source || !target) {
      continue;
    }

    const hasReverse = edge.kind === "nodeCommunication" && nodeCommunicationKeys.has(edge.targetId + "->" + edge.sourceId);
    const bendDirection = edge.sourceId.localeCompare(edge.targetId) < 0 ? 1 : -1;
    const bendOffset = hasReverse ? bendDirection * 38 : 0;
    const labelProgress = hasReverse ? (bendDirection > 0 ? 0.34 : 0.66) : 0.5;
    const geometry = getRosGraphEdgeGeometry(source, target, bendOffset, labelProgress, hasReverse ? 18 : 14);
    const path = document.createElementNS(SVG_NS, "path");
    path.setAttribute("d", geometry.pathData);
    path.classList.add("ros-graph-edge");
    if (hasSelection) {
      if (edge.sourceId === rosGraphSelectedId || edge.targetId === rosGraphSelectedId) {
        path.classList.add("is-highlighted");
      } else if (!focusIds.has(edge.sourceId) || !focusIds.has(edge.targetId)) {
        path.classList.add("is-dimmed");
      }
    }

    if (edge.kind === "nodeCommunication" && edge.labels && edge.labels.length > 0) {
      const edgeTitle = document.createElementNS(SVG_NS, "title");
      edgeTitle.textContent = edge.labels.join("\n");
      path.appendChild(edgeTitle);
    }

    elements.rosGraphViewportGroup.appendChild(path);

    if (shouldRenderRosGraphEdgeLabel(edge, hasSelection)) {
      const label = document.createElementNS(SVG_NS, "text");
      label.classList.add("ros-graph-edge-label");
      label.setAttribute("x", geometry.labelX.toFixed(1));
      label.setAttribute("y", geometry.labelY.toFixed(1));
      label.textContent = rosGraphEdgeLabelText(edge.labels ?? []);
      label.classList.add("is-highlighted");
      const labelTitle = document.createElementNS(SVG_NS, "title");
      labelTitle.textContent = (edge.labels ?? []).join("\n");
      label.appendChild(labelTitle);
      elements.rosGraphViewportGroup.appendChild(label);
    }
  }

  const orderedEntities = visible.entities.slice().sort((left, right) => {
    if (left.id === rosGraphSelectedId) {
      return 1;
    }
    if (right.id === rosGraphSelectedId) {
      return -1;
    }
    if (left.y !== right.y) {
      return left.y - right.y;
    }
    return left.x - right.x;
  });

  for (const entity of orderedEntities) {
    const group = document.createElementNS(SVG_NS, "g");
    group.classList.add("ros-graph-entity", entity.kind === "node" ? "is-node" : "is-topic");
    if (entity.id === rosGraphSelectedId) {
      group.classList.add("is-selected");
    } else if (hasSelection && !focusIds.has(entity.id)) {
      group.classList.add("is-dimmed");
    }
    group.setAttribute("transform", "translate(" + entity.x.toFixed(1) + " " + entity.y.toFixed(1) + ")");
    group.addEventListener("click", (event) => {
      event.stopPropagation();
      selectRosGraphEntity(entity.id);
    });

    const title = document.createElementNS(SVG_NS, "title");
    title.textContent = entity.kind === "topic" && entity.type
      ? entity.name + " (" + entity.type + ")"
      : entity.name;
    group.appendChild(title);

    if (entity.kind === "node") {
      const ellipse = document.createElementNS(SVG_NS, "ellipse");
      ellipse.classList.add("ros-graph-entity-shape");
      ellipse.setAttribute("cx", "0");
      ellipse.setAttribute("cy", "0");
      ellipse.setAttribute("rx", String(entity.width / 2));
      ellipse.setAttribute("ry", String(entity.height / 2));
      group.appendChild(ellipse);
    } else {
      const rect = document.createElementNS(SVG_NS, "rect");
      rect.classList.add("ros-graph-entity-shape");
      rect.setAttribute("x", String(-entity.width / 2));
      rect.setAttribute("y", String(-entity.height / 2));
      rect.setAttribute("width", String(entity.width));
      rect.setAttribute("height", String(entity.height));
      rect.setAttribute("rx", "8");
      rect.setAttribute("ry", "8");
      group.appendChild(rect);
    }

    const label = document.createElementNS(SVG_NS, "text");
    label.classList.add("ros-graph-entity-label");
    label.textContent = rosGraphLabel(entity.name, entity.kind === "node" ? 24 : 34);
    group.appendChild(label);

    elements.rosGraphViewportGroup.appendChild(group);
  }

  applyRosGraphTransform();
}

function renderRosGraphModal(): void {
  const refreshLabel = t(currentLanguage, "button.refresh");
  const fitLabel = t(currentLanguage, "button.fitView");
  const modeLabel = t(currentLanguage, rosGraphViewMode === "detail" ? "button.showRosGraphNodeConnections" : "button.showRosGraphDetail");
  const closeLabel = t(currentLanguage, "button.close");
  elements.rosGraphSearch.value = rosGraphSearchQuery;
  elements.rosGraphSearch.placeholder = t(currentLanguage, "label.rosGraphSearch");
  elements.rosGraphSearch.setAttribute("aria-label", t(currentLanguage, "label.rosGraphSearch"));
  elements.rosGraphRefreshBtn.title = refreshLabel;
  elements.rosGraphRefreshBtn.setAttribute("aria-label", refreshLabel);
  elements.rosGraphFitBtn.title = fitLabel;
  elements.rosGraphFitBtn.setAttribute("aria-label", fitLabel);
  elements.rosGraphModeBtn.title = modeLabel;
  elements.rosGraphModeBtn.setAttribute("aria-label", modeLabel);
  elements.rosGraphModeBtn.setAttribute("aria-pressed", rosGraphViewMode === "nodeCommunication" ? "true" : "false");
  elements.rosGraphModeBtn.classList.toggle("is-active", rosGraphViewMode === "nodeCommunication");
  elements.rosGraphCloseBtn.title = closeLabel;
  elements.rosGraphCloseBtn.setAttribute("aria-label", closeLabel);
  elements.rosGraphRefreshBtn.disabled = !rosClient.isConnected() || rosGraphBusy;
  elements.rosGraphModeBtn.disabled = !rosGraphSnapshot;
  const visible = getRosGraphVisibleData();
  elements.rosGraphFitBtn.disabled = visible.entities.length === 0;
  elements.rosGraphSearch.disabled = !rosClient.isConnected() && !rosGraphSnapshot;
  elements.rosGraphMeta.textContent = rosGraphMetaText();

  if (rosGraphSnapshot && rosGraphSnapshot.partialFailures.length > 0) {
    elements.rosGraphBanner.hidden = false;
    elements.rosGraphBanner.textContent = t(currentLanguage, "state.rosGraphPartial", {
      count: rosGraphSnapshot.partialFailures.length
    });
    elements.rosGraphBanner.title = rosGraphSnapshot.partialFailures.join("\n");
  } else {
    elements.rosGraphBanner.hidden = true;
    elements.rosGraphBanner.textContent = "";
    elements.rosGraphBanner.removeAttribute("title");
  }

  renderRosGraphCanvas(visible);
  renderRosGraphDetail(visible.entityIds);
}

applyTheme(currentTheme);
sceneManager.setTheme(currentTheme);
sceneManager.setPlannedTrajectoryVisible(plannedTrajectoryVisible);
elements.plannedTrajectoryToggleBtn.innerHTML = TRAJ_ICON_PLAN_PATH;
elements.rosGraphToggleBtn.innerHTML = ROS_GRAPH_ICON;
elements.trajectoryChartsToggleBtn.innerHTML = TRAJECTORY_CHARTS_ICON;
elements.rosGraphRefreshBtn.innerHTML = ROS_GRAPH_REFRESH_ICON;
elements.rosGraphFitBtn.innerHTML = ROS_GRAPH_FIT_ICON;
elements.rosGraphModeBtn.innerHTML = ROS_GRAPH_MODE_ICON;
elements.rosGraphCloseBtn.innerHTML = ROS_GRAPH_CLOSE_ICON;
elements.trajectoryChartsCloseBtn.innerHTML = ROS_GRAPH_CLOSE_ICON;
elements.resetViewBtn.innerHTML = VIEW_ICON_RESET;
updatePlannedTrajectoryToggleUi();
updateResetViewButtonUi();
renderConfigToUi();
updatePointCloudOptions([], config.defaultPointCloudTopic);
enhanceSelect(elements.pointCloudTopic);
enhanceSelect(elements.cartesianFrame);
jointAngleUnit = elements.jointAngleUnit.value as AngleUnit;
cartesianLengthUnit = elements.cartesianLengthUnit.value as LengthUnit;
cartesianAngleUnit = elements.cartesianAngleUnit.value as AngleUnit;
updateRightPanelUnitButtons();
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

  if (state === "disconnected" || state === "error") {
    clearRosGraphState();
    clearTrajectoryChartsState();
  }

  if (state === "connected" && rosGraphOpen && !rosGraphSnapshot && !rosGraphBusy) {
    void refreshRosGraphSnapshot();
  }

  renderRosGraphModal();
  renderTrajectoryChartsModal();
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
  scheduleTrajectoryChartsRender();
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

elements.rosGraphToggleBtn.addEventListener("click", () => {
  setRosGraphOpen(!rosGraphOpen);
});

elements.trajectoryChartsToggleBtn.addEventListener("click", () => {
  setTrajectoryChartsOpen(!trajectoryChartsOpen);
});

elements.resetViewBtn.addEventListener("click", () => {
  sceneManager.resetView();
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

elements.rosGraphCloseBtn.addEventListener("click", () => {
  setRosGraphOpen(false);
});

elements.trajectoryChartsCloseBtn.addEventListener("click", () => {
  setTrajectoryChartsOpen(false);
});

elements.rosGraphModalBackdrop.addEventListener("click", () => {
  setRosGraphOpen(false);
});

elements.trajectoryChartsModalBackdrop.addEventListener("click", () => {
  setTrajectoryChartsOpen(false);
});

elements.rosGraphRefreshBtn.addEventListener("click", () => {
  void refreshRosGraphSnapshot();
});

elements.rosGraphFitBtn.addEventListener("click", () => {
  fitRosGraphToView();
});

elements.rosGraphModeBtn.addEventListener("click", () => {
  toggleRosGraphViewMode();
});

elements.rosGraphSearch.addEventListener("input", () => {
  rosGraphSearchQuery = elements.rosGraphSearch.value;
  renderRosGraphModal();
  window.requestAnimationFrame(() => {
    if (rosGraphOpen) {
      fitRosGraphToView();
    }
  });
});

elements.trajectoryChartsPositionDegBtn.addEventListener("click", () => {
  trajectoryChartsPositionUnit = "deg";
  renderTrajectoryChartsModal();
});

elements.trajectoryChartsPositionRadBtn.addEventListener("click", () => {
  trajectoryChartsPositionUnit = "rad";
  renderTrajectoryChartsModal();
});

elements.trajectoryChartsTcpMmBtn.addEventListener("click", () => {
  trajectoryChartsTcpUnit = "mm";
  renderTrajectoryChartsModal();
});

elements.trajectoryChartsTcpMBtn.addEventListener("click", () => {
  trajectoryChartsTcpUnit = "m";
  renderTrajectoryChartsModal();
});

elements.trajectoryChartsVelocityDegBtn.addEventListener("click", () => {
  trajectoryChartsVelocityUnit = "deg";
  renderTrajectoryChartsModal();
});

elements.trajectoryChartsVelocityRadBtn.addEventListener("click", () => {
  trajectoryChartsVelocityUnit = "rad";
  renderTrajectoryChartsModal();
});

elements.trajectoryChartsAccelerationDegBtn.addEventListener("click", () => {
  trajectoryChartsAccelerationUnit = "deg";
  renderTrajectoryChartsModal();
});

elements.trajectoryChartsAccelerationRadBtn.addEventListener("click", () => {
  trajectoryChartsAccelerationUnit = "rad";
  renderTrajectoryChartsModal();
});

elements.trajectoryChartsShowAllBtn.addEventListener("click", () => {
  const segment = getTrajectoryChartsSegment();
  if (!segment) {
    return;
  }
  trajectoryChartsVisibleJoints = new Set(segment.jointNames);
  renderTrajectoryChartsJointChips(segment);
  scheduleTrajectoryChartsRender();
});

elements.trajectoryChartsResetBtn.addEventListener("click", () => {
  const segment = getTrajectoryChartsSegment();
  if (!segment) {
    return;
  }
  trajectoryChartsVisibleJoints = defaultTrajectoryChartsVisibleJoints(segment.jointNames);
  renderTrajectoryChartsJointChips(segment);
  scheduleTrajectoryChartsRender();
});

elements.rosGraphSvg.addEventListener("click", (event) => {
  const target = event.target;
  if (target instanceof Element && target.closest(".ros-graph-entity")) {
    return;
  }
  if (rosGraphSelectedId) {
    rosGraphSelectedId = null;
    renderRosGraphModal();
  }
});

elements.rosGraphSvg.addEventListener("pointerdown", (event) => {
  const target = event.target;
  if (target instanceof Element && target.closest(".ros-graph-entity")) {
    return;
  }
  if (!rosGraphSnapshot || getRosGraphVisibleData().entities.length === 0) {
    return;
  }
  rosGraphPointerState = {
    pointerId: event.pointerId,
    startX: event.clientX,
    startY: event.clientY,
    panX: rosGraphPanX,
    panY: rosGraphPanY
  };
  elements.rosGraphSvg.setPointerCapture(event.pointerId);
  elements.rosGraphSvg.classList.add("is-panning");
});

elements.rosGraphSvg.addEventListener("pointermove", (event) => {
  if (!rosGraphPointerState || rosGraphPointerState.pointerId !== event.pointerId) {
    return;
  }
  rosGraphPanX = rosGraphPointerState.panX + (event.clientX - rosGraphPointerState.startX);
  rosGraphPanY = rosGraphPointerState.panY + (event.clientY - rosGraphPointerState.startY);
  applyRosGraphTransform();
});

elements.rosGraphSvg.addEventListener("pointerup", (event) => {
  if (!rosGraphPointerState || rosGraphPointerState.pointerId !== event.pointerId) {
    return;
  }
  rosGraphPointerState = null;
  elements.rosGraphSvg.classList.remove("is-panning");
  if (elements.rosGraphSvg.hasPointerCapture(event.pointerId)) {
    elements.rosGraphSvg.releasePointerCapture(event.pointerId);
  }
});

elements.rosGraphSvg.addEventListener("pointercancel", (event) => {
  if (!rosGraphPointerState || rosGraphPointerState.pointerId !== event.pointerId) {
    return;
  }
  rosGraphPointerState = null;
  elements.rosGraphSvg.classList.remove("is-panning");
  if (elements.rosGraphSvg.hasPointerCapture(event.pointerId)) {
    elements.rosGraphSvg.releasePointerCapture(event.pointerId);
  }
});

elements.rosGraphSvg.addEventListener("wheel", (event) => {
  if (!rosGraphSnapshot || getRosGraphVisibleData().entities.length === 0) {
    return;
  }
  event.preventDefault();
  const rect = elements.rosGraphSvg.getBoundingClientRect();
  if (rect.width <= 0 || rect.height <= 0) {
    return;
  }
  const pointerX = event.clientX - rect.left;
  const pointerY = event.clientY - rect.top;
  const nextScale = clampNumber(rosGraphScale * Math.exp(-event.deltaY * 0.001), ROS_GRAPH_MIN_SCALE, ROS_GRAPH_MAX_SCALE);
  const worldX = (pointerX - rosGraphPanX) / rosGraphScale;
  const worldY = (pointerY - rosGraphPanY) / rosGraphScale;
  rosGraphScale = nextScale;
  rosGraphPanX = pointerX - worldX * rosGraphScale;
  rosGraphPanY = pointerY - worldY * rosGraphScale;
  applyRosGraphTransform();
}, { passive: false });

window.addEventListener("keydown", (event) => {
  if (event.key === "Escape") {
    hideDetailModal();
    if (rosGraphOpen) {
      setRosGraphOpen(false);
    }
    if (trajectoryChartsOpen) {
      setTrajectoryChartsOpen(false);
    }
  }
});

window.addEventListener("resize", () => {
  scheduleTrajectoryChartsRender();
});

elements.jointAngleUnit.addEventListener("change", () => {
  jointAngleUnit = elements.jointAngleUnit.value as AngleUnit;
  updateRightPanelUnitButtons();
  scheduleRightPanelUpdate();
});

elements.jointAngleDegBtn.addEventListener("click", () => {
  jointAngleUnit = "deg";
  elements.jointAngleUnit.value = jointAngleUnit;
  updateRightPanelUnitButtons();
  scheduleRightPanelUpdate();
});

elements.jointAngleRadBtn.addEventListener("click", () => {
  jointAngleUnit = "rad";
  elements.jointAngleUnit.value = jointAngleUnit;
  updateRightPanelUnitButtons();
  scheduleRightPanelUpdate();
});

elements.cartesianLengthUnit.addEventListener("change", () => {
  cartesianLengthUnit = elements.cartesianLengthUnit.value as LengthUnit;
  updateRightPanelUnitButtons();
  scheduleRightPanelUpdate();
});

elements.cartesianLengthMmBtn.addEventListener("click", () => {
  cartesianLengthUnit = "mm";
  elements.cartesianLengthUnit.value = cartesianLengthUnit;
  updateRightPanelUnitButtons();
  scheduleRightPanelUpdate();
});

elements.cartesianLengthMBtn.addEventListener("click", () => {
  cartesianLengthUnit = "m";
  elements.cartesianLengthUnit.value = cartesianLengthUnit;
  updateRightPanelUnitButtons();
  scheduleRightPanelUpdate();
});

elements.cartesianAngleUnit.addEventListener("change", () => {
  cartesianAngleUnit = elements.cartesianAngleUnit.value as AngleUnit;
  updateRightPanelUnitButtons();
  scheduleRightPanelUpdate();
});

elements.cartesianAngleDegBtn.addEventListener("click", () => {
  cartesianAngleUnit = "deg";
  elements.cartesianAngleUnit.value = cartesianAngleUnit;
  updateRightPanelUnitButtons();
  scheduleRightPanelUpdate();
});

elements.cartesianAngleRadBtn.addEventListener("click", () => {
  cartesianAngleUnit = "rad";
  elements.cartesianAngleUnit.value = cartesianAngleUnit;
  updateRightPanelUnitButtons();
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
