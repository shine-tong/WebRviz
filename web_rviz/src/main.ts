import "./style.css";
import { RuntimeConfig, loadConfig, saveConfig } from "./config";
import { decodePointCloud2 } from "./ros/pointcloud";
import { RosClient, TopicInfo } from "./ros/rosClient";
import { loadRvizSyncHints } from "./rviz/rvizConfig";
import { loadRobotModel } from "./visualization/robotLoader";
import { SceneManager } from "./visualization/sceneManager";

interface AppElements {
  rosbridgeUrl: HTMLInputElement;
  rvizConfigPath: HTMLInputElement;
  urdfFallbackPath: HTMLInputElement;
  packageRootUrl: HTMLInputElement;
  pointCloudTopic: HTMLSelectElement;
  connectBtn: HTMLButtonElement;
  syncBtn: HTMLButtonElement;
  fixedFrameValue: HTMLElement;
  connectionStatus: HTMLElement;
  logBox: HTMLElement;
  viewport: HTMLElement;
}

interface TopicSubscriptions {
  jointStates?: any;
  tf?: any;
  tfStatic?: any;
  pointCloud?: any;
}

const POINTCLOUD2_TYPE = "sensor_msgs/PointCloud2";

function getElements(): AppElements {
  const byId = <T extends HTMLElement>(id: string): T => {
    const element = document.getElementById(id);
    if (!element) {
      throw new Error(`missing element: ${id}`);
    }
    return element as T;
  };

  return {
    rosbridgeUrl: byId<HTMLInputElement>("rosbridgeUrl"),
    rvizConfigPath: byId<HTMLInputElement>("rvizConfigPath"),
    urdfFallbackPath: byId<HTMLInputElement>("urdfFallbackPath"),
    packageRootUrl: byId<HTMLInputElement>("packageRootUrl"),
    pointCloudTopic: byId<HTMLSelectElement>("pointCloudTopic"),
    connectBtn: byId<HTMLButtonElement>("connectBtn"),
    syncBtn: byId<HTMLButtonElement>("syncBtn"),
    fixedFrameValue: byId<HTMLElement>("fixedFrameValue"),
    connectionStatus: byId<HTMLElement>("connectionStatus"),
    logBox: byId<HTMLElement>("logBox"),
    viewport: byId<HTMLElement>("viewport")
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
    sceneManager.updateJointStates(message);
  });

  subscriptions.tf = rosClient.createTopic("/tf", "tf2_msgs/TFMessage");
  subscriptions.tf.subscribe((message: unknown) => {
    sceneManager.upsertTfMessage(message);
  });

  subscriptions.tfStatic = rosClient.createTopic("/tf_static", "tf2_msgs/TFMessage");
  subscriptions.tfStatic.subscribe((message: unknown) => {
    sceneManager.upsertTfMessage(message);
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

  log(`sync applied: fixed_frame=${config.fixedFrame}, pointcloud=${desiredTopic}`);
}

renderConfigToUi();
updatePointCloudOptions([], config.defaultPointCloudTopic);
updateStatusLabel("disconnected");

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
