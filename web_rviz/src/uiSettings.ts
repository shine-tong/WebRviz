export type Language = "en" | "zh";
export type ThemeMode = "dark" | "light";

const LANGUAGE_STORAGE_KEY = "webrviz-language";
const THEME_STORAGE_KEY = "webrviz-theme";

type TranslationMap = Record<string, string>;

const translations: Record<Language, TranslationMap> = {
  en: {
    "app.title": "WebRviz",
    "label.rosbridgeUrl": "rosbridge URL",
    "label.rvizConfigPath": "RViz config URL",
    "label.urdfFallbackPath": "URDF fallback URL",
    "label.packageRootUrl": "URDF package root URL",
    "label.pointCloudTopic": "PointCloud2 topic",
    "label.fixedFrame": "Fixed Frame",
    "label.status": "Status",
    "label.frame": "TCP link",
    "label.rosGraphSearch": "Search nodes or topics",
    "label.trajectoryChartsStatus": "Status",
    "label.trajectoryChartsDuration": "Duration",
    "label.trajectoryChartsSamples": "Samples",
    "label.trajectoryChartsTcpFrame": "TCP Frame",
    "panel.robotState": "Robot State",
    "panel.rosInfo": "ROS Info",
    "section.jointAngles": "Joint Angles",
    "section.cartesian": "Cartesian Position",
    "section.tfTree": "TF Tree",
    "section.trajectoryJointPosition": "Joint Trajectory",
    "section.trajectoryTcpPosition": "TCP Trajectory",
    "section.trajectoryJointVelocity": "Joint Velocity",
    "section.trajectoryJointAcceleration": "Joint Acceleration",
    "section.trajectoryJointEffort": "Torque / Effort",
    "section.topics": "Topics",
    "section.services": "Services",
    "section.params": "Params",
    "button.connect": "Connect",
    "button.disconnect": "Disconnect",
    "button.syncRviz": "Sync RViz",
    "button.loadAllParams": "Load All Params",
    "button.close": "Close",
    "button.view": "View",
    "button.showSidebar": "Show Sidebar",
    "button.hideSidebar": "Hide Sidebar",
    "button.showPlannedTrajectory": "Show plan path",
    "button.hidePlannedTrajectory": "Hide plan path",
    "button.resetView": "Reset view",
    "button.switchLanguage": "Switch language",
    "button.switchTheme": "Switch theme",
    "button.themeLight": "Light",
    "button.themeDark": "Dark",
    "button.record": "Record",
    "button.stop": "Stop",
    "button.play": "Play",
    "button.pause": "Pause",
    "button.clear": "Clear",
    "button.showAllTf": "Show All",
    "button.hideAllTf": "Hide All",
    "button.selectTf": "Select Link",
    "button.showRightSidebar": "Show right sidebar",
    "button.hideRightSidebar": "Hide right sidebar",
    "button.showRosGraph": "Show ROS Graph",
    "button.hideRosGraph": "Hide ROS Graph",
    "button.refresh": "Refresh",
    "button.fitView": "Fit View",
    "button.showRosGraphNodeConnections": "Show node communication graph",
    "button.showRosGraphDetail": "Show detailed graph",
    "button.showTrajectoryCharts": "Show motion curves",
    "button.hideTrajectoryCharts": "Hide motion curves",
    "button.showAllShort": "All",
    "button.resetSelection": "Reset",
    "modal.detailTitle": "DETAIL",
    "modal.trajectoryChartsTitle": "Motion Curves",
    "modal.rosGraphTitle": "ROS Graph",
    "status.disconnected": "disconnected",
    "status.connecting": "connecting",
    "status.connected": "connected",
    "status.error": "error",
    "statusDetail.connected": "connected",
    "statusDetail.disconnected": "disconnected",
    "statusDetail.connectionClosed": "connection closed",
    "detail.noTypeForTopic": "No type for this topic",
    "detail.loadingMessage": "Loading message details for {type} ...",
    "detail.messageError": "Error getting message info for {type}: {detail}",
    "detail.loadingService": "Loading service details for {type} ...",
    "detail.serviceUnavailable": "Service detail is unavailable for {type}: {detail}",
    "detail.request": "[Request]",
    "detail.response": "[Response]",
    "detail.noMessageInfo": "No message info for {type}",
    "detail.loadingParam": "Loading param value: {paramName} ...",
    "detail.paramError": "Error getting param '{paramName}': {detail}",
    "detail.paramValueLabel": "Value of param '{paramName}'",
    "detail.parametersTitle": "Parameters:",
    "detail.errorValue": "<error: {detail}>",
    "detail.rosGraphNode": "Node",
    "detail.rosGraphTopic": "Topic",
    "detail.rosGraphPublishing": "Publishing",
    "detail.rosGraphSubscribing": "Subscribing",
    "detail.rosGraphPublishers": "Publishers",
    "detail.rosGraphSubscribers": "Subscribers",
    "detail.rosGraphServices": "Services",
    "detail.rosGraphType": "Message Type",
    "state.notConnected": "not connected",
    "state.rosGraphHint": "Select a node or topic to inspect details",
    "state.rosGraphNodeHint": "Select a node to inspect details",
    "state.rosGraphLoading": "Loading ROS graph...",
    "state.rosGraphEmpty": "No nodes or topics found",
    "state.rosGraphNoMatch": "No graph items match the current search",
    "state.rosGraphRefreshFailed": "Failed to load ROS graph: {detail}",
    "state.rosGraphPartial": "{count} nodes failed to load",
    "state.rosGraphUpdatedAt": "Updated {time}",
    "state.rosGraphNeverUpdated": "Not refreshed yet",
    "state.trajectoryChartsEmpty": "Waiting for motion data",
    "state.trajectoryChartsCapturing": "Capturing",
    "state.trajectoryChartsLatest": "Latest segment",
    "state.trajectoryChartsNoTcpFrame": "No TCP frame",
    "state.trajectoryChartsDisconnected": "Connect ROS to collect motion curves",
    "state.trajectoryChartsMetaCapturing": "Capturing current motion segment",
    "state.trajectoryChartsMetaLatest": "Showing latest completed motion segment",
    "state.noTopics": "no topics",
    "state.noServices": "no services",
    "state.noParams": "no params",
    "state.noJointData": "no joint data",
    "state.noFrame": "no frame",
    "state.unavailable": "unavailable",
    "state.noTfData": "no tf data",
    "state.noVisibleTf": "no visible tf",
    "tf.fixed": "fixed",
    "tf.selected": "selected",
    "tf.loop": "loop",
    "common.unknown": "unknown",
    "unit.seconds": "s",
    "log.ready": "WebRviz ready. Click Connect to start."
  },
  zh: {
    "app.title": "WebRviz",
    "label.rosbridgeUrl": "rosbridge 地址",
    "label.rvizConfigPath": "RViz 配置地址",
    "label.urdfFallbackPath": "URDF 备用地址",
    "label.packageRootUrl": "URDF 包根地址",
    "label.pointCloudTopic": "PointCloud2 话题",
    "label.fixedFrame": "固定坐标系",
    "label.status": "状态",
    "label.frame": "TCP link",
    "label.rosGraphSearch": "搜索节点或话题",
    "panel.robotState": "机器人状态",
    "panel.rosInfo": "ROS 信息",
    "section.jointAngles": "关节角度",
    "section.cartesian": "笛卡尔位置",
    "section.tfTree": "TF 树",
    "section.topics": "话题",
    "section.services": "服务",
    "section.params": "参数",
    "button.connect": "连接",
    "button.disconnect": "断开",
    "button.syncRviz": "同步 RViz",
    "button.loadAllParams": "加载全部参数",
    "button.close": "关闭",
    "button.view": "查看",
    "button.showSidebar": "显示侧边栏",
    "button.hideSidebar": "隐藏侧边栏",
    "button.showPlannedTrajectory": "显示规划路径",
    "button.hidePlannedTrajectory": "隐藏规划路径",
    "button.resetView": "重置视图",
    "button.switchLanguage": "切换语言",
    "button.switchTheme": "切换主题",
    "button.themeLight": "亮色",
    "button.themeDark": "暗色",
    "button.record": "录制",
    "button.stop": "停止",
    "button.play": "播放",
    "button.pause": "暂停",
    "button.clear": "清除",
    "button.showAllTf": "显示全部",
    "button.hideAllTf": "隐藏全部",
    "button.selectTf": "选择 Link",
    "button.showRightSidebar": "显示右侧栏",
    "button.hideRightSidebar": "隐藏右侧栏",
    "button.showRosGraph": "显示 ROS 图谱",
    "button.hideRosGraph": "隐藏 ROS 图谱",
    "button.refresh": "\u5237\u65b0",
    "button.fitView": "\u9002\u914d\u89c6\u56fe",
    "button.showRosGraphNodeConnections": "\u663e\u793a\u8282\u70b9\u901a\u4fe1\u56fe",
    "button.showRosGraphDetail": "\u663e\u793a\u8be6\u7ec6\u56fe\u8c31",
    "button.showTrajectoryCharts": "\u663e\u793a\u8fd0\u52a8\u66f2\u7ebf",
    "button.hideTrajectoryCharts": "\u9690\u85cf\u8fd0\u52a8\u66f2\u7ebf",
    "button.showAllShort": "\u5168\u90e8",
    "button.resetSelection": "\u91cd\u7f6e",
    "modal.detailTitle": "详情",
    "modal.rosGraphTitle": "ROS 图谱",
    "status.disconnected": "未连接",
    "status.connecting": "连接中",
    "status.connected": "已连接",
    "status.error": "错误",
    "statusDetail.connected": "已连接",
    "statusDetail.disconnected": "未连接",
    "statusDetail.connectionClosed": "连接已关闭",
    "detail.noTypeForTopic": "该话题没有类型信息",
    "detail.loadingMessage": "正在加载 {type} 的消息详情...",
    "detail.messageError": "获取 {type} 的消息详情失败: {detail}",
    "detail.loadingService": "正在加载 {type} 的服务详情...",
    "detail.serviceUnavailable": "无法获取 {type} 的服务详情: {detail}",
    "detail.request": "[请求]",
    "detail.response": "[响应]",
    "detail.noMessageInfo": "{type} 没有消息信息",
    "detail.loadingParam": "正在加载参数值: {paramName} ...",
    "detail.paramError": "获取参数 '{paramName}' 失败: {detail}",
    "detail.paramValueLabel": "参数 '{paramName}' 的值",
    "detail.parametersTitle": "参数列表:",
    "detail.errorValue": "<错误: {detail}>",
    "detail.rosGraphNode": "节点",
    "detail.rosGraphTopic": "话题",
    "detail.rosGraphPublishing": "发布话题",
    "detail.rosGraphSubscribing": "订阅话题",
    "detail.rosGraphPublishers": "发布者",
    "detail.rosGraphSubscribers": "订阅者",
    "detail.rosGraphServices": "服务",
    "detail.rosGraphType": "消息类型",
    "state.notConnected": "\u672a\u8fde\u63a5",
    "state.rosGraphHint": "\u9009\u62e9\u4e00\u4e2a\u8282\u70b9\u6216\u8bdd\u9898\u4ee5\u67e5\u770b\u8be6\u7ec6\u4fe1\u606f",
    "state.rosGraphNodeHint": "\u9009\u62e9\u4e00\u4e2a\u8282\u70b9\u4ee5\u67e5\u770b\u8be6\u7ec6\u4fe1\u606f",
    "state.rosGraphLoading": "\u6b63\u5728\u52a0\u8f7d ROS \u56fe\u8c31...",
    "state.rosGraphEmpty": "\u6ca1\u6709\u53d1\u73b0\u8282\u70b9\u6216\u8bdd\u9898",
    "state.rosGraphNoMatch": "\u5f53\u524d\u641c\u7d22\u6ca1\u6709\u5339\u914d\u7684\u56fe\u8c31\u9879",
    "state.rosGraphRefreshFailed": "加载 ROS 图谱失败: {detail}",
    "state.rosGraphPartial": "有 {count} 个节点加载失败",
    "state.rosGraphUpdatedAt": "更新于 {time}",
    "state.rosGraphNeverUpdated": "尚未刷新",
    "state.trajectoryChartsEmpty": "\u7b49\u5f85\u8fd0\u52a8\u6570\u636e",
    "state.trajectoryChartsCapturing": "\u91c7\u96c6\u4e2d",
    "state.trajectoryChartsLatest": "\u6700\u65b0\u8fd0\u52a8\u6bb5",
    "state.trajectoryChartsNoTcpFrame": "\u65e0 TCP \u5750\u6807\u7cfb",
    "state.trajectoryChartsDisconnected": "\u8bf7\u5148\u8fde\u63a5 ROS \u4ee5\u91c7\u96c6\u8fd0\u52a8\u66f2\u7ebf",
    "state.trajectoryChartsMetaCapturing": "\u6b63\u5728\u91c7\u96c6\u5f53\u524d\u8fd0\u52a8\u6bb5",
    "state.trajectoryChartsMetaLatest": "\u6b63\u5728\u663e\u793a\u6700\u65b0\u5b8c\u6210\u7684\u8fd0\u52a8\u6bb5",
    "state.noTopics": "没有话题",
    "state.noServices": "没有服务",
    "state.noParams": "没有参数",
    "state.noJointData": "没有关节数据",
    "state.noFrame": "没有坐标系",
    "state.unavailable": "不可用",
    "state.noTfData": "没有 TF 数据",
    "state.noVisibleTf": "没有可见 TF",
    "tf.fixed": "固定",
    "tf.selected": "已选中",
    "tf.loop": "环路",
    "common.unknown": "未知",
    "unit.seconds": "秒",
    "log.ready": "WebRviz 已就绪，点击连接开始。"
  }
};

export type TranslationKey = keyof (typeof translations)["en"];

export function loadLanguage(): Language {
  const stored = window.localStorage.getItem(LANGUAGE_STORAGE_KEY);
  if (stored === "en" || stored === "zh") {
    return stored;
  }

  return "zh";
}

export function saveLanguage(language: Language): void {
  window.localStorage.setItem(LANGUAGE_STORAGE_KEY, language);
}

export function loadTheme(): ThemeMode {
  const stored = window.localStorage.getItem(THEME_STORAGE_KEY);
  if (stored === "dark" || stored === "light") {
    return stored;
  }

  return "light";
}

export function saveTheme(theme: ThemeMode): void {
  window.localStorage.setItem(THEME_STORAGE_KEY, theme);
}

export function t(language: Language, key: TranslationKey, vars: Record<string, string | number> = {}): string {
  let text = translations[language][key] ?? translations.en[key] ?? key;

  for (const [name, value] of Object.entries(vars)) {
    text = text.split(`{${name}}`).join(String(value));
  }

  return text;
}

export function applyStaticTranslations(language: Language): void {
  document.documentElement.lang = language === "zh" ? "zh-CN" : "en";
  document.title = t(language, "app.title");

  const items = document.querySelectorAll<HTMLElement>("[data-i18n]");
  for (const item of items) {
    const key = item.dataset.i18n as TranslationKey | undefined;
    if (!key) {
      continue;
    }
    item.textContent = t(language, key);
  }
}

export function applyTheme(theme: ThemeMode): void {
  document.documentElement.dataset.theme = theme;
}
