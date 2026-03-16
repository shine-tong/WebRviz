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
    "panel.robotState": "Robot State",
    "panel.rosInfo": "ROS Info",
    "section.jointAngles": "Joint Angles",
    "section.cartesian": "Cartesian Position",
    "section.tfTree": "TF Tree",
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
    "button.selectTf": "Select TF",
    "button.showRightSidebar": "Show right sidebar",
    "button.hideRightSidebar": "Hide right sidebar",
    "modal.detailTitle": "DETAIL",
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
    "state.notConnected": "not connected",
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
    "button.selectTf": "选择TF",
    "button.showRightSidebar": "显示右侧栏",
    "button.hideRightSidebar": "隐藏右侧栏",
    "modal.detailTitle": "详情",
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
    "state.notConnected": "未连接",
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
