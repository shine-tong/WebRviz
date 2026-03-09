export interface RuntimeConfig {
  rosbridgeUrl: string;
  rvizConfigPath: string;
  urdfFallbackPath: string;
  packageRootUrl: string;
  fixedFrame: string;
  defaultPointCloudTopic: string;
  maxPoints: number;
  targetFps: number;
}

const STORAGE_KEY = "webrviz-runtime-config";

function buildDefaultRosbridgeUrl(): string {
  const host = window.location.hostname || "127.0.0.1";
  return `ws://${host}:9090`;
}

export const defaultConfig: RuntimeConfig = {
  rosbridgeUrl: buildDefaultRosbridgeUrl(),
  rvizConfigPath: "/rviz/moveit.rviz",
  urdfFallbackPath: "/urdf/five_axis.urdf",
  packageRootUrl: "/ros_pkgs",
  fixedFrame: "base_link",
  defaultPointCloudTopic: "/pointcloud/output",
  maxPoints: 50000,
  targetFps: 20
};

export function loadConfig(): RuntimeConfig {
  const raw = window.localStorage.getItem(STORAGE_KEY);
  if (!raw) {
    return { ...defaultConfig };
  }

  try {
    const parsed = JSON.parse(raw) as Partial<RuntimeConfig>;
    return {
      ...defaultConfig,
      ...parsed
    };
  } catch {
    return { ...defaultConfig };
  }
}

export function saveConfig(config: RuntimeConfig): void {
  window.localStorage.setItem(STORAGE_KEY, JSON.stringify(config));
}
