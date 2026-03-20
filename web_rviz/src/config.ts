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
const STORAGE_VERSION = 2;

function buildDefaultRosbridgeUrl(): string {
  const host = window.location.hostname || "127.0.0.1";
  return `ws://${host}:9090`;
}

export const defaultConfig: RuntimeConfig = {
  rosbridgeUrl: buildDefaultRosbridgeUrl(),
  rvizConfigPath: "/ros_pkgs/mr12_moveit_config/config/moveit.rviz",
  urdfFallbackPath: "/ros_pkgs/mr12urdf20240605/urdf/mr12urdf20240605.urdf",
  packageRootUrl: "/ros_pkgs",
  fixedFrame: "base_link",
  defaultPointCloudTopic: "/pointcloud/output",
  maxPoints: 50000,
  targetFps: 20
};

const LEGACY_URDF_FALLBACKS = new Set([
  "/urdf/five_axis.urdf",
  "urdf/five_axis.urdf"
]);

function migrateStoredConfig(parsed: Partial<RuntimeConfig> & { _version?: number }): Partial<RuntimeConfig> {
  const next = { ...parsed };
  const version = typeof next._version === "number" ? next._version : 0;

  if (typeof next.urdfFallbackPath === "string" && LEGACY_URDF_FALLBACKS.has(next.urdfFallbackPath.trim())) {
    next.urdfFallbackPath = "";
  }

  if (version < STORAGE_VERSION && (!next.urdfFallbackPath || next.urdfFallbackPath.trim().length === 0)) {
    next.urdfFallbackPath = defaultConfig.urdfFallbackPath;
  }

  delete (next as { _version?: number })._version;
  return next;
}

export function loadConfig(): RuntimeConfig {
  const raw = window.localStorage.getItem(STORAGE_KEY);
  if (!raw) {
    return { ...defaultConfig };
  }

  try {
    const parsed = migrateStoredConfig(JSON.parse(raw) as Partial<RuntimeConfig> & { _version?: number });
    return {
      ...defaultConfig,
      ...parsed
    };
  } catch {
    return { ...defaultConfig };
  }
}

export function saveConfig(config: RuntimeConfig): void {
  window.localStorage.setItem(STORAGE_KEY, JSON.stringify({
    ...config,
    _version: STORAGE_VERSION
  }));
}
