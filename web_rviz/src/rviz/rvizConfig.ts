import yaml from "js-yaml";

export interface RvizSyncHints {
  fixedFrame?: string;
  pointCloudTopics: string[];
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null;
}

function pushUnique(target: string[], value: string): void {
  if (!value || target.includes(value)) {
    return;
  }
  target.push(value);
}

function collectPointCloudTopics(node: unknown, topics: string[]): void {
  if (Array.isArray(node)) {
    for (const item of node) {
      collectPointCloudTopics(item, topics);
    }
    return;
  }

  if (!isRecord(node)) {
    return;
  }

  const className = String(node["Class"] ?? "");
  if (className.includes("PointCloud")) {
    const topic = node["Topic"];
    if (typeof topic === "string") {
      pushUnique(topics, topic);
    }
  }

  for (const value of Object.values(node)) {
    collectPointCloudTopics(value, topics);
  }
}

export function parseRvizConfig(content: string): RvizSyncHints {
  const loaded = yaml.load(content);
  const pointCloudTopics: string[] = [];

  if (!isRecord(loaded)) {
    return { pointCloudTopics };
  }

  const visualizationManager = loaded["Visualization Manager"];
  if (!isRecord(visualizationManager)) {
    return { pointCloudTopics };
  }

  collectPointCloudTopics(visualizationManager["Displays"], pointCloudTopics);

  let fixedFrame: string | undefined;
  const globalOptions = visualizationManager["Global Options"];
  if (isRecord(globalOptions)) {
    const parsedFrame = globalOptions["Fixed Frame"];
    if (typeof parsedFrame === "string" && parsedFrame.trim() && parsedFrame !== "<Fixed Frame>") {
      fixedFrame = parsedFrame;
    }
  }

  return {
    fixedFrame,
    pointCloudTopics
  };
}

export async function loadRvizSyncHints(url: string): Promise<RvizSyncHints> {
  const response = await fetch(url, { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`failed to fetch RViz config: ${response.status} ${response.statusText}`);
  }

  const content = await response.text();
  return parseRvizConfig(content);
}