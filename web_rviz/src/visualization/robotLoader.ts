import { RosClient } from "../ros/rosClient";

export interface LoadedRobotModel {
  robot: any;
  rootLink: string;
  source: string;
}

interface LoadedUrdfText {
  text: string;
  source: string;
}

async function fetchText(url: string): Promise<string> {
  const response = await fetch(url, { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`failed to fetch ${url}: ${response.status} ${response.statusText}`);
  }
  return response.text();
}

function normalizeUrdfText(raw: string): string {
  let text = (raw || "").trim();
  if (!text) {
    return text;
  }

  const looksQuoted =
    (text.startsWith('"') && text.endsWith('"')) ||
    (text.startsWith("'") && text.endsWith("'"));

  if (looksQuoted) {
    try {
      const parsed = JSON.parse(text);
      if (typeof parsed === "string") {
        text = parsed.trim();
      }
    } catch {
      text = text.slice(1, -1).trim();
    }
  }

  if (!text.includes("\n") && text.includes("\\n")) {
    text = text.replace(/\\n/g, "\n");
  }
  if (text.includes('\\"')) {
    text = text.replace(/\\"/g, '"');
  }

  return text;
}

function ensureRobotTag(xml: string): void {
  if (!xml.includes("<robot")) {
    throw new Error("URDF text does not contain <robot>");
  }
}

function parseUrdfOrThrow(urdfXml: string): Document {
  const parser = new DOMParser();
  const documentXml = parser.parseFromString(urdfXml, "text/xml");
  const parserError = documentXml.querySelector("parsererror");
  if (parserError) {
    throw new Error(`invalid URDF XML: ${parserError.textContent || "parse error"}`);
  }
  return documentXml;
}

function inferRootLink(documentXml: Document): string {
  const linkElements = Array.from(documentXml.querySelectorAll("robot > link"));
  if (linkElements.length === 0) {
    return "base_link";
  }

  const linkNames = linkElements
    .map((element) => element.getAttribute("name") || "")
    .filter((name) => name.length > 0);

  if (linkNames.includes("base_link")) {
    return "base_link";
  }

  const childNames = new Set(
    Array.from(documentXml.querySelectorAll("robot > joint > child"))
      .map((element) => element.getAttribute("link") || "")
      .filter((name) => name.length > 0)
  );

  const rootCandidate = linkNames.find((name) => !childNames.has(name));
  return rootCandidate || linkNames[0] || "base_link";
}

function normalizeBaseUrl(baseUrl: string): string {
  return (baseUrl || "").trim().replace(/\/+$/, "");
}

function resolvePackageUrl(packageName: string, packageRootUrl: string): string {
  const root = normalizeBaseUrl(packageRootUrl);
  if (!root) {
    return `/${packageName}`;
  }
  return `${root}/${packageName}`;
}

function normalizeParamName(name: string): string {
  return (name || "").trim().replace(/^\/+/, "").toLowerCase();
}

function isRobotDescriptionParam(name: string): boolean {
  const normalized = normalizeParamName(name);
  return normalized === "robot_description"
    || normalized.endsWith(":robot_description")
    || normalized.endsWith(".robot_description")
    || normalized.endsWith("/robot_description");
}

function robotDescriptionParamPriority(name: string): number {
  const normalized = normalizeParamName(name);
  if (normalized === "robot_description") {
    return 0;
  }
  if (normalized.includes("robot_state_publisher")) {
    return 1;
  }
  if (normalized.includes("move_group")) {
    return 2;
  }
  if (normalized.includes("moveit_cpp")) {
    return 3;
  }
  return 4;
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

function commonRobotDescriptionCandidates(): string[] {
  return [
    "/move_group:robot_description",
    "move_group:robot_description",
    "/robot_state_publisher:robot_description",
    "robot_state_publisher:robot_description",
    "/moveit_cpp:robot_description",
    "moveit_cpp:robot_description"
  ];
}

function buildNodeRobotDescriptionCandidates(nodes: string[]): string[] {
  const scoped: string[] = [];
  for (const node of nodes) {
    const raw = (node || "").trim().replace(/\/+$/, "");
    const trimmed = raw.replace(/^\/+/, "");
    if (!trimmed) {
      continue;
    }

    scoped.push(`${raw.startsWith("/") ? raw : "/" + raw}:robot_description`);
    scoped.push(`${trimmed}:robot_description`);
  }

  return orderedUniqueStrings(
    scoped
      .filter((name) => isRobotDescriptionParam(name))
      .sort((left, right) => {
        const priorityDiff = robotDescriptionParamPriority(left) - robotDescriptionParamPriority(right);
        return priorityDiff !== 0 ? priorityDiff : left.localeCompare(right);
      })
  );
}

async function tryRobotDescriptionCandidates(
  rosClient: RosClient,
  candidates: string[]
): Promise<LoadedUrdfText | null> {
  for (const candidate of orderedUniqueStrings(candidates)) {
    try {
      const paramValue = await rosClient.getParam(candidate);
      const normalized = normalizeUrdfText(paramValue);
      ensureRobotTag(normalized);
      return {
        text: normalized,
        source: `param ${candidate}`
      };
    } catch {
      // ignore and continue probing likely names
    }
  }

  return null;
}

async function getParamUrdf(rosClient: RosClient): Promise<LoadedUrdfText> {
  const directMatch = await tryRobotDescriptionCandidates(rosClient, commonRobotDescriptionCandidates());
  if (directMatch) {
    return directMatch;
  }

  let lastError: unknown = null;

  try {
    const discoveredParams = await rosClient.listParams();
    const paramMatch = await tryRobotDescriptionCandidates(
      rosClient,
      discoveredParams
        .filter((name) => isRobotDescriptionParam(name))
        .sort((left, right) => {
          const priorityDiff = robotDescriptionParamPriority(left) - robotDescriptionParamPriority(right);
          return priorityDiff !== 0 ? priorityDiff : left.localeCompare(right);
        })
    );

    if (paramMatch) {
      return paramMatch;
    }
  } catch (error) {
    lastError = error;
  }

  try {
    const nodes = await rosClient.listNodes();
    const nodeMatch = await tryRobotDescriptionCandidates(rosClient, buildNodeRobotDescriptionCandidates(nodes));
    if (nodeMatch) {
      return nodeMatch;
    }
  } catch (error) {
    lastError = error;
  }

  if (lastError instanceof Error) {
    throw lastError;
  }
  throw new Error("robot_description parameter not found");
}

async function getFallbackUrdf(fallbackPath: string): Promise<LoadedUrdfText> {
  if (!fallbackPath) {
    throw new Error("robot_description unavailable and urdfFallbackPath is empty");
  }

  const raw = await fetchText(fallbackPath);
  const normalized = normalizeUrdfText(raw);
  ensureRobotTag(normalized);
  return {
    text: normalized,
    source: `fallback ${fallbackPath}`
  };
}

export async function loadRobotUrdfText(rosClient: RosClient, fallbackPath: string): Promise<LoadedUrdfText> {
  try {
    return await getParamUrdf(rosClient);
  } catch {
    return getFallbackUrdf(fallbackPath);
  }
}

export async function buildRobotFromUrdf(urdfXml: string, packageRootUrl: string): Promise<LoadedRobotModel> {
  const module = await import("urdf-loader");
  const URDFLoader = module.default;

  const parsed = parseUrdfOrThrow(urdfXml);
  const rootLink = inferRootLink(parsed);

  const loader = new URDFLoader();
  loader.packages = (packageName: string) => resolvePackageUrl(packageName, packageRootUrl);

  const robot = loader.parse(parsed);
  return {
    robot,
    rootLink,
    source: ""
  };
}

export async function loadRobotModel(
  rosClient: RosClient,
  fallbackPath: string,
  packageRootUrl: string
): Promise<LoadedRobotModel> {
  const loaded = await loadRobotUrdfText(rosClient, fallbackPath);
  const model = await buildRobotFromUrdf(loaded.text, packageRootUrl);
  return {
    ...model,
    source: loaded.source
  };
}
