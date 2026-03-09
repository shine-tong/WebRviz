import { RosClient } from "../ros/rosClient";

export interface LoadedRobotModel {
  robot: any;
  rootLink: string;
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
  if (text.includes("\\\"")) {
    text = text.replace(/\\\"/g, '"');
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

async function getParamUrdf(rosClient: RosClient): Promise<string> {
  const paramValue = await rosClient.getParam("/robot_description");
  const normalized = normalizeUrdfText(paramValue);
  ensureRobotTag(normalized);
  return normalized;
}

async function getFallbackUrdf(fallbackPath: string): Promise<string> {
  if (!fallbackPath) {
    throw new Error("robot_description unavailable and urdfFallbackPath is empty");
  }

  const raw = await fetchText(fallbackPath);
  const normalized = normalizeUrdfText(raw);
  ensureRobotTag(normalized);
  return normalized;
}

export async function loadRobotUrdfText(rosClient: RosClient, fallbackPath: string): Promise<string> {
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
  return { robot, rootLink };
}

export async function loadRobotModel(
  rosClient: RosClient,
  fallbackPath: string,
  packageRootUrl: string
): Promise<LoadedRobotModel> {
  const urdfXml = await loadRobotUrdfText(rosClient, fallbackPath);
  return buildRobotFromUrdf(urdfXml, packageRootUrl);
}
