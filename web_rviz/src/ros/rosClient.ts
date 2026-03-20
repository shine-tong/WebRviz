import ROSLIB from "roslib";

export type ConnectionState = "disconnected" | "connecting" | "connected" | "error";

export interface TopicInfo {
  name: string;
  type: string;
}

export interface ServiceInfo {
  name: string;
  type: string;
}

export interface MessageTypeDef {
  type: string;
  fieldnames: string[];
  fieldtypes: string[];
  fieldarraylen: number[];
}

export interface MessageDetailsResponse {
  typedefs?: MessageTypeDef[];
}

export interface ServiceDetailsResponse {
  request: MessageDetailsResponse;
  response: MessageDetailsResponse;
}

export interface NodeDetails {
  publishing: string[];
  subscribing: string[];
  services: string[];
}

export interface RosVersionInfo {
  version: number;
  distro: string;
}

type StateListener = (state: ConnectionState, message?: string) => void;

interface TopicsResponse {
  topics?: string[];
  types?: string[];
}

interface TopicTypeResponse {
  type: string;
}

interface ServicesResponse {
  services: string[];
}

interface ServiceTypeResponse {
  type: string;
}

interface GetParamNamesResponse {
  names: string[];
}

interface GetParamResponse {
  value: string;
  success?: boolean;
  reason?: string;
}

interface NodesResponse {
  nodes: string[];
}

interface NodeDetailsResponse {
  publishing?: string[];
  subscribing?: string[];
  services?: string[];
}

interface ActionServersResponse {
  action_servers?: string[];
  actions?: string[];
  names?: string[];
}

interface ActionTypeResponse {
  type?: string;
  action_type?: string;
}

interface RosVersionResponse {
  version?: number | string;
  distro?: string;
}

interface TopicsAndRawTypesResponse {
  topics?: string[];
  types?: string[];
  typedefs_full_text?: string[];
  typedefsFullText?: string[];
}

interface ListParametersResultResponse {
  names?: string[];
  prefixes?: string[];
}

interface ListParametersResponse {
  result?: ListParametersResultResponse;
}

interface ParameterValueResponse {
  type?: number;
  bool_value?: boolean;
  integer_value?: number | string;
  double_value?: number;
  string_value?: string;
  byte_array_value?: number[];
  bool_array_value?: boolean[];
  integer_array_value?: Array<number | string>;
  double_array_value?: number[];
  string_array_value?: string[];
}

interface GetParametersResponse {
  values?: ParameterValueResponse[];
}

interface GetInterfaceTextResponse {
  success?: boolean;
  message?: string;
  raw_text?: string;
  rawText?: string;
}

function normalizeTypeName(type: string): string {
  return (type || "").trim().replace(/\/msg\//g, "/");
}

function uniqueStrings(values: unknown): string[] {
  if (!Array.isArray(values)) {
    return [];
  }

  const seen = new Set<string>();
  const result: string[] = [];
  for (const value of values) {
    if (typeof value !== "string" || value.length === 0 || seen.has(value)) {
      continue;
    }
    seen.add(value);
    result.push(value);
  }

  return result;
}

export class RosClient {
  private ros: any | null = null;
  private state: ConnectionState = "disconnected";
  private listeners: StateListener[] = [];

  onStateChange(listener: StateListener): void {
    this.listeners.push(listener);
  }

  getState(): ConnectionState {
    return this.state;
  }

  isConnected(): boolean {
    return this.state === "connected" && this.ros !== null;
  }

  async connect(url: string): Promise<void> {
    this.disconnect();
    this.setState("connecting", `connecting to ${url}`);

    const ros = new ROSLIB.Ros({ url });

    await new Promise<void>((resolve, reject) => {
      let settled = false;

      ros.on("connection", () => {
        settled = true;
        this.ros = ros;
        this.setState("connected", "connected");
        resolve();
      });

      ros.on("error", (error: unknown) => {
        const detail = error instanceof Error ? error.message : String(error);
        this.setState("error", detail);
        if (!settled) {
          reject(new Error(detail));
        }
      });

      ros.on("close", () => {
        this.ros = null;
        this.setState("disconnected", "connection closed");
      });
    });
  }

  disconnect(): void {
    if (this.ros) {
      try {
        this.ros.close();
      } catch {
        // ignore close errors
      }
    }
    this.ros = null;
    this.setState("disconnected", "disconnected");
  }

  createTopic(name: string, messageType: string, throttleRateMs = 0): any {
    if (!this.ros) {
      throw new Error("ROS is not connected");
    }

    return new ROSLIB.Topic({
      ros: this.ros,
      name,
      messageType,
      throttle_rate: throttleRateMs
    });
  }

  async listTopicsWithTypes(): Promise<TopicInfo[]> {
    const topicsResponse = await this.callService<Record<string, never>, TopicsResponse>(
      "/rosapi/topics",
      "rosapi_msgs/srv/Topics",
      {}
    );

    const topicNames = uniqueStrings(topicsResponse.topics);
    if (Array.isArray(topicsResponse.types) && topicsResponse.types.length === topicNames.length) {
      return topicNames.map((name, index) => ({
        name,
        type: typeof topicsResponse.types?.[index] === "string" ? topicsResponse.types[index] : ""
      }));
    }

    const result: TopicInfo[] = [];
    for (const name of topicNames) {
      try {
        const typeResponse = await this.callService<{ topic: string }, TopicTypeResponse>(
          "/rosapi/topic_type",
          "rosapi_msgs/srv/TopicType",
          { topic: name }
        );

        result.push({ name, type: typeResponse.type });
      } catch {
        result.push({ name, type: "" });
      }
    }

    return result;
  }

  async listServicesWithTypes(): Promise<ServiceInfo[]> {
    const servicesResponse = await this.callService<Record<string, never>, ServicesResponse>(
      "/rosapi/services",
      "rosapi_msgs/srv/Services",
      {}
    );

    const result: ServiceInfo[] = [];
    for (const name of uniqueStrings(servicesResponse.services)) {
      try {
        const typeResponse = await this.callService<{ service: string }, ServiceTypeResponse>(
          "/rosapi/service_type",
          "rosapi_msgs/srv/ServiceType",
          { service: name }
        );

        result.push({ name, type: typeResponse.type });
      } catch {
        result.push({ name, type: "" });
      }
    }

    return result;
  }

  async listParams(): Promise<string[]> {
    try {
      const response = await this.callService<Record<string, never>, GetParamNamesResponse>(
        "/rosapi/get_param_names",
        "rosapi_msgs/srv/GetParamNames",
        {}
      );

      return uniqueStrings(response.names);
    } catch {
      return this.listParamsViaNodeServices();
    }
  }

  async listNodes(): Promise<string[]> {
    const response = await this.callService<Record<string, never>, NodesResponse>(
      "/rosapi/nodes",
      "rosapi_msgs/srv/Nodes",
      {}
    );

    return uniqueStrings(response.nodes);
  }

  async getNodeDetails(node: string): Promise<NodeDetails> {
    const response = await this.callService<{ node: string }, NodeDetailsResponse>(
      "/rosapi/node_details",
      "rosapi_msgs/srv/NodeDetails",
      { node }
    );

    return {
      publishing: Array.isArray(response.publishing) ? response.publishing : [],
      subscribing: Array.isArray(response.subscribing) ? response.subscribing : [],
      services: Array.isArray(response.services) ? response.services : []
    };
  }

  async getMessageDetails(type: string): Promise<MessageDetailsResponse> {
    const candidates = uniqueStrings([
      type,
      type.replace("/msg/", "/")
    ]);

    let lastError: unknown = null;
    for (const candidate of candidates) {
      try {
        return await this.callService<{ type: string }, MessageDetailsResponse>(
          "/rosapi/message_details",
          "rosapi_msgs/srv/MessageDetails",
          { type: candidate }
        );
      } catch (error) {
        lastError = error;
      }
    }

    throw lastError instanceof Error ? lastError : new Error(`message details unavailable for ${type}`);
  }

  async getServiceDetails(type: string): Promise<ServiceDetailsResponse> {
    const [request, response] = await Promise.all([
      this.getServiceRequestDetails(type),
      this.getServiceResponseDetails(type)
    ]);

    return { request, response };
  }

  async getParam(name: string, defaultValue = ""): Promise<string> {
    try {
      const response = await this.callService<{ name: string; default_value: string }, GetParamResponse>(
        "/rosapi/get_param",
        "rosapi_msgs/srv/GetParam",
        { name, default_value: defaultValue }
      );

      if (response.success === false) {
        throw new Error(response.reason || `parameter ${name} unavailable`);
      }

      return response.value;
    } catch {
      try {
        const response = await this.callService<{ name: string }, GetParamResponse>(
          "/rosapi/get_param",
          "rosapi_msgs/srv/GetParam",
          { name }
        );

        if (response.success === false) {
          throw new Error(response.reason || `parameter ${name} unavailable`);
        }

        return response.value;
      } catch {
        const qualified = this.splitQualifiedParamName(name);
        if (!qualified) {
          throw new Error(`parameter ${name} unavailable`);
        }

        const response = await this.callService<{ names: string[] }, GetParametersResponse>(
          `${qualified.node}/get_parameters`,
          "rcl_interfaces/srv/GetParameters",
          { names: [qualified.param] }
        );

        const value = Array.isArray(response.values) ? response.values[0] : undefined;
        return this.parameterValueToString(value, defaultValue);
      }
    }
  }

  async getRosVersion(): Promise<RosVersionInfo> {
    const response = await this.callService<Record<string, never>, RosVersionResponse>(
      "/rosapi/get_ros_version",
      "rosapi_msgs/srv/GetROSVersion",
      {}
    );

    const version = typeof response.version === "number"
      ? response.version
      : Number.parseInt(String(response.version || ""), 10);

    return {
      version: Number.isFinite(version) ? version : 0,
      distro: typeof response.distro === "string" ? response.distro : ""
    };
  }

  async listActionServers(): Promise<string[]> {
    const response = await this.callService<Record<string, never>, ActionServersResponse>(
      "/rosapi/action_servers",
      "rosapi_msgs/srv/GetActionServers",
      {}
    );

    return uniqueStrings(response.action_servers ?? response.actions ?? response.names);
  }

  async getActionType(actionName: string): Promise<string> {
    try {
      const response = await this.callService<{ action: string }, ActionTypeResponse>(
        "/rosapi/action_type",
        "rosapi_msgs/srv/ActionType",
        { action: actionName }
      );

      return typeof response.type === "string"
        ? response.type
        : typeof response.action_type === "string"
          ? response.action_type
          : "";
    } catch {
      const response = await this.callService<{ name: string }, ActionTypeResponse>(
        "/rosapi/action_type",
        "rosapi_msgs/srv/ActionType",
        { name: actionName }
      );

      return typeof response.type === "string"
        ? response.type
        : typeof response.action_type === "string"
          ? response.action_type
          : "";
    }
  }

  async getTypeDescription(type: string): Promise<string | null> {
    try {
      const response = await this.callService<{
        type_name: string;
      }, GetInterfaceTextResponse>(
        "/webrviz/get_interface_text",
        "webrviz_interfaces/srv/GetInterfaceText",
        {
          type_name: type
        }
      );

      const success = response.success === true;
      if (!success) {
        return null;
      }

      const raw = typeof response.raw_text === "string"
        ? response.raw_text
        : typeof response.rawText === "string"
          ? response.rawText
          : "";
      return raw.trim().length > 0 ? raw : null;
    } catch {
      return null;
    }
  }

  async getRawMessageDefinition(type: string): Promise<string | null> {
    const response = await this.callService<Record<string, never>, TopicsAndRawTypesResponse>(
      "/rosapi/topics_and_raw_types",
      "rosapi_msgs/srv/TopicsAndRawTypes",
      {}
    );

    const types = Array.isArray(response.types) ? response.types : [];
    const rawTypes = Array.isArray(response.typedefs_full_text)
      ? response.typedefs_full_text
      : Array.isArray(response.typedefsFullText)
        ? response.typedefsFullText
        : [];
    const candidates = uniqueStrings([type, type.replace("/msg/", "/")]).map((value) => normalizeTypeName(value));

    for (let index = 0; index < types.length; index += 1) {
      const rawType = rawTypes[index];
      if (typeof rawType !== "string" || rawType.trim().length === 0) {
        continue;
      }

      if (candidates.includes(normalizeTypeName(types[index] || ""))) {
        return rawType;
      }
    }

    return null;
  }

  private async getServiceRequestDetails(type: string): Promise<MessageDetailsResponse> {
    const candidates = uniqueStrings([
      type,
      type.replace("/srv/", "/")
    ]);

    let lastError: unknown = null;
    for (const candidate of candidates) {
      try {
        return await this.callService<{ type: string }, MessageDetailsResponse>(
          "/rosapi/service_request_details",
          "rosapi_msgs/srv/ServiceRequestDetails",
          { type: candidate }
        );
      } catch (error) {
        lastError = error;
      }

      try {
        return await this.callService<{ service: string }, MessageDetailsResponse>(
          "/rosapi/service_request_details",
          "rosapi_msgs/srv/ServiceRequestDetails",
          { service: candidate }
        );
      } catch (error) {
        lastError = error;
      }
    }

    throw lastError instanceof Error ? lastError : new Error(`service request details unavailable for ${type}`);
  }

  private async getServiceResponseDetails(type: string): Promise<MessageDetailsResponse> {
    const candidates = uniqueStrings([
      type,
      type.replace("/srv/", "/")
    ]);

    let lastError: unknown = null;
    for (const candidate of candidates) {
      try {
        return await this.callService<{ type: string }, MessageDetailsResponse>(
          "/rosapi/service_response_details",
          "rosapi_msgs/srv/ServiceResponseDetails",
          { type: candidate }
        );
      } catch (error) {
        lastError = error;
      }

      try {
        return await this.callService<{ service: string }, MessageDetailsResponse>(
          "/rosapi/service_response_details",
          "rosapi_msgs/srv/ServiceResponseDetails",
          { service: candidate }
        );
      } catch (error) {
        lastError = error;
      }
    }

    throw lastError instanceof Error ? lastError : new Error(`service response details unavailable for ${type}`);
  }

  private async listParamsViaNodeServices(): Promise<string[]> {
    const nodes = await this.listNodes();
    const tasks = nodes.map(async (node) => {
      const normalizedNode = this.normalizeNodeName(node);
      if (!normalizedNode) {
        return [] as string[];
      }

      try {
        const response = await this.callService<{ prefixes: string[]; depth: number }, ListParametersResponse>(
          `${normalizedNode}/list_parameters`,
          "rcl_interfaces/srv/ListParameters",
          { prefixes: [], depth: 0 }
        );

        const names = uniqueStrings(response.result?.names);
        return names.map((paramName) => `${normalizedNode}:${paramName}`);
      } catch {
        return [] as string[];
      }
    });

    const resolved = await Promise.all(tasks);
    return uniqueStrings(resolved.flat());
  }

  private normalizeNodeName(node: string): string {
    const trimmed = (node || "").trim().replace(/\/+$/, "");
    if (!trimmed) {
      return "";
    }
    return trimmed.startsWith("/") ? trimmed : `/${trimmed}`;
  }

  private splitQualifiedParamName(name: string): { node: string; param: string } | null {
    const trimmed = (name || "").trim();
    const separatorIndex = trimmed.lastIndexOf(":");
    if (separatorIndex <= 0 || separatorIndex >= trimmed.length - 1) {
      return null;
    }

    const node = this.normalizeNodeName(trimmed.slice(0, separatorIndex));
    const param = trimmed.slice(separatorIndex + 1).trim();
    if (!node || !param) {
      return null;
    }

    return { node, param };
  }

  private parameterValueToString(value: ParameterValueResponse | undefined, defaultValue = ""): string {
    if (!value || typeof value.type !== "number") {
      return defaultValue;
    }

    switch (value.type) {
      case 0:
        return defaultValue;
      case 1:
        return JSON.stringify(Boolean(value.bool_value));
      case 2:
        return JSON.stringify(value.integer_value ?? 0);
      case 3:
        return JSON.stringify(value.double_value ?? 0);
      case 4:
        return typeof value.string_value === "string" ? value.string_value : defaultValue;
      case 5:
        return JSON.stringify(Array.isArray(value.byte_array_value) ? value.byte_array_value : []);
      case 6:
        return JSON.stringify(Array.isArray(value.bool_array_value) ? value.bool_array_value : []);
      case 7:
        return JSON.stringify(Array.isArray(value.integer_array_value) ? value.integer_array_value : []);
      case 8:
        return JSON.stringify(Array.isArray(value.double_array_value) ? value.double_array_value : []);
      case 9:
        return JSON.stringify(Array.isArray(value.string_array_value) ? value.string_array_value : []);
      default:
        return defaultValue;
    }
  }

  private async callService<TRequest extends object, TResponse>(
    name: string,
    serviceType: string,
    request: TRequest
  ): Promise<TResponse> {
    if (!this.ros) {
      throw new Error("ROS is not connected");
    }

    const service = new ROSLIB.Service({
      ros: this.ros,
      name,
      serviceType
    });

    return new Promise<TResponse>((resolve, reject) => {
      const requestMessage = new ROSLIB.ServiceRequest(request);
      service.callService(
        requestMessage,
        (response: TResponse) => resolve(response),
        (error: unknown) => {
          const detail = error instanceof Error ? error.message : String(error);
          reject(new Error(`service ${name} failed: ${detail}`));
        }
      );
    });
  }

  private setState(state: ConnectionState, message?: string): void {
    this.state = state;
    for (const listener of this.listeners) {
      listener(state, message);
    }
  }
}
