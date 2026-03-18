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

type StateListener = (state: ConnectionState, message?: string) => void;

interface TopicsResponse {
  topics: string[];
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
}

interface NodesResponse {
  nodes: string[];
}

interface NodeDetailsResponse {
  publishing?: string[];
  subscribing?: string[];
  services?: string[];
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
      "rosapi/Topics",
      {}
    );

    const result: TopicInfo[] = [];
    for (const name of topicsResponse.topics) {
      try {
        const typeResponse = await this.callService<{ topic: string }, TopicTypeResponse>(
          "/rosapi/topic_type",
          "rosapi/TopicType",
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
      "rosapi/Services",
      {}
    );

    const result: ServiceInfo[] = [];
    for (const name of servicesResponse.services) {
      try {
        const typeResponse = await this.callService<{ service: string }, ServiceTypeResponse>(
          "/rosapi/service_type",
          "rosapi/ServiceType",
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
    const response = await this.callService<Record<string, never>, GetParamNamesResponse>(
      "/rosapi/get_param_names",
      "rosapi/GetParamNames",
      {}
    );

    return response.names || [];
  }

  async listNodes(): Promise<string[]> {
    const response = await this.callService<Record<string, never>, NodesResponse>(
      "/rosapi/nodes",
      "rosapi/Nodes",
      {}
    );

    return response.nodes || [];
  }

  async getNodeDetails(node: string): Promise<NodeDetails> {
    const response = await this.callService<{ node: string }, NodeDetailsResponse>(
      "/rosapi/node_details",
      "rosapi/NodeDetails",
      { node }
    );

    return {
      publishing: Array.isArray(response.publishing) ? response.publishing : [],
      subscribing: Array.isArray(response.subscribing) ? response.subscribing : [],
      services: Array.isArray(response.services) ? response.services : []
    };
  }

  async getMessageDetails(type: string): Promise<MessageDetailsResponse> {
    return this.callService<{ type: string }, MessageDetailsResponse>(
      "/rosapi/message_details",
      "rosapi/MessageDetails",
      { type }
    );
  }

  async getServiceDetails(type: string): Promise<ServiceDetailsResponse> {
    const [request, response] = await Promise.all([
      this.getServiceRequestDetails(type),
      this.getServiceResponseDetails(type)
    ]);

    return { request, response };
  }

  async getParam(name: string): Promise<string> {
    const response = await this.callService<{ name: string }, GetParamResponse>(
      "/rosapi/get_param",
      "rosapi/GetParam",
      { name }
    );

    return response.value;
  }

  private async getServiceRequestDetails(type: string): Promise<MessageDetailsResponse> {
    try {
      return await this.callService<{ type: string }, MessageDetailsResponse>(
        "/rosapi/service_request_details",
        "rosapi/ServiceRequestDetails",
        { type }
      );
    } catch {
      // compatibility fallback for rosapi variants that expect `service`.
      return this.callService<{ service: string }, MessageDetailsResponse>(
        "/rosapi/service_request_details",
        "rosapi/ServiceRequestDetails",
        { service: type }
      );
    }
  }

  private async getServiceResponseDetails(type: string): Promise<MessageDetailsResponse> {
    try {
      return await this.callService<{ type: string }, MessageDetailsResponse>(
        "/rosapi/service_response_details",
        "rosapi/ServiceResponseDetails",
        { type }
      );
    } catch {
      // compatibility fallback for rosapi variants that expect `service`.
      return this.callService<{ service: string }, MessageDetailsResponse>(
        "/rosapi/service_response_details",
        "rosapi/ServiceResponseDetails",
        { service: type }
      );
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