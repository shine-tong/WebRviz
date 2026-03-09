import ROSLIB from "roslib";

export type ConnectionState = "disconnected" | "connecting" | "connected" | "error";

export interface TopicInfo {
  name: string;
  type: string;
}

type StateListener = (state: ConnectionState, message?: string) => void;

interface TopicsResponse {
  topics: string[];
}

interface TopicTypeResponse {
  type: string;
}

interface GetParamResponse {
  value: string;
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

  async getParam(name: string): Promise<string> {
    const response = await this.callService<{ name: string }, GetParamResponse>(
      "/rosapi/get_param",
      "rosapi/GetParam",
      { name }
    );

    return response.value;
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
