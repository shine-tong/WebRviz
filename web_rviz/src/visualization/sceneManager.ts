import {
  AmbientLight,
  AxesHelper,
  BufferAttribute,
  BufferGeometry,
  Color,
  DirectionalLight,
  GridHelper,
  Group,
  Matrix4,
  PerspectiveCamera,
  Points,
  PointsMaterial,
  Quaternion,
  Scene,
  Vector3,
  WebGLRenderer
} from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";

export type SceneTheme = "dark" | "light";

interface TfRecord {
  parent: string;
  translation: Vector3;
  rotation: Quaternion;
}

interface ParsedPointCloud {
  frameId: string;
  positions: Float32Array;
  colors: Float32Array;
  count: number;
}

export interface TfNode {
  frame: string;
  parent: string;
  translation: Vector3;
  rotation: Quaternion;
}

export class SceneManager {
  private readonly container: HTMLElement;
  private readonly scene: Scene;
  private readonly camera: PerspectiveCamera;
  private readonly renderer: WebGLRenderer;
  private readonly controls: OrbitControls;
  private readonly robotLayer: Group;
  private readonly tfLayer: Group;
  private readonly worldGrid: GridHelper;
  private readonly tfRecords = new Map<string, TfRecord>();
  private readonly tfAxes = new Map<string, AxesHelper>();
  private endEffectorAxis: AxesHelper | null = null;
  private endEffectorFrame = "";
  private showOnlyEndEffector = false;
  private pointCloud: Points | null = null;
  private pointCloudFrameId = "";
  private pointCloudMaterial: PointsMaterial;
  private robot: any | null = null;
  private robotBaseFrame = "base_link";
  private fixedFrame = "base_link";
  private targetFps: number;
  private lastRenderTimestamp = 0;
  private currentTheme: SceneTheme = "dark";
  private visibleTfFrames: Set<string> | null = null;

  constructor(container: HTMLElement, targetFps: number) {
    this.container = container;
    this.targetFps = Math.max(1, targetFps);

    this.scene = new Scene();
    this.scene.background = new Color(0x0b1016);

    this.camera = new PerspectiveCamera(55, 1, 0.01, 1500);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(2.2, 2.2, 1.4);

    this.renderer = new WebGLRenderer({ antialias: false, powerPreference: "low-power" });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
    this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = false;
    this.controls.target.set(0, 0, 0.5);

    this.robotLayer = new Group();
    this.tfLayer = new Group();

    this.worldGrid = new GridHelper(10, 20, 0x4e6f94, 0x3a5470);
    // RViz uses Z-up with Grid plane XY, so rotate Three.js grid from XZ to XY.
    this.worldGrid.rotation.x = Math.PI / 2;
    this.worldGrid.position.set(0, 0, 0);

    const ambientLight = new AmbientLight(0xffffff, 0.7);
    const directionalLight = new DirectionalLight(0xffffff, 0.7);
    directionalLight.position.set(3, 2, 5);

    this.pointCloudMaterial = new PointsMaterial({
      size: 0.02,
      vertexColors: true,
      transparent: true,
      opacity: 0.96,
      sizeAttenuation: true
    });

    this.scene.add(this.worldGrid);
    this.scene.add(ambientLight);
    this.scene.add(directionalLight);
    this.scene.add(this.robotLayer);
    this.scene.add(this.tfLayer);

    this.container.appendChild(this.renderer.domElement);
    this.setTheme("dark");

    this.onResize = this.onResize.bind(this);
    window.addEventListener("resize", this.onResize);

    this.loop = this.loop.bind(this);
    window.requestAnimationFrame(this.loop);
  }

  setTargetFps(value: number): void {
    this.targetFps = Math.max(1, value);
  }

  setTheme(theme: SceneTheme): void {
    this.currentTheme = theme;

    if (theme === "light") {
      this.scene.background = new Color(0xeef3f8);
      this.updateGridColors(0x9fb7cf, 0xc8d8e6);
      return;
    }

    this.scene.background = new Color(0x0b1016);
    this.updateGridColors(0x4e6f94, 0x3a5470);
  }

  setFixedFrame(frame: string): void {
    this.fixedFrame = frame || "base_link";
    this.refreshFrameTransforms();
  }

  getFixedFrame(): string {
    return this.fixedFrame;
  }

  getRobotBaseFrame(): string {
    return this.robotBaseFrame;
  }

  setEndEffectorFrame(frame: string): void {
    this.endEffectorFrame = frame || "";
    this.attachEndEffectorAxis();
    this.refreshFrameTransforms();
  }

  setShowOnlyEndEffector(show: boolean): void {
    this.showOnlyEndEffector = show;
    if (!show && this.endEffectorAxis) {
      this.endEffectorAxis.visible = false;
    }
    this.attachEndEffectorAxis();
    this.refreshFrameTransforms();
  }


  setVisibleTfFrames(frames: Iterable<string> | null): void {
    this.visibleTfFrames = frames ? new Set(frames) : null;
    this.refreshFrameTransforms();
  }
  getDefaultEndEffectorFrame(): string {
    if (!this.robot) {
      return this.robotBaseFrame;
    }

    const linkMap = this.robot.links;
    if (linkMap) {
      const linkKeys = Object.keys(linkMap);
      if (linkKeys.length > 0) {
        return linkKeys[linkKeys.length - 1];
      }
    }

    const frameMap = this.robot.frames;
    if (frameMap) {
      const frameKeys = Object.keys(frameMap);
      if (frameKeys.length > 0) {
        return frameKeys[frameKeys.length - 1];
      }
    }

    return this.robotBaseFrame;
  }

  getLinkList(): string[] {
    if (this.robot && this.robot.links) {
      return Object.keys(this.robot.links).sort((left, right) => left.localeCompare(right));
    }
    return [];
  }

  getFrameList(): string[] {
    const frames = new Set<string>();
    if (this.fixedFrame) {
      frames.add(this.fixedFrame);
    }
    if (this.robotBaseFrame) {
      frames.add(this.robotBaseFrame);
    }
    for (const [child, record] of this.tfRecords.entries()) {
      frames.add(child);
      if (record.parent) {
        frames.add(record.parent);
      }
    }
    if (this.robot) {
      const frameMap = this.robot.frames || this.robot.links;
      if (frameMap) {
        for (const frame of Object.keys(frameMap)) {
          frames.add(frame);
        }
      }
    }
    return Array.from(frames).filter(Boolean).sort((left, right) => left.localeCompare(right));
  }

  getTfSnapshot(): Array<{ parent: string; child: string }> {
    const snapshot: Array<{ parent: string; child: string }> = [];
    for (const [child, record] of this.tfRecords.entries()) {
      snapshot.push({ parent: record.parent, child });
    }
    return snapshot;
  }

  clearTfRecords(): void {
    this.tfRecords.clear();
    for (const axis of this.tfAxes.values()) {
      axis.visible = false;
    }
    this.refreshFrameTransforms();
  }

  private ensureEndEffectorAxis(): AxesHelper {
    if (!this.endEffectorAxis) {
      this.endEffectorAxis = new AxesHelper(0.12);
      this.endEffectorAxis.visible = false;
      this.tfLayer.add(this.endEffectorAxis);
    }
    return this.endEffectorAxis;
  }

  private attachEndEffectorAxis(): void {
    if (!this.endEffectorFrame) {
      return;
    }

    const axis = this.ensureEndEffectorAxis();
    const frameObject = this.getRobotFrameObject(this.endEffectorFrame);
    if (frameObject) {
      if (axis.parent !== frameObject) {
        if (axis.parent) {
          axis.parent.remove(axis);
        }
        frameObject.add(axis);
      }
      axis.visible = this.showOnlyEndEffector;
      return;
    }

    if (axis.parent !== this.tfLayer) {
      if (axis.parent) {
        axis.parent.remove(axis);
      }
      this.tfLayer.add(axis);
    }
    axis.visible = this.showOnlyEndEffector;
  }
  private getRobotFrameObject(frame: string): any | null {
    if (!this.robot || !frame) {
      return null;
    }

    if (typeof this.robot.getFrame === "function") {
      try {
        const found = this.robot.getFrame(frame);
        if (found) {
          return found;
        }
      } catch {
        // ignore lookup errors
      }
    }

    const frameMap = this.robot.frames || this.robot.links;
    if (frameMap && frameMap[frame]) {
      return frameMap[frame];
    }

    return null;
  }
  getRelativeTransform(frame: string): { translation: Vector3; rotation: Quaternion } | null {
    const cache = new Map<string, Matrix4 | null>();
    const relative = this.computeRelativeMatrix(frame, cache);
    if (relative) {
      const position = new Vector3();
      const rotation = new Quaternion();
      const scale = new Vector3();
      relative.decompose(position, rotation, scale);
      return { translation: position, rotation };
    }

    const frameObject = this.getRobotFrameObject(frame);
    if (!frameObject || !this.robot) {
      return null;
    }

    this.robot.updateMatrixWorld(true);

    const position = new Vector3();
    const rotation = new Quaternion();
    frameObject.getWorldPosition(position);
    frameObject.getWorldQuaternion(rotation);
    return { translation: position, rotation };
  }

  setRobot(robot: any, baseFrame = "base_link"): void {
    if (this.robot) {
      this.robotLayer.remove(this.robot);
    }

    this.robot = robot;
    this.robotBaseFrame = baseFrame || "base_link";

    if (this.robot) {      this.robotLayer.add(this.robot);    }    this.attachEndEffectorAxis();    this.refreshFrameTransforms();
  }

  updateJointStates(message: unknown): void {
    const payload = message as { name?: string[]; position?: number[] };
    if (!this.robot || !payload || !Array.isArray(payload.name) || !Array.isArray(payload.position)) {
      return;
    }

    if (!this.robot.joints) {
      return;
    }

    const count = Math.min(payload.name.length, payload.position.length);
    for (let index = 0; index < count; index += 1) {
      const jointName = payload.name[index];
      const jointValue = payload.position[index];
      const joint = this.robot.joints[jointName];
      if (joint && typeof joint.setJointValue === "function") {
        joint.setJointValue(jointValue);
      }
    }
  }

  upsertTfMessage(message: unknown): void {
    const payload = message as {
      transforms?: Array<{
        child_frame_id?: string;
        header?: { frame_id?: string };
        transform?: {
          translation?: { x?: number; y?: number; z?: number };
          rotation?: { x?: number; y?: number; z?: number; w?: number };
        };
      }>;
    };

    if (!payload || !Array.isArray(payload.transforms)) {
      return;
    }

    for (const tf of payload.transforms) {
      const child = (tf.child_frame_id || "").replace(/^\//, "");
      const parent = (tf.header?.frame_id || "").replace(/^\//, "");
      if (!child || !parent) {
        continue;
      }

      const translation = new Vector3(
        tf.transform?.translation?.x ?? 0,
        tf.transform?.translation?.y ?? 0,
        tf.transform?.translation?.z ?? 0
      );

      const rotation = new Quaternion(
        tf.transform?.rotation?.x ?? 0,
        tf.transform?.rotation?.y ?? 0,
        tf.transform?.rotation?.z ?? 0,
        tf.transform?.rotation?.w ?? 1
      );

      this.tfRecords.set(child, { parent, translation, rotation });

      if (!this.tfAxes.has(child)) {
        const axis = new AxesHelper(0.09);
        axis.visible = false;
        this.tfLayer.add(axis);
        this.tfAxes.set(child, axis);
      }
    }

    this.refreshFrameTransforms();
  }

  getTfNodes(): TfNode[] {
    const nodes: TfNode[] = [];
    for (const [frame, record] of this.tfRecords.entries()) {
      nodes.push({
        frame,
        parent: record.parent,
        translation: record.translation.clone(),
        rotation: record.rotation.clone()
      });
    }
    nodes.sort((a, b) => a.frame.localeCompare(b.frame));
    return nodes;
  }

  setPointCloud(pointCloud: ParsedPointCloud | null): void {
    if (!pointCloud) {
      if (this.pointCloud) {
        this.tfLayer.remove(this.pointCloud);
        this.pointCloud.geometry.dispose();
      }
      this.pointCloud = null;
      this.pointCloudFrameId = "";
      return;
    }

    const geometry = new BufferGeometry();
    geometry.setAttribute("position", new BufferAttribute(pointCloud.positions, 3));
    geometry.setAttribute("color", new BufferAttribute(pointCloud.colors, 3));

    if (this.pointCloud) {
      this.tfLayer.remove(this.pointCloud);
      this.pointCloud.geometry.dispose();
    }

    this.pointCloud = new Points(geometry, this.pointCloudMaterial);
    this.pointCloudFrameId = pointCloud.frameId || this.fixedFrame;
    this.tfLayer.add(this.pointCloud);

    this.refreshFrameTransforms();
  }

  dispose(): void {
    window.removeEventListener("resize", this.onResize);
    this.controls.dispose();

    if (this.pointCloud) {
      this.pointCloud.geometry.dispose();
      this.pointCloud = null;
    }

    this.pointCloudMaterial.dispose();
    this.renderer.dispose();
    this.container.removeChild(this.renderer.domElement);
  }

  private refreshFrameTransforms(): void {
    const cache = new Map<string, Matrix4 | null>();

    for (const [frame, axis] of this.tfAxes.entries()) {
      if (this.showOnlyEndEffector) {
        axis.visible = false;
        continue;
      }


      if (this.visibleTfFrames !== null && !this.visibleTfFrames.has(frame)) {
        axis.visible = false;
        continue;
      }
      const relative = this.computeRelativeMatrix(frame, cache);
      axis.visible = relative !== null;
      if (relative) {
        this.applyMatrixToObject(axis, relative);
      }
    }

    if (this.endEffectorAxis && this.showOnlyEndEffector && this.endEffectorAxis.parent === this.tfLayer) {
      const relative = this.computeRelativeMatrix(this.endEffectorFrame, cache);
      this.endEffectorAxis.visible = relative !== null;
      if (relative) {
        this.applyMatrixToObject(this.endEffectorAxis, relative);
      }
    }

    if (this.pointCloud) {
      const frame = this.pointCloudFrameId || this.fixedFrame;
      const relative = this.computeRelativeMatrix(frame, cache);
      this.pointCloud.visible = relative !== null;
      if (relative) {
        this.applyMatrixToObject(this.pointCloud, relative);
      }
    }

    if (this.robot) {
      const relative = this.computeRelativeMatrix(this.robotBaseFrame || "base_link", cache);
      this.robot.visible = relative !== null;
      if (relative) {
        this.applyMatrixToObject(this.robot, relative);
      }
    }
  }
  private computeRelativeMatrix(frame: string, cache: Map<string, Matrix4 | null>): Matrix4 | null {
    const fixedMatrix = this.computeAbsoluteMatrix(this.fixedFrame, cache, new Set());
    const frameMatrix = this.computeAbsoluteMatrix(frame, cache, new Set());
    if (!fixedMatrix || !frameMatrix) {
      return null;
    }
    return fixedMatrix.clone().invert().multiply(frameMatrix);
  }

  private computeAbsoluteMatrix(frame: string, cache: Map<string, Matrix4 | null>, stack: Set<string>): Matrix4 | null {
    if (!frame) {
      return null;
    }

    if (cache.has(frame)) {
      const cached = cache.get(frame);
      return cached ? cached.clone() : null;
    }

    if (stack.has(frame)) {
      cache.set(frame, null);
      return null;
    }

    if (frame === this.fixedFrame) {
      const identity = new Matrix4();
      cache.set(frame, identity);
      return identity.clone();
    }

    const record = this.tfRecords.get(frame);
    if (!record) {
      cache.set(frame, null);
      return null;
    }

    stack.add(frame);
    const parentAbsolute = this.computeAbsoluteMatrix(record.parent, cache, stack);
    stack.delete(frame);

    if (!parentAbsolute) {
      cache.set(frame, null);
      return null;
    }

    const local = new Matrix4().compose(record.translation, record.rotation, new Vector3(1, 1, 1));
    const absolute = parentAbsolute.clone().multiply(local);

    cache.set(frame, absolute);
    return absolute.clone();
  }

  private applyMatrixToObject(object3D: { position: Vector3; quaternion: Quaternion }, matrix: Matrix4): void {
    const position = new Vector3();
    const rotation = new Quaternion();
    const scale = new Vector3();
    matrix.decompose(position, rotation, scale);

    object3D.position.copy(position);
    object3D.quaternion.copy(rotation);
  }

  private updateGridColors(centerLine: number, gridLine: number): void {
    const materials = Array.isArray(this.worldGrid.material) ? this.worldGrid.material : [this.worldGrid.material];
    if (materials[0]) {
      materials[0].color.setHex(centerLine);
    }
    if (materials[1]) {
      materials[1].color.setHex(gridLine);
    }
  }

  private onResize(): void {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }

  private loop(timestamp: number): void {
    const minInterval = 1000 / this.targetFps;
    if (timestamp - this.lastRenderTimestamp >= minInterval) {
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
      this.lastRenderTimestamp = timestamp;
    }

    window.requestAnimationFrame(this.loop);
  }
}












