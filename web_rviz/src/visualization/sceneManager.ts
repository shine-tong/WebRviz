import {
  AmbientLight,
  AxesHelper,
  BufferAttribute,
  BufferGeometry,
  CanvasTexture,
  Color,
  CylinderGeometry,
  DirectionalLight,
  Euler,
  EdgesGeometry,
  GridHelper,
  Group,
  Line,
  LineBasicMaterial,
  LineSegments,
  Material,
  MeshBasicMaterial,
  Matrix4,
  Mesh,
  MeshStandardMaterial,
  PerspectiveCamera,
  Points,
  PointsMaterial,
  Quaternion,
  Scene,
  SphereGeometry,
  Sprite,
  SpriteMaterial,
  Vector3,
  WebGLRenderer
} from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { Line2 } from "three/examples/jsm/lines/Line2.js";
import { LineGeometry } from "three/examples/jsm/lines/LineGeometry.js";
import { LineMaterial } from "three/examples/jsm/lines/LineMaterial.js";

export type SceneTheme = "dark" | "light";

type RobotPartRole = "body" | "joint" | "tool" | "cable" | "accent";

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

export interface RobotJointConnection {
  name: string;
  parent: string;
  child: string;
  type: string;
}

export interface RobotTransformDetail {
  xyz: [number, number, number];
  rpy: [number, number, number];
}

export interface RobotInertiaDetail {
  ixx: number | null;
  ixy: number | null;
  ixz: number | null;
  iyy: number | null;
  iyz: number | null;
  izz: number | null;
}

export interface RobotLinkDetail {
  name: string;
  parentJoint: string | null;
  childJoints: string[];
  visualCount: number;
  collisionCount: number;
  materialNames: string[];
  mass: number | null;
  inertialOrigin: RobotTransformDetail | null;
  inertia: RobotInertiaDetail | null;
  pose: RobotTransformDetail | null;
}

export interface RobotJointLimitDetail {
  lower: number | null;
  upper: number | null;
  effort: number | null;
  velocity: number | null;
}

export interface RobotJointDynamicsDetail {
  damping: number | null;
  friction: number | null;
}

export interface RobotJointMimicDetail {
  joint: string;
  multiplier: number | null;
  offset: number | null;
}

export interface RobotJointDetail {
  name: string;
  type: string;
  parentLink: string | null;
  childLink: string | null;
  axis: [number, number, number] | null;
  origin: RobotTransformDetail | null;
  limit: RobotJointLimitDetail | null;
  dynamics: RobotJointDynamicsDetail | null;
  mimic: RobotJointMimicDetail | null;
  currentValue: number[];
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
  private readonly ambientLight: AmbientLight;
  private readonly keyLight: DirectionalLight;
  private readonly fillLight: DirectionalLight;
  private plannedTrajectoryLine: Line2 | null = null;
  private plannedTrajectoryStartMarker: Group | null = null;
  private plannedTrajectoryEndMarker: Group | null = null;
  private plannedTrajectoryVisible = true;
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
    this.controls.update();
    this.controls.saveState();

    this.robotLayer = new Group();
    this.tfLayer = new Group();

    this.worldGrid = new GridHelper(10, 20, 0x4e6f94, 0x3a5470);
    // RViz uses Z-up with Grid plane XY, so rotate Three.js grid from XZ to XY.
    this.worldGrid.rotation.x = Math.PI / 2;
    this.worldGrid.position.set(0, 0, 0);

    this.ambientLight = new AmbientLight(0xf2f6ff, 0.68);
    this.keyLight = new DirectionalLight(0xfff1d4, 1.0);
    this.keyLight.position.set(4.2, -2.6, 6.4);
    this.fillLight = new DirectionalLight(0xd7e7ff, 0.34);
    this.fillLight.position.set(-4.8, 3.4, 3.2);

    this.pointCloudMaterial = new PointsMaterial({
      size: 0.02,
      vertexColors: true,
      transparent: true,
      opacity: 0.96,
      sizeAttenuation: true
    });

    this.scene.add(this.worldGrid);
    this.scene.add(this.ambientLight);
    this.scene.add(this.keyLight);
    this.scene.add(this.fillLight);
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

  resetView(): void {
    this.camera.up.set(0, 0, 1);
    this.controls.reset();
    this.controls.update();
  }

  setTheme(theme: SceneTheme): void {
    this.currentTheme = theme;

    if (theme === "light") {
      this.scene.background = new Color(0xeef3f8);
      this.updateGridColors(0x9fb7cf, 0xc8d8e6);
    } else {
      this.scene.background = new Color(0x0b1016);
      this.updateGridColors(0x4e6f94, 0x3a5470);
    }

    this.updateLighting();
    this.refreshRobotAppearance();
    this.updatePlannedTrajectoryAppearance();
  }

  setPlannedTrajectoryVisible(visible: boolean): void {
    this.plannedTrajectoryVisible = visible;
    this.updatePlannedTrajectoryVisibility();
  }

  setPlannedTrajectoryPath(
    points: ReadonlyArray<Vector3>,
    startPose?: { translation: Vector3; rotation: { x: number; y: number; z: number; w: number } } | null,
    endPose?: { translation: Vector3; rotation: { x: number; y: number; z: number; w: number } } | null
  ): void {
    this.clearPlannedTrajectory();

    if (points.length < 2) {
      return;
    }

    const positions = new Float32Array(points.length * 3);
    for (let index = 0; index < points.length; index += 1) {
      const point = points[index];
      const offset = index * 3;
      positions[offset] = point.x;
      positions[offset + 1] = point.y;
      positions[offset + 2] = point.z;
    }

    const geometry = new LineGeometry();
    geometry.setPositions(positions);

    const material = new LineMaterial({
      color: this.getPlannedTrajectoryColor(),
      linewidth: this.getPlannedTrajectoryLineWidth(),
      transparent: true,
      opacity: this.getPlannedTrajectoryOpacity(),
      depthWrite: false,
      depthTest: false,
      toneMapped: false
    });
    material.resolution.set(
      Math.max(1, this.container.clientWidth),
      Math.max(1, this.container.clientHeight)
    );

    this.plannedTrajectoryLine = new Line2(geometry, material);
    this.plannedTrajectoryLine.renderOrder = 3;
    this.plannedTrajectoryLine.frustumCulled = false;
    this.scene.add(this.plannedTrajectoryLine);

    if (startPose) {
      this.plannedTrajectoryStartMarker = this.createPlannedTrajectoryPoseMarker("start", startPose);
      this.scene.add(this.plannedTrajectoryStartMarker);
    }

    if (endPose) {
      this.plannedTrajectoryEndMarker = this.createPlannedTrajectoryPoseMarker("end", endPose);
      this.scene.add(this.plannedTrajectoryEndMarker);
    }

    this.updatePlannedTrajectoryAppearance();
    this.updatePlannedTrajectoryVisibility();
  }

  clearPlannedTrajectory(): void {
    if (this.plannedTrajectoryStartMarker) {
      this.disposePlannedTrajectoryPoseMarker(this.plannedTrajectoryStartMarker);
      this.plannedTrajectoryStartMarker = null;
    }

    if (this.plannedTrajectoryEndMarker) {
      this.disposePlannedTrajectoryPoseMarker(this.plannedTrajectoryEndMarker);
      this.plannedTrajectoryEndMarker = null;
    }

    if (!this.plannedTrajectoryLine) {
      return;
    }

    this.scene.remove(this.plannedTrajectoryLine);
    this.plannedTrajectoryLine.geometry.dispose();

    const materials = Array.isArray(this.plannedTrajectoryLine.material)
      ? this.plannedTrajectoryLine.material
      : [this.plannedTrajectoryLine.material];
    for (const material of materials) {
      if (material instanceof Material) {
        material.dispose();
      }
    }

    this.plannedTrajectoryLine = null;
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

    const urdfLeafLink = this.getDefaultEndEffectorLinkFromUrdf();
    if (urdfLeafLink) {
      return urdfLeafLink;
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

  getRobotJointConnections(): RobotJointConnection[] {
    if (!this.robot || !this.robot.joints) {
      return [];
    }

    const connections: RobotJointConnection[] = [];
    for (const [jointKey, joint] of Object.entries<any>(this.robot.joints)) {
      const parentName = typeof joint?.parent?.urdfName === "string" ? joint.parent.urdfName : "";
      const childName = this.getJointChildLinkName(joint) || "";

      if (!parentName || !childName) {
        continue;
      }

      connections.push({
        name: typeof joint?.urdfName === "string" && joint.urdfName ? joint.urdfName : jointKey,
        parent: parentName,
        child: childName,
        type: typeof joint?.jointType === "string" ? joint.jointType : ""
      });
    }

    connections.sort((left, right) => {
      const parentCompare = left.parent.localeCompare(right.parent);
      if (parentCompare !== 0) {
        return parentCompare;
      }
      const childCompare = left.child.localeCompare(right.child);
      if (childCompare !== 0) {
        return childCompare;
      }
      return left.name.localeCompare(right.name);
    });

    return connections;
  }

  getRobotLinkDetail(linkName: string): RobotLinkDetail | null {
    if (!this.robot || !this.robot.links || !linkName) {
      return null;
    }

    const link = this.robot.links[linkName];
    if (!link) {
      return null;
    }

    let parentJoint: string | null = null;
    const childJoints: string[] = [];
    for (const [jointKey, joint] of Object.entries<any>(this.robot.joints || {})) {
      const jointName = typeof joint?.urdfName === "string" && joint.urdfName ? joint.urdfName : jointKey;
      const parentLinkName = typeof joint?.parent?.urdfName === "string" ? joint.parent.urdfName : "";
      const childLinkName = this.getJointChildLinkName(joint);
      if (childLinkName === linkName && !parentJoint) {
        parentJoint = jointName;
      }
      if (parentLinkName === linkName) {
        childJoints.push(jointName);
      }
    }
    childJoints.sort((left, right) => left.localeCompare(right));

    const linkNode = link.urdfNode as Element | null;
    const childNodes = linkNode ? Array.from(linkNode.children) : [];
    const visualNodes = childNodes.filter((node) => node.nodeName.toLowerCase() === "visual");
    const collisionNodes = childNodes.filter((node) => node.nodeName.toLowerCase() === "collision");
    const materialNames = Array.from(new Set(
      visualNodes
        .map((node) => Array.from(node.children).find((child) => child.nodeName.toLowerCase() === "material")?.getAttribute("name") || "")
        .filter((name) => !!name)
    )).sort((left, right) => left.localeCompare(right));

    let mass: number | null = null;
    let inertialOrigin: RobotTransformDetail | null = null;
    let inertia: RobotInertiaDetail | null = null;
    const inertialNode = childNodes.find((node) => node.nodeName.toLowerCase() === "inertial") || null;
    if (inertialNode) {
      const inertialChildren = Array.from(inertialNode.children);
      const massNode = inertialChildren.find((node) => node.nodeName.toLowerCase() === "mass") || null;
      const originNode = inertialChildren.find((node) => node.nodeName.toLowerCase() === "origin") || null;
      const inertiaNode = inertialChildren.find((node) => node.nodeName.toLowerCase() === "inertia") || null;
      mass = this.parseNumberAttribute(massNode, "value");
      inertialOrigin = this.parseOriginElement(originNode) || { xyz: [0, 0, 0], rpy: [0, 0, 0] };
      if (inertiaNode) {
        inertia = {
          ixx: this.parseNumberAttribute(inertiaNode, "ixx"),
          ixy: this.parseNumberAttribute(inertiaNode, "ixy"),
          ixz: this.parseNumberAttribute(inertiaNode, "ixz"),
          iyy: this.parseNumberAttribute(inertiaNode, "iyy"),
          iyz: this.parseNumberAttribute(inertiaNode, "iyz"),
          izz: this.parseNumberAttribute(inertiaNode, "izz")
        };
      }
    }

    const transform = this.getRelativeTransform(linkName);
    return {
      name: linkName,
      parentJoint,
      childJoints,
      visualCount: visualNodes.length,
      collisionCount: collisionNodes.length,
      materialNames,
      mass,
      inertialOrigin,
      inertia,
      pose: transform ? this.toTransformDetailFromPose(transform.translation, transform.rotation) : null
    };
  }

  getRobotJointDetail(jointName: string): RobotJointDetail | null {
    if (!this.robot || !this.robot.joints || !jointName) {
      return null;
    }

    const joint = this.robot.joints[jointName];
    if (!joint) {
      return null;
    }

    const jointNode = joint.urdfNode as Element | null;
    const childNodes = jointNode ? Array.from(jointNode.children) : [];
    const originNode = childNodes.find((node) => node.nodeName.toLowerCase() === "origin") || null;
    const limitNode = childNodes.find((node) => node.nodeName.toLowerCase() === "limit") || null;
    const dynamicsNode = childNodes.find((node) => node.nodeName.toLowerCase() === "dynamics") || null;
    const mimicNode = childNodes.find((node) => node.nodeName.toLowerCase() === "mimic") || null;
    const axis = joint.axis
      ? [joint.axis.x, joint.axis.y, joint.axis.z] as [number, number, number]
      : null;

    return {
      name: typeof joint?.urdfName === "string" && joint.urdfName ? joint.urdfName : jointName,
      type: typeof joint?.jointType === "string" ? joint.jointType : "",
      parentLink: typeof joint?.parent?.urdfName === "string" ? joint.parent.urdfName : null,
      childLink: this.getJointChildLinkName(joint),
      axis,
      origin: this.parseOriginElement(originNode) || { xyz: [0, 0, 0], rpy: [0, 0, 0] },
      limit: limitNode ? {
        lower: this.parseNumberAttribute(limitNode, "lower"),
        upper: this.parseNumberAttribute(limitNode, "upper"),
        effort: this.parseNumberAttribute(limitNode, "effort"),
        velocity: this.parseNumberAttribute(limitNode, "velocity")
      } : null,
      dynamics: dynamicsNode ? {
        damping: this.parseNumberAttribute(dynamicsNode, "damping"),
        friction: this.parseNumberAttribute(dynamicsNode, "friction")
      } : null,
      mimic: mimicNode ? {
        joint: mimicNode.getAttribute("joint") || "",
        multiplier: this.parseNumberAttribute(mimicNode, "multiplier"),
        offset: this.parseNumberAttribute(mimicNode, "offset")
      } : null,
      currentValue: Array.isArray(joint.jointValue)
        ? joint.jointValue.filter((value: unknown): value is number => typeof value === "number" && Number.isFinite(value))
        : []
    };
  }

  private getDefaultEndEffectorLinkFromUrdf(): string {
    if (!this.robot?.links) {
      return "";
    }

    const linkNames = Object.keys(this.robot.links).filter((name) => typeof name === "string" && name.length > 0);
    if (linkNames.length === 0) {
      return "";
    }

    const linkSet = new Set(linkNames);
    const linkOrder = new Map<string, number>();
    for (let index = 0; index < linkNames.length; index += 1) {
      linkOrder.set(linkNames[index], index);
    }

    const childrenByParent = new Map<string, string[]>();
    const hasParent = new Set<string>();
    for (const joint of Object.values<any>(this.robot.joints || {})) {
      const parentName = typeof joint?.parent?.urdfName === "string" ? joint.parent.urdfName : "";
      const childName = this.getJointChildLinkName(joint) || "";
      if (!parentName || !childName || !linkSet.has(childName)) {
        continue;
      }
      if (!childrenByParent.has(parentName)) {
        childrenByParent.set(parentName, []);
      }
      childrenByParent.get(parentName)?.push(childName);
      hasParent.add(childName);
    }

    const leafLinks = linkNames.filter((linkName) => (childrenByParent.get(linkName)?.length ?? 0) === 0);
    if (leafLinks.length === 0) {
      return linkNames[linkNames.length - 1] || "";
    }

    const rootCandidate =
      (this.robotBaseFrame && linkSet.has(this.robotBaseFrame) ? this.robotBaseFrame : "") ||
      linkNames.find((linkName) => !hasParent.has(linkName)) ||
      linkNames[0] ||
      "";

    const depthByLink = new Map<string, number>();
    const queue: string[] = [];
    if (rootCandidate) {
      depthByLink.set(rootCandidate, 0);
      queue.push(rootCandidate);
    }

    while (queue.length > 0) {
      const linkName = queue.shift();
      if (!linkName) {
        continue;
      }
      const depth = depthByLink.get(linkName) ?? 0;
      for (const childName of childrenByParent.get(linkName) || []) {
        const nextDepth = depth + 1;
        const knownDepth = depthByLink.get(childName);
        if (knownDepth === undefined || nextDepth > knownDepth) {
          depthByLink.set(childName, nextDepth);
          queue.push(childName);
        }
      }
    }

    let bestLeaf = leafLinks[0] || "";
    let bestDepth = depthByLink.get(bestLeaf) ?? -1;
    let bestOrder = linkOrder.get(bestLeaf) ?? -1;

    for (let index = 1; index < leafLinks.length; index += 1) {
      const candidate = leafLinks[index];
      const candidateDepth = depthByLink.get(candidate) ?? -1;
      const candidateOrder = linkOrder.get(candidate) ?? -1;
      if (candidateDepth > bestDepth || (candidateDepth === bestDepth && candidateOrder > bestOrder)) {
        bestLeaf = candidate;
        bestDepth = candidateDepth;
        bestOrder = candidateOrder;
      }
    }

    return bestLeaf;
  }

  private getJointChildLinkName(joint: any): string | null {
    const childLink = Array.isArray(joint?.children)
      ? joint.children.find((child: any) => child && (child.isURDFLink === true || child.type === "URDFLink"))
      : null;
    return typeof childLink?.urdfName === "string" ? childLink.urdfName : null;
  }

  private toTransformDetailFromPose(translation: Vector3, rotation: Quaternion): RobotTransformDetail {
    const euler = new Euler().setFromQuaternion(rotation, "XYZ");
    return {
      xyz: [translation.x, translation.y, translation.z],
      rpy: [euler.x, euler.y, euler.z]
    };
  }

  private parseOriginElement(node: Element | null | undefined): RobotTransformDetail | null {
    if (!node) {
      return null;
    }

    return {
      xyz: this.parseTripleAttribute(node, "xyz", [0, 0, 0]),
      rpy: this.parseTripleAttribute(node, "rpy", [0, 0, 0])
    };
  }

  private parseTripleAttribute(
    node: Element | null | undefined,
    attribute: string,
    fallback: [number, number, number]
  ): [number, number, number] {
    if (!node) {
      return [...fallback] as [number, number, number];
    }

    const raw = node.getAttribute(attribute);
    if (!raw) {
      return [...fallback] as [number, number, number];
    }

    const parts = raw.trim().split(/\s+/g).map((value) => Number(value));
    if (parts.length < 3 || parts.slice(0, 3).some((value) => !Number.isFinite(value))) {
      return [...fallback] as [number, number, number];
    }

    return [parts[0], parts[1], parts[2]];
  }

  private parseNumberAttribute(node: Element | null | undefined, attribute: string): number | null {
    if (!node) {
      return null;
    }

    const raw = node.getAttribute(attribute);
    if (raw == null || raw === "") {
      return null;
    }

    const value = Number(raw);
    return Number.isFinite(value) ? value : null;
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

    return this.getRobotRelativeTransform(frame);
  }

  getRobotRelativeTransform(frame: string): { translation: Vector3; rotation: Quaternion } | null {
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
      this.cleanupRobotAppearance(this.robot);
      this.robotLayer.remove(this.robot);
    }

    this.robot = robot;
    this.robotBaseFrame = baseFrame || "base_link";

    if (this.robot) {
      this.robotLayer.add(this.robot);
      this.styleRobotAppearance();
    }

    this.attachEndEffectorAxis();
    this.refreshFrameTransforms();
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

    if (this.robot) {
      this.cleanupRobotAppearance(this.robot);
    }

    this.clearPlannedTrajectory();
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

  private updateLighting(): void {
    if (this.currentTheme === "light") {
      this.ambientLight.color.setHex(0xffffff);
      this.ambientLight.intensity = 0.86;
      this.keyLight.color.setHex(0xfff3dc);
      this.keyLight.intensity = 0.88;
      this.fillLight.color.setHex(0xdce8f7);
      this.fillLight.intensity = 0.30;
      return;
    }

    this.ambientLight.color.setHex(0xe9f0fb);
    this.ambientLight.intensity = 0.66;
    this.keyLight.color.setHex(0xfff0d0);
    this.keyLight.intensity = 1.05;
    this.fillLight.color.setHex(0xcddcf1);
    this.fillLight.intensity = 0.36;
  }

  private styleRobotAppearance(): void {
    if (!this.robot) {
      return;
    }

    this.robot.traverse((object: any) => {
      if (!(object instanceof Mesh)) {
        return;
      }

      const role = this.getRobotPartRole(object);
      object.userData.__webRvizMaterialRole = role;
      object.material = this.ensureStyledMaterials(object.material);
      this.applyMaterialPreset(object.material, role);
      this.ensureEdgeHelper(object);
    });

    this.updateRobotEdgeAppearance();
  }

  private refreshRobotAppearance(): void {
    if (!this.robot) {
      return;
    }

    this.robot.traverse((object: any) => {
      if (!(object instanceof Mesh)) {
        return;
      }

      const role = (object.userData.__webRvizMaterialRole as RobotPartRole | undefined) ?? this.getRobotPartRole(object);
      object.userData.__webRvizMaterialRole = role;
      this.applyMaterialPreset(object.material, role);
    });

    this.updateRobotEdgeAppearance();
  }

  private ensureStyledMaterials(material: Material | Material[]): Material | Material[] {
    if (Array.isArray(material)) {
      return material.map((entry) => this.ensureStyledMaterial(entry));
    }
    return this.ensureStyledMaterial(material);
  }

  private ensureStyledMaterial(material: Material): MeshStandardMaterial {
    if (material instanceof MeshStandardMaterial && material.userData.__webRvizStyledMaterial) {
      return material;
    }

    const baseMaterial = material instanceof MeshStandardMaterial ? material.clone() : new MeshStandardMaterial();
    const source = material as Material & {
      color?: Color;
      map?: MeshStandardMaterial["map"];
      transparent?: boolean;
      opacity?: number;
      side?: number;
    };

    if (!(baseMaterial instanceof MeshStandardMaterial)) {
      return new MeshStandardMaterial();
    }

    baseMaterial.color.copy(source.color ?? new Color(0xb8b8b8));
    baseMaterial.map = source.map ?? null;
    baseMaterial.transparent = source.transparent ?? false;
    baseMaterial.opacity = source.opacity ?? 1;
    baseMaterial.side = source.side ?? baseMaterial.side;
    baseMaterial.flatShading = false;
    baseMaterial.userData.__webRvizStyledMaterial = true;
    return baseMaterial;
  }

  private applyMaterialPreset(material: Material | Material[], role: RobotPartRole): void {
    const preset = this.getMaterialPreset(role);
    const materials = Array.isArray(material) ? material : [material];

    for (const entry of materials) {
      if (!(entry instanceof MeshStandardMaterial)) {
        continue;
      }
      entry.color.setHex(preset.color);
      entry.metalness = preset.metalness;
      entry.roughness = preset.roughness;
      entry.needsUpdate = true;
    }
  }

  private getMaterialPreset(role: RobotPartRole): { color: number; metalness: number; roughness: number } {
    if (this.currentTheme === "light") {
      switch (role) {
        case "joint":
          return { color: 0x8e95a3, metalness: 0.3, roughness: 0.5 };
        case "tool":
          return { color: 0x5b6675, metalness: 0.28, roughness: 0.56 };
        case "cable":
          return { color: 0x4f5661, metalness: 0.06, roughness: 0.82 };
        case "accent":
          return { color: 0x97996d, metalness: 0.24, roughness: 0.62 };
        case "body":
        default:
          return { color: 0xb1ae75, metalness: 0.24, roughness: 0.64 };
      }
    }

    switch (role) {
      case "joint":
        return { color: 0xaeb1bc, metalness: 0.34, roughness: 0.46 };
      case "tool":
        return { color: 0x79808b, metalness: 0.31, roughness: 0.52 };
      case "cable":
        return { color: 0x5e6470, metalness: 0.08, roughness: 0.8 };
      case "accent":
        return { color: 0xa0a271, metalness: 0.26, roughness: 0.58 };
      case "body":
      default:
        return { color: 0xc8c697, metalness: 0.28, roughness: 0.6 };
    }
  }

  private getRobotPartRole(mesh: Mesh): RobotPartRole {
    const names = [mesh.name, mesh.parent?.name ?? "", mesh.parent?.parent?.name ?? ""]
      .join(" ")
      .toLowerCase();

    if (/(tool|tcp|gripper|flange|wrist|ee|end)/.test(names)) {
      return "tool";
    }
    if (/(joint|motor|gear|bearing|axis)/.test(names)) {
      return "joint";
    }
    if (/(cable|wire|hose)/.test(names)) {
      return "cable";
    }
    if (/(base|pedestal|mount)/.test(names)) {
      return "accent";
    }
    return "body";
  }

  private ensureEdgeHelper(mesh: Mesh): void {
    const existingEdge = mesh.children.find((child) => child.userData.__webRvizEdgeHelper === true) as LineSegments | undefined;
    if (existingEdge) {
      return;
    }

    const edgeGeometry = new EdgesGeometry(mesh.geometry, 30);
    const edgeMaterial = new LineBasicMaterial({
      color: this.getEdgeColor(),
      transparent: true,
      opacity: this.getEdgeOpacity(),
      depthWrite: false,
      toneMapped: false
    });

    const edgeLines = new LineSegments(edgeGeometry, edgeMaterial);
    edgeLines.userData.__webRvizEdgeHelper = true;
    edgeLines.renderOrder = 2;
    mesh.add(edgeLines);
  }

  private updateRobotEdgeAppearance(): void {
    if (!this.robot) {
      return;
    }

    const color = this.getEdgeColor();
    const opacity = this.getEdgeOpacity();

    this.robot.traverse((object: any) => {
      if (object.userData.__webRvizEdgeHelper !== true) {
        return;
      }

      const materials = Array.isArray(object.material) ? object.material : [object.material];
      for (const entry of materials) {
        if (!(entry instanceof LineBasicMaterial)) {
          continue;
        }
        entry.color.setHex(color);
        entry.opacity = opacity;
        entry.needsUpdate = true;
      }
    });
  }

  private getEdgeColor(): number {
    return this.currentTheme === "light" ? 0x6b84a3 : 0x90b8da;
  }

  private getEdgeOpacity(): number {
    return this.currentTheme === "light" ? 0.24 : 0.28;
  }

  private updatePlannedTrajectoryAppearance(): void {
    if (this.plannedTrajectoryStartMarker) {
      this.updatePlannedTrajectoryPoseMarkerAppearance(this.plannedTrajectoryStartMarker, "start");
    }

    if (this.plannedTrajectoryEndMarker) {
      this.updatePlannedTrajectoryPoseMarkerAppearance(this.plannedTrajectoryEndMarker, "end");
    }

    if (!this.plannedTrajectoryLine) {
      return;
    }

    const materials = Array.isArray(this.plannedTrajectoryLine.material)
      ? this.plannedTrajectoryLine.material
      : [this.plannedTrajectoryLine.material];
    for (const material of materials) {
      if (!(material instanceof LineMaterial)) {
        continue;
      }
      material.color.setHex(this.getPlannedTrajectoryColor());
      material.opacity = this.getPlannedTrajectoryOpacity();
      material.linewidth = this.getPlannedTrajectoryLineWidth();
      material.needsUpdate = true;
    }
  }

  private updatePlannedTrajectoryVisibility(): void {
    if (this.plannedTrajectoryLine) {
      this.plannedTrajectoryLine.visible = this.plannedTrajectoryVisible;
    }
    if (this.plannedTrajectoryStartMarker) {
      this.plannedTrajectoryStartMarker.visible = this.plannedTrajectoryVisible;
    }
    if (this.plannedTrajectoryEndMarker) {
      this.plannedTrajectoryEndMarker.visible = this.plannedTrajectoryVisible;
    }
  }

  private getPlannedTrajectoryColor(): number {
    return this.currentTheme === "light" ? 0x1178c7 : 0x62d99f;
  }

  private getPlannedTrajectoryOpacity(): number {
    return this.currentTheme === "light" ? 0.88 : 0.92;
  }

  private getPlannedTrajectoryLineWidth(): number {
    return 1.25;
  }

  private createPlannedTrajectoryPoseMarker(
    kind: "start" | "end",
    pose: { translation: Vector3; rotation: { x: number; y: number; z: number; w: number } }
  ): Group {
    const marker = new Group();
    marker.position.copy(pose.translation);
    marker.quaternion.set(
      pose.rotation.x,
      pose.rotation.y,
      pose.rotation.z,
      pose.rotation.w
    );
    marker.renderOrder = 4;

    marker.add(this.createPlannedTrajectoryAxisMesh("x", 0xff5a5a));
    marker.add(this.createPlannedTrajectoryAxisMesh("y", 0x4bd37b));
    marker.add(this.createPlannedTrajectoryAxisMesh("z", 0x4aa8ff));

    const sphere = new Mesh(
      new SphereGeometry(0.014, 14, 12),
      new MeshBasicMaterial({
        color: kind === "start" ? this.getPlannedTrajectoryStartMarkerColor() : this.getPlannedTrajectoryEndMarkerColor(),
        transparent: true,
        opacity: this.getPlannedTrajectoryMarkerOpacity(),
        depthWrite: false,
        depthTest: false,
        toneMapped: false
      })
    );
    sphere.name = "planned-trajectory-origin";
    sphere.userData.__plannedTrajectoryOrigin = true;
    sphere.renderOrder = 4;
    marker.add(sphere);

    const label = this.createPlannedTrajectoryLabelSprite(kind);
    const labelOffset = new Vector3(0, 0, this.getPlannedTrajectoryMarkerLabelOffset()).applyQuaternion(
      new Quaternion(
        pose.rotation.x,
        pose.rotation.y,
        pose.rotation.z,
        pose.rotation.w
      ).invert()
    );
    label.position.copy(labelOffset);
    marker.add(label);

    this.updatePlannedTrajectoryPoseMarkerAppearance(marker, kind);
    return marker;
  }

  private createPlannedTrajectoryAxisMesh(axis: "x" | "y" | "z", color: number): Mesh {
    const length = this.getPlannedTrajectoryMarkerAxisLength();
    const radius = this.getPlannedTrajectoryMarkerAxisRadius();
    const mesh = new Mesh(
      new CylinderGeometry(radius, radius, length, 10),
      new MeshBasicMaterial({
        color,
        transparent: true,
        opacity: this.getPlannedTrajectoryMarkerAxisOpacity(),
        depthWrite: false,
        depthTest: false,
        toneMapped: false
      })
    );
    mesh.name = "planned-trajectory-axis-" + axis;
    mesh.userData.__plannedTrajectoryAxis = true;
    mesh.userData.__plannedTrajectoryAxisColor = color;
    mesh.renderOrder = 4;

    if (axis === "x") {
      mesh.rotation.z = -Math.PI / 2;
      mesh.position.x = length / 2;
    } else if (axis === "y") {
      mesh.position.y = length / 2;
    } else {
      mesh.rotation.x = Math.PI / 2;
      mesh.position.z = length / 2;
    }

    return mesh;
  }

  private createPlannedTrajectoryLabelSprite(kind: "start" | "end"): Sprite {
    const material = new SpriteMaterial({
      map: this.createPlannedTrajectoryLabelTexture(kind),
      transparent: true,
      depthWrite: false,
      depthTest: false,
      toneMapped: false
    });
    const sprite = new Sprite(material);
    sprite.name = "planned-trajectory-label";
    sprite.userData.__plannedTrajectoryLabelKind = kind;
    sprite.scale.set(0.08, 0.044, 1);
    sprite.renderOrder = 5;
    return sprite;
  }

  private createPlannedTrajectoryLabelTexture(kind: "start" | "end"): CanvasTexture {
    const canvas = document.createElement("canvas");
    canvas.width = 96;
    canvas.height = 56;
    const context = canvas.getContext("2d");
    if (!context) {
      return new CanvasTexture(canvas);
    }

    const width = canvas.width;
    const height = canvas.height;
    const radius = 14;
    const color = "#" + (kind === "start" ? this.getPlannedTrajectoryStartMarkerColor() : this.getPlannedTrajectoryEndMarkerColor()).toString(16).padStart(6, "0");
    const label = kind === "start" ? "S" : "E";

    context.clearRect(0, 0, width, height);
    context.fillStyle = color;
    context.strokeStyle = "rgba(255,255,255,0.22)";
    context.lineWidth = 2;
    context.beginPath();
    context.moveTo(radius, 4);
    context.lineTo(width - radius, 4);
    context.quadraticCurveTo(width - 4, 4, width - 4, radius);
    context.lineTo(width - 4, height - radius);
    context.quadraticCurveTo(width - 4, height - 4, width - radius, height - 4);
    context.lineTo(radius, height - 4);
    context.quadraticCurveTo(4, height - 4, 4, height - radius);
    context.lineTo(4, radius);
    context.quadraticCurveTo(4, 4, radius, 4);
    context.closePath();
    context.fill();
    context.stroke();

    context.fillStyle = "#ffffff";
    context.font = "700 28px Segoe UI";
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillText(label, width / 2, height / 2 + 1);

    const texture = new CanvasTexture(canvas);
    texture.needsUpdate = true;
    return texture;
  }

  private disposePlannedTrajectoryPoseMarker(marker: Group): void {
    this.scene.remove(marker);
    marker.traverse((object: any) => {
      if (object.geometry && typeof object.geometry.dispose === "function") {
        object.geometry.dispose();
      }

      const materials = Array.isArray(object.material) ? object.material : [object.material];
      for (const material of materials) {
        if (material && "map" in material && material.map && typeof material.map.dispose === "function") {
          material.map.dispose();
        }
        if (material instanceof Material) {
          material.dispose();
        }
      }
    });
  }

  private updatePlannedTrajectoryPoseMarkerAppearance(marker: Group, kind: "start" | "end"): void {
    marker.traverse((object: any) => {
      if (object instanceof Sprite && object.material instanceof SpriteMaterial) {
        if (object.material.map) {
          object.material.map.dispose();
        }
        object.material.map = this.createPlannedTrajectoryLabelTexture(kind);
        object.material.needsUpdate = true;
        return;
      }

      const materials = Array.isArray(object.material) ? object.material : [object.material];
      for (const material of materials) {
        if (material instanceof MeshBasicMaterial) {
          if (object.userData.__plannedTrajectoryOrigin === true) {
            material.color.setHex(kind === "start" ? this.getPlannedTrajectoryStartMarkerColor() : this.getPlannedTrajectoryEndMarkerColor());
            material.opacity = this.getPlannedTrajectoryMarkerOpacity();
          } else if (object.userData.__plannedTrajectoryAxis === true) {
            material.color.setHex(object.userData.__plannedTrajectoryAxisColor ?? 0xffffff);
            material.opacity = this.getPlannedTrajectoryMarkerAxisOpacity();
          }
          material.depthWrite = false;
          material.depthTest = false;
          material.needsUpdate = true;
        }
      }
    });
  }

  private getPlannedTrajectoryStartMarkerColor(): number {
    return this.currentTheme === "light" ? 0x0f9f6e : 0x62d99f;
  }

  private getPlannedTrajectoryEndMarkerColor(): number {
    return this.currentTheme === "light" ? 0xd9485f : 0xff8c72;
  }

  private getPlannedTrajectoryMarkerOpacity(): number {
    return this.currentTheme === "light" ? 0.96 : 0.98;
  }

  private getPlannedTrajectoryMarkerAxisOpacity(): number {
    return this.currentTheme === "light" ? 0.92 : 0.96;
  }

  private getPlannedTrajectoryMarkerAxisLength(): number {
    return 0.075 * 0.75;
  }

  private getPlannedTrajectoryMarkerAxisRadius(): number {
    return 0.00375;
  }

  private getPlannedTrajectoryMarkerLabelOffset(): number {
    return 0.05;
  }

  private cleanupRobotAppearance(root: Group): void {
    root.traverse((object: any) => {
      if (object instanceof Mesh) {
        const materials = Array.isArray(object.material) ? object.material : [object.material];
        for (const entry of materials) {
          if (entry instanceof MeshStandardMaterial && entry.userData.__webRvizStyledMaterial) {
            entry.dispose();
          }
        }
      }

      const edgeChildren = object.children.filter((child: any) => child.userData.__webRvizEdgeHelper === true);
      for (const child of edgeChildren) {
        object.remove(child);
        if (child instanceof LineSegments) {
          child.geometry.dispose();
          const materials = Array.isArray(child.material) ? child.material : [child.material];
          for (const entry of materials) {
            if (entry instanceof LineBasicMaterial) {
              entry.dispose();
            }
          }
        }
      }
    });
  }

  private onResize(): void {
    const width = Math.max(1, this.container.clientWidth);
    const height = Math.max(1, this.container.clientHeight);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
    this.updatePlannedTrajectoryResolution(width, height);
  }

  private updatePlannedTrajectoryResolution(width: number, height: number): void {
    if (!this.plannedTrajectoryLine) {
      return;
    }

    const materials = Array.isArray(this.plannedTrajectoryLine.material)
      ? this.plannedTrajectoryLine.material
      : [this.plannedTrajectoryLine.material];
    for (const material of materials) {
      if (!(material instanceof LineMaterial)) {
        continue;
      }
      material.resolution.set(width, height);
    }
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

