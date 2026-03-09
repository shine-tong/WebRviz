export interface ParsedPointCloud {
  frameId: string;
  positions: Float32Array;
  colors: Float32Array;
  count: number;
}

interface PointField {
  name: string;
  offset: number;
  datatype: number;
  count: number;
}

interface PointCloud2Message {
  header?: {
    frame_id?: string;
  };
  width: number;
  height: number;
  point_step: number;
  is_bigendian: boolean;
  fields: PointField[];
  data: number[] | string;
}

const DATATYPE_INT8 = 1;
const DATATYPE_UINT8 = 2;
const DATATYPE_INT16 = 3;
const DATATYPE_UINT16 = 4;
const DATATYPE_INT32 = 5;
const DATATYPE_UINT32 = 6;
const DATATYPE_FLOAT32 = 7;
const DATATYPE_FLOAT64 = 8;

function decodeBase64(input: string): Uint8Array {
  const binary = atob(input);
  const data = new Uint8Array(binary.length);
  for (let index = 0; index < binary.length; index += 1) {
    data[index] = binary.charCodeAt(index);
  }
  return data;
}

function toByteArray(data: PointCloud2Message["data"]): Uint8Array {
  if (typeof data === "string") {
    return decodeBase64(data);
  }

  return Uint8Array.from(data);
}

function readNumericField(view: DataView, baseOffset: number, field: PointField, littleEndian: boolean): number {
  const offset = baseOffset + field.offset;
  switch (field.datatype) {
    case DATATYPE_INT8:
      return view.getInt8(offset);
    case DATATYPE_UINT8:
      return view.getUint8(offset);
    case DATATYPE_INT16:
      return view.getInt16(offset, littleEndian);
    case DATATYPE_UINT16:
      return view.getUint16(offset, littleEndian);
    case DATATYPE_INT32:
      return view.getInt32(offset, littleEndian);
    case DATATYPE_UINT32:
      return view.getUint32(offset, littleEndian);
    case DATATYPE_FLOAT32:
      return view.getFloat32(offset, littleEndian);
    case DATATYPE_FLOAT64:
      return view.getFloat64(offset, littleEndian);
    default:
      return Number.NaN;
  }
}

function readRgbBits(view: DataView, baseOffset: number, field: PointField, littleEndian: boolean): number {
  const offset = baseOffset + field.offset;
  if (field.datatype === DATATYPE_FLOAT32) {
    return view.getUint32(offset, littleEndian);
  }

  if (field.datatype === DATATYPE_UINT32 || field.datatype === DATATYPE_INT32) {
    return view.getUint32(offset, littleEndian);
  }

  if (field.datatype === DATATYPE_UINT8 && field.count >= 3) {
    const r = view.getUint8(offset);
    const g = view.getUint8(offset + 1);
    const b = view.getUint8(offset + 2);
    return (r << 16) | (g << 8) | b;
  }

  return 0xffffff;
}

function unpackRgb(colorBits: number): [number, number, number] {
  const r = (colorBits >> 16) & 0xff;
  const g = (colorBits >> 8) & 0xff;
  const b = colorBits & 0xff;

  return [r / 255, g / 255, b / 255];
}

export function decodePointCloud2(rawMessage: unknown, maxPoints: number): ParsedPointCloud | null {
  const message = rawMessage as PointCloud2Message;
  if (!message || !Array.isArray(message.fields) || message.point_step <= 0) {
    return null;
  }

  const fields = message.fields;
  const xField = fields.find((field) => field.name === "x");
  const yField = fields.find((field) => field.name === "y");
  const zField = fields.find((field) => field.name === "z");
  const rgbField = fields.find((field) => field.name === "rgb" || field.name === "rgba");
  const intensityField = fields.find((field) => field.name === "intensity");

  if (!xField || !yField || !zField) {
    return null;
  }

  const bytes = toByteArray(message.data);
  const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  const littleEndian = !message.is_bigendian;

  const maxByPayload = Math.floor(bytes.byteLength / message.point_step);
  const totalPoints = Math.min(maxByPayload, Math.max(0, message.width * message.height));
  if (totalPoints <= 0) {
    return null;
  }

  const safeMaxPoints = Math.max(1, maxPoints);
  const sampleStep = Math.max(1, Math.floor(totalPoints / safeMaxPoints));
  const estimatedCount = Math.ceil(totalPoints / sampleStep);

  const positions = new Float32Array(estimatedCount * 3);
  const colors = new Float32Array(estimatedCount * 3);

  let writeIndex = 0;
  for (let pointIndex = 0; pointIndex < totalPoints; pointIndex += sampleStep) {
    const baseOffset = pointIndex * message.point_step;

    const x = readNumericField(view, baseOffset, xField, littleEndian);
    const y = readNumericField(view, baseOffset, yField, littleEndian);
    const z = readNumericField(view, baseOffset, zField, littleEndian);

    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) {
      continue;
    }

    positions[writeIndex * 3] = x;
    positions[writeIndex * 3 + 1] = y;
    positions[writeIndex * 3 + 2] = z;

    let r = 0.82;
    let g = 0.88;
    let b = 0.95;

    if (rgbField) {
      [r, g, b] = unpackRgb(readRgbBits(view, baseOffset, rgbField, littleEndian));
    } else if (intensityField) {
      const intensity = readNumericField(view, baseOffset, intensityField, littleEndian);
      const normalized = Number.isFinite(intensity) ? Math.max(0, Math.min(1, intensity)) : 0.65;
      r = normalized;
      g = normalized;
      b = normalized;
    }

    colors[writeIndex * 3] = r;
    colors[writeIndex * 3 + 1] = g;
    colors[writeIndex * 3 + 2] = b;
    writeIndex += 1;
  }

  if (writeIndex === 0) {
    return null;
  }

  return {
    frameId: (message.header?.frame_id || "").replace(/^\//, ""),
    positions: positions.slice(0, writeIndex * 3),
    colors: colors.slice(0, writeIndex * 3),
    count: writeIndex
  };
}