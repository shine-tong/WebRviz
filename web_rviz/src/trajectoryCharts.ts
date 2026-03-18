export interface TrajectoryChartSeries {
  label: string;
  color: string;
  values: number[];
  visible?: boolean;
}

export interface TrajectoryChartTheme {
  background: string;
  border: string;
  grid: string;
  text: string;
  muted: string;
  accent: string;
}

export interface TrajectoryLineChartOptions {
  canvas: HTMLCanvasElement;
  times: number[];
  series: TrajectoryChartSeries[];
  unitLabel: string;
  emptyLabel: string;
  theme: TrajectoryChartTheme;
  playheadTime?: number | null;
}

interface Point {
  time: number;
  value: number;
}

interface ChartLayout {
  width: number;
  height: number;
  paddingTop: number;
  paddingRight: number;
  paddingBottom: number;
  paddingLeft: number;
  plotWidth: number;
  plotHeight: number;
}

interface HoverSelection {
  sampleIndex: number;
  seriesLabel: string;
}

interface HoverDisplay {
  label: string;
  color: string;
  time: number;
  value: number;
  x: number;
  y: number;
}

interface ChartInteractiveState {
  options: TrajectoryLineChartOptions;
  tooltip: HTMLDivElement;
  tooltipDot: HTMLSpanElement;
  tooltipLabel: HTMLSpanElement;
  tooltipValue: HTMLDivElement;
  tooltipTime: HTMLDivElement;
  hover: HoverSelection | null;
  rafPending: boolean;
}

const chartStates = new WeakMap<HTMLCanvasElement, ChartInteractiveState>();
const HOVER_DISTANCE_PX = 18;

function resizeCanvasToDisplaySize(canvas: HTMLCanvasElement): { width: number; height: number; context: CanvasRenderingContext2D } | null {
  const rect = canvas.getBoundingClientRect();
  const cssWidth = Math.max(1, Math.round(rect.width));
  const cssHeight = Math.max(1, Math.round(rect.height));
  const context = canvas.getContext("2d");
  if (!context || cssWidth <= 0 || cssHeight <= 0) {
    return null;
  }

  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.max(1, Math.round(cssWidth * dpr));
  const height = Math.max(1, Math.round(cssHeight * dpr));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }

  context.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { width: cssWidth, height: cssHeight, context };
}

function formatAxisValue(value: number): string {
  const absolute = Math.abs(value);
  if (absolute >= 1000) {
    return value.toFixed(0);
  }
  if (absolute >= 100) {
    return value.toFixed(1);
  }
  if (absolute >= 10) {
    return value.toFixed(2);
  }
  if (absolute >= 1) {
    return value.toFixed(3);
  }
  return value.toFixed(4);
}

function formatSeconds(seconds: number): string {
  if (!Number.isFinite(seconds) || seconds <= 0) {
    return "0s";
  }
  if (seconds >= 60) {
    return (seconds / 60).toFixed(seconds >= 600 ? 0 : 1) + "m";
  }
  if (seconds >= 10) {
    return seconds.toFixed(1) + "s";
  }
  return seconds.toFixed(2) + "s";
}

function collectRange(seriesList: TrajectoryChartSeries[]): { min: number; max: number } | null {
  let min = Number.POSITIVE_INFINITY;
  let max = Number.NEGATIVE_INFINITY;

  for (const series of seriesList) {
    if (series.visible === false) {
      continue;
    }
    for (const value of series.values) {
      if (!Number.isFinite(value)) {
        continue;
      }
      min = Math.min(min, value);
      max = Math.max(max, value);
    }
  }

  if (!Number.isFinite(min) || !Number.isFinite(max)) {
    return null;
  }

  if (min === max) {
    const padding = min === 0 ? 1 : Math.max(1e-6, Math.abs(min) * 0.08);
    return { min: min - padding, max: max + padding };
  }

  const padding = Math.max(1e-6, (max - min) * 0.08);
  return { min: min - padding, max: max + padding };
}

function decimateSeries(times: number[], values: number[], pixelWidth: number): Point[] {
  const finiteCount = values.reduce((count, value) => count + (Number.isFinite(value) ? 1 : 0), 0);
  if (finiteCount === 0) {
    return [];
  }

  const maxBuckets = Math.max(24, Math.floor(pixelWidth));
  if (times.length <= maxBuckets * 2) {
    const raw: Point[] = [];
    for (let index = 0; index < times.length; index += 1) {
      const value = values[index];
      if (!Number.isFinite(value)) {
        continue;
      }
      raw.push({ time: times[index], value });
    }
    return raw;
  }

  const bucketSize = Math.max(1, Math.ceil(times.length / maxBuckets));
  const output: Point[] = [];
  for (let start = 0; start < times.length; start += bucketSize) {
    const end = Math.min(times.length, start + bucketSize);
    let firstIndex = -1;
    let lastIndex = -1;
    let minIndex = -1;
    let maxIndex = -1;
    let minValue = Number.POSITIVE_INFINITY;
    let maxValue = Number.NEGATIVE_INFINITY;

    for (let index = start; index < end; index += 1) {
      const value = values[index];
      if (!Number.isFinite(value)) {
        continue;
      }
      if (firstIndex === -1) {
        firstIndex = index;
      }
      lastIndex = index;
      if (value < minValue) {
        minValue = value;
        minIndex = index;
      }
      if (value > maxValue) {
        maxValue = value;
        maxIndex = index;
      }
    }

    if (firstIndex === -1) {
      continue;
    }

    const indexes = Array.from(new Set([firstIndex, minIndex, maxIndex, lastIndex].filter((index) => index >= 0))).sort((left, right) => left - right);
    for (const index of indexes) {
      output.push({ time: times[index], value: values[index] });
    }
  }

  return output;
}

function lineY(value: number, min: number, max: number, top: number, height: number): number {
  const ratio = (value - min) / Math.max(1e-9, max - min);
  return top + height - ratio * height;
}

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

function chartLayout(width: number, height: number): ChartLayout {
  const paddingTop = 18;
  const paddingRight = 18;
  const paddingBottom = 32;
  const paddingLeft = 58;
  return {
    width,
    height,
    paddingTop,
    paddingRight,
    paddingBottom,
    paddingLeft,
    plotWidth: Math.max(1, width - paddingLeft - paddingRight),
    plotHeight: Math.max(1, height - paddingTop - paddingBottom)
  };
}

function findNearestTimeIndex(times: number[], targetTime: number): number {
  if (times.length === 0) {
    return -1;
  }

  let low = 0;
  let high = times.length - 1;
  while (low < high) {
    const middle = Math.floor((low + high) / 2);
    if (times[middle] < targetTime) {
      low = middle + 1;
    } else {
      high = middle;
    }
  }

  if (low <= 0) {
    return 0;
  }
  const previous = low - 1;
  return Math.abs(times[low] - targetTime) < Math.abs(times[previous] - targetTime) ? low : previous;
}

function ensureChartState(canvas: HTMLCanvasElement): ChartInteractiveState {
  const existing = chartStates.get(canvas);
  if (existing) {
    return existing;
  }

  const shell = canvas.parentElement;
  if (!(shell instanceof HTMLElement)) {
    throw new Error("Trajectory chart canvas must be wrapped in an element");
  }

  const tooltip = document.createElement("div");
  tooltip.className = "trajectory-chart-tooltip";
  const tooltipHeader = document.createElement("div");
  tooltipHeader.className = "trajectory-chart-tooltip-header";
  const tooltipDot = document.createElement("span");
  tooltipDot.className = "trajectory-chart-tooltip-dot";
  const tooltipLabel = document.createElement("span");
  tooltipLabel.className = "trajectory-chart-tooltip-label";
  tooltipHeader.appendChild(tooltipDot);
  tooltipHeader.appendChild(tooltipLabel);
  const tooltipValue = document.createElement("div");
  tooltipValue.className = "trajectory-chart-tooltip-value";
  const tooltipTime = document.createElement("div");
  tooltipTime.className = "trajectory-chart-tooltip-time";
  tooltip.appendChild(tooltipHeader);
  tooltip.appendChild(tooltipValue);
  tooltip.appendChild(tooltipTime);
  shell.appendChild(tooltip);

  const state: ChartInteractiveState = {
    options: {
      canvas,
      times: [],
      series: [],
      unitLabel: "",
      emptyLabel: "",
      theme: { background: "", border: "", grid: "", text: "", muted: "", accent: "" }
    },
    tooltip,
    tooltipDot,
    tooltipLabel,
    tooltipValue,
    tooltipTime,
    hover: null,
    rafPending: false
  };

  const scheduleDraw = () => {
    if (state.rafPending) {
      return;
    }
    state.rafPending = true;
    requestAnimationFrame(() => {
      state.rafPending = false;
      drawTrajectoryLineChart(state);
    });
  };

  const clearHover = () => {
    if (!state.hover && !state.tooltip.classList.contains("active")) {
      return;
    }
    state.hover = null;
    state.tooltip.classList.remove("active");
    scheduleDraw();
  };

  canvas.addEventListener("pointermove", (event) => {
    const rect = canvas.getBoundingClientRect();
    const resized = resizeCanvasToDisplaySize(canvas);
    if (!resized) {
      clearHover();
      return;
    }
    const layout = chartLayout(resized.width, resized.height);
    const visibleSeries = state.options.series.filter((series) => series.visible !== false);
    const range = collectRange(visibleSeries);
    if (!range || state.options.times.length === 0) {
      clearHover();
      return;
    }
    const pointerX = event.clientX - rect.left;
    const pointerY = event.clientY - rect.top;
    const nextHover = resolveHoverSelection(state.options, layout, range, pointerX, pointerY);
    const changed = nextHover?.sampleIndex !== state.hover?.sampleIndex || nextHover?.seriesLabel !== state.hover?.seriesLabel;
    if (changed) {
      state.hover = nextHover;
      scheduleDraw();
    }
  });
  canvas.addEventListener("pointerleave", clearHover);
  canvas.addEventListener("pointercancel", clearHover);

  chartStates.set(canvas, state);
  return state;
}

function resolveHoverSelection(
  options: TrajectoryLineChartOptions,
  layout: ChartLayout,
  range: { min: number; max: number },
  pointerX: number,
  pointerY: number
): HoverSelection | null {
  const withinPlot = pointerX >= layout.paddingLeft
    && pointerX <= layout.width - layout.paddingRight
    && pointerY >= layout.paddingTop
    && pointerY <= layout.height - layout.paddingBottom;
  if (!withinPlot || options.times.length === 0) {
    return null;
  }

  const totalDurationMs = Math.max(1, options.times[options.times.length - 1] || 1);
  const targetTime = clamp((pointerX - layout.paddingLeft) / layout.plotWidth, 0, 1) * totalDurationMs;
  const sampleIndex = findNearestTimeIndex(options.times, targetTime);
  if (sampleIndex < 0) {
    return null;
  }

  let bestLabel: string | null = null;
  let bestDistance = Number.POSITIVE_INFINITY;
  for (const series of options.series) {
    if (series.visible === false) {
      continue;
    }
    const value = series.values[sampleIndex];
    if (!Number.isFinite(value)) {
      continue;
    }
    const y = lineY(value, range.min, range.max, layout.paddingTop, layout.plotHeight);
    const distance = Math.abs(y - pointerY);
    if (distance < bestDistance) {
      bestDistance = distance;
      bestLabel = series.label;
    }
  }

  if (!bestLabel || bestDistance > HOVER_DISTANCE_PX) {
    return null;
  }

  return { sampleIndex, seriesLabel: bestLabel };
}

function resolveHoverDisplay(
  options: TrajectoryLineChartOptions,
  layout: ChartLayout,
  range: { min: number; max: number },
  hover: HoverSelection | null
): HoverDisplay | null {
  if (!hover || hover.sampleIndex < 0 || hover.sampleIndex >= options.times.length) {
    return null;
  }

  const series = options.series.find((entry) => entry.label === hover.seriesLabel && entry.visible !== false);
  if (!series) {
    return null;
  }

  const value = series.values[hover.sampleIndex];
  if (!Number.isFinite(value)) {
    return null;
  }

  const totalDurationMs = Math.max(1, options.times[options.times.length - 1] || 1);
  const time = options.times[hover.sampleIndex];
  const x = layout.paddingLeft + (time / totalDurationMs) * layout.plotWidth;
  const y = lineY(value, range.min, range.max, layout.paddingTop, layout.plotHeight);
  return {
    label: series.label,
    color: series.color,
    time,
    value,
    x,
    y
  };
}

function updateTooltip(state: ChartInteractiveState, layout: ChartLayout, hoverDisplay: HoverDisplay | null): void {
  if (!hoverDisplay) {
    state.tooltip.classList.remove("active");
    return;
  }

  state.tooltipDot.style.backgroundColor = hoverDisplay.color;
  state.tooltipLabel.textContent = hoverDisplay.label;
  state.tooltipValue.textContent = `${formatAxisValue(hoverDisplay.value)} ${state.options.unitLabel}`.trim();
  state.tooltipTime.textContent = `t = ${formatSeconds(hoverDisplay.time / 1000)}`;
  state.tooltip.classList.add("active");

  const margin = 8;
  const tooltipWidth = state.tooltip.offsetWidth || 120;
  const tooltipHeight = state.tooltip.offsetHeight || 58;
  const left = hoverDisplay.x <= layout.width * 0.6
    ? hoverDisplay.x + 12
    : hoverDisplay.x - tooltipWidth - 12;
  const top = hoverDisplay.y - tooltipHeight - 12 >= margin
    ? hoverDisplay.y - tooltipHeight - 12
    : hoverDisplay.y + 12;
  state.tooltip.style.left = `${clamp(left, margin, layout.width - tooltipWidth - margin)}px`;
  state.tooltip.style.top = `${clamp(top, margin, layout.height - tooltipHeight - margin)}px`;
}

function drawTrajectoryLineChart(state: ChartInteractiveState): void {
  const resized = resizeCanvasToDisplaySize(state.options.canvas);
  if (!resized) {
    return;
  }

  const { width, height, context } = resized;
  const layout = chartLayout(width, height);

  context.clearRect(0, 0, width, height);
  context.fillStyle = state.options.theme.background;
  context.fillRect(0, 0, width, height);
  context.strokeStyle = state.options.theme.border;
  context.lineWidth = 1;
  context.strokeRect(0.5, 0.5, Math.max(0, width - 1), Math.max(0, height - 1));

  context.font = "12px system-ui, sans-serif";
  context.textBaseline = "middle";

  const visibleSeries = state.options.series.filter((series) => series.visible !== false);
  const range = collectRange(visibleSeries);
  const totalDurationMs = state.options.times.length > 0 ? Math.max(1, state.options.times[state.options.times.length - 1]) : 1;

  context.fillStyle = state.options.theme.muted;
  context.textAlign = "left";
  context.fillText("0s", layout.paddingLeft, height - 14);
  context.textAlign = "right";
  context.fillText(formatSeconds(totalDurationMs / 1000), width - layout.paddingRight, height - 14);
  context.textAlign = "left";
  context.fillText(state.options.unitLabel, layout.paddingLeft, 10);

  const gridRows = 4;
  const gridColumns = 4;
  context.strokeStyle = state.options.theme.grid;
  context.lineWidth = 1;

  for (let row = 0; row <= gridRows; row += 1) {
    const y = layout.paddingTop + (layout.plotHeight * row) / gridRows;
    context.beginPath();
    context.moveTo(layout.paddingLeft, y + 0.5);
    context.lineTo(width - layout.paddingRight, y + 0.5);
    context.stroke();
  }

  for (let column = 0; column <= gridColumns; column += 1) {
    const x = layout.paddingLeft + (layout.plotWidth * column) / gridColumns;
    context.beginPath();
    context.moveTo(x + 0.5, layout.paddingTop);
    context.lineTo(x + 0.5, height - layout.paddingBottom);
    context.stroke();
  }

  if (!range || state.options.times.length === 0) {
    context.fillStyle = state.options.theme.muted;
    context.textAlign = "center";
    context.fillText(state.options.emptyLabel, width / 2, height / 2);
    updateTooltip(state, layout, null);
    return;
  }

  context.fillStyle = state.options.theme.muted;
  context.textAlign = "right";
  context.fillText(formatAxisValue(range.max), layout.paddingLeft - 8, layout.paddingTop + 2);
  context.fillText(formatAxisValue((range.max + range.min) / 2), layout.paddingLeft - 8, layout.paddingTop + layout.plotHeight / 2);
  context.fillText(formatAxisValue(range.min), layout.paddingLeft - 8, layout.paddingTop + layout.plotHeight - 2);

  for (const series of visibleSeries) {
    const points = decimateSeries(state.options.times, series.values, layout.plotWidth);
    if (points.length === 0) {
      continue;
    }

    context.beginPath();
    context.strokeStyle = series.color;
    context.lineWidth = 1.8;
    let started = false;
    for (const point of points) {
      const x = layout.paddingLeft + (point.time / totalDurationMs) * layout.plotWidth;
      const y = lineY(point.value, range.min, range.max, layout.paddingTop, layout.plotHeight);
      if (!started) {
        context.moveTo(x, y);
        started = true;
      } else {
        context.lineTo(x, y);
      }
    }
    context.stroke();
  }

  if (typeof state.options.playheadTime === "number" && Number.isFinite(state.options.playheadTime)) {
    const clamped = Math.max(0, Math.min(totalDurationMs, state.options.playheadTime));
    const x = layout.paddingLeft + (clamped / totalDurationMs) * layout.plotWidth;
    context.beginPath();
    context.strokeStyle = state.options.theme.accent;
    context.lineWidth = 1;
    context.moveTo(x + 0.5, layout.paddingTop);
    context.lineTo(x + 0.5, layout.paddingTop + layout.plotHeight);
    context.stroke();
  }

  const hoverDisplay = resolveHoverDisplay(state.options, layout, range, state.hover);
  if (hoverDisplay) {
    context.save();
    context.strokeStyle = state.options.theme.accent;
    context.lineWidth = 1;
    context.setLineDash([4, 4]);
    context.beginPath();
    context.moveTo(hoverDisplay.x + 0.5, layout.paddingTop);
    context.lineTo(hoverDisplay.x + 0.5, layout.paddingTop + layout.plotHeight);
    context.stroke();
    context.restore();

    context.beginPath();
    context.fillStyle = hoverDisplay.color;
    context.arc(hoverDisplay.x, hoverDisplay.y, 4, 0, Math.PI * 2);
    context.fill();
    context.beginPath();
    context.strokeStyle = state.options.theme.background;
    context.lineWidth = 1.5;
    context.arc(hoverDisplay.x, hoverDisplay.y, 4, 0, Math.PI * 2);
    context.stroke();
  }

  updateTooltip(state, layout, hoverDisplay);
}

export function renderTrajectoryLineChart(options: TrajectoryLineChartOptions): void {
  const state = ensureChartState(options.canvas);
  state.options = options;
  drawTrajectoryLineChart(state);
}
