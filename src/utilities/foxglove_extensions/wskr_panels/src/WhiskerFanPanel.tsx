import {
  PanelExtensionContext,
  MessageEvent,
  SettingsTreeAction,
} from "@foxglove/extension";
import { useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

import {
  AXIS_X_COLOR,
  AXIS_Y_COLOR,
  BACKGROUND_FILL,
  CHASSIS_FILL,
  CHASSIS_STROKE,
  HEADING_COLOR_DEAD_RECK,
  HEADING_COLOR_VISUAL,
  LABEL_FILL,
  ROBOT_LENGTH_MM,
  ROBOT_WIDTH_MM,
  TARGET_MARKER_FILL,
  TARGET_MARKER_STROKE,
  WHEEL_FILL,
  WHEEL_STROKE,
  WHISKER_ANGLES_DEG,
  WHISKER_RANGE_MM,
  whiskerColor,
} from "./fanGeometry";

interface PanelConfig {
  whiskerTopic: string;
  targetWhiskerTopic: string;
  headingTopic: string;
  trackingModeTopic: string;
}

const DEFAULT_CONFIG: PanelConfig = {
  whiskerTopic: "/WSKR/whisker_lengths",
  targetWhiskerTopic: "/WSKR/target_whisker_lengths",
  headingTopic: "/WSKR/heading_to_target",
  trackingModeTopic: "/WSKR/tracking_mode",
};

type Float32MultiArrayMsg = { data: number[] | Float32Array };
type Float32Msg = { data: number };
type StringMsg = { data: string };

function drawTargetDiamond(
  ctx: CanvasRenderingContext2D,
  x: number,
  y: number,
): void {
  const r = 5;
  ctx.beginPath();
  ctx.moveTo(x, y - r);
  ctx.lineTo(x + r, y);
  ctx.lineTo(x, y + r);
  ctx.lineTo(x - r, y);
  ctx.closePath();
  ctx.fillStyle = TARGET_MARKER_FILL;
  ctx.strokeStyle = TARGET_MARKER_STROKE;
  ctx.lineWidth = 1;
  ctx.fill();
  ctx.stroke();
}

function WhiskerFanPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const wrapRef = useRef<HTMLDivElement>(null);

  const [config, setConfig] = useState<PanelConfig>(() => {
    const saved = context.initialState as Partial<PanelConfig> | undefined;
    return { ...DEFAULT_CONFIG, ...(saved ?? {}) };
  });

  const whiskersRef = useRef<number[] | undefined>(undefined);
  const targetWhiskersRef = useRef<number[] | undefined>(undefined);
  const headingRef = useRef<number | undefined>(undefined);
  const modeRef = useRef<string>("—");

  useEffect(() => {
    const actionHandler = (action: SettingsTreeAction) => {
      if (action.action === "update") {
        const { path, value } = action.payload;
        setConfig((prev) => {
          const key = path[path.length - 1] as keyof PanelConfig;
          if (!(key in prev)) return prev;
          const next = { ...prev, [key]: value } as PanelConfig;
          context.saveState(next);
          return next;
        });
      }
    };
    context.updatePanelSettingsEditor({
      actionHandler,
      nodes: {
        general: {
          label: "Whisker Fan",
          fields: {
            whiskerTopic: {
              label: "Whisker lengths",
              input: "string",
              value: config.whiskerTopic,
            },
            targetWhiskerTopic: {
              label: "Target whisker lengths",
              input: "string",
              value: config.targetWhiskerTopic,
            },
            headingTopic: {
              label: "Heading (deg)",
              input: "string",
              value: config.headingTopic,
            },
            trackingModeTopic: {
              label: "Tracking mode",
              input: "string",
              value: config.trackingModeTopic,
            },
          },
        },
      },
    });
  }, [context, config]);

  useEffect(() => {
    context.subscribe([
      { topic: config.whiskerTopic },
      { topic: config.targetWhiskerTopic },
      { topic: config.headingTopic },
      { topic: config.trackingModeTopic },
    ]);
    return () => {
      context.unsubscribeAll();
    };
  }, [context, config.whiskerTopic, config.targetWhiskerTopic, config.headingTopic, config.trackingModeTopic]);

  const [renderTick, setRenderTick] = useState(0);
  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      if (renderState.currentFrame) {
        for (const msg of renderState.currentFrame) {
          if (msg.topic === config.whiskerTopic) {
            const arr = (msg as MessageEvent<Float32MultiArrayMsg>).message.data;
            whiskersRef.current = Array.from(arr).map(Number);
          } else if (msg.topic === config.targetWhiskerTopic) {
            const arr = (msg as MessageEvent<Float32MultiArrayMsg>).message.data;
            targetWhiskersRef.current = Array.from(arr).map(Number);
          } else if (msg.topic === config.headingTopic) {
            headingRef.current = Number((msg as MessageEvent<Float32Msg>).message.data);
          } else if (msg.topic === config.trackingModeTopic) {
            modeRef.current = (msg as MessageEvent<StringMsg>).message.data ?? "—";
          }
        }
      }
      setRenderTick((t) => t + 1);
      done();
    };
    context.watch("currentFrame");
  }, [context, config.whiskerTopic, config.targetWhiskerTopic, config.headingTopic, config.trackingModeTopic]);

  // Redraw loop.
  useEffect(() => {
    const canvas = canvasRef.current;
    const wrap = wrapRef.current;
    if (!canvas || !wrap) return;

    const dpr = window.devicePixelRatio || 1;
    const areaW = wrap.clientWidth || 480;
    const areaH = wrap.clientHeight || 360;
    canvas.width = Math.max(1, Math.floor(areaW * dpr));
    canvas.height = Math.max(1, Math.floor(areaH * dpr));
    canvas.style.width = `${areaW}px`;
    canvas.style.height = `${areaH}px`;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

    ctx.fillStyle = BACKGROUND_FILL;
    ctx.fillRect(0, 0, areaW, areaH);

    if (areaW < 60 || areaH < 60) return;

    // px-per-mm chosen to fit whisker fan + robot length vertically
    // and twice the whisker radius horizontally. Matches the tkinter logic.
    const availableH = Math.max(areaH - 18, 1);
    const availableW = Math.max(areaW - 12, 1);
    const pxPerMm = Math.min(
      availableH / (WHISKER_RANGE_MM + ROBOT_LENGTH_MM),
      availableW / (2 * WHISKER_RANGE_MM),
    );
    const maxRayPx = WHISKER_RANGE_MM * pxPerMm;
    const bodyW = ROBOT_WIDTH_MM * pxPerMm;
    const bodyH = ROBOT_LENGTH_MM * pxPerMm;
    const wheelR = Math.max(bodyH * 0.11, 3);

    const cx = areaW / 2;
    const robotBottom = areaH - 10;
    const robotTop = robotBottom - bodyH;
    const bodyCx = cx;
    const bodyCy = (robotTop + robotBottom) / 2;
    const ox = cx;
    const oy = robotTop;

    const whiskers = whiskersRef.current;
    const target = targetWhiskersRef.current;

    if (whiskers && whiskers.length > 0) {
      const n = whiskers.length;
      for (let i = 0; i < n; i++) {
        const mm = whiskers[i];
        const thetaDeg =
          n === WHISKER_ANGLES_DEG.length
            ? WHISKER_ANGLES_DEG[i]
            : -90.0 + i * (180.0 / Math.max(n - 1, 1));
        const theta = (thetaDeg * Math.PI) / 180;
        const lengthFrac = Math.max(0, Math.min(1, mm / WHISKER_RANGE_MM));
        const length = maxRayPx * lengthFrac;
        const sinT = Math.sin(theta);
        const cosT = Math.cos(theta);
        const ex = ox - length * sinT;
        const ey = oy - length * cosT;

        const col = whiskerColor(mm);
        ctx.strokeStyle = col;
        ctx.fillStyle = col;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(ox, oy);
        ctx.lineTo(ex, ey);
        ctx.stroke();

        ctx.beginPath();
        ctx.arc(ex, ey, 3, 0, Math.PI * 2);
        ctx.fill();

        ctx.fillStyle = LABEL_FILL;
        ctx.font = "8px Consolas, monospace";
        const nudge = 10;
        ctx.fillText(
          `${Math.round(mm)}`,
          ex - nudge * sinT,
          ey - nudge * cosT,
        );

        if (target && target.length === n) {
          const tmm = target[i];
          if (tmm < WHISKER_RANGE_MM - 1.0) {
            const tFrac = Math.max(0, Math.min(1, tmm / WHISKER_RANGE_MM));
            const tLen = maxRayPx * tFrac;
            drawTargetDiamond(ctx, ox - tLen * sinT, oy - tLen * cosT);
          }
        }
      }
    }

    // Chassis body.
    ctx.fillStyle = CHASSIS_FILL;
    ctx.strokeStyle = CHASSIS_STROKE;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.rect(cx - bodyW / 2, robotTop, bodyW, bodyH);
    ctx.fill();
    ctx.stroke();

    // Wheels at the four body corners.
    const wheelPositions: Array<[number, number]> = [
      [cx - bodyW / 2, robotTop],
      [cx + bodyW / 2, robotTop],
      [cx - bodyW / 2, robotBottom],
      [cx + bodyW / 2, robotBottom],
    ];
    for (const [wx, wy] of wheelPositions) {
      ctx.fillStyle = WHEEL_FILL;
      ctx.strokeStyle = WHEEL_STROKE;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.arc(wx, wy, wheelR, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }

    // X (forward, red) and Y (left, green) axes from the body centre.
    const axisLen = bodyW * 0.32;
    ctx.lineWidth = 2;
    ctx.strokeStyle = AXIS_X_COLOR;
    ctx.beginPath();
    ctx.moveTo(bodyCx, bodyCy);
    ctx.lineTo(bodyCx, bodyCy - axisLen);
    ctx.stroke();
    ctx.strokeStyle = AXIS_Y_COLOR;
    ctx.beginPath();
    ctx.moveTo(bodyCx, bodyCy);
    ctx.lineTo(bodyCx - axisLen, bodyCy);
    ctx.stroke();

    // Heading arrow.
    const heading = headingRef.current;
    if (heading !== undefined) {
      const theta = (heading * Math.PI) / 180;
      const arrowLen = maxRayPx * 1.05;
      const ex = Math.max(6, Math.min(areaW - 6, bodyCx - arrowLen * Math.sin(theta)));
      const ey = Math.max(6, Math.min(areaH - 6, bodyCy - arrowLen * Math.cos(theta)));
      ctx.strokeStyle =
        modeRef.current === "visual" ? HEADING_COLOR_VISUAL : HEADING_COLOR_DEAD_RECK;
      ctx.lineWidth = 5;
      ctx.lineCap = "round";
      ctx.beginPath();
      ctx.moveTo(bodyCx, bodyCy);
      ctx.lineTo(ex, ey);
      ctx.stroke();
      // Arrowhead.
      const ah = 10;
      const dx = ex - bodyCx;
      const dy = ey - bodyCy;
      const len = Math.max(Math.hypot(dx, dy), 1);
      const ux = dx / len;
      const uy = dy / len;
      const leftX = ex - ah * ux + ah * 0.5 * uy;
      const leftY = ey - ah * uy - ah * 0.5 * ux;
      const rightX = ex - ah * ux - ah * 0.5 * uy;
      const rightY = ey - ah * uy + ah * 0.5 * ux;
      ctx.beginPath();
      ctx.moveTo(ex, ey);
      ctx.lineTo(leftX, leftY);
      ctx.lineTo(rightX, rightY);
      ctx.closePath();
      ctx.fillStyle = ctx.strokeStyle;
      ctx.fill();
      ctx.lineCap = "butt";
    }
  }, [renderTick]);

  return (
    <div
      ref={wrapRef}
      style={{
        width: "100%",
        height: "100%",
        background: BACKGROUND_FILL,
      }}
    >
      <canvas ref={canvasRef} />
    </div>
  );
}

export function initWhiskerFanPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<WhiskerFanPanel context={context} />);
  return () => {
    root.unmount();
  };
}
