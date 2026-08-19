import {
  PanelExtensionContext,
  MessageEvent,
  SettingsTreeAction,
} from "@foxglove/extension";
import { useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

import {
  COLOR_NON_TARGET,
  COLOR_SERVER_TRACKING,
  COLOR_TARGET_HIGHLIGHT,
  HIGHLIGHT_STROKE_WIDTH,
  LABEL_FONT,
  NON_TARGET_STROKE_WIDTH,
} from "./arucoConfig";

interface PanelConfig {
  cameraTopic: string;
  markersTopic: string;
  trackedBboxTopic: string;
  targetArucoId: number;
  showHud: boolean;
}

const DEFAULT_CONFIG: PanelConfig = {
  cameraTopic: "/camera1/throttled/compressed",
  markersTopic: "/WSKR/aruco_markers",
  trackedBboxTopic: "/WSKR/tracked_bbox",
  targetArucoId: 1,
  showHud: true,
};

type CompressedImageMsg = {
  format: string;
  data: Uint8Array | ArrayBuffer | number[] | string;
};

type Float32MultiArrayMsg = {
  data: number[] | Float32Array;
};

type TrackedBboxMsg = {
  x_norm: number;
  y_norm: number;
  w_norm: number;
  h_norm: number;
  source: string;
};

type TrackedBbox = {
  x: number; // normalized by frame width
  y: number;
  w: number;
  h: number;
};

type ArucoMarker = {
  id: number;
  // Corners are width-normalized fractions (same convention as tracked_bbox):
  // both x and y components were divided by the source frame's width on the
  // publisher side, so multiplying by the displayed drawW yields pixel coords
  // within the fitted image.
  corners: Array<{ x: number; y: number }>;
};

// Foxglove's ROS bridges deliver message fields in a handful of shapes
// depending on version and schema — normalize to a Uint8Array so the rest
// of the pipeline can stop caring.
function normalizeImageData(raw: unknown): Uint8Array | undefined {
  if (raw == null) return undefined;
  if (raw instanceof Uint8Array) return raw;
  if (raw instanceof ArrayBuffer) return new Uint8Array(raw);
  if (ArrayBuffer.isView(raw)) {
    const v = raw as ArrayBufferView;
    return new Uint8Array(v.buffer, v.byteOffset, v.byteLength);
  }
  if (Array.isArray(raw)) return new Uint8Array(raw as number[]);
  if (typeof raw === "string") {
    try {
      const bin = atob(raw);
      const arr = new Uint8Array(bin.length);
      for (let i = 0; i < bin.length; i++) arr[i] = bin.charCodeAt(i);
      return arr;
    } catch {
      return undefined;
    }
  }
  return undefined;
}

// Parse the flat Float32MultiArray layout published by the Jetson-side
// detector: [id, x1, y1, x2, y2, x3, y3, x4, y4, id, x1, ..., y4, ...].
function parseMarkers(msg: Float32MultiArrayMsg): ArucoMarker[] {
  const data = msg.data ?? [];
  const markers: ArucoMarker[] = [];
  const stride = 9;
  for (let base = 0; base + stride <= data.length; base += stride) {
    markers.push({
      id: Math.round(Number(data[base])),
      corners: [
        { x: Number(data[base + 1]), y: Number(data[base + 2]) },
        { x: Number(data[base + 3]), y: Number(data[base + 4]) },
        { x: Number(data[base + 5]), y: Number(data[base + 6]) },
        { x: Number(data[base + 7]), y: Number(data[base + 8]) },
      ],
    });
  }
  return markers;
}

function CameraArucoPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const wrapRef = useRef<HTMLDivElement>(null);

  const [config, setConfig] = useState<PanelConfig>(() => {
    const saved = context.initialState as Partial<PanelConfig> | undefined;
    return { ...DEFAULT_CONFIG, ...(saved ?? {}) };
  });

  const latestImageRef = useRef<CompressedImageMsg | undefined>(undefined);
  const latestMarkersRef = useRef<ArucoMarker[]>([]);
  const latestBboxRef = useRef<TrackedBbox | undefined>(undefined);

  // Cache the most recently decoded JPEG so redraws triggered by marker
  // updates (at the full marker-publish rate) can draw synchronously and
  // not flicker the canvas to black while an async decode is in flight.
  // The decoded ImageBitmap stays alive until the image *message* changes.
  const imageCacheRef = useRef<{
    msg: CompressedImageMsg;
    bitmap: ImageBitmap;
  } | undefined>(undefined);
  // Which message we're currently decoding, so concurrent effect runs don't
  // spawn parallel decoders for the same frame.
  const decodingForRef = useRef<CompressedImageMsg | undefined>(undefined);

  // Diagnostic counters rendered on the HUD overlay.
  const framesSeenRef = useRef(0);
  const markerMsgsSeenRef = useRef(0);
  const decodeOkRef = useRef(0);
  const decodeFailRef = useRef(0);
  const lastDecodeErrRef = useRef<string>("");

  const unmountedRef = useRef(false);
  useEffect(() => () => { unmountedRef.current = true; }, []);

  // Settings editor.
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
          label: "Camera + ArUco",
          fields: {
            cameraTopic: {
              label: "Camera topic",
              input: "string",
              value: config.cameraTopic,
            },
            markersTopic: {
              label: "Markers topic",
              input: "string",
              value: config.markersTopic,
            },
            trackedBboxTopic: {
              label: "Tracked bbox topic",
              input: "string",
              value: config.trackedBboxTopic,
            },
            targetArucoId: {
              label: "Target ArUco ID",
              input: "number",
              value: config.targetArucoId,
              step: 1,
            },
            showHud: {
              label: "Show diagnostic HUD",
              input: "boolean",
              value: config.showHud,
            },
          },
        },
      },
    });
  }, [context, config]);

  useEffect(() => {
    context.subscribe([
      { topic: config.cameraTopic },
      { topic: config.markersTopic },
      { topic: config.trackedBboxTopic },
    ]);
    return () => {
      context.unsubscribeAll();
    };
  }, [
    context,
    config.cameraTopic,
    config.markersTopic,
    config.trackedBboxTopic,
  ]);

  const [renderTick, setRenderTick] = useState(0);
  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      if (renderState.currentFrame) {
        for (const msg of renderState.currentFrame) {
          if (msg.topic === config.cameraTopic) {
            latestImageRef.current = (msg as MessageEvent<CompressedImageMsg>).message;
            framesSeenRef.current += 1;
          } else if (msg.topic === config.markersTopic) {
            latestMarkersRef.current = parseMarkers(
              (msg as MessageEvent<Float32MultiArrayMsg>).message,
            );
            markerMsgsSeenRef.current += 1;
          } else if (msg.topic === config.trackedBboxTopic) {
            const bboxMsg = (msg as MessageEvent<TrackedBboxMsg>).message;
            if (bboxMsg.source) {
              latestBboxRef.current = {
                x: Number(bboxMsg.x_norm), y: Number(bboxMsg.y_norm),
                w: Number(bboxMsg.w_norm), h: Number(bboxMsg.h_norm),
              };
            } else {
              latestBboxRef.current = undefined;
            }
          }
        }
      }
      setRenderTick((t) => t + 1);
      done();
    };
    context.watch("currentFrame");
  }, [
    context,
    config.cameraTopic,
    config.markersTopic,
    config.trackedBboxTopic,
  ]);

  // Draw loop: runs synchronously whenever renderTick changes. The JPEG
  // decode happens asynchronously only when the image *message* actually
  // changes; in between, every redraw reuses the cached ImageBitmap so
  // marker-only redraws at 10 Hz don't flash the canvas to black while
  // awaiting a fresh decode.
  useEffect(() => {
    const canvas = canvasRef.current;
    const wrap = wrapRef.current;
    if (!canvas || !wrap) return;

    const msg = latestImageRef.current;
    if (
      msg !== undefined &&
      imageCacheRef.current?.msg !== msg &&
      decodingForRef.current !== msg
    ) {
      decodingForRef.current = msg;
      (async () => {
        const bytes = normalizeImageData(msg.data);
        if (!bytes || bytes.byteLength === 0) {
          decodeFailRef.current += 1;
          lastDecodeErrRef.current = `empty/unknown data (typeof=${typeof msg.data})`;
          if (decodingForRef.current === msg) decodingForRef.current = undefined;
          if (!unmountedRef.current) setRenderTick((t) => t + 1);
          return;
        }
        try {
          const blob = new Blob([bytes as BlobPart], {
            type: `image/${msg.format || "jpeg"}`,
          });
          const bitmap = await createImageBitmap(blob);
          if (unmountedRef.current) { bitmap.close?.(); return; }
          imageCacheRef.current?.bitmap.close?.();
          imageCacheRef.current = { msg, bitmap };
          decodeOkRef.current += 1;
        } catch (err) {
          decodeFailRef.current += 1;
          lastDecodeErrRef.current = String(err);
        } finally {
          if (decodingForRef.current === msg) decodingForRef.current = undefined;
        }
        if (!unmountedRef.current) setRenderTick((t) => t + 1);
      })();
    }

    drawNow(canvas, wrap);

    function drawNow(canvas: HTMLCanvasElement, wrap: HTMLDivElement): void {
      const wrapW = Math.max(1, wrap.clientWidth);
      const wrapH = Math.max(1, wrap.clientHeight);
      if (canvas.width !== wrapW) canvas.width = wrapW;
      if (canvas.height !== wrapH) canvas.height = wrapH;
      const ctx = canvas.getContext("2d");
      if (!ctx) return;
      ctx.fillStyle = "#000";
      ctx.fillRect(0, 0, wrapW, wrapH);

      const cache = imageCacheRef.current;
      if (!cache) {
        // No image yet — or the very first decode hasn't finished.
        ctx.fillStyle = lastDecodeErrRef.current ? "#ff6060" : "#a0a0a0";
        ctx.font = LABEL_FONT;
        ctx.fillText(
          lastDecodeErrRef.current
            ? "JPEG decode failed — see HUD."
            : `Waiting for ${config.cameraTopic}…`,
          12,
          24,
        );
        drawHud(ctx, wrapW);
        return;
      }

      const bitmap = cache.bitmap;
      const scale = Math.min(wrapW / bitmap.width, wrapH / bitmap.height);
      const drawW = Math.max(1, Math.floor(bitmap.width * scale));
      const drawH = Math.max(1, Math.floor(bitmap.height * scale));
      const offX = Math.floor((wrapW - drawW) / 2);
      const offY = Math.floor((wrapH - drawH) / 2);
      ctx.drawImage(bitmap, offX, offY, drawW, drawH);

      // Draw ArUco markers received from /WSKR/aruco_markers.
      const serverTracking = latestBboxRef.current !== undefined;
      for (const m of latestMarkersRef.current) {
        const isTarget = m.id === config.targetArucoId;
        const highlight = isTarget && !serverTracking;
        const color = highlight ? COLOR_TARGET_HIGHLIGHT : COLOR_NON_TARGET;
        const stroke = highlight ? HIGHLIGHT_STROKE_WIDTH : NON_TARGET_STROKE_WIDTH;
        ctx.strokeStyle = color;
        ctx.fillStyle = color;
        ctx.lineWidth = stroke;
        ctx.font = LABEL_FONT;
        const pts = m.corners.map((p) => ({
          x: offX + p.x * drawW,
          y: offY + p.y * drawW,
        }));
        ctx.beginPath();
        ctx.moveTo(pts[0].x, pts[0].y);
        for (let i = 1; i < 4; i++) ctx.lineTo(pts[i].x, pts[i].y);
        ctx.closePath();
        ctx.stroke();
        const xMin = Math.min(...pts.map((p) => p.x));
        const yMin = Math.min(...pts.map((p) => p.y));
        ctx.fillText(
          `ID:${m.id}${highlight ? " [TARGET]" : ""}`,
          xMin,
          Math.max(14, yMin - 6),
        );
      }

      // Server-tracked bbox (cyan). tracked_bbox is width-normalized:
      // both x and w were divided by the source frame width on the publisher
      // side, so multiplying by drawW gives pixel coords within the fitted image.
      const bbox = latestBboxRef.current;
      if (bbox) {
        const x1 = offX + bbox.x * drawW;
        const y1 = offY + bbox.y * drawW;
        const x2 = offX + (bbox.x + bbox.w) * drawW;
        const y2 = offY + (bbox.y + bbox.h) * drawW;
        ctx.strokeStyle = COLOR_SERVER_TRACKING;
        ctx.fillStyle = COLOR_SERVER_TRACKING;
        ctx.lineWidth = 3;
        ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
        ctx.fillText("TRACKING", x1, Math.max(14, y1 - 8));
      }

      drawHud(ctx, wrapW);
    }

    function drawHud(ctx: CanvasRenderingContext2D, w: number): void {
      if (!config.showHud) return;
      const lines = [
        `camera: ${config.cameraTopic}`,
        `markers: ${config.markersTopic}  (msgs: ${markerMsgsSeenRef.current}, last N: ${latestMarkersRef.current.length})`,
        `frames seen: ${framesSeenRef.current}  decoded: ${decodeOkRef.current}  decode-fail: ${decodeFailRef.current}`,
      ];
      if (lastDecodeErrRef.current) {
        lines.push(`last decode err: ${lastDecodeErrRef.current}`);
      }
      const padX = 6;
      const padY = 4;
      const lineH = 14;
      const boxH = padY * 2 + lineH * lines.length;
      const boxW = Math.min(w - 8, 540);
      ctx.fillStyle = "rgba(0,0,0,0.6)";
      ctx.fillRect(4, 4, boxW, boxH);
      ctx.fillStyle = "#d0d0d0";
      ctx.font = "11px Consolas, monospace";
      for (let i = 0; i < lines.length; i++) {
        ctx.fillText(lines[i], 4 + padX, 4 + padY + (i + 1) * lineH - 3);
      }
    }
  }, [
    renderTick,
    config.targetArucoId,
    config.cameraTopic,
    config.markersTopic,
    config.showHud,
  ]);

  return (
    <div
      ref={wrapRef}
      style={{
        width: "100%",
        height: "100%",
        background: "#000",
        position: "relative",
      }}
    >
      <canvas
        ref={canvasRef}
        style={{
          width: "100%",
          height: "100%",
          display: "block",
        }}
      />
    </div>
  );
}

export function initCameraArucoPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<CameraArucoPanel context={context} />);
  return () => {
    root.unmount();
  };
}
