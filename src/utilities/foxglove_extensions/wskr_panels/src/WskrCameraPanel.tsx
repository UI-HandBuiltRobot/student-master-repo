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

// ── message types ──────────────────────────────────────────────────────

type CompressedImageMsg = {
  format: string;
  data: Uint8Array | ArrayBuffer | number[] | string;
};

type Float32MultiArrayMsg = { data: number[] | Float32Array };

type TrackedBboxMsg = {
  x_norm: number;
  y_norm: number;
  w_norm: number;
  h_norm: number;
  source: string;
};

type ImgDetectionDataMsg = {
  image_width: number;
  image_height: number;
  inference_time: number;
  detection_ids: string[];
  x: number[];
  y: number[];
  width: number[];
  height: number[];
  class_name: string[];
  confidence: number[];
};

type ApproachTargetInfoMsg = {
  class_name: string;
  track_id: number;
  target_type: number; // 0 = TOY, 1 = BOX
  active: boolean;
};

type ArucoMarker = {
  id: number;
  corners: Array<{ x: number; y: number }>;
};

type TrackedBbox = { x: number; y: number; w: number; h: number };

// ── config ─────────────────────────────────────────────────────────────

interface PanelConfig {
  cameraTopic: string;
  markersTopic: string;
  trackedBboxTopic: string;
  detectionsTopic: string;
  selectedTopic: string;
  targetInfoTopic: string;
  targetArucoId: number;
  showHud: boolean;
}

const DEFAULT_CONFIG: PanelConfig = {
  cameraTopic: "/camera1/throttled/compressed",
  markersTopic: "/WSKR/aruco_markers",
  trackedBboxTopic: "/WSKR/tracked_bbox",
  detectionsTopic: "/vision/yolo/detections",
  selectedTopic: "/vision/selected_object",
  targetInfoTopic: "/WSKR/approach_target_info",
  targetArucoId: 1,
  showHud: true,
};

// ── YOLO drawing constants ─────────────────────────────────────────────

const TARGET_TYPE_TOY = 0;
const TARGET_TYPE_BOX = 1;

const COLOR_YOLO_BOX = "#ffffff";
const COLOR_YOLO_SELECTED = "#ffff00";
const COLOR_YOLO_SELECTED_FILL = "rgba(255, 255, 0, 0.25)";

// ── helpers ────────────────────────────────────────────────────────────

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

// ── panel ──────────────────────────────────────────────────────────────

function WskrCameraPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const wrapRef = useRef<HTMLDivElement>(null);

  const [config, setConfig] = useState<PanelConfig>(() => {
    const saved = context.initialState as Partial<PanelConfig> | undefined;
    return { ...DEFAULT_CONFIG, ...(saved ?? {}) };
  });

  // Latest message state — kept in refs so the draw function is always current.
  const latestImageRef = useRef<CompressedImageMsg | undefined>(undefined);
  const latestMarkersRef = useRef<ArucoMarker[]>([]);
  const latestBboxRef = useRef<TrackedBbox | undefined>(undefined);
  const latestDetectionsRef = useRef<ImgDetectionDataMsg | undefined>(undefined);
  const latestSelectedRef = useRef<ImgDetectionDataMsg | undefined>(undefined);
  const targetInfoRef = useRef<ApproachTargetInfoMsg>({ class_name: "", track_id: -1, target_type: 0, active: false });

  // JPEG decode cache — async decode keyed to the message object so concurrent
  // effect runs never spawn duplicate decoders for the same frame.
  const imageCacheRef = useRef<{ msg: CompressedImageMsg; bitmap: ImageBitmap } | undefined>(undefined);
  const decodingForRef = useRef<CompressedImageMsg | undefined>(undefined);

  // Diagnostic counters.
  const framesSeenRef = useRef(0);
  const decodeOkRef = useRef(0);
  const decodeFailRef = useRef(0);
  const lastDecodeErrRef = useRef("");
  const markerMsgsRef = useRef(0);
  const detectionMsgsRef = useRef(0);

  const unmountedRef = useRef(false);
  useEffect(() => () => { unmountedRef.current = true; }, []);

  // ── settings editor ──────────────────────────────────────────────────

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
          label: "WSKR Camera",
          fields: {
            cameraTopic:    { label: "Camera topic",          input: "string",  value: config.cameraTopic },
            markersTopic:   { label: "ArUco markers topic",   input: "string",  value: config.markersTopic },
            trackedBboxTopic: { label: "Tracked bbox topic",  input: "string",  value: config.trackedBboxTopic },
            detectionsTopic:  { label: "YOLO detections",     input: "string",  value: config.detectionsTopic },
            selectedTopic:    { label: "Selected object",      input: "string",  value: config.selectedTopic },
            targetInfoTopic:  { label: "Approach target info", input: "string",  value: config.targetInfoTopic },
            targetArucoId:  { label: "Target ArUco ID",       input: "number",  value: config.targetArucoId, step: 1 },
            showHud:        { label: "Show diagnostic HUD",   input: "boolean", value: config.showHud },
          },
        },
      },
    });
  }, [context, config]);

  // ── subscriptions ─────────────────────────────────────────────────────

  useEffect(() => {
    context.subscribe([
      { topic: config.cameraTopic },
      { topic: config.markersTopic },
      { topic: config.trackedBboxTopic },
      { topic: config.detectionsTopic },
      { topic: config.selectedTopic },
      { topic: config.targetInfoTopic },
    ]);
    return () => { context.unsubscribeAll(); };
  }, [
    context,
    config.cameraTopic,
    config.markersTopic,
    config.trackedBboxTopic,
    config.detectionsTopic,
    config.selectedTopic,
    config.targetInfoTopic,
  ]);

  // ── message ingestion ─────────────────────────────────────────────────

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
            markerMsgsRef.current += 1;
          } else if (msg.topic === config.trackedBboxTopic) {
            const m = (msg as MessageEvent<TrackedBboxMsg>).message;
            latestBboxRef.current = m.source
              ? { x: Number(m.x_norm), y: Number(m.y_norm), w: Number(m.w_norm), h: Number(m.h_norm) }
              : undefined;
          } else if (msg.topic === config.detectionsTopic) {
            latestDetectionsRef.current = (msg as MessageEvent<ImgDetectionDataMsg>).message;
            detectionMsgsRef.current += 1;
          } else if (msg.topic === config.selectedTopic) {
            latestSelectedRef.current = (msg as MessageEvent<ImgDetectionDataMsg>).message;
          } else if (msg.topic === config.targetInfoTopic) {
            targetInfoRef.current = (msg as MessageEvent<ApproachTargetInfoMsg>).message;
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
    config.detectionsTopic,
    config.selectedTopic,
    config.targetInfoTopic,
  ]);

  // ── draw loop ─────────────────────────────────────────────────────────

  useEffect(() => {
    const canvas = canvasRef.current;
    const wrap = wrapRef.current;
    if (!canvas || !wrap) return;

    const msg = latestImageRef.current;
    if (msg !== undefined && imageCacheRef.current?.msg !== msg && decodingForRef.current !== msg) {
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
          const blob = new Blob([bytes as BlobPart], { type: `image/${msg.format || "jpeg"}` });
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

    // ── draw ───────────────────────────────────────────────────────────

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
        ctx.fillStyle = lastDecodeErrRef.current ? "#ff6060" : "#a0a0a0";
        ctx.font = LABEL_FONT;
        ctx.fillText(
          lastDecodeErrRef.current
            ? "JPEG decode failed — see HUD."
            : `Waiting for ${config.cameraTopic}…`,
          12, 24,
        );
        drawHud(ctx, wrapW);
        return;
      }

      // Fit-letterbox the bitmap into the canvas.
      const bitmap = cache.bitmap;
      const scale = Math.min(wrapW / bitmap.width, wrapH / bitmap.height);
      const drawW = Math.max(1, Math.floor(bitmap.width * scale));
      const drawH = Math.max(1, Math.floor(bitmap.height * scale));
      const offX = Math.floor((wrapW - drawW) / 2);
      const offY = Math.floor((wrapH - drawH) / 2);
      ctx.drawImage(bitmap, offX, offY, drawW, drawH);

      // ── mode-switch rendering ──────────────────────────────────────

      const info = targetInfoRef.current;
      const approachActive = info.active;

      if (!approachActive) {
        // IDLE — show YOLO detections and ArUco markers.
        drawYoloDetections(ctx, offX, offY, drawW, drawH);
        drawArucoMarkers(ctx, offX, offY, drawW, /*serverTracking=*/false);
      } else if (info.target_type === TARGET_TYPE_TOY) {
        // TOY approach — tracked bbox only.
        drawTrackedBbox(ctx, offX, offY, drawW);
      } else if (info.target_type === TARGET_TYPE_BOX) {
        // BOX approach — ArUco markers + tracked bbox.
        drawArucoMarkers(ctx, offX, offY, drawW, /*serverTracking=*/latestBboxRef.current !== undefined);
        drawTrackedBbox(ctx, offX, offY, drawW);
      }

      drawHud(ctx, wrapW);
    }

    // ── layer: YOLO detections ─────────────────────────────────────────

    function drawYoloDetections(
      ctx: CanvasRenderingContext2D,
      offX: number, offY: number,
      drawW: number, drawH: number,
    ): void {
      const dets = latestDetectionsRef.current;
      if (!dets?.x?.length) return;

      const yoloW = dets.image_width || 640;
      const yoloH = dets.image_height || 640;
      const sx = drawW / yoloW;
      const sy = drawH / yoloH;

      const sel = latestSelectedRef.current;
      const selectedId = sel?.detection_ids?.[0] ?? undefined;

      const n = Math.min(dets.x.length, dets.y.length, dets.width.length, dets.height.length);
      let pick: { x1: number; y1: number; w: number; h: number; label: string } | undefined;

      for (let i = 0; i < n; i++) {
        const cx = Number(dets.x[i]);
        const cy = Number(dets.y[i]);
        const bw = Number(dets.width[i]);
        const bh = Number(dets.height[i]);
        const x1 = offX + (cx - bw / 2) * sx;
        const y1 = offY + (cy - bh / 2) * sy;
        const rw = bw * sx;
        const rh = bh * sy;

        const cls   = dets.class_name?.[i] ?? "";
        const conf  = Number(dets.confidence?.[i] ?? 0);
        const detId = dets.detection_ids?.[i] ?? "";
        const label = `${cls} ${conf.toFixed(2)}`;

        if (selectedId !== undefined && String(detId) === String(selectedId)) {
          pick = { x1, y1, w: rw, h: rh, label: `[PICK] ${label}` };
        } else {
          ctx.strokeStyle = COLOR_YOLO_BOX;
          ctx.lineWidth = 1;
          ctx.strokeRect(x1, y1, rw, rh);
          ctx.fillStyle = COLOR_YOLO_BOX;
          ctx.font = LABEL_FONT;
          ctx.fillText(label, x1, Math.max(12, y1 - 4));
        }
      }

      // Draw the pick last so it sits on top of all other boxes.
      if (pick) {
        ctx.fillStyle = COLOR_YOLO_SELECTED_FILL;
        ctx.fillRect(pick.x1, pick.y1, pick.w, pick.h);
        ctx.strokeStyle = COLOR_YOLO_SELECTED;
        ctx.lineWidth = 3;
        ctx.strokeRect(pick.x1, pick.y1, pick.w, pick.h);
        ctx.fillStyle = COLOR_YOLO_SELECTED;
        ctx.font = "bold 14px Consolas, monospace";
        ctx.fillText(pick.label, pick.x1, Math.max(14, pick.y1 - 6));
      }
    }

    // ── layer: ArUco markers ───────────────────────────────────────────

    function drawArucoMarkers(
      ctx: CanvasRenderingContext2D,
      offX: number, offY: number,
      drawW: number,
      serverTracking: boolean,
    ): void {
      for (const m of latestMarkersRef.current) {
        const isTarget = m.id === config.targetArucoId;
        const highlight = isTarget && !serverTracking;
        const color  = highlight ? COLOR_TARGET_HIGHLIGHT : COLOR_NON_TARGET;
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
    }

    // ── layer: server-tracked bbox ─────────────────────────────────────

    function drawTrackedBbox(
      ctx: CanvasRenderingContext2D,
      offX: number, offY: number,
      drawW: number,
    ): void {
      const bbox = latestBboxRef.current;
      if (!bbox) return;
      // tracked_bbox is width-normalized: multiply by drawW for both x and y.
      const x1 = offX + bbox.x * drawW;
      const y1 = offY + bbox.y * drawW;
      const x2 = offX + (bbox.x + bbox.w) * drawW;
      const y2 = offY + (bbox.y + bbox.h) * drawW;
      ctx.strokeStyle = COLOR_SERVER_TRACKING;
      ctx.fillStyle = COLOR_SERVER_TRACKING;
      ctx.lineWidth = 3;
      ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
      ctx.font = LABEL_FONT;
      ctx.fillText("TRACKING", x1, Math.max(14, y1 - 8));
    }

    // ── layer: HUD ────────────────────────────────────────────────────

    function drawHud(ctx: CanvasRenderingContext2D, w: number): void {
      if (!config.showHud) return;
      const info = targetInfoRef.current;
      const dets = latestDetectionsRef.current;
      const mode = info.active
        ? (info.target_type === TARGET_TYPE_TOY ? `TOY approach — ${info.class_name}` : "BOX approach")
        : "idle";
      const lines = [
        `mode: ${mode}`,
        `markers: ${latestMarkersRef.current.length}  (msgs: ${markerMsgsRef.current})`,
        `detections: ${dets?.x?.length ?? 0}  (msgs: ${detectionMsgsRef.current})`,
        `frames seen: ${framesSeenRef.current}  decoded: ${decodeOkRef.current}  fail: ${decodeFailRef.current}`,
      ];
      if (lastDecodeErrRef.current) lines.push(`decode err: ${lastDecodeErrRef.current}`);

      const padX = 6;
      const padY = 4;
      const lineH = 14;
      const boxH = padY * 2 + lineH * lines.length;
      const boxW = Math.min(w - 8, 560);
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
    config.cameraTopic,
    config.targetArucoId,
    config.showHud,
  ]);

  return (
    <div
      ref={wrapRef}
      style={{ width: "100%", height: "100%", background: "#000", position: "relative" }}
    >
      <canvas
        ref={canvasRef}
        style={{ width: "100%", height: "100%", display: "block" }}
      />
    </div>
  );
}

export function initWskrCameraPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<WskrCameraPanel context={context} />);
  return () => { root.unmount(); };
}
