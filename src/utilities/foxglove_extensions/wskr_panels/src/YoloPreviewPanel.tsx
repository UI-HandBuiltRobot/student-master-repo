import {
  PanelExtensionContext,
  MessageEvent,
  SettingsTreeAction,
} from "@foxglove/extension";
import { useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

// ── message types ──────────────────────────────────────────────────────

type CompressedImageMsg = {
  format: string;
  data: Uint8Array | ArrayBuffer | number[] | string;
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
  active: boolean;
};

// ── config ─────────────────────────────────────────────────────────────

interface PanelConfig {
  cameraTopic: string;
  detectionsTopic: string;
  selectedTopic: string;
  targetInfoTopic: string;
  showHud: boolean;
}

const DEFAULT_CONFIG: PanelConfig = {
  cameraTopic: "/camera1/throttled/compressed",
  detectionsTopic: "/vision/yolo/detections",
  selectedTopic: "/vision/selected_object",
  targetInfoTopic: "/WSKR/approach_target_info",
  showHud: true,
};

// ── drawing constants ──────────────────────────────────────────────────

const COLOR_BOX = "#ffffff";
const COLOR_SELECTED = "#ffff00";
const COLOR_SELECTED_FILL = "rgba(255, 255, 0, 0.25)";
const LABEL_FONT = "bold 13px Consolas, monospace";

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

// ── panel ──────────────────────────────────────────────────────────────

function YoloPreviewPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const wrapRef = useRef<HTMLDivElement>(null);

  const [config, setConfig] = useState<PanelConfig>(() => {
    const saved = context.initialState as Partial<PanelConfig> | undefined;
    return { ...DEFAULT_CONFIG, ...(saved ?? {}) };
  });

  const latestImageRef = useRef<CompressedImageMsg | undefined>(undefined);
  const latestDetectionsRef = useRef<ImgDetectionDataMsg | undefined>(undefined);
  const latestSelectedRef = useRef<ImgDetectionDataMsg | undefined>(undefined);
  const approachActiveRef = useRef(false);

  const imageCacheRef = useRef<{ msg: CompressedImageMsg; bitmap: ImageBitmap } | undefined>(undefined);
  const decodingForRef = useRef<CompressedImageMsg | undefined>(undefined);

  const framesSeenRef = useRef(0);
  const decodeOkRef = useRef(0);
  const decodeFailRef = useRef(0);
  const detectionMsgsRef = useRef(0);

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
          label: "YOLO Preview",
          fields: {
            cameraTopic: { label: "Camera topic", input: "string", value: config.cameraTopic },
            detectionsTopic: { label: "Detections topic", input: "string", value: config.detectionsTopic },
            selectedTopic: { label: "Selected object topic", input: "string", value: config.selectedTopic },
            targetInfoTopic: { label: "Approach target info", input: "string", value: config.targetInfoTopic },
            showHud: { label: "Show diagnostic HUD", input: "boolean", value: config.showHud },
          },
        },
      },
    });
  }, [context, config]);

  // Subscriptions.
  useEffect(() => {
    context.subscribe([
      { topic: config.cameraTopic },
      { topic: config.detectionsTopic },
      { topic: config.selectedTopic },
      { topic: config.targetInfoTopic },
    ]);
    return () => { context.unsubscribeAll(); };
  }, [context, config.cameraTopic, config.detectionsTopic, config.selectedTopic, config.targetInfoTopic]);

  const [renderTick, setRenderTick] = useState(0);

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      if (renderState.currentFrame) {
        for (const msg of renderState.currentFrame) {
          if (msg.topic === config.cameraTopic) {
            latestImageRef.current = (msg as MessageEvent<CompressedImageMsg>).message;
            framesSeenRef.current += 1;
          } else if (msg.topic === config.detectionsTopic) {
            latestDetectionsRef.current = (msg as MessageEvent<ImgDetectionDataMsg>).message;
            detectionMsgsRef.current += 1;
          } else if (msg.topic === config.selectedTopic) {
            latestSelectedRef.current = (msg as MessageEvent<ImgDetectionDataMsg>).message;
          } else if (msg.topic === config.targetInfoTopic) {
            approachActiveRef.current = (msg as MessageEvent<ApproachTargetInfoMsg>).message.active;
          }
        }
      }
      setRenderTick((t) => t + 1);
      done();
    };
    context.watch("currentFrame");
  }, [context, config.cameraTopic, config.detectionsTopic, config.selectedTopic, config.targetInfoTopic]);

  // Draw loop.
  useEffect(() => {
    let cancelled = false;
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
          if (decodingForRef.current === msg) decodingForRef.current = undefined;
          if (!cancelled) setRenderTick((t) => t + 1);
          return;
        }
        try {
          const blob = new Blob([bytes as BlobPart], { type: `image/${msg.format || "jpeg"}` });
          const bitmap = await createImageBitmap(blob);
          if (cancelled) { bitmap.close?.(); return; }
          imageCacheRef.current?.bitmap.close?.();
          imageCacheRef.current = { msg, bitmap };
          decodeOkRef.current += 1;
        } catch {
          decodeFailRef.current += 1;
        } finally {
          if (decodingForRef.current === msg) decodingForRef.current = undefined;
        }
        if (!cancelled) setRenderTick((t) => t + 1);
      })();
    }

    drawNow(canvas, wrap);
    return () => { cancelled = true; };

    function drawNow(canvas: HTMLCanvasElement, wrap: HTMLDivElement): void {
      const wrapW = Math.max(1, wrap.clientWidth);
      const wrapH = Math.max(1, wrap.clientHeight);
      canvas.width = wrapW;
      canvas.height = wrapH;
      const ctx = canvas.getContext("2d");
      if (!ctx) return;
      ctx.fillStyle = "#000";
      ctx.fillRect(0, 0, wrapW, wrapH);

      const cache = imageCacheRef.current;
      if (!cache) {
        ctx.fillStyle = "#a0a0a0";
        ctx.font = LABEL_FONT;
        ctx.fillText(`Waiting for ${config.cameraTopic}…`, 12, 24);
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

      if (!approachActiveRef.current) {
        drawDetections(ctx, offX, offY, drawW, drawH);
      }

      drawHud(ctx, wrapW);
    }

    function drawDetections(
      ctx: CanvasRenderingContext2D,
      offX: number,
      offY: number,
      drawW: number,
      drawH: number,
    ): void {
      const dets = latestDetectionsRef.current;
      if (!dets || !dets.x?.length) return;

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

        const cls = dets.class_name?.[i] ?? "";
        const conf = Number(dets.confidence?.[i] ?? 0);
        const detId = dets.detection_ids?.[i] ?? "";
        const label = `${cls} ${conf.toFixed(2)}`;

        if (selectedId !== undefined && String(detId) === String(selectedId)) {
          pick = { x1, y1, w: rw, h: rh, label: `[PICK] ${label}` };
        } else {
          ctx.strokeStyle = COLOR_BOX;
          ctx.lineWidth = 1;
          ctx.strokeRect(x1, y1, rw, rh);
          ctx.fillStyle = COLOR_BOX;
          ctx.font = LABEL_FONT;
          ctx.fillText(label, x1, Math.max(12, y1 - 4));
        }
      }

      if (pick) {
        ctx.fillStyle = COLOR_SELECTED_FILL;
        ctx.fillRect(pick.x1, pick.y1, pick.w, pick.h);
        ctx.strokeStyle = COLOR_SELECTED;
        ctx.lineWidth = 3;
        ctx.strokeRect(pick.x1, pick.y1, pick.w, pick.h);
        ctx.fillStyle = COLOR_SELECTED;
        ctx.font = "bold 14px Consolas, monospace";
        ctx.fillText(pick.label, pick.x1, Math.max(14, pick.y1 - 6));
      }
    }

    function drawHud(ctx: CanvasRenderingContext2D, w: number): void {
      if (!config.showHud) return;
      const dets = latestDetectionsRef.current;
      const n = dets?.x?.length ?? 0;
      const ms = dets?.inference_time?.toFixed(1) ?? "—";
      const lines = [
        `detections: ${n}  inference: ${ms} ms  (msgs: ${detectionMsgsRef.current})`,
        `frames seen: ${framesSeenRef.current}  decoded: ${decodeOkRef.current}  fail: ${decodeFailRef.current}`,
      ];
      if (approachActiveRef.current) {
        lines.unshift("APPROACH ACTIVE — detections hidden");
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
  }, [renderTick, config.cameraTopic, config.showHud]);

  return (
    <div ref={wrapRef} style={{ width: "100%", height: "100%", background: "#000", position: "relative" }}>
      <canvas ref={canvasRef} style={{ width: "100%", height: "100%", display: "block" }} />
    </div>
  );
}

export function initYoloPreviewPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<YoloPreviewPanel context={context} />);
  return () => { root.unmount(); };
}
