import {
  PanelExtensionContext,
  MessageEvent,
  SettingsTreeAction,
} from "@foxglove/extension";
import { useCallback, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

// ── message types ──────────────────────────────────────────────────────

type StringMsg = { data: string };

// ── config ─────────────────────────────────────────────────────────────

interface PanelConfig {
  knownModels: string;        // comma-separated list of model filenames
  activeModelTopic: string;
  modelFilenameTopic: string;
  speedScaleTopic: string;
}

const DEFAULT_CONFIG: PanelConfig = {
  knownModels: "model_ppo_v2_002.json",
  activeModelTopic: "/WSKR/autopilot/active_model",
  modelFilenameTopic: "/WSKR/autopilot/model_filename",
  speedScaleTopic: "/WSKR/autopilot/speed_scale",
};

// ── styles ─────────────────────────────────────────────────────────────

const CONTAINER: React.CSSProperties = {
  width: "100%", height: "100%",
  background: "#1e1e1e", color: "#e0e0e0",
  fontFamily: "Consolas, monospace", fontSize: 13,
  padding: 12, boxSizing: "border-box",
  display: "flex", flexDirection: "column", gap: 10, overflow: "auto",
};
const SECTION: React.CSSProperties = {
  margin: 0, fontSize: 13, color: "#80d0ff",
  borderBottom: "1px solid #333", paddingBottom: 3,
  textTransform: "uppercase", letterSpacing: "0.05em",
};
const DIVIDER: React.CSSProperties = { borderTop: "1px solid #333", margin: "2px 0" };
const ROW: React.CSSProperties    = { display: "flex", gap: 8, alignItems: "center" };
const LABEL: React.CSSProperties  = { color: "#888", minWidth: 72 };
const VALUE: React.CSSProperties  = { color: "#e0e0e0" };
const MUTED: React.CSSProperties  = { color: "#666", fontSize: 11, wordBreak: "break-all" };
const BTN_BASE: React.CSSProperties = {
  fontFamily: "Consolas, monospace", fontSize: 13, fontWeight: "bold",
  border: "none", borderRadius: 4, padding: "7px 16px", cursor: "pointer",
};
const BTN_GO:  React.CSSProperties = { ...BTN_BASE, background: "#2e7d32", color: "#fff" };
const BTN_OFF: React.CSSProperties = { ...BTN_BASE, background: "#555",    color: "#999", cursor: "default" };

const SELECT_STYLE: React.CSSProperties = {
  fontFamily: "Consolas, monospace", fontSize: 13,
  background: "#2a2a2a", color: "#e0e0e0",
  border: "1px solid #555", borderRadius: 4, padding: "4px 6px", flex: 1,
};

const SLIDER_WRAP: React.CSSProperties = { display: "flex", gap: 8, alignItems: "center", flex: 1 };
const SLIDER: React.CSSProperties = { flex: 1, accentColor: "#4caf50" };
const SCALE_VAL: React.CSSProperties = { color: "#4caf50", fontWeight: "bold", minWidth: 36, textAlign: "right" };

const STATUS_MSG: React.CSSProperties = {
  color: "#aaa", fontSize: 11, marginTop: 4,
  whiteSpace: "pre-wrap", wordBreak: "break-word",
  maxHeight: 56, overflow: "auto",
};

// ── panel ──────────────────────────────────────────────────────────────

function AutopilotPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [config, setConfig] = useState<PanelConfig>(() => ({
    ...DEFAULT_CONFIG,
    ...((context.initialState as Partial<PanelConfig>) ?? {}),
  }));

  const [selectedModel, setSelectedModel] = useState("");
  const [activeModel,   setActiveModel]   = useState<string | undefined>(undefined);
  const [speedScale,    setSpeedScale]    = useState(0.25);
  const [statusMsg,     setStatusMsg]     = useState("");
  const [busy,          setBusy]          = useState(false);

  const modelOptions = config.knownModels
    .split(",")
    .map((s) => s.trim())
    .filter(Boolean);

  // ── settings ──────────────────────────────────────────────────────────

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
          label: "Autopilot Panel",
          fields: {
            knownModels:       { label: "Known models (comma-sep)",  input: "string", value: config.knownModels },
            activeModelTopic:  { label: "Active model topic",        input: "string", value: config.activeModelTopic },
            modelFilenameTopic:{ label: "Load model topic",          input: "string", value: config.modelFilenameTopic },
            speedScaleTopic:   { label: "Speed scale topic",         input: "string", value: config.speedScaleTopic },
          },
        },
      },
    });
  }, [context, config]);

  // ── advertise publish topics ──────────────────────────────────────────

  useEffect(() => {
    context.advertise?.(config.modelFilenameTopic, "std_msgs/msg/String");
    context.advertise?.(config.speedScaleTopic,    "std_msgs/msg/Float32");
    return () => {
      context.unadvertise?.(config.modelFilenameTopic);
      context.unadvertise?.(config.speedScaleTopic);
    };
  }, [context, config.modelFilenameTopic, config.speedScaleTopic]);

  // ── subscriptions ─────────────────────────────────────────────────────

  useEffect(() => {
    context.subscribe([{ topic: config.activeModelTopic }]);
    return () => { context.unsubscribeAll(); };
  }, [context, config.activeModelTopic]);

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      if (renderState.currentFrame) {
        for (const msg of renderState.currentFrame) {
          if (msg.topic === config.activeModelTopic) {
            const s = (msg as MessageEvent<StringMsg>).message;
            setActiveModel(s.data || "");
          }
        }
      }
      done();
    };
    context.watch("currentFrame");
  }, [context, config.activeModelTopic]);

  // ── actions ───────────────────────────────────────────────────────────

  const handleLoad = useCallback(() => {
    const name = selectedModel || modelOptions[0] || "";
    if (!name) { setStatusMsg("No model selected."); return; }
    if (!context.publish) { setStatusMsg("context.publish not available in this Foxglove version."); return; }
    setBusy(true);
    setStatusMsg(`Loading ${name}…`);
    try {
      context.publish(config.modelFilenameTopic, { data: name });
      setStatusMsg(`Load requested: ${name}`);
    } catch (err) {
      setStatusMsg(`Publish failed: ${err}`);
    } finally {
      setBusy(false);
    }
  }, [context, config.modelFilenameTopic, selectedModel, modelOptions]);

  const handleSpeedChange = useCallback((e: React.ChangeEvent<HTMLInputElement>) => {
    const v = parseFloat(e.target.value);
    setSpeedScale(v);
    context.publish?.(config.speedScaleTopic, { data: v });
  }, [context, config.speedScaleTopic]);

  // ── render ────────────────────────────────────────────────────────────

  const activeBasename = activeModel
    ? activeModel.split(/[\\/]/).pop() ?? activeModel
    : undefined;

  const canLoad = !busy && (selectedModel !== "" || modelOptions.length > 0);

  return (
    <div style={CONTAINER}>

      {/* ── Model ── */}
      <div style={SECTION}>Autopilot Model</div>

      <div style={ROW}>
        <span style={LABEL}>Select:</span>
        <select
          style={SELECT_STYLE}
          value={selectedModel || modelOptions[0] || ""}
          onChange={(e) => setSelectedModel(e.target.value)}
        >
          {modelOptions.length === 0
            ? <option value="">— add models in settings —</option>
            : modelOptions.map((m) => <option key={m} value={m}>{m}</option>)
          }
        </select>
        <button
          style={canLoad ? BTN_GO : BTN_OFF}
          disabled={!canLoad}
          onClick={handleLoad}
        >
          LOAD
        </button>
      </div>

      <div style={ROW}>
        <span style={LABEL}>Active:</span>
        <div>
          <div style={VALUE}>{activeBasename ?? "—"}</div>
          {activeModel && activeBasename !== activeModel && (
            <div style={MUTED}>{activeModel}</div>
          )}
        </div>
      </div>

      {statusMsg && <div style={STATUS_MSG}>{statusMsg}</div>}

      <div style={DIVIDER} />

      {/* ── Speed Scale ── */}
      <div style={SECTION}>Speed Scale</div>

      <div style={ROW}>
        <span style={LABEL}>Scale:</span>
        <div style={SLIDER_WRAP}>
          <input
            type="range"
            min={0} max={1} step={0.01}
            value={speedScale}
            onChange={handleSpeedChange}
            style={SLIDER}
          />
          <span style={SCALE_VAL}>{speedScale.toFixed(2)}</span>
        </div>
      </div>

      <div style={{ ...ROW, fontSize: 11, color: "#666" }}>
        <span style={{ minWidth: 72 }} />
        <span>0.0 = stopped&nbsp;&nbsp;&nbsp;1.0 = full policy output</span>
      </div>

    </div>
  );
}

export function initAutopilotPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<AutopilotPanel context={context} />);
  return () => { root.unmount(); };
}
