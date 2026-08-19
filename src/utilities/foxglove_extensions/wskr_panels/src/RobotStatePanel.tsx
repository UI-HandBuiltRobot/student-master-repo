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
  stateTopic: string;
  commandTopic: string;
}

const DEFAULT_CONFIG: PanelConfig = {
  stateTopic: "/robot_state",
  commandTopic: "/robot_command",
};

// ── state definitions ──────────────────────────────────────────────────

interface StateInfo {
  label: string;
  command: string;
  color: string;
}

const PIPELINE_STATES: StateInfo[] = [
  { label: "SEARCH",       command: "search",       color: "#2196f3" },
  { label: "SELECT",       command: "select",       color: "#00acc1" },
  { label: "APPROACH OBJ", command: "approach_obj",  color: "#ff9800" },
  { label: "GRASP",        command: "grasp",        color: "#9c27b0" },
  { label: "FIND BOX",     command: "find_box",     color: "#2196f3" },
  { label: "APPROACH BOX", command: "approach_box",  color: "#ff9800" },
  { label: "DROP",         command: "drop",         color: "#4caf50" },
];

const CONTROL_STATES: StateInfo[] = [
  { label: "IDLE",    command: "idle",    color: "#607d8b" },
  { label: "WANDER",  command: "wander",  color: "#26a69a" },
  { label: "STOPPED", command: "stopped", color: "#f44336" },
  { label: "ERROR",   command: "error",   color: "#b71c1c" },
];

function stateColor(state: string): string {
  const all = [...PIPELINE_STATES, ...CONTROL_STATES];
  const match = all.find((s) => s.command === state.toLowerCase() || s.label === state.replace("_", " "));
  return match?.color ?? "#888";
}

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
const BTN_GRID: React.CSSProperties = {
  display: "grid",
  gridTemplateColumns: "repeat(auto-fill, minmax(120px, 1fr))",
  gap: 6,
};
const STATUS_MSG: React.CSSProperties = {
  color: "#aaa", fontSize: 11, marginTop: 4,
  whiteSpace: "pre-wrap", wordBreak: "break-word",
  maxHeight: 56, overflow: "auto",
};

function stateButtonStyle(info: StateInfo, isActive: boolean): React.CSSProperties {
  return {
    fontFamily: "Consolas, monospace",
    fontSize: 12,
    fontWeight: "bold",
    border: isActive ? `2px solid ${info.color}` : "2px solid transparent",
    borderRadius: 4,
    padding: "8px 6px",
    cursor: "pointer",
    background: isActive ? info.color : "#333",
    color: isActive ? "#fff" : "#ccc",
    textAlign: "center",
    transition: "background 0.15s, border-color 0.15s",
    letterSpacing: "0.03em",
  };
}

function indicatorStyle(color: string): React.CSSProperties {
  return {
    display: "flex",
    alignItems: "center",
    justifyContent: "center",
    gap: 10,
    padding: "10px 16px",
    background: "#252525",
    borderRadius: 6,
    borderLeft: `4px solid ${color}`,
  };
}

// ── panel ──────────────────────────────────────────────────────────────

function RobotStatePanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [config, setConfig] = useState<PanelConfig>(() => ({
    ...DEFAULT_CONFIG,
    ...((context.initialState as Partial<PanelConfig>) ?? {}),
  }));

  const [currentState, setCurrentState] = useState("—");
  const [statusMsg, setStatusMsg] = useState("");

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
          label: "Robot State Panel",
          fields: {
            stateTopic:   { label: "State topic",   input: "string", value: config.stateTopic },
            commandTopic: { label: "Command topic",  input: "string", value: config.commandTopic },
          },
        },
      },
    });
  }, [context, config]);

  // ── advertise ─────────────────────────────────────────────────────────

  useEffect(() => {
    context.advertise?.(config.commandTopic, "std_msgs/msg/String");
    return () => { context.unadvertise?.(config.commandTopic); };
  }, [context, config.commandTopic]);

  // ── subscribe ─────────────────────────────────────────────────────────

  useEffect(() => {
    context.subscribe([{ topic: config.stateTopic }]);
    return () => { context.unsubscribeAll(); };
  }, [context, config.stateTopic]);

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      if (renderState.currentFrame) {
        for (const msg of renderState.currentFrame) {
          if (msg.topic === config.stateTopic) {
            const s = (msg as MessageEvent<StringMsg>).message;
            setCurrentState(s.data ?? "—");
          }
        }
      }
      done();
    };
    context.watch("currentFrame");
  }, [context, config.stateTopic]);

  // ── publish command ───────────────────────────────────────────────────

  const sendCommand = useCallback((command: string) => {
    if (!context.publish) {
      setStatusMsg("context.publish not available in this Foxglove version.");
      return;
    }
    try {
      context.publish(config.commandTopic, { data: command });
      setStatusMsg(`Sent: ${command}`);
    } catch (err) {
      setStatusMsg(`Publish failed: ${err}`);
    }
  }, [context, config.commandTopic]);

  // ── render ────────────────────────────────────────────────────────────

  const activeColor = stateColor(currentState);
  const isStopped = currentState === "STOPPED";

  const renderButton = (info: StateInfo) => {
    const isActive = currentState === info.label.replace(" ", "_");
    return (
      <button
        key={info.command}
        style={stateButtonStyle(info, isActive)}
        onClick={() => sendCommand(info.command)}
        onMouseEnter={(e) => {
          if (!isActive) {
            e.currentTarget.style.background = info.color + "88";
            e.currentTarget.style.color = "#fff";
          }
        }}
        onMouseLeave={(e) => {
          if (!isActive) {
            e.currentTarget.style.background = "#333";
            e.currentTarget.style.color = "#ccc";
          }
        }}
      >
        {info.label}
      </button>
    );
  };

  return (
    <div style={CONTAINER}>
      {/* ── Current State ── */}
      <div style={SECTION}>Current State</div>
      <div style={indicatorStyle(activeColor)}>
        <span style={{
          fontSize: 22,
          fontWeight: "bold",
          color: activeColor,
          letterSpacing: "0.05em",
        }}>
          {currentState}
        </span>
      </div>

      {isStopped && (
        <div style={{ color: "#f44336", fontSize: 11, textAlign: "center" }}>
          SAFETY STOP ACTIVE — only IDLE and STOP commands accepted
        </div>
      )}

      <div style={DIVIDER} />

      {/* ── Pipeline ── */}
      <div style={SECTION}>Pipeline</div>
      <div style={BTN_GRID}>
        {PIPELINE_STATES.map(renderButton)}
      </div>

      <div style={DIVIDER} />

      {/* ── Control ── */}
      <div style={SECTION}>Control</div>
      <div style={BTN_GRID}>
        {CONTROL_STATES.map(renderButton)}
      </div>

      {statusMsg && <div style={STATUS_MSG}>{statusMsg}</div>}
    </div>
  );
}

export function initRobotStatePanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<RobotStatePanel context={context} />);
  return () => { root.unmount(); };
}
