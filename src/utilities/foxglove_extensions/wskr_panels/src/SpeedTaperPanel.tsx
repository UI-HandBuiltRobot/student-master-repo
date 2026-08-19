import {
  PanelExtensionContext,
  SettingsTreeAction,
} from "@foxglove/extension";
import { useCallback, useEffect, useState } from "react";
import { createRoot } from "react-dom/client";

// ── config ─────────────────────────────────────────────────────────────

interface PanelConfig {
  proximityLimitsTopic: string;
}

const DEFAULT_CONFIG: PanelConfig = {
  proximityLimitsTopic: "/WSKR/autopilot/proximity_limits",
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
const GRID: React.CSSProperties = {
  display: "grid",
  gridTemplateColumns: "max-content 1fr 1fr",
  gap: "6px 10px",
  alignItems: "center",
};
const COL_HEADER: React.CSSProperties = { color: "#888", fontSize: 11, textAlign: "center" };
const ROW_LABEL:  React.CSSProperties = { color: "#888" };
const INPUT_STYLE: React.CSSProperties = {
  fontFamily: "Consolas, monospace", fontSize: 13,
  background: "#2a2a2a", color: "#e0e0e0",
  border: "1px solid #555", borderRadius: 4,
  padding: "4px 8px", width: "100%", boxSizing: "border-box",
};
const BTN_BASE: React.CSSProperties = {
  fontFamily: "Consolas, monospace", fontSize: 13, fontWeight: "bold",
  border: "none", borderRadius: 4, padding: "7px 20px", cursor: "pointer",
};
const BTN_GO:  React.CSSProperties = { ...BTN_BASE, background: "#2e7d32", color: "#fff" };
const BTN_OFF: React.CSSProperties = { ...BTN_BASE, background: "#555",    color: "#999", cursor: "default" };
const STATUS_MSG: React.CSSProperties = {
  color: "#aaa", fontSize: 11, marginTop: 4,
  whiteSpace: "pre-wrap", wordBreak: "break-word",
};
const NOTE: React.CSSProperties = { color: "#555", fontSize: 11 };

// ── panel ──────────────────────────────────────────────────────────────

function SpeedTaperPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [config, setConfig] = useState<PanelConfig>(() => ({
    ...DEFAULT_CONFIG,
    ...((context.initialState as Partial<PanelConfig>) ?? {}),
  }));

  // Ramp point values (raw string so user can type freely; validated on Apply)
  const [farMm,      setFarMm]      = useState("500");
  const [nearMm,     setNearMm]     = useState("100");
  const [speedAtFar, setSpeedAtFar] = useState("1.00");
  const [speedAtNear,setSpeedAtNear]= useState("0.10");

  const [statusMsg,  setStatusMsg]  = useState("");
  const [busy,       setBusy]       = useState(false);

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
          label: "Speed Taper Panel",
          fields: {
            proximityLimitsTopic: {
              label: "Proximity limits topic",
              input: "string",
              value: config.proximityLimitsTopic,
            },
          },
        },
      },
    });
  }, [context, config]);

  // ── advertise ─────────────────────────────────────────────────────────

  useEffect(() => {
    context.advertise?.(config.proximityLimitsTopic, "std_msgs/msg/Float32MultiArray");
    return () => { context.unadvertise?.(config.proximityLimitsTopic); };
  }, [context, config.proximityLimitsTopic]);

  // ── apply ─────────────────────────────────────────────────────────────

  const handleApply = useCallback(() => {
    const far   = parseFloat(farMm);
    const near  = parseFloat(nearMm);
    const sFar  = parseFloat(speedAtFar);
    const sNear = parseFloat(speedAtNear);

    if (isNaN(far) || isNaN(near) || isNaN(sFar) || isNaN(sNear)) {
      setStatusMsg("All fields must be valid numbers.");
      return;
    }
    if (far <= near) {
      setStatusMsg("Far distance must be greater than near distance.");
      return;
    }
    if (sFar < 0 || sFar > 1 || sNear < 0 || sNear > 1) {
      setStatusMsg("Speed values must be in [0.0, 1.0].");
      return;
    }
    if (!context.publish) {
      setStatusMsg("context.publish not available in this Foxglove version.");
      return;
    }
    setBusy(true);
    setStatusMsg("");
    try {
      // Payload: [near_mm, far_mm, speed_at_near, speed_at_far]
      // Autopilot parses: data[0]=min_mm, data[1]=max_mm, data[2]=speed_min, data[3]=speed_max
      context.publish(config.proximityLimitsTopic, {
        layout: { dim: [], data_offset: 0 },
        data: [near, far, sNear, sFar],
      });
      setStatusMsg(
        `Applied: far=${far.toFixed(0)}mm @ ${sFar.toFixed(2)}  near=${near.toFixed(0)}mm @ ${sNear.toFixed(2)}`
      );
    } catch (err) {
      setStatusMsg(`Publish failed: ${err}`);
    } finally {
      setBusy(false);
    }
  }, [context, config.proximityLimitsTopic, farMm, nearMm, speedAtFar, speedAtNear]);

  // ── render ────────────────────────────────────────────────────────────

  const valid =
    !isNaN(parseFloat(farMm)) && !isNaN(parseFloat(nearMm)) &&
    !isNaN(parseFloat(speedAtFar)) && !isNaN(parseFloat(speedAtNear)) &&
    parseFloat(farMm) > parseFloat(nearMm) && !busy;

  return (
    <div style={CONTAINER}>
      <div style={SECTION}>Speed Taper</div>

      <div style={GRID}>
        {/* Header row */}
        <span />
        <span style={COL_HEADER}>Distance (mm)</span>
        <span style={COL_HEADER}>Speed scale</span>

        {/* Far row */}
        <span style={ROW_LABEL}>Far</span>
        <input
          type="number" min={0} step={10}
          value={farMm}
          onChange={(e) => setFarMm(e.target.value)}
          style={INPUT_STYLE}
        />
        <input
          type="number" min={0} max={1} step={0.01}
          value={speedAtFar}
          onChange={(e) => setSpeedAtFar(e.target.value)}
          style={INPUT_STYLE}
        />

        {/* Near row */}
        <span style={ROW_LABEL}>Near</span>
        <input
          type="number" min={0} step={10}
          value={nearMm}
          onChange={(e) => setNearMm(e.target.value)}
          style={INPUT_STYLE}
        />
        <input
          type="number" min={0} max={1} step={0.01}
          value={speedAtNear}
          onChange={(e) => setSpeedAtNear(e.target.value)}
          style={INPUT_STYLE}
        />
      </div>

      <div style={NOTE}>
        Drive speed scales linearly from Far to Near. Rotation is unaffected.
      </div>

      <div style={{ display: "flex", justifyContent: "flex-end" }}>
        <button
          style={valid ? BTN_GO : BTN_OFF}
          disabled={!valid}
          onClick={handleApply}
        >
          APPLY
        </button>
      </div>

      {statusMsg && <div style={STATUS_MSG}>{statusMsg}</div>}
    </div>
  );
}

export function initSpeedTaperPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<SpeedTaperPanel context={context} />);
  return () => { root.unmount(); };
}
