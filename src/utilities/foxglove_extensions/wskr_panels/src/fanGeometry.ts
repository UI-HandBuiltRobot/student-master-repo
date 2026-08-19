// Top-down whisker-fan geometry — ported verbatim from wskr_dashboard.py:
//   _WHISKER_ANGLES_DEG, _ROBOT_WIDTH_MM, _ROBOT_LENGTH_MM, _whisker_color.
// Any change here must match the autopilot's understanding of the array order
// (see wskr_range_node: published array ascends by whisker label, -90 → +90).

export const WHISKER_ANGLES_DEG: number[] = [-90, -60, -45, -30, -15, 0, 15, 30, 45, 60, 90];

export const ROBOT_WIDTH_MM = 160.0;   // left-right
export const ROBOT_LENGTH_MM = 200.0;  // front-back
export const WHISKER_RANGE_MM = 500.0; // max ray length in mm (matches wskr_range_node max_range_mm)

export function whiskerColor(mm: number): string {
  if (mm < 150.0) return "#ff6060";   // red — close obstacle
  if (mm < 400.0) return "#ffcc40";   // yellow — caution
  return "#60c0ff";                    // cyan — clear
}

// Colors for the chassis + axes + target markers + heading arrow, matching
// the tkinter rendering in wskr_dashboard._draw_robot_diagram.
export const CHASSIS_FILL = "#2a63d6";
export const CHASSIS_STROKE = "#1d4ba3";
export const WHEEL_FILL = "#101010";
export const WHEEL_STROKE = "#303030";
export const AXIS_X_COLOR = "#ff6060"; // forward (red)
export const AXIS_Y_COLOR = "#60ff60"; // left (green)
export const TARGET_MARKER_FILL = "#ff40d0";
export const TARGET_MARKER_STROKE = "#ffffff";
export const HEADING_COLOR_VISUAL = "#ffd84a";     // gold when tracking_mode=='visual'
export const HEADING_COLOR_DEAD_RECK = "#ff9040";  // orange otherwise

export const BACKGROUND_FILL = "#181818";
export const LABEL_FILL = "#b0b0b0";
