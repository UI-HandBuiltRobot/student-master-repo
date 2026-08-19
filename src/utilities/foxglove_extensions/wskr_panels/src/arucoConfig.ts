// Styling constants for the browser-side ArUco overlay. ArUco detection
// itself runs on the Jetson (see utilities/wskr_foxglove_aruco_detector.py);
// this file only carries the drawing conventions.

export const COLOR_TARGET_HIGHLIGHT = "#FFFF00"; // yellow — target ID, no server-tracking
export const COLOR_NON_TARGET = "#787878";       // grey
export const COLOR_SERVER_TRACKING = "#00FFFF";  // cyan — /WSKR/tracked_bbox

export const LABEL_FONT = "bold 14px Consolas, monospace";
export const HIGHLIGHT_STROKE_WIDTH = 3;
export const NON_TARGET_STROKE_WIDTH = 1;
