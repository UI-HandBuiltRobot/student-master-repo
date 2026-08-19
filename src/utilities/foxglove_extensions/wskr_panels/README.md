# WSKR Panels — Foxglove extension

Two thin client-side panels for the WSKR web dashboard:

- **CameraArucoPanel** — subscribes to the throttled camera stream
  (`/camera1/throttled/compressed`), the Jetson-side marker topic
  (`/WSKR/aruco_markers`), and `/WSKR/tracked_bbox`. Decodes the JPEG
  and draws rectangles from the received marker corners (grey for
  non-target IDs, yellow for the target) plus the cyan "TRACKING"
  server-tracking overlay. **No in-browser ArUco detection** — the
  Jetson runs `cv2.aruco` once per dashboard frame and ships the
  corners over.
- **WhiskerFanPanel** — subscribes to `WSKR/whisker_lengths`,
  `WSKR/target_whisker_lengths`, `WSKR/heading_to_target`, and
  `WSKR/tracking_mode`, and renders the top-down whisker fan,
  chassis, target diamonds, and heading arrow.

Both panels are thin HTML-Canvas renderers with no native-library
dependencies — the whole `.foxe` bundle is small, loads fast, and works
identically in Foxglove Studio (desktop), self-hosted containers, and
app.foxglove.dev.

## Build the `.foxe` bundle (on a dev machine with Node.js)

```bash
cd Project5_WS/src/utilities/foxglove_extensions/wskr_panels
npm install
npm run package
```

This produces `wskr.wskr-panels-<version>.foxe` in the extension root.

## Install in Foxglove

Drag the `.foxe` into the Foxglove window (Studio desktop, self-hosted
`ghcr.io/foxglove/studio`, or app.foxglove.dev). It registers two panel
types, `wskr.wskr-panels.CameraArucoPanel` and
`wskr.wskr-panels.WhiskerFanPanel`.

## Use with the shipped layout

1. On the Jetson (one-time):
   `sudo apt install ros-humble-foxglove-bridge ros-humble-topic-tools`.
2. On the Jetson: `ros2 launch utilities wskr_foxglove.launch.py` — starts
   foxglove_bridge on port 8765, the image throttles, the ArUco detector,
   the model-list helper, and the approach-service bridge.
3. In Foxglove, "Open connection" → `ws://<jetson-ip>:8765`.
4. Layout → Import →
   `install/utilities/share/utilities/layouts/wskr_default.json`.
5. The two custom panels should populate; if they don't, check that the
   extension is installed and that topic names match your namespace.

## Panel settings

Each panel exposes editable topic paths and (for CameraArucoPanel) the
target ArUco ID and a diagnostic HUD toggle, in Foxglove's panel-settings
sidebar. Edits persist in the layout.

## Diagnosing an empty CameraArucoPanel

The HUD overlay shows:
- `camera: <topic>` — confirms the configured camera topic.
- `markers: <topic>  (msgs: N, last N: M)` — message count + markers in
  the most recent message.
- `frames seen / decoded / decode-fail` — image pipeline health.

Common failure patterns:
- `frames seen: 0` → bridge isn't delivering the camera topic. Verify
  `ros2 topic hz /camera1/throttled/compressed` on the Jetson, and
  make sure the topic name in the panel settings matches.
- `msgs: 0` in the markers line → the detector node isn't publishing.
  `ros2 topic hz /WSKR/aruco_markers` on the Jetson; check
  `wskr_foxglove.launch.py` logs for detector startup.
- `decode-fail > 0` → the JPEG decoder rejects the frames. The first
  camera message's field types are logged to the browser DevTools
  console for inspection.
