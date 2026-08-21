import cv2
import numpy as np
import tkinter as tk
from tkinter import filedialog

# File browser — also grab screen dimensions before destroying root
root = tk.Tk()
root.withdraw()
screen_w = root.winfo_screenwidth()
screen_h = root.winfo_screenheight()
image_path = filedialog.askopenfilename(
    title="Select an image",
    filetypes=[("Image files", "*.jpg *.jpeg *.png")]
)
root.destroy()

if not image_path:
    print("No image selected.")
    exit()

img_orig = cv2.imread(image_path)
orig_h, orig_w = img_orig.shape[:2]

# Initial canvas fills 90% of the screen; image is scaled to fit inside it.
# This is just the STARTING size — the window is resizable/maximizable and
# the canvas will track whatever size the window actually is at draw time.
canvas_w = int(screen_w * 0.9)
canvas_h = int(screen_h * 0.9)
fit_scale = min(canvas_w / orig_w, canvas_h / orig_h)

# View state
scale = fit_scale
pan_x = (canvas_w - orig_w * scale) / 2.0   # canvas x of image left edge
pan_y = (canvas_h - orig_h * scale) / 2.0   # canvas y of image top edge

ZOOM_FACTOR = 1.15
MIN_SCALE   = fit_scale * 0.1
MAX_SCALE   = 20.0

points = []   # list of (orig_x, orig_y) integer pixel coords


def resize_canvas(new_w, new_h):
    """Update canvas_w/canvas_h to match the actual window size, keeping
    whatever image point was at the center of the view still centered,
    and scaling the image proportionally to avoid cropping."""
    global canvas_w, canvas_h, scale, pan_x, pan_y, fit_scale, MIN_SCALE

    new_w = max(1, new_w)
    new_h = max(1, new_h)
    if new_w == canvas_w and new_h == canvas_h:
        return

    # Image-space coordinate currently at the center of the old canvas
    old_center_x = canvas_w / 2.0
    old_center_y = canvas_h / 2.0
    img_cx = (old_center_x - pan_x) / scale
    img_cy = (old_center_y - pan_y) / scale

    # Recalculate fit_scale for the new window size
    new_fit_scale = min(new_w / orig_w, new_h / orig_h)

    # Calculate new scale maintaining the relative zoom level
    zoom_level = scale / fit_scale
    scale = new_fit_scale * zoom_level
    fit_scale = new_fit_scale
    MIN_SCALE = fit_scale * 0.1

    canvas_w, canvas_h = new_w, new_h

    # Re-center on the same image point in the new canvas with the adjusted scale
    new_center_x = canvas_w / 2.0
    new_center_y = canvas_h / 2.0
    pan_x = new_center_x - img_cx * scale
    pan_y = new_center_y - img_cy * scale


def redraw():
    canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

    # Destination rectangle on the canvas where the image is visible
    dst_x0 = max(0, int(pan_x))
    dst_y0 = max(0, int(pan_y))
    dst_x1 = min(canvas_w, int(np.ceil(pan_x + orig_w * scale)))
    dst_y1 = min(canvas_h, int(np.ceil(pan_y + orig_h * scale)))

    if dst_x0 < dst_x1 and dst_y0 < dst_y1:
        # Map destination rectangle back to source (original) coords
        src_x0 = max(0, int(np.floor((dst_x0 - pan_x) / scale)))
        src_y0 = max(0, int(np.floor((dst_y0 - pan_y) / scale)))
        src_x1 = min(orig_w, int(np.ceil((dst_x1 - pan_x) / scale)))
        src_y1 = min(orig_h, int(np.ceil((dst_y1 - pan_y) / scale)))

        if src_x0 < src_x1 and src_y0 < src_y1:
            patch = img_orig[src_y0:src_y1, src_x0:src_x1]

            # Recalculate exact destination for the (integer-snapped) src patch
            rdst_x0 = max(0, int(src_x0 * scale + pan_x))
            rdst_y0 = max(0, int(src_y0 * scale + pan_y))
            rdst_x1 = min(canvas_w, int(np.ceil(src_x1 * scale + pan_x)))
            rdst_y1 = min(canvas_h, int(np.ceil(src_y1 * scale + pan_y)))
            dw = rdst_x1 - rdst_x0
            dh = rdst_y1 - rdst_y0

            if dw > 0 and dh > 0:
                interp = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
                canvas[rdst_y0:rdst_y1, rdst_x0:rdst_x1] = cv2.resize(
                    patch, (dw, dh), interpolation=interp)

    # Draw clicked points
    font = cv2.FONT_HERSHEY_SIMPLEX
    for i, (ox, oy) in enumerate(points):
        dx = int(ox * scale + pan_x)
        dy = int(oy * scale + pan_y)
        cv2.circle(canvas, (dx, dy), 5, (0, 0, 255), -1)
        cv2.putText(canvas, str(i + 1), (dx + 7, dy - 7), font, 0.6, (0, 255, 255), 2)

    # Draw Legend / Help Menu
    legend_x = 10
    legend_y = 10
    legend_w = 260
    legend_h = 105
    # Create semi-transparent overlay
    sub_img = canvas[legend_y:legend_y+legend_h, legend_x:legend_x+legend_w]
    rect = np.zeros(sub_img.shape, dtype=np.uint8)
    rect[:] = (40, 40, 40) # Dark gray background
    res = cv2.addWeighted(sub_img, 0.3, rect, 0.7, 0)
    canvas[legend_y:legend_y+legend_h, legend_x:legend_x+legend_w] = res

    cv2.rectangle(canvas, (legend_x, legend_y), (legend_x + legend_w, legend_y + legend_h), (200, 200, 200), 1)

    help_text = [
        "Controls:",
        "  L-Click: Place Point",
        "  R-Click: Undo Last Point",
        "  Scroll: Zoom In/Out",
        "  Esc / Close: Finish & Save"
    ]
    for i, line in enumerate(help_text):
        font_scale = 0.45 if i > 0 else 0.5
        font_weight = 1 if i > 0 else 2
        color = (255, 255, 255) if i > 0 else (0, 255, 255)
        cv2.putText(canvas, line, (legend_x + 10, legend_y + 18 + i*18), font, font_scale, color, font_weight, cv2.LINE_AA)

    cv2.imshow('image', canvas)


def click_event(event, x, y, flags, params):
    global scale, pan_x, pan_y

    if event == cv2.EVENT_LBUTTONDOWN:
        ox = int(round((x - pan_x) / scale))
        oy = int(round((y - pan_y) / scale))
        ox = max(0, min(orig_w - 1, ox))
        oy = max(0, min(orig_h - 1, oy))
        nx = ox / (orig_w - 1) if orig_w > 1 else 0.0
        ny = oy / (orig_h - 1) if orig_h > 1 else 0.0
        points.append((ox, oy))
        n = len(points)
        print(f"  {n}: pixel=({ox}, {oy})  normalized=({nx:.4f}, {ny:.4f})")
        redraw()

    elif event == cv2.EVENT_RBUTTONDOWN:
        if points:
            points.pop()
            print(f"  (removed last point, {len(points)} remaining)")
            redraw()

    elif event == cv2.EVENT_MOUSEWHEEL:
        # flags > 0: scroll up → zoom in; flags < 0: scroll down → zoom out
        factor = ZOOM_FACTOR if flags > 0 else 1.0 / ZOOM_FACTOR
        new_scale = max(MIN_SCALE, min(MAX_SCALE, scale * factor))
        ratio = new_scale / scale
        # Keep the pixel under the cursor fixed
        pan_x = x - (x - pan_x) * ratio
        pan_y = y - (y - pan_y) * ratio
        scale = new_scale
        redraw()


# WINDOW_NORMAL (instead of WINDOW_AUTOSIZE) makes the window resizable
# and lets the OS maximize button / double-click-titlebar work.
# WINDOW_GUI_NORMAL removes the Qt status bar/toolbar (bottom task bar displaying pixel coords & RGB).
cv2.namedWindow('image', cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
cv2.resizeWindow('image', canvas_w, canvas_h)
redraw()
cv2.setMouseCallback('image', click_event)

last_w, last_h = canvas_w, canvas_h

# Poll instead of a single blocking waitKey(0) so we notice window
# resizes/maximizes even when the user isn't clicking or scrolling.
while True:
    key = cv2.waitKey(30) & 0xFF

    # Window closed via the OS 'X' button
    try:
        if cv2.getWindowProperty('image', cv2.WND_PROP_VISIBLE) < 1:
            break
    except cv2.error:
        break

    if key == 27:  # Esc
        break

    # Check current window size and resync the canvas if it changed
    # (covers manual resize and OS-level maximize/restore)
    try:
        _, _, win_w, win_h = cv2.getWindowImageRect('image')
    except cv2.error:
        win_w, win_h = last_w, last_h

    if win_w > 0 and win_h > 0 and (win_w != last_w or win_h != last_h):
        resize_canvas(win_w, win_h)
        last_w, last_h = win_w, win_h
        redraw()

cv2.destroyAllWindows()

print("\nFinal points:")
if points:
    print(f"  {'#':<4} {'pixel_x':>10} {'pixel_y':>10} {'norm_x':>10} {'norm_y':>10}")
    for i, (ox, oy) in enumerate(points):
        nx = ox / (orig_w - 1) if orig_w > 1 else 0.0
        ny = oy / (orig_h - 1) if orig_h > 1 else 0.0
        print(f"  {i+1:<4} {ox:>10} {oy:>10} {nx:>10.4f} {ny:>10.4f}")
else:
    print("  (none)")