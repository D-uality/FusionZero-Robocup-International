# ─── high-level imports ────────────────────────────────────────────────────────
import sys, pathlib, cv2, numpy as np
root = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(root))

from hardware.robot import *          # brings in evac_camera etc.
from core.utilities import *          # start_display(), stop_display() …

# ─── optional: flip settings for evac_camera if you need them ────────────────
# evac_camera.set_vflip(False)
# evac_camera.set_hflip(False)

# ─── UI helpers ───────────────────────────────────────────────────────────────
def nothing(_=None):                 # dummy cb for track-bars
    pass

def processHoughTransform(img, p):
    """Return a copy of `img` with detected circles drawn on it."""
    gray     = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred  = cv2.GaussianBlur(gray, (9, 9), 2)
    circles  = cv2.HoughCircles(
        blurred, cv2.HOUGH_GRADIENT,
        dp        = p["dp"],
        minDist   = p["minDist"],
        param1    = p["param1"],
        param2    = p["param2"],
        minRadius = p["minRadius"],
        maxRadius = p["maxRadius"]
    )

    out = img.copy()
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for x, y, r in circles[0]:
            cv2.circle(out, (x, y), r, (0, 255,   0), 2)  # outer ring
            cv2.circle(out, (x, y), 2, (0,   0, 255), 3)  # centre dot
    return out

# ─── initialise OpenCV display and track-bars ────────────────────────────────
start_display()                      # your existing helper (opens cv2 window)
cv2.startWindowThread()
cv2.namedWindow("image", cv2.WINDOW_NORMAL)

# dp slider is *10 so the integer range 10-50 becomes 1.0-5.0
cv2.createTrackbar("dp",        "image", 10,  50, nothing)
cv2.createTrackbar("minDist",   "image", 20, 300, nothing)
cv2.createTrackbar("param1",    "image", 50, 200, nothing)
cv2.createTrackbar("param2",    "image", 30, 100, nothing)
cv2.createTrackbar("minRadius", "image",  0, 100, nothing)
cv2.createTrackbar("maxRadius", "image",  0, 300, nothing)

# ─── main loop ────────────────────────────────────────────────────────────────
try:
    while True:
        frame = evac_camera.capture()                # BGR expected
        # frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_I420)  # ← uncomment if YUV

        # read UI values
        params = {
            "dp":        cv2.getTrackbarPos("dp",        "image") / 10.0,
            "minDist":   cv2.getTrackbarPos("minDist",   "image"),
            "param1":    cv2.getTrackbarPos("param1",    "image"),
            "param2":    cv2.getTrackbarPos("param2",    "image"),
            "minRadius": cv2.getTrackbarPos("minRadius", "image"),
            "maxRadius": cv2.getTrackbarPos("maxRadius", "image") or 0
        }

        out = processHoughTransform(frame, params)

        # overlay the live parameter read-out
        y0, dy = 20, 25
        for i, (k, v) in enumerate(params.items()):
            cv2.putText(out, f"{k}: {v}", (10, y0 + i*dy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 0, 0), 2)

        cv2.imshow("image", out)

        # quit on ESC
        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    cv2.destroyAllWindows()
    stop_display()                   # tidy up if you have such a helper