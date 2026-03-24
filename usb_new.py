import cv2
import time
from flask import Flask, Response, render_template_string
from ultralytics import YOLO

app = Flask(__name__)

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

CONFIDENCE_THRESHOLD = 0.5
BOTTLE_CLASS_ID = 39  # "bottle" in COCO

# Set to True for USB camera, False for CSI (IMX219) camera
USE_USB_CAMERA = True

# NEW: The GStreamer pipeline for your Arducam USB camera
USB_GSTREAMER_PIPELINE = (
    "v4l2src device=/dev/video0 ! "
    "video/x-raw, width=1280, height=720 ! "
    "videoflip method=rotate-180 ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! appsink drop=true"
)

# Existing CSI pipeline (untouched)
CSI_GSTREAMER_PIPELINE = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=21/1 ! "
    "nvvidconv flip-method=0 ! "
    "video/x-raw, width=640, height=480, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! appsink"
)


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------

def open_camera():
    if USE_USB_CAMERA:
        # Bypass GStreamer and use native Linux V4L2
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        # Force MJPEG format to prevent slow raw feeds
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    else:
        # Keep GStreamer for the CSI camera if needed later
        cap = cv2.VideoCapture(CSI_GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        raise RuntimeError("Failed to open camera")

    print("Camera opened successfully!")
    return cap    


# ---------------------------------------------------------------------------
# Detection + streaming
# ---------------------------------------------------------------------------

def detect_and_stream(model):
    cap = open_camera()

    fps_start = time.time()
    fps_count = 0
    fps = 0.0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue
            if USE_USB_CAMERA:
                frame = cv2.flip(frame, -1)
            # Run YOLO inference (runs on GPU automatically if available)
            results = model(frame, conf=CONFIDENCE_THRESHOLD, verbose=False)[0]

            # Filter for bottles only and draw detections
            bottles_detected = 0

            for box in results.boxes:
                class_id = int(box.cls[0])
                if class_id != BOTTLE_CLASS_ID:
                    continue

                bottles_detected += 1
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    frame, f"Bottle {conf * 100:.1f}%", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2,
                )

            # FPS counter
            fps_count += 1
            if fps_count >= 30:
                fps = fps_count / (time.time() - fps_start)
                fps_start = time.time()
                fps_count = 0

            # HUD overlay
            cv2.putText(frame, f"Bottles: {bottles_detected}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"FPS: {fps:.1f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # Encode and yield JPEG frame
            _, buffer = cv2.imencode(".jpg", frame)
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n"
            )

    finally:
        cap.release()


# ---------------------------------------------------------------------------
# Flask routes
# ---------------------------------------------------------------------------

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>Bottle Detection — Jetson</title>
    <style>
        body {
            background: #1a1a1a;
            color: #eee;
            font-family: sans-serif;
            display: flex;
            flex-direction: column;
            align-items: center;
            padding: 20px;
        }
        h1 { margin-bottom: 10px; }
        img {
            border: 2px solid #444;
            border-radius: 8px;
            max-width: 100%;
        }
    </style>
</head>
<body>
    <h1>Bottle Detection</h1>
    <p>YOLOv8x &middot; Live stream from Jetson</p>
    <img src="/feed">
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML_PAGE)

@app.route("/feed")
def video_feed():
    return Response(
        detect_and_stream(model),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print("Loading YOLOv8x model...")
    model = YOLO("yolov8x.pt")  # downloads automatically on first run
    print(f"Model loaded on: {model.device}")

    print("\nStarting stream at http://0.0.0.0:5000")
    print("Open this in your browser from another machine using your Jetson's IP.\n")

    app.run(host="0.0.0.0", port=5000, threaded=True)
