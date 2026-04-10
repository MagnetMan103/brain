import cv2
import time
import threading
from ultralytics import YOLO
from stream import start_stream

CSI_GSTREAMER_PIPELINE = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=21/1 ! "
    "nvvidconv flip-method=0 ! "
    "video/x-raw, width=640, height=480, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! appsink"
)

class Vision():
    CONFIDENCE_THRESHOLD = 0.50
    BOTTLE_CLASS_ID = 39

    def __init__(self, cameraType, model_version, stream=False):
        """
            cameraType: "usb" or "csi"
            model_version: "yolov8x.pt"
        """
        self.cameraType = cameraType
        
        if self.cameraType == "usb":
            self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            # Attempt to set buffer size to 1 (Supported on some Linux drivers)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        else:
            self.cap = cv2.VideoCapture(CSI_GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open {cameraType} camera")
        print(f"SYSTEM: {cameraType.upper()} Camera opened successfully!")

        self.model = YOLO(model_version)
        self.stream = stream
        self.bottles = []
        self.frame = None
        self.ret = None
        self.encoded_frame = None
        self._frame_event = threading.Event()

        # --- Frame Capture Threading Variables ---
        self._latest_raw_frame = None
        self._latest_ret = False
        self.camera_running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        if self.stream:
            start_stream(self)

    def _capture_loop(self):
        """Background thread that constantly pulls frames to empty the buffer."""
        while self.camera_running:
            if self.cap.isOpened():
                ret, frame = self.cap.read()
                self._latest_ret = ret
                if ret:
                    self._latest_raw_frame = frame
            else:
                time.sleep(0.01)

    def get_frame(self):
        """Returns the most recent capture of the camera and applies transforms"""
        self.ret = self._latest_ret
        
        if self.ret and self._latest_raw_frame is not None:
            # Create a copy so we don't manipulate the frame while the thread updates it
            frame_copy = self._latest_raw_frame.copy()
            
            if self.cameraType == "usb":
                self.frame = cv2.flip(frame_copy, -1)
            else:
                self.frame = frame_copy
        else:
            self.frame = None

    def detect_bottles(self):
        """
        Detects the water bottles in the frame
        Sets self.bottles to be an array of the bottles detected in the current frame.
        """
        self.get_frame()
        if self.frame is None:
            return
            
        results = self.model(self.frame, conf=self.CONFIDENCE_THRESHOLD, verbose=False)[0]
        self.bottles = []
        
        for box in results.boxes:
            class_id = int(box.cls[0])
            if class_id == self.BOTTLE_CLASS_ID:
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                data = {"x1": x1, "y1": y1, "x2": x2, "y2": y2, "conf": conf}
                self.bottles.append(data)

    def setup_overlay(self):
        """
        Draws rectangle and confidence text around each detected water bottle.
        Stores the JPEG-encoded frame in self.encoded_frame.
        """
        for bottle in self.bottles:
            cv2.rectangle(self.frame, (bottle["x1"], bottle["y1"]), (bottle["x2"], bottle["y2"]), (0, 255, 0), 2)
            cv2.putText(
                self.frame, f"Bottle {bottle['conf'] * 100:.1f}%", (bottle["x1"], bottle["y1"] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
            )
        _, buffer = cv2.imencode(".jpg", self.frame)
        self.encoded_frame = buffer
        self._frame_event.set()

    def generate_frames(self):
        """Generator for Flask streaming - yields the latest encoded frame"""
        while True:
            self._frame_event.wait()
            self._frame_event.clear()
            if self.encoded_frame is not None:
                yield (b"--frame\r\n"
                       b"Content-Type: image/jpeg\r\n\r\n" + self.encoded_frame.tobytes() + b"\r\n")

    def update(self):
        """Main loop call - runs detection and prepares stream frame if enabled"""
        self.detect_bottles()
        if self.stream and self.frame is not None:
            self.setup_overlay()
            
    def close(self):
        """Safely shuts down the capture thread and releases hardware"""
        self.camera_running = False
        if self.cap.isOpened():
            self.cap.release()
