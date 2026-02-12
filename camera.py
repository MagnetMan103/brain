import cv2
import time
import threading
from ultralytics import YOLO
from stream import start_stream

class Vision():
    cap = None
    CONFIDENCE_THRESHOLD = 0.50
    BOTTLE_CLASS_ID = 39

    def __init__(self, cameraType, model_version, stream=False):
        """
            cameraType: "usb"
            model_version: "yolov8x.pt"
        """
        if cameraType == "usb":
            self.cap = cv2.VideoCapture(0)
        self.model = YOLO(model_version)
        self.stream = stream
        self.bottles = []
        self.frame = None
        self.ret = None
        self.encoded_frame = None
        self._frame_event = threading.Event()

        if self.stream:
            start_stream(self)

    def get_frame(self):
        """Returns the current capture of the camera"""
        ret, frame = self.cap.read()
        self.ret = ret
        self.frame = frame

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
