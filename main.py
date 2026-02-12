from camera import Vision

Brain = Vision("usb", "yolov8x.pt", stream=True)

while True:
    Brain.update()

