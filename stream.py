from flask import Flask, Response, render_template_string
import threading

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

def start_stream(vision, host="0.0.0.0", port=5000):
    """Called by Vision to start Flask server in a background thread"""
    app = Flask(__name__)

    @app.route("/")
    def index():
        return render_template_string(HTML_PAGE)

    @app.route("/feed")
    def video_feed():
        return Response(
            vision.generate_frames(),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    thread = threading.Thread(target=lambda: app.run(host=host, port=port, threaded=True))
    thread.daemon = True
    thread.start()
