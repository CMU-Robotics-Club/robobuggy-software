#!/usr/bin/env python3
"""
Serves the annotated (YOLO-overlaid) camera feed over HTTP as a low-res MJPEG
stream, so it can be watched from any browser, including over the T-Mobile 5G
modem connected to the robot's router.

Input : CompressedImage on `debug/annotated_camera_frame` (published by the
        detector node, full-res JPEG with detections drawn on).
Output: http://<robot-ip>:<port>/        simple viewer page
        http://<robot-ip>:<port>/stream  multipart MJPEG stream
        http://<robot-ip>:<port>/snapshot.jpg  latest single frame

Bandwidth/CPU design:
  * Frames are decoded, downscaled, and re-encoded at low JPEG quality.
  * Output is rate limited to `max_fps`.
  * If nobody is connected, incoming frames are dropped without decoding.
  * One encode per frame is shared across all viewers.
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

INDEX_HTML = b"""<!doctype html>
<html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Buggy camera</title>
<style>html,body{margin:0;background:#000;height:100%}
img{width:100%;height:100%;object-fit:contain}</style></head>
<body><img src="/stream"></body></html>"""


class FrameBuffer:
    """Latest encoded frame, shared between the ROS thread and HTTP threads."""

    def __init__(self):
        self._cond = threading.Condition()
        self._jpeg = None
        self._seq = 0
        self.viewers = 0

    def put(self, jpeg):
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self._cond.notify_all()

    def latest(self):
        with self._cond:
            return self._jpeg

    def wait_next(self, last_seq, timeout=5.0):
        """Block until a frame newer than last_seq exists. Returns (seq, jpeg)."""
        with self._cond:
            self._cond.wait_for(lambda: self._seq != last_seq, timeout)
            return self._seq, self._jpeg

    def add_viewer(self, delta):
        with self._cond:
            self.viewers += delta


def make_handler(buffer):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):  # keep the ROS console clean
            pass

        def do_GET(self):  # pylint: disable=invalid-name
            if self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(INDEX_HTML)))
                self.end_headers()
                self.wfile.write(INDEX_HTML)
            elif self.path.startswith("/snapshot.jpg"):
                jpeg = buffer.latest()
                if jpeg is None:
                    self.send_error(503, "no frame yet")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpeg)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(jpeg)
            elif self.path.startswith("/stream"):
                self.stream()
            else:
                self.send_error(404)

        def stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            buffer.add_viewer(1)
            seq = -1
            try:
                while True:
                    seq, jpeg = buffer.wait_next(seq)
                    if jpeg is None:
                        continue
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                        + str(len(jpeg)).encode()
                        + b"\r\n\r\n"
                        + jpeg
                        + b"\r\n"
                    )
            except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                pass
            finally:
                buffer.add_viewer(-1)

    return Handler


class VideoStream(Node):
    def __init__(self):
        super().__init__("video_stream")

        self.declare_parameter("input_topic", "debug/annotated_camera_frame")
        self.declare_parameter("port", 8080)
        self.declare_parameter("width", 480)  # output width in px, height keeps aspect
        self.declare_parameter("jpeg_quality", 40)  # 1-100, lower = smaller
        self.declare_parameter("max_fps", 8.0)

        self.width = int(self.get_parameter("width").value)
        self.quality = int(self.get_parameter("jpeg_quality").value)
        self.min_period = 1.0 / float(self.get_parameter("max_fps").value)
        port = int(self.get_parameter("port").value)

        self.buffer = FrameBuffer()
        self.last_emit = 0.0

        self.server = ThreadingHTTPServer(("0.0.0.0", port), make_handler(self.buffer))
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

        self.create_subscription(
            CompressedImage, self.get_parameter("input_topic").value, self.on_frame, 1
        )
        self.get_logger().info(f"Serving MJPEG on 0.0.0.0:{port} (/stream, /snapshot.jpg)")

    def on_frame(self, msg):
        now = time.monotonic()
        # Skip all work if nobody is watching or we're over the fps budget.
        # (/snapshot.jpg alone won't wake the encoder; open /stream or the index.)
        if self.buffer.viewers == 0 or now - self.last_emit < self.min_period:
            return
        self.last_emit = now

        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        h, w = img.shape[:2]
        if w > self.width:
            img = cv2.resize(
                img, (self.width, int(h * self.width / w)), interpolation=cv2.INTER_AREA
            )
        ok, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if ok:
            self.buffer.put(enc.tobytes())

    def destroy_node(self):
        self.server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoStream()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
