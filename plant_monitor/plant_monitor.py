import atexit
import serial
from typing import Dict, Optional
import base64
from threading import Thread, Event
from queue import Queue

import cv2 as cv

from flask import Flask, request
from werkzeug import serving


def jetson_pipeline(capture_width=1280, capture_height=720,
                    display_width=1280, display_height=720, flip_method=0):
    args = [f"nvarguscamerasrc",
            f"video/x-raw(memory:NVMM), width={int(capture_width)}, height={int(capture_height)},"
            " format=NV12",
            f"nvvidconv flip-method={int(flip_method)}",
            f"video/x-raw, width={int(display_width)}, height={int(display_height)}, format=BGRx",
            "videoconvert",
            "video/x-raw, format=BGR",
            "appsink"]
    return " ! ".join(args)


def gstreamer_pipeline(width=1280, height=720, flip_180=False):
    args = ["libcamerasrc", f"video/x-raw, width={width}, height={height}"]
    if flip_180:
        args.append("videoflip method=rotate-180")
    args.append("appsink")
    return (" ! ".join(args))


class CH34X:
    def __init__(self, port, baudrate):
        self._serial_port = port
        self.start()

    def start(self):
        self.port = serial.Serial(self._serial_port, 9600, timeout=0.1, write_timeout=0.1)

    def move(self, servo: int, pos: int, delay: int):
         """Move servo to position

        Args:
            servo: servo motor number
            pos: position to go to, usually between 300 - 2500
            delay: Time taken for the traversal in miliseconds

        """
         self.port.write(f"#{servo}P{pos}T{delay}\r\n".encode())

    def set_pos_delay(self, pin, pos, delay):
        print(f"Writing #{pin}P{pos}T{delay}\r\n")
        self.port.write(f"#{pin}P{pos}T{delay}\r\n".encode())

    def stop(self):
        self.port.close()


# Bufferless VideoCapture
# Adapted from https://stackoverflow.com/a/54577746/16723964
class VideoCapture:
    def __init__(self, pipeline, cap_type):
        self._cap = cv.VideoCapture(pipeline, cap_type)
        self.q = Queue()
        self._should_read = Event()
        self._should_read.set()
        self._reader_thread = Thread(target=self._reader)
        self._reader_thread.start()

    # read frames as soon as they are available, keeping only most recent one
    def _reader(self):
        while self._should_read.is_set():
            ret, frame = self._cap.read()
            if not ret:
                break
            if not self.q.empty():
                try:
                    self.q.get_nowait()  # discard previous (unprocessed) frame
                except Queue.Empty:
                    pass
            self.q.put(frame)

    def stop(self):
        self._should_read.clear()
        self._reader_thread.join()
        self._cap.release()

    def isOpened(self):
        return self._should_read.is_set()

    def release(self):
        self.stop()

    def read(self):
        if self._should_read.is_set():
            return True, self.q.get()
        else:
            return False, None


class PlantMonitor4DOF:
    """A class for controlling Two DOF Robotic Arm with two servo motors and a
    camera at the head. Uses an SC08A controller to control the two motors.

    The camera is captured and images are read on demand via HTTP requests.

    Args:
        width: Image width to capture
        height: Image height to capture
        http_port: HTTP port on which to listen
        pins: A :class:`dict` of pins which designate which motor rotates in the
              horizontal plane and which in the vertical plane
        serial_port: The serial port to which :class:`SC08A` is connected
        baudrate: Optional baudrate of the serial port, defaults to 9600 in :code:`SC08A`

    """
    def __init__(self, width: int, height: int, http_port: int,
                 pins: Dict[str, int], serial_port: str,
                 baudrate: Optional[int] = None):
        self._width = width
        self._height = height
        self._flip = True
        self._gst_pipeline = jetson_pipeline(width, height, flip_method=int(self._flip))
        # self._gst_pipeline = gstreamer_pipeline(width, height, flip_180=self._flip)
        # self._cap = cv.VideoCapture(self._gst_pipeline, cv.CAP_GSTREAMER)
        self._cap = VideoCapture(self._gst_pipeline, cv.CAP_GSTREAMER)
        self.port = http_port
        self.app = Flask("Frame Server")
        self._pos_horizontal = 8000
        self._pos_rotate = 1200
        self._pos_a = 1200
        self._pos_b = 1200
        self._min_val = 500
        self._max_val = 2200
        self.pins = pins
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.init_controller()
        self.default_delay = 1000
        self.default_increment = 100
        self.app = Flask("Servo")

    def set_capture_properties(self, width, height, flip_180):
        self._width = width
        self._height = height
        self._flip = flip_180
        self._gst_pipeline = jetson_pipeline(self._width, self._height, self._flip)
        if self._cap.isOpened():
            self._cap.release()
        self._cap = cv.VideoCapture(self._gst_pipeline, cv.CAP_GSTREAMER)

    def init_controller(self):
        self.controller = CH34X(self.serial_port, self.baudrate)

    def start(self):
        self.init_routes()
        serving.run_simple("0.0.0.0", self.port, self.app)

    def _move_servo(self, pin, pos, delay):
        self.controller.set_pos_delay(pin, pos, delay)
        return f"Setting position for motor: {pin} at: {pos} and delay: {delay}"

    def _horizontal(self, delta):
        pin = self.pins["horizontal"]
        if delta < 0:
            self.controller.set_pos_delay(pin, 2000, 100)
            moving = "left"
        elif delta == 0:
            self.controller.set_pos_delay(pin, 8000, 100)
            moving = "stopping"
        elif delta >= 0:
            self.controller.set_pos_delay(pin, 1000, 100)
            moving = "right"
        return f"Moving servo {pin} delta {delta} {moving}"

    def _rotate(self, delta, delay):
        pin = self.pins["rotate"]
        cur_pos = self._pos_rotate
        if cur_pos + delta < self._max_val and cur_pos + delta > self._min_val:
            self._pos_rotate = cur_pos + delta
            return self._move_servo(pin, cur_pos+delta, delay)
        else:
            return "At max/min value"

    def _swing_a(self, delta, delay):
        pin = self.pins["a"]
        cur_pos = self._pos_a
        if cur_pos + delta < self._max_val and cur_pos + delta > self._min_val:
            self._pos_a = cur_pos + delta
            return self._move_servo(pin, cur_pos+delta, delay)
        else:
            return "At max/min value"

    def _swing_b(self, delta, delay):
        pin = self.pins["b"]
        cur_pos = self._pos_b
        if cur_pos + delta < self._max_val and cur_pos + delta > self._min_val:
            self._pos_b = cur_pos + delta
            return self._move_servo(pin, cur_pos+delta, delay)
        else:
            return "At max/min value"

    def _go_rotate(self, direction, delay, delta=None):
        delta = delta or self.default_increment
        if direction == "clockwise":
            pos = cur_pos + delta
        else:
            pos = cur_pos - delta
        return self._rotate(delta, delay)

    def _go_swing_a(self, direction, delay, delta=None):
        delta = delta or self.default_increment
        if direction == "clockwise":
            pos = cur_pos + delta
        else:
            pos = cur_pos - delta
        return self._swing_a(delta, delay)

    def _go_swing_b(self, direction, delay, delta=None):
        delta = delta or self.default_increment
        if direction == "clockwise":
            pos = cur_pos + delta
        else:
            pos = cur_pos - delta
        return self._swing_b(delta, delay)

    def init_routes(self):
        def _maybe_get_delay(request):
            if "delay" not in request.args:
                print("delay not given. Will use 1000")
                delay = self.default_delay
            else:
                delay = int(request.args.get("delay"))
            return delay

        def _maybe_get_delta(request):
            if "delta" in request.args:
                return int(request.args.get("delta"))
            else:
                return None

        def _get_pin():
            if "pin" not in request.args:
                return "Pin not given"
            pin = int(request.args.get("pin"))
            return pin

        @self.app.route("/set_motion_delta", methods=["GET"])
        def _set_motion_delta():
            if "delta" not in request.args:
                return "Delta not given"
            else:
                delta = int(request.args.get("delta"))
            self.default_increment = delta
            return f"Delta set to {delta}"

        @self.app.route("/set_delay", methods=["GET"])
        def _set_speed():
            if "delay" not in request.args:
                return "Delay not given"
            else:
                delay = int(request.args.get("delay"))
            self.default_delay = delay
            return f"Delay set to {delay}"

        @self.app.route("/set_capture_properties", methods=["GET"])
        def _set_capture_properties():
            width = request.args.get("width")
            height = request.args.get("height")
            flip = request.args.get("flip")
            self.set_capture_properties(width, height, flip)

        @self.app.route("/get_frame", methods=["GET"])
        def __get_frame():
            status, img = self._cap.read()
            status, buf = cv.imencode(".jpg", img)
            data = base64.b64encode(buf)
            return data

        @self.app.route("/horizontal", methods=["GET"])
        def _horizontal():
            delta = _maybe_get_delta(request)
            return self._horizontal(delta)

        @self.app.route("/rotate", methods=["GET"])
        def _rotate():
            delay = _maybe_get_delay(request)
            delta = _maybe_get_delta(request)
            return self._rotate(delta, delay)

        @self.app.route("/swing_a", methods=["GET"])
        def _swing_a():
            delay = _maybe_get_delay(request)
            delta = _maybe_get_delta(request)
            return self._swing_a(delta, delay)

        @self.app.route("/swing_b", methods=["GET"])
        def _swing_b():
            delay = _maybe_get_delay(request)
            delta = _maybe_get_delta(request)
            return self._swing_b(delta, delay)

        @self.app.route("/reset", methods=["GET"])
        def _reset_all():
            self.controller.stop()
            self.controller.start()
            return "Issued STOP and START"

        @self.app.route("/stop", methods=["GET"])
        def _close():
            self.controller.stop()
            return "Issued STOP"

        @self.app.route("/start", methods=["GET"])
        def _start():
            self.controller.start()
            return "Initialized the controller"


if __name__ == '__main__':
    try:
        arm = PlantMonitor4DOF(640, 480, 8080, {"horizontal": 5, "rotate": 6, "a": 7, "b": 8}, "/dev/ttyUSB0")
        arm.start()
    except KeyboardInterrupt:
        arm._cap._cap.release()
        arm.controller.stop()
