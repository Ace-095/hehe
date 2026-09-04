"""
camera_source.py — The single camera abstraction layer for the entire pipeline.

Ground rule (Phase 1 spec, rule 5): every downstream consumer (camera_manager.py,
qr_pipeline.py, test scripts) acquires frames through CameraSource.get_frame().
They never know or care whether the frame came from real hardware, a Gazebo
simulation, or a synthetic test image.

Three concrete implementations are provided here:

    SyntheticQRSource  — cheapest and fastest tier. Pure Python, no sim needed.
                         Use for rapid iteration on vision pipeline logic.
    GazeboCameraSource — dev-machine sim tier. Subscribes to a ROS 2 topic
                         bridged from Gazebo. Never imported on the Pi in
                         production — ROS 2 is not installed there.
    Picamera2Source    — real hardware. Stub only until Phase 9 when the camera
                         modules are physically available to verify the API.

Select via: CAMERA_SOURCE=synthetic|gazebo|picamera2 env var (see config.py).

Factory: call get_camera_source() — it reads config and returns the right instance.
"""

import time
import threading
from abc import ABC, abstractmethod
from typing import Optional

import numpy as np


# ---------------------------------------------------------------------------
# Abstract base
# ---------------------------------------------------------------------------

class CameraSource(ABC):
    """
    Abstract camera source.

    Lifecycle:
        source = get_camera_source()
        source.start()
        try:
            frame = source.get_frame()   # blocks until frame available
        finally:
            source.stop()

    Contract:
        - start() must be idempotent (calling twice is a no-op).
        - get_frame() returns a uint8 numpy array shaped (H, W, 3) in BGR
          colour order, matching OpenCV conventions. Callers must not mutate
          the returned array.
        - stop() is idempotent and releases all resources.
        - All methods are thread-safe: get_frame() may be called from a
          different thread than start()/stop().
    """

    @abstractmethod
    def start(self) -> None:
        """Open the camera / subscription / generator. Idempotent."""
        ...

    @abstractmethod
    def get_frame(self) -> np.ndarray:
        """
        Return the next frame as a uint8 BGR numpy array (H, W, 3).
        Blocks until a frame is available or raises RuntimeError on failure.
        """
        ...

    @abstractmethod
    def stop(self) -> None:
        """Release resources. Idempotent."""
        ...


# ---------------------------------------------------------------------------
# Synthetic QR source (no simulator, no hardware — pure Python)
# ---------------------------------------------------------------------------

class SyntheticQRSource(CameraSource):
    """
    Generates frames containing a real, decodable QR code composited onto a
    background, with optional slow pan/zoom for motion-robustness testing.

    Primary dependency: opencv (cv2.QRCodeEncoder — available via system OpenCV,
    no pip install needed). Fallback: qrcode[pil] if installed.

    Parameters
    ----------
    payload : str
        QR code data string. Default read from config.SYNTHETIC_QR_PAYLOAD.
    width, height : int
        Output frame dimensions in pixels.
    fps : float
        Target frame rate (limits CPU usage in tight loops).
    pan_speed : float
        Pixels per second the QR code target moves across the frame.
        Set to 0 to keep the target centred (static).
    zoom_amplitude : float
        Fraction of frame width the QR target scales up/down over time
        (0 = no zoom, 0.2 = ±20% of base size). Simulates altitude change.
    """

    def __init__(
        self,
        payload: Optional[str] = None,
        width: int = 640,
        height: int = 480,
        fps: float = 10.0,
        pan_speed: float = 20.0,
        zoom_amplitude: float = 0.15,
    ):
        if payload is None:
            from config import config
            payload = config.SYNTHETIC_QR_PAYLOAD

        self._payload = payload
        self._width = width
        self._height = height
        self._frame_interval = 1.0 / fps
        self._pan_speed = pan_speed
        self._zoom_amplitude = zoom_amplitude

        self._started = False
        self._lock = threading.Lock()
        self._t0: float = 0.0
        self._qr_image = None   # uint8 greyscale numpy array — generated once in start()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        with self._lock:
            if self._started:
                return
            self._t0 = time.time()
            self._qr_image = self._render_qr_image()
            self._started = True

    def stop(self) -> None:
        with self._lock:
            self._started = False
            self._qr_image = None

    # ------------------------------------------------------------------
    # Frame generation
    # ------------------------------------------------------------------

    def get_frame(self) -> np.ndarray:
        if not self._started:
            raise RuntimeError("SyntheticQRSource.start() must be called before get_frame()")

        elapsed = time.time() - self._t0

        # --- Compute current pan position (wraps at frame boundary) ---
        pan_range = self._width * 0.5   # half-frame pan range
        pan_x = int(pan_range * 0.5 * (1.0 + np.sin(elapsed * self._pan_speed / pan_range)))
        pan_y = int(pan_range * 0.5 * (1.0 + np.cos(elapsed * self._pan_speed / pan_range * 0.7)))

        # --- Compute current QR size (zoom) ---
        base_size = min(self._width, self._height) // 4
        zoom_factor = 1.0 + self._zoom_amplitude * np.sin(elapsed * 0.3)
        qr_size = max(32, int(base_size * zoom_factor))

        # --- Composite QR onto background ---
        frame_bgr = self._composite(pan_x, pan_y, qr_size)

        # Throttle to target fps
        time.sleep(max(0.0, self._frame_interval - 0.001))

        return frame_bgr

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _render_qr_image(self) -> np.ndarray:
        """Generate the base QR code as a uint8 greyscale numpy array.

        Primary method: cv2.QRCodeEncoder (available via system OpenCV — no
        extra packages needed beyond numpy + opencv).

        Fallback: qrcode + Pillow (if installed — produces higher-quality codes
        with configurable error correction levels, better at larger sizes).
        """
        import cv2

        # --- Primary: cv2.QRCodeEncoder ---
        try:
            encoder = cv2.QRCodeEncoder.create()
            qr_grey = encoder.encode(self._payload)
            # qr_grey is a 2D uint8 array (greyscale). Values: 0=black, 255=white.
            return qr_grey
        except AttributeError:
            pass  # cv2 build doesn't have QRCodeEncoder — try fallback

        # --- Fallback: qrcode + Pillow ---
        try:
            import qrcode as _qrcode
            qr = _qrcode.QRCode(
                version=None,
                error_correction=_qrcode.constants.ERROR_CORRECT_M,
                box_size=10,
                border=4,
            )
            qr.add_data(self._payload)
            qr.make(fit=True)
            img_pil = qr.make_image(fill_color="black", back_color="white").convert("L")
            return np.array(img_pil, dtype=np.uint8)
        except ImportError:
            pass

        raise RuntimeError(
            "SyntheticQRSource: cannot generate QR code. "
            "Ensure opencv-contrib-python (cv2.QRCodeEncoder) or "
            "'qrcode[pil]' is installed. "
            "System cv2 check: python3 -c \"import cv2; cv2.QRCodeEncoder.create()\""
        )

    def _composite(self, qr_x: int, qr_y: int, qr_size: int) -> np.ndarray:
        """
        Resize the QR image to qr_size × qr_size and paste it at (qr_x, qr_y)
        onto a grey background. Returns uint8 BGR ndarray.
        """
        import cv2

        # Background: 50% grey (muted grass/tarmac stand-in)
        bg = np.full((self._height, self._width, 3), 100, dtype=np.uint8)

        # Resize QR to current zoom size using nearest-neighbour
        # (preserves hard black/white edges, critical for QR detection)
        qr_resized = cv2.resize(
            self._qr_image, (qr_size, qr_size), interpolation=cv2.INTER_NEAREST
        )

        # Ensure the QR array is 2D greyscale before converting to BGR
        if qr_resized.ndim == 3:
            qr_resized = cv2.cvtColor(qr_resized, cv2.COLOR_BGR2GRAY)

        # Convert greyscale to BGR for compositing
        qr_bgr = cv2.cvtColor(qr_resized, cv2.COLOR_GRAY2BGR)

        # Clamp paste position so QR stays fully in-frame
        paste_x = min(max(qr_x, 0), self._width - qr_size)
        paste_y = min(max(qr_y, 0), self._height - qr_size)

        bg[paste_y:paste_y + qr_size, paste_x:paste_x + qr_size] = qr_bgr
        return bg


# ---------------------------------------------------------------------------
# Gazebo camera source (simulation, dev machine only, requires ROS 2)
# ---------------------------------------------------------------------------

class GazeboCameraSource(CameraSource):
    """
    Reads camera frames from a ROS 2 topic bridged out of Gazebo via
    gz-ros-bridge (ros_gz_bridge).

    SIMULATION-ONLY: This class imports rclpy and sensor_msgs, which are only
    available on the dev machine. It must never be instantiated on the Pi.
    The import of rclpy is deferred to start() so that importing camera_source
    at the top of backend modules on the Pi doesn't fail — only instantiation
    and calling start() will fail if ROS 2 is absent.

    The camera topic name comes from config.ROS2_CAMERA_TOPIC (default
    /iris/camera/image_raw). This value is NEED TEST — verify the actual
    topic name by running `ros2 topic list` after launching the world file
    in Phase 1b, and update ROS2_CAMERA_TOPIC in your env accordingly.

    Parameters
    ----------
    topic : str | None
        Override ROS 2 topic. Defaults to config.ROS2_CAMERA_TOPIC.
    node_name : str
        ROS 2 node name (must be unique within the ROS graph).
    timeout_s : float
        How long get_frame() waits for the first message before raising.
    """

    def __init__(
        self,
        topic: Optional[str] = None,
        node_name: str = "camera_source_node",
        timeout_s: float = 5.0,
    ):
        if topic is None:
            from config import config
            topic = config.ROS2_CAMERA_TOPIC

        self._topic = topic
        self._node_name = node_name
        self._timeout_s = timeout_s

        self._started = False
        self._lock = threading.Lock()
        self._frame_event = threading.Event()
        self._latest_frame: Optional[np.ndarray] = None

        # ROS 2 objects — initialised in start()
        self._node = None
        self._subscription = None
        self._spin_thread: Optional[threading.Thread] = None
        self._spin_executor = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        with self._lock:
            if self._started:
                return

            # Deferred ROS 2 import — fails clearly on Pi
            try:
                import rclpy
                from rclpy.executors import SingleThreadedExecutor
                from sensor_msgs.msg import Image as ROSImage
            except ImportError:
                raise ImportError(
                    "rclpy / sensor_msgs not found. GazeboCameraSource requires "
                    "ROS 2 Humble on the dev machine. This source must not be "
                    "used on the Raspberry Pi."
                )

            rclpy.init()
            self._node = rclpy.create_node(self._node_name)

            def _image_callback(msg: "ROSImage"):  # noqa: F821
                """Convert ROS Image message (rgb8 or bgr8) to BGR ndarray."""
                try:
                    import cv2
                    # Build numpy array from raw bytes
                    arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                        msg.height, msg.width, -1
                    )
                    # ros_gz_bridge publishes rgb8 by default; convert to BGR
                    if msg.encoding in ("rgb8", "RGB8"):
                        arr = arr[:, :, ::-1].copy()
                    elif msg.encoding not in ("bgr8", "BGR8"):
                        # Attempt greyscale -> BGR fallback
                        if arr.ndim == 2 or arr.shape[2] == 1:
                            arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
                    self._latest_frame = arr
                    self._frame_event.set()
                except Exception as exc:
                    self._node.get_logger().warning(
                        f"GazeboCameraSource: frame decode failed: {exc}"
                    )

            from sensor_msgs.msg import Image as ROSImage  # noqa: F811
            self._subscription = self._node.create_subscription(
                ROSImage, self._topic, _image_callback, qos_profile=10
            )

            self._spin_executor = SingleThreadedExecutor()
            self._spin_executor.add_node(self._node)

            self._spin_thread = threading.Thread(
                target=self._spin_executor.spin,
                name="ros2-camera-spin",
                daemon=True,
            )
            self._spin_thread.start()
            self._started = True

    def stop(self) -> None:
        with self._lock:
            if not self._started:
                return
            self._started = False
            if self._spin_executor is not None:
                self._spin_executor.shutdown(timeout_sec=1.0)
                self._spin_executor = None
            if self._node is not None:
                self._node.destroy_node()
                self._node = None
            try:
                import rclpy
                rclpy.shutdown()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Frame retrieval
    # ------------------------------------------------------------------

    def get_frame(self) -> np.ndarray:
        if not self._started:
            raise RuntimeError("GazeboCameraSource.start() must be called before get_frame()")

        # Wait for the first frame (or a refreshed event) up to timeout
        got = self._frame_event.wait(timeout=self._timeout_s)
        if not got or self._latest_frame is None:
            raise RuntimeError(
                f"GazeboCameraSource: no frame received on '{self._topic}' "
                f"within {self._timeout_s}s. Check that Gazebo + gz-ros-bridge "
                f"are running and that ROS2_CAMERA_TOPIC matches the actual topic "
                f"(run `ros2 topic list` to verify). NEED TEST."
            )

        # Clear event so next call blocks until a fresh frame arrives
        self._frame_event.clear()
        return self._latest_frame


# ---------------------------------------------------------------------------
# Picamera2 source (real hardware stub — Phase 9)
# ---------------------------------------------------------------------------

class Picamera2Source(CameraSource):
    """
    Real hardware camera source for the Raspberry Pi.

    STUB — NOT IMPLEMENTED YET. Phase 9 hardware bring-up task.

    This class is intentionally incomplete. Picamera2's full API surface
    (configuration dicts, format strings, ISP tuning YAML path, autofocus
    trigger semantics) must be verified against the actual camera modules
    before this is written. Guessing it now would introduce hard-to-detect
    bugs at competition time.

    What to do in Phase 9:
      1. `libcamera-hello --list-cameras` — confirm both cameras are visible.
      2. Check focus motor sanity: `libcamera-still --autofocus-mode=auto -o test.jpg`
      3. Install picamera2: `pip install picamera2`
      4. Read: https://datasheets.raspberrypi.com/camera/picamera2-manual.pdf
      5. Implement start(), get_frame(), stop() using Picamera2.capture_array()
         in a background thread feeding a queue — same pattern as this file.
      6. Set CAMERA_SOURCE=picamera2 and verify frames decode QR codes.

    Parameters
    ----------
    camera_index : int
        Camera index (0 = Camera 1 belly, 1 = Camera 2 belly — NEED TEST,
        depends on how the cameras are enumerated after wiring).
    width, height : int
        Desired capture resolution. NEED TEST against actual module.
    """

    def __init__(self, camera_index: int = 0, width: int = 1920, height: int = 1080):
        self._camera_index = camera_index
        self._width = width
        self._height = height

    def start(self) -> None:
        raise NotImplementedError(
            "Picamera2Source is not yet implemented. "
            "See Phase 9 in the implementation plan and the docstring above."
        )

    def get_frame(self) -> np.ndarray:
        raise NotImplementedError(
            "Picamera2Source is not yet implemented. "
            "See Phase 9 in the implementation plan and the docstring above."
        )

    def stop(self) -> None:
        # Idempotent — nothing to clean up until implemented
        pass


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------

def get_camera_source(override: Optional[str] = None, **kwargs) -> CameraSource:
    """
    Factory: return a CameraSource instance selected by config.CAMERA_SOURCE
    (or the ``override`` argument for testing).

    Parameters
    ----------
    override : str | None
        If provided, overrides config.CAMERA_SOURCE. Valid: synthetic|gazebo|picamera2.
    **kwargs
        Passed through to the selected CameraSource constructor.

    Raises
    ------
    ValueError
        Unknown camera source name.
    """
    from config import config

    source_name = (override or config.CAMERA_SOURCE).strip().lower()

    if source_name == "synthetic":
        return SyntheticQRSource(**kwargs)
    elif source_name == "gazebo":
        return GazeboCameraSource(**kwargs)
    elif source_name == "picamera2":
        return Picamera2Source(**kwargs)
    else:
        raise ValueError(
            f"Unknown CAMERA_SOURCE '{source_name}'. "
            "Valid values: synthetic | gazebo | picamera2"
        )
