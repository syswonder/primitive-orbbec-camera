#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""orbbec_camera_rbnx — Orbbec Gemini 330-series RGBD primitive
(capability_id=orbbec_camera).

Owns `robonix/primitive/camera/*`. Uses the apt-installed
ros-humble-orbbec-camera package (no source build needed).

Capability surface:
  primitive/camera/driver         rpc gRPC (lifecycle)
  primitive/camera/rgb            topic_out ROS2 (continuous, raw)
  primitive/camera/depth          topic_out ROS2 (continuous, raw)
  primitive/camera/snapshot       rpc MCP (one-shot RGB JPEG — VLM-facing)
  primitive/camera/depth_snapshot rpc MCP (one-shot depth as 8-bit JPEG)
  primitive/camera/extrinsics     topic_out ROS2 (latched TF)
  primitive/camera/intrinsics     topic_out ROS2 (latched CameraInfo)

Lifecycle:
    on_init      — spawn orbbec_camera.launch.py with camera_name + profiles
                   → subscribe rgb+depth + intrinsics relay →
                   resolve extrinsics via tf2 → wait for first RGB frame →
                   declare rgb/depth/intrinsics/extrinsics topic_out
                   + snapshot + depth_snapshot.
    on_shutdown  — kill orbbec subprocess.

Config (from manifest):
    camera_name          default "camera"
    camera_model         default "gemini330_series"
    color_profile        default "640x480x15"
    depth_profile        default "640x480x15"
    depth_registration   default false
    enable_color         default true
    enable_depth         default true
    enable_point_cloud   default false
    enable_imu           default false
    serial_number        default "" (optional device selector)
    usb_port             default "" (optional device selector)
    sentinel_timeout_s   default 30.0
    camera_info_topic    default "/<camera_name>/color/camera_info"
    intrinsics_topic     default "/<camera_name>/intrinsics"
    extrinsics_topic     default "/<camera_name>/extrinsics"
    base_frame           default "base_link"
    cam_frame            default "camera_color_optical_frame"
"""
from __future__ import annotations

import logging
import os
import signal
import subprocess
import threading
import time
from io import BytesIO
from pathlib import Path

import numpy as np

from robonix_api import Primitive, Ok, Err
from .launch_config import device_preset_args, device_selector_args

logging.basicConfig(
    level=os.environ.get("ORBBEC_LOG_LEVEL", "INFO"),
    format="[orbbec] %(message)s",
)
log = logging.getLogger("orbbec")

cap = Primitive(id="orbbec_camera", namespace="robonix/primitive/camera")

_pkg_root: Path = Path(__file__).resolve().parent.parent
_orbbec_proc: subprocess.Popen | None = None

# ── snapshot state ───────────────────────────────────────────────────────────
_state_lock = threading.Lock()
_latest_rgb_jpeg: bytes | None = None
_latest_depth_jpeg: bytes | None = None
_rgb_frame_id: str = "camera_color_optical_frame"
_depth_frame_id: str = "camera_depth_optical_frame"
_rgb_received = threading.Event()
_extrinsics_pub = None  # rclpy publisher for the latched TF
_intrinsics_pub = None  # rclpy publisher for the latched CameraInfo
_intrinsics_published = False  # publish K once; intrinsics are static


def _parse_profile(profile: str) -> tuple[int, int, int]:
    """Parse 'WxHxFPS' string → (width, height, fps)."""
    parts = profile.split("x")
    if len(parts) != 3:
        raise ValueError(f"invalid profile format '{profile}' — expected WxHxFPS")
    return int(parts[0]), int(parts[1]), int(parts[2])


def _spawn_orbbec(cfg: dict) -> None:
    """Launch ros2 launch orbbec_camera orbbec_camera.launch.py with config args."""
    global _orbbec_proc
    cam = cfg.get("camera_name", "camera")
    cam_model = cfg.get("camera_model", "gemini330_series")
    depth_reg = cfg.get("depth_registration", False)
    enable_color = cfg.get("enable_color", True)
    enable_depth = cfg.get("enable_depth", True)
    enable_pc = cfg.get("enable_point_cloud", False)
    enable_imu = cfg.get("enable_imu", False)

    # Parse profiles — use 15fps default to reduce USB3 bus bandwidth on
    # embedded platforms where USB3 and Ethernet share a PCIe root complex.
    color_w, color_h, color_fps = 0, 0, 0
    depth_w, depth_h, depth_fps = 0, 0, 0
    try:
        color_w, color_h, color_fps = _parse_profile(cfg.get("color_profile", "640x480x15"))
    except ValueError:
        log.warning("invalid color_profile, using default (640x480x15)")
        color_w, color_h, color_fps = 640, 480, 30
    try:
        depth_w, depth_h, depth_fps = _parse_profile(cfg.get("depth_profile", "640x480x15"))
    except ValueError:
        log.warning("invalid depth_profile, using default (640x480x15)")
        depth_w, depth_h, depth_fps = 640, 480, 30

    # Use the model-specific launch file for proper parameter handling.
    # gemini_330_series.launch.py handles all flat params directly (no YAML
    # merging) and is designed for Gemini 330-series cameras (335/336/330/etc).
    # Fall back to orbbec_camera.launch.py for other models.
    if cam_model == "gemini330_series":
        launch_file = "gemini_330_series.launch.py"
    else:
        launch_file = "orbbec_camera.launch.py"

    selectors = device_selector_args(cfg)
    preset = device_preset_args(cfg)
    args = [
        "ros2", "launch", "orbbec_camera", launch_file,
        f"camera_name:={cam}",
        f"depth_registration:={'true' if depth_reg else 'false'}",
        f"enable_color:={'true' if enable_color else 'false'}",
        f"enable_depth:={'true' if enable_depth else 'false'}",
        f"enable_point_cloud:={'true' if enable_pc else 'false'}",
        f"enable_imu:={'true' if enable_imu else 'false'}",
        f"color_width:={color_w}",
        f"color_height:={color_h}",
        f"color_fps:={color_fps}",
        f"depth_width:={depth_w}",
        f"depth_height:={depth_h}",
        f"depth_fps:={depth_fps}",
    ]
    # Preserve the upstream string-typed Default preset unless the deployment
    # explicitly selects another named preset.
    args.extend(preset)
    # Multi-camera robots must select the intended physical camera explicitly;
    # otherwise the upstream driver may bind whichever Orbbec enumerates first.
    args.extend(selectors)
    if launch_file == "orbbec_camera.launch.py":
        args.append(f"camera_model:={cam_model}")

    log_path = _pkg_root / "rbnx-build" / "data" / "orbbec.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_fh = open(log_path, "ab", buffering=0)
    selector_text = ", ".join(selectors) if selectors else "auto-discovery"
    log.info("spawning orbbec camera (model=%s, cam=%s, device=%s) → %s",
             cam_model, cam, selector_text, log_path)
    log.debug("launch args: %s", " ".join(args))
    _orbbec_proc = subprocess.Popen(
        args, stdout=log_fh, stderr=log_fh, start_new_session=True,
    )


def _kill_orbbec() -> None:
    p = _orbbec_proc
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        p.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


# ── extrinsics: tf2 lookup once at startup, republish on a latched topic ────
def _publish_extrinsics_when_ready(base_frame: str, cam_frame: str, topic: str) -> None:
    """Resolve `base_frame → cam_frame` from tf2, publish on latched extrinsics
    topic, exit. tf2 is used here purely as the local-to-this-primitive
    mechanism for reading the URDF chain — consumers never touch tf2; they
    go through the declared `primitive/camera/extrinsics` contract."""
    from rclpy.duration import Duration  # type: ignore
    from rclpy.time import Time  # type: ignore
    from tf2_ros import Buffer, TransformListener  # type: ignore
    from robonix_api.ros import RosBackend
    node = RosBackend.get().node
    tf_buf = Buffer()
    TransformListener(tf_buf, node)
    deadline = time.monotonic() + 60.0
    while time.monotonic() < deadline:
        try:
            tf = tf_buf.lookup_transform(base_frame, cam_frame, Time(), Duration(seconds=0.5))
        except Exception:  # noqa: BLE001
            time.sleep(0.5)
            continue
        tf.header.frame_id = base_frame
        tf.child_frame_id = cam_frame
        if _extrinsics_pub is not None:
            _extrinsics_pub.publish(tf)
        t = tf.transform.translation
        log.info("published extrinsics %s→%s: (%.3f, %.3f, %.3f) → %s",
                 base_frame, cam_frame, t.x, t.y, t.z, topic)
        return
    log.warning("extrinsics publish gave up — tf2 chain %s→%s not resolvable.",
                base_frame, cam_frame)


# ── image conversion ─────────────────────────────────────────────────────────
def _ros_image_to_jpeg(msg) -> bytes:
    """Encode a sensor_msgs/Image into JPEG bytes.
    Supports: rgb8, bgr8, rgba8, bgra8, mono8, 16uc1, 32fc1."""
    h, w = msg.height, msg.width
    enc = msg.encoding.lower()
    if enc == "rgb8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
    elif enc == "bgr8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)[:, :, ::-1]
    elif enc == "rgba8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)[:, :, :3]
    elif enc == "bgra8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)[:, :, :3][:, :, ::-1]
    elif enc == "mono8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        arr = np.stack([arr, arr, arr], axis=-1)
    elif enc == "16uc1":
        raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
        arr = (raw / raw.max() * 255).astype(np.uint8) if raw.max() > 0 else np.zeros((h, w), np.uint8)
        arr = np.stack([arr, arr, arr], axis=-1)
    elif enc == "32fc1":
        raw = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
        valid = np.isfinite(raw)
        if valid.any():
            mn, mx = raw[valid].min(), raw[valid].max()
            norm = np.where(valid, (raw - mn) / max(mx - mn, 1e-6) * 255, 0).astype(np.uint8)
        else:
            norm = np.zeros((h, w), np.uint8)
        arr = np.stack([norm, norm, norm], axis=-1)
    else:
        raise ValueError(f"unsupported image encoding: {enc}")
    from PIL import Image as PILImage
    buf = BytesIO()
    PILImage.fromarray(np.ascontiguousarray(arr)).save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def _on_rgb(msg) -> None:
    global _latest_rgb_jpeg, _rgb_frame_id
    # INIT uses the permanent RGB subscription as its readiness sentinel.
    # Avoid Capability.wait_for_topic(): destroying its temporary subscription
    # while the Zenoh executor is spinning can terminate the shared spin loop.
    _rgb_received.set()
    try:
        jpg = _ros_image_to_jpeg(msg)
        with _state_lock:
            _latest_rgb_jpeg = jpg
            if msg.header.frame_id:
                _rgb_frame_id = msg.header.frame_id
    except Exception as e:  # noqa: BLE001
        log.warning("RGB conversion error: %s", e)


def _on_depth(msg) -> None:
    global _latest_depth_jpeg, _depth_frame_id
    try:
        jpg = _ros_image_to_jpeg(msg)
        with _state_lock:
            _latest_depth_jpeg = jpg
            if msg.header.frame_id:
                _depth_frame_id = msg.header.frame_id
    except Exception as e:  # noqa: BLE001
        log.warning("depth conversion error: %s", e)


# ── MCP snapshot tools (typed against codegen MCP dataclasses) ──────────────
import builtin_interfaces_mcp  # noqa: E402
import std_msgs_mcp  # noqa: E402
from sensor_msgs_mcp import Image  # noqa: E402
from std_msgs_mcp import Empty  # noqa: E402


def _now_header(frame_id: str) -> std_msgs_mcp.Header:
    now = time.time()
    sec = int(now)
    ns = int((now % 1) * 1e9) % 1_000_000_000
    return std_msgs_mcp.Header(
        stamp=builtin_interfaces_mcp.Time(sec=sec, nanosec=ns),
        frame_id=frame_id,
    )


def _jpeg_to_image_mcp(jpg: bytes, frame_id: str) -> Image:
    from PIL import Image as PILImage
    im = PILImage.open(BytesIO(jpg))
    w, h = im.size
    return Image(
        header=_now_header(frame_id),
        height=h, width=w,
        encoding="jpeg",
        is_bigendian=0,
        step=len(jpg),
        data=jpg,
    )


def _empty_image_error(reason: str) -> Image:
    """Return a tiny black 1x1 JPEG when we can't deliver a frame."""
    from PIL import Image as PILImage
    buf = BytesIO()
    PILImage.new("RGB", (1, 1), (0, 0, 0)).save(buf, format="JPEG")
    return _jpeg_to_image_mcp(buf.getvalue(), f"error:{reason}")


@cap.mcp("robonix/primitive/camera/snapshot")
def snapshot(msg: Empty) -> Image:
    """PRIMARY perception tool. Use freely — between every chassis/cmd
    burst — to see what's in front of the robot and decide what to do
    next. Returns the current RGB frame as a JPEG-encoded
    sensor_msgs/Image (encoding='jpeg', data=JPEG bytes)."""
    with _state_lock:
        data = _latest_rgb_jpeg
        frame_id = _rgb_frame_id
    if data is None:
        return _empty_image_error("no RGB frame received yet")
    return _jpeg_to_image_mcp(data, frame_id)


@cap.mcp("robonix/primitive/camera/depth_snapshot")
def depth_snapshot(msg: Empty) -> Image:
    """Depth snapshot as 8-bit JPEG (normalized for visualization).
    Returns sensor_msgs/Image with encoding='jpeg'. For actual metric
    depth, subscribe to robonix/primitive/camera/depth (16UC1)."""
    with _state_lock:
        data = _latest_depth_jpeg
        frame_id = _depth_frame_id
    if data is None:
        return _empty_image_error("no depth frame received yet")
    return _jpeg_to_image_mcp(data, frame_id)


# ── lifecycle ────────────────────────────────────────────────────────────────
@cap.on_init
def init(cfg: dict):
    """REGISTERED → INACTIVE: spawn orbbec, subscribe RGB+depth, declare."""
    global _extrinsics_pub, _intrinsics_pub
    cam = cfg.get("camera_name", "camera")
    # Orbbec topics — orbbec_camera.launch.py publishes under /<camera_name>/
    rgb_topic = cfg.get("rgb_topic", f"/{cam}/color/image_raw")
    depth_topic = cfg.get("depth_topic", f"/{cam}/depth/image_raw")
    sentinel_timeout = float(cfg.get("sentinel_timeout_s", 30.0))
    _rgb_received.clear()

    # ── extrinsics + intrinsics config ───────────────────────────────────
    camera_info_topic = cfg.get("camera_info_topic") or os.environ.get(
        "ORBBEC_CAMERA_INFO_TOPIC", f"/{cam}/color/camera_info")
    intrinsics_topic = cfg.get("intrinsics_topic") or os.environ.get(
        "ORBBEC_INTRINSICS_TOPIC", f"/{cam}/intrinsics")
    extrinsics_topic = cfg.get("extrinsics_topic") or os.environ.get(
        "ORBBEC_EXTRINSICS_TOPIC", f"/{cam}/extrinsics")
    base_frame = cfg.get("base_frame") or os.environ.get(
        "ORBBEC_BASE_FRAME", "base_link")
    cam_frame = cfg.get("cam_frame") or os.environ.get(
        "ORBBEC_RGB_FRAME_ID", "camera_color_optical_frame")

    try:
        _spawn_orbbec(cfg)
    except Exception as e:  # noqa: BLE001
        return Err(f"spawn orbbec failed: {e}")

    # Subscribe RGB + depth via robonix_api (declare=False — we declare
    # the ros2 topic_out interfaces explicitly below, after sentinel passes).
    cap.create_subscription(
        "robonix/primitive/camera/rgb",
        topic=rgb_topic, msg_type="Image",
        callback=_on_rgb, qos="best_effort", declare=False,
    )
    cap.create_subscription(
        "robonix/primitive/camera/depth",
        topic=depth_topic, msg_type="Image",
        callback=_on_depth, qos="best_effort", declare=False,
    )

    # ── latched extrinsics publisher ─────────────────────────────────────
    from geometry_msgs.msg import TransformStamped  # type: ignore
    _extrinsics_pub = cap.create_publisher(
        "robonix/primitive/camera/extrinsics",
        topic=extrinsics_topic, msg_type=TransformStamped, qos="latched",
    )
    threading.Thread(
        target=_publish_extrinsics_when_ready,
        args=(base_frame, cam_frame, extrinsics_topic),
        daemon=True,
    ).start()

    # ── latched intrinsics relay ─────────────────────────────────────────
    from sensor_msgs.msg import CameraInfo  # type: ignore
    _intrinsics_pub = cap.create_publisher(
        "robonix/primitive/camera/intrinsics",
        topic=intrinsics_topic, msg_type=CameraInfo, qos="latched",
    )

    def _on_camera_info(msg, _topic=intrinsics_topic):
        global _intrinsics_published
        if _intrinsics_pub is None:
            return
        # Validate K before relaying: skip zero/partial CameraInfo.
        k = list(msg.k) if hasattr(msg, "k") else list(getattr(msg, "K", []))
        if len(k) < 6 or k[0] <= 0 or k[4] <= 0:
            return
        # Relay on EVERY frame, not once. Scene subscribes with
        # DURABILITY=VOLATILE so a one-shot publish before the subscriber
        # connects would be missed. Stream K continuously (K is static,
        # cost is negligible; publisher is latched for TRANSIENT_LOCAL).
        _intrinsics_pub.publish(msg)
        if not _intrinsics_published:
            _intrinsics_published = True
            log.info("publishing intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f "
                     "%dx%d -> %s",
                     k[0], k[4], k[2], k[5], msg.width, msg.height, _topic)

    cap.create_subscription(
        "robonix/primitive/camera/intrinsics",
        topic=camera_info_topic, msg_type="CameraInfo",
        callback=_on_camera_info, qos="best_effort", declare=False,
    )

    # Give-up watchdog: warn if no usable CameraInfo lands within deadline.
    def _warn_if_no_intrinsics(source_topic: str, deadline_s: float = 60.0) -> None:
        end = time.monotonic() + deadline_s
        while time.monotonic() < end:
            if _intrinsics_published:
                return
            time.sleep(0.5)
        if not _intrinsics_published:
            log.warning("intrinsics never published — no usable CameraInfo "
                        "seen on %s", source_topic)

    threading.Thread(
        target=_warn_if_no_intrinsics,
        args=(camera_info_topic,),
        daemon=True,
    ).start()

    # Gate INIT on the permanent RGB subscription. USB cameras can lag on cold
    # boot, but no temporary ROS subscription should be created or destroyed
    # while the process-wide executor is spinning.
    if not _rgb_received.wait(timeout=sentinel_timeout):
        _kill_orbbec()
        return Err(f"no Image on {rgb_topic} within {sentinel_timeout:.1f}s")

    cap.declare_ros2_topic(
        "robonix/primitive/camera/rgb",
        topic=rgb_topic, qos="best_effort",
    )
    cap.declare_ros2_topic(
        "robonix/primitive/camera/depth",
        topic=depth_topic, qos="best_effort",
    )
    log.info("init complete: rgb=%s depth=%s + intrinsics=%s extrinsics=%s "
             "+ snapshot/depth_snapshot MCP exposed",
             rgb_topic, depth_topic, intrinsics_topic, extrinsics_topic)
    return Ok()


@cap.on_shutdown
def shutdown():
    _kill_orbbec()
    return Ok()


if __name__ == "__main__":
    cap.run()
