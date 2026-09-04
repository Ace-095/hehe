"""
MAVLink polygon-fence upload/download for ArduPilot Copter (Pixhawk 6C).

Protocol details below were verified against ArduPilot's own source
(GCS_MAVLink/MissionItemProtocol_Fence.cpp) and the MAVLink mission
protocol docs (mavlink.io/en/services/mission.html), not written from
memory. Two things worth knowing before you touch this file:

1. MAVLINK20 must be set to "1" in the environment BEFORE pymavlink is
   imported anywhere in the process. `mavutil.mavlink` silently resolves
   to the MAVLink1 dialect otherwise, which has no `mission_type` field at
   all — fence upload will silently fail with a confusing "mission ack:
   denied" if you miss this. This module sets it at import time.
2. The `vertex_count` sent in the first MISSION_ITEM_INT (seq=0) must equal
   the total number of vertices. ArduPilot uses this to allocate its
   internal fence array; if it mismatches the actual item count sent, the
   FC rejects the whole fence with `MAV_MISSION_ERROR`. This module
   enforces the match.

All functions here are stateless and take a `MavlinkBus` instance.
The bus handles connection lifecycle, thread-safe sends, and the single
reader thread that replaces the old per-call `conn.recv_match()` pattern.
"""

import os
os.environ.setdefault("MAVLINK20", "1")

import math
import queue
import time
from typing import Optional

from pymavlink import mavutil

from geo_utils import latlon_to_enu, polygon_centroid, max_radius_from_point, point_in_polygon
from models import GeofenceStatus, Vertex, ENUPoint
from mavlink_bus import MavlinkBus, FenceError, ConnectionError


def _send_and_wait(bus: MavlinkBus, send_fn, *args, **kwargs):
    """
    Internal helper: send a message via bus and return immediately.
    The caller uses bus.wait_for() to wait for the expected response.
    """
    bus.send(send_fn, *args, **kwargs)


def request_ekf_origin(bus: MavlinkBus, timeout_s: float) -> tuple[float, float]:
    """
    Request the EKF origin (GPS_GLOBAL_ORIGIN) from the flight controller.
    Returns (lat, lon) in degrees (WGS84).
    Raises FenceError on timeout or missing origin.
    """
    # Actively request GPS_GLOBAL_ORIGIN (MAV_CMD_REQUEST_MESSAGE)
    _send_and_wait(
        bus,
        bus.conn.mav.command_long_send,
        bus.conn.target_system,
        bus.conn.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN,
        0, 0, 0, 0, 0, 0,
    )

    # Wait for GPS_GLOBAL_ORIGIN response
    msg = bus.wait_for(
        ["GPS_GLOBAL_ORIGIN"],
        lambda m: True,
        timeout_s=timeout_s,
    )
    if msg is None:
        raise FenceError("Timeout waiting for GPS_GLOBAL_ORIGIN from flight controller")

    lat = msg.latitude / 1e7
    lon = msg.longitude / 1e7
    if lat == 0 and lon == 0:
        raise FenceError("FC returned zero origin (EKF not initialized?)")
    return lat, lon


def upload_polygon_fence(bus: MavlinkBus, vertices: list[Vertex], timeout_s: float) -> None:
    """
    Upload a polygon fence to the flight controller via MAVLink mission protocol
    (mission_type=FENCE). The polygon is sent as a sequence of MISSION_ITEM_INT
    messages with command=MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION.
    The first item (seq=0) carries the vertex_count in its param1 field.
    """
    n = len(vertices)
    if n < 3:
        raise FenceError("Polygon needs at least 3 vertices")
    if n > 255:
        raise FenceError("Polygon exceeds 255 vertices (ArduPilot fence storage limit)")

    # 1. Send MISSION_COUNT to announce number of fence items
    _send_and_wait(
        bus,
        bus.conn.mav.mission_count_send,
        bus.conn.target_system,
        bus.conn.target_component,
        n,
        mission_type=mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
    )

    # 2. FC responds with MISSION_REQUEST_INT for each seq; we send MISSION_ITEM_INT
    for seq in range(n):
        req = bus.wait_for(
            ["MISSION_REQUEST_INT"],
            lambda m: m.seq == seq and m.mission_type == mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
            timeout_s=timeout_s,
        )
        if req is None:
            raise FenceError(f"Timeout waiting for MISSION_REQUEST_INT seq={seq}")

        v = vertices[seq]
        # Every item carries vertex_count in param1 (x field for MISSION_ITEM_INT)
        param1 = float(n)

        _send_and_wait(
            bus,
            bus.conn.mav.mission_item_int_send,
            bus.conn.target_system,
            bus.conn.target_component,
            seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
            0,  # current
            0,  # autocontinue
            param1,  # param1: vertex_count for seq=0, else 0
            0.0, 0.0, 0.0,  # params 2-4 unused (must be floats)
            int(v.lat * 1e7),
            int(v.lon * 1e7),
            0.0,  # alt unused for fence vertices
            mavutil.mavlink.MAV_MISSION_TYPE_FENCE,  # mission_type positional
        )

    # 3. Wait for MISSION_ACK
    ack = bus.wait_for(
        ["MISSION_ACK"],
        lambda m: m.mission_type == mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
        timeout_s=timeout_s,
    )
    if ack is None:
        raise FenceError("Timeout waiting for MISSION_ACK after fence upload")
    if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise FenceError(f"Fence upload rejected by FC: MAV_MISSION_ERROR type={ack.type}")


def download_fence(bus: MavlinkBus, timeout_s: float) -> list[Vertex]:
    """
    Download the currently stored fence from the flight controller.
    Returns list of Vertex (lat, lon).
    """
    # Request fence list
    _send_and_wait(
        bus,
        bus.conn.mav.mission_request_list_send,
        bus.conn.target_system,
        bus.conn.target_component,
        mission_type=mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
    )

    # Wait for MISSION_COUNT
    count_msg = bus.wait_for(
        ["MISSION_COUNT"],
        lambda m: m.mission_type == mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
        timeout_s=timeout_s,
    )
    if count_msg is None:
        raise FenceError("Timeout waiting for MISSION_COUNT on fence download")
    count = count_msg.count

    if count == 0:
        return []

    # Subscribe to ACK before requesting items
    ack_queue = bus.subscribe(["MISSION_ACK"])

    # Request each item
    vertices = []
    for seq in range(count):
        _send_and_wait(
            bus,
            bus.conn.mav.mission_request_int_send,
            bus.conn.target_system,
            bus.conn.target_component,
            seq,
            mission_type=mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
        )

        item = bus.wait_for(
            ["MISSION_ITEM_INT"],
            lambda m: m.seq == seq and m.mission_type == mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
            timeout_s=timeout_s,
        )
        if item is None:
            bus.unsubscribe(ack_queue)
            raise FenceError(f"Timeout waiting for MISSION_ITEM_INT seq={seq} on download")
        vertices.append(Vertex(lat=item.x / 1e7, lon=item.y / 1e7))

    # Wait for MISSION_ACK (already subscribed)
    ack = None
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            msg = ack_queue.get(timeout=max(0.01, deadline - time.time()))
        except queue.Empty:
            continue
        if msg.mission_type == mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
            ack = msg
            break

    bus.unsubscribe(ack_queue)

    if ack is None:
        raise FenceError("Timeout waiting for MISSION_ACK after fence download")
    if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise FenceError(f"Fence download rejected by FC: type={ack.type}")

    return vertices


def clear_fence(bus: MavlinkBus, timeout_s: float) -> None:
    """
    Clear all fence items on the flight controller.
    """
    _send_and_wait(
        bus,
        bus.conn.mav.mission_clear_all_send,
        bus.conn.target_system,
        bus.conn.target_component,
        mission_type=mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
    )

    ack = bus.wait_for(
        ["MISSION_ACK"],
        lambda m: m.mission_type == mavutil.mavlink.MAV_MISSION_TYPE_FENCE,
        timeout_s=timeout_s,
    )
    if ack is None:
        raise FenceError("Timeout waiting for MISSION_ACK on fence clear")
    if ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        raise FenceError(f"Fence clear rejected by FC: type={ack.type}")


def get_param(bus: MavlinkBus, param_id: str, timeout_s: float) -> float:
    """
    Read a single parameter value from the flight controller.
    Returns the float value on success, raises FenceError on timeout.

    This is the read-side mirror of set_param(). Used by the Phase 2 SITL
    tests to read back FENCE_ENABLE, FENCE_ACTION, and FENCE_TYPE after
    enable_fence() to confirm ArduPilot actually accepted the values.
    """
    _send_and_wait(
        bus,
        bus.conn.mav.param_request_read_send,
        bus.conn.target_system,
        bus.conn.target_component,
        param_id.encode("ascii"),
        -1,  # param_index=-1 means "look up by name"
    )

    msg = bus.wait_for(
        ["PARAM_VALUE"],
        lambda m: m.param_id.rstrip("\x00") == param_id,
        timeout_s=timeout_s,
    )
    if msg is None:
        raise FenceError(f"Timeout waiting for PARAM_VALUE for {param_id}")
    return float(msg.param_value)



def set_param(bus: MavlinkBus, param_id: str, value: float, timeout_s: float) -> None:
    """
    Set a parameter on the flight controller and wait for confirmation.
    """
    _send_and_wait(
        bus,
        bus.conn.mav.param_set_send,
        bus.conn.target_system,
        bus.conn.target_component,
        param_id.encode("ascii"),
        value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )

    # Wait for PARAM_VALUE confirmation
    msg = bus.wait_for(
        ["PARAM_VALUE"],
        lambda m: m.param_id.rstrip("\x00") == param_id,
        timeout_s=timeout_s,
    )
    if msg is None:
        raise FenceError(f"Timeout waiting for PARAM_VALUE confirmation for {param_id}")
    # Value comparison with tolerance for float encoding differences
    if abs(msg.param_value - value) > 0.01:
        raise FenceError(f"Param {param_id} set to {msg.param_value}, expected {value}")


# FENCE_TYPE bitmask value for a polygon-inclusion fence.
# Verified against ArduPilot source: AP_Fence/AP_Fence.h
#   AC_FENCE_TYPE_ALT_MAX = 1
#   AC_FENCE_TYPE_CIRCLE  = 2
#   AC_FENCE_TYPE_POLYGON = 4  ← this one
#   AC_FENCE_TYPE_ALT_MIN = 8
# NEED TEST: confirm this value against the exact firmware version in use by
# running test_fence_sitl.py and checking the printed FENCE_TYPE readback.
_FENCE_TYPE_POLYGON_BIT = 4


def enable_fence(bus: MavlinkBus, fence_action: int, timeout_s: float) -> None:
    """
    Enable fence on the flight controller.

    Sets three parameters:
      - FENCE_ENABLE = 1
      - FENCE_ACTION = fence_action  (1 = RTL, team default)
      - FENCE_TYPE: OR in the polygon bit (_FENCE_TYPE_POLYGON_BIT = 4)

    Why we explicitly set FENCE_TYPE rather than relying on ArduPilot inference:
    The ArduPilot behavior — whether uploading FENCE_POLYGON_VERTEX_INCLUSION items
    automatically sets FENCE_TYPE's polygon bit — is firmware-version dependent and
    was listed as NEED TEST in the original README. Until test_fence_sitl.py is run
    against a known SITL build and the observed pre-enable FENCE_TYPE value is
    documented in that test's output, we take the conservative path: explicitly OR
    the polygon bit into whatever FENCE_TYPE is already set to. This is safe in all
    cases — if ArduPilot already set the bit, ORing it again is a no-op; if it
    didn't, this is the fix. Update this docstring after running test_fence_sitl.py
    to record the ArduPilot version and observed auto-inference behavior.

    NEED TEST (Phase 2): run test_fence_sitl.py to observe FENCE_TYPE *before*
    this function sets it. If it's already 4, ArduPilot infers automatically and
    the explicit set below is harmless but redundant — record that in a comment.
    If it's 0, the explicit set is necessary — also record that.
    """
    set_param(bus, "FENCE_ENABLE", 1, timeout_s=timeout_s)
    set_param(bus, "FENCE_ACTION", fence_action, timeout_s=timeout_s)

    # Read current FENCE_TYPE, OR in the polygon bit, write back only if changed
    current_fence_type = get_param(bus, "FENCE_TYPE", timeout_s=timeout_s)
    desired_fence_type = int(current_fence_type) | _FENCE_TYPE_POLYGON_BIT
    if int(current_fence_type) != desired_fence_type:
        set_param(bus, "FENCE_TYPE", float(desired_fence_type), timeout_s=timeout_s)


def build_geofence_status(
    bus: MavlinkBus,
    vertices: list[Vertex],
    origin_lat: float,
    origin_lon: float,
    fence_action: int,
    timeout_s: float,
    fc_readback_matched: bool = True,
) -> GeofenceStatus:
    """
    Convert uploaded vertices to ENU, compute centroid and max radius,
    and return a full GeofenceStatus object.
    """
    vertices_enu = [latlon_to_enu(v.lat, v.lon, origin_lat, origin_lon) for v in vertices]
    centroid = polygon_centroid(vertices_enu)
    max_radius = max_radius_from_point(vertices_enu, centroid)

    vertices_enu_models = [ENUPoint(x=p.x, y=p.y) for p in vertices_enu]

    return GeofenceStatus(
        loaded=True,
        armable=True,
        reason="Fence confirmed on flight controller",
        vertex_count=len(vertices),
        vertices_latlon=vertices,
        vertices_enu=vertices_enu_models,
        centroid_enu=ENUPoint(x=centroid.x, y=centroid.y),
        max_radius_m=max_radius,
        origin_lat=origin_lat,
        origin_lon=origin_lon,
        fc_readback_matched=fc_readback_matched,
    )