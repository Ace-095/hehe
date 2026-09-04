# Hardware Setup — Raspberry Pi + Pixhawk + Cameras

> **Platform:** Raspberry Pi 4 (or 5) running Ubuntu 22.04 / Raspberry Pi OS (64-bit).
> **Audience:** whoever does the hardware bring-up in Phase 9.

This document is the hardware mirror of `SIMULATION_SETUP.md`. Both paths
are documented from the start so the team can validate them in parallel and
so nothing is discovered for the first time at the competition venue.

---

## Bill of Materials (Phase 9 scope)

| Item | Notes |
|------|-------|
| Raspberry Pi 4 or 5 | 4 GB RAM minimum recommended |
| Pixhawk (with ArduCopter firmware) | Firmware version: **NEED TEAM INPUT** — confirm before Phase 9 |
| Camera module × 2 | Model: **NEED TEAM INPUT** — arducam, imx708, etc. Affects picamera2 config |
| USB-to-Serial adapter or UART cable | For Pixhawk serial connection to Pi |

---

## 1. Pixhawk Serial Connection

Prefer a stable device path that doesn't renumber on reboot:

```bash
# List stable by-id paths — pick the one that corresponds to the Pixhawk
ls /dev/serial/by-id/

# Example (NEED TEST — actual string depends on Pixhawk USB chip):
# /dev/serial/by-id/usb-ArduPilot_Pixhawk1_...
```

Set this path as the MAVLink device:
```bash
export MAVLINK_DEVICE=/dev/serial/by-id/usb-<your-actual-path-here>
export MAVLINK_BAUD=115200     # or 921600 if using TELEM2 high-speed
```

> **NEED TEST:** the baud rate depends on how the Pixhawk SERIALX_BAUD
> parameter is configured for the port you're using. Confirm before wiring.

---

## 2. Camera Enumeration

```bash
# List all cameras the Pi sees
libcamera-hello --list-cameras

# Take a quick test shot (confirms hardware + driver)
libcamera-still --autofocus-mode=auto -o /tmp/cam_test.jpg

# Inspect the image
eog /tmp/cam_test.jpg        # or scp it to your laptop
```

Note the camera indices from `--list-cameras` output. The mapping
**Camera 1 belly / Camera 2 belly → `/dev/videoX` → `picamera2` index** is
**NEED TEST** — it depends on the physical wiring and the driver enumeration
order. Document it here once confirmed:

```
Camera 1 (belly, forward): index = ?     NEED TEST
Camera 2 (belly, wide):    index = ?     NEED TEST
```

---

## 3. Focus Motor Sanity Check

```bash
# Trigger autofocus and confirm lens responds
libcamera-still --autofocus-mode=auto --autofocus-range=full -o /tmp/focus_test.jpg
```

If autofocus fails, check:
- Cable seating in the CSI connector
- `libcamera` version: `libcamera-hello --version`
- Camera model vs. driver support

---

## 4. Python Environment

```bash
# Create and activate venv (same as simulation)
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt

# picamera2 — NOT a standard pip package:
# On Raspberry Pi OS: sudo apt install python3-picamera2
# On Ubuntu/Pi: pip install picamera2 --extra-index-url https://www.piwheels.org/simple
# NEED TEST: verify which index/method works on your Pi OS version.
```

> **Do NOT** `pip install` `picamera2` from PyPI without the piwheels index —
> the PyPI package is a stub and may install silently without the actual library.

---

## 5. System Libraries

```bash
# zbar (for pyzbar QR decoding)
sudo apt install libzbar0

# For any video/GStreamer testing (optional, sim-only in practice)
# sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base
```

---

## 6. Environment Variables (Pi production)

Create `/etc/environment.d/addc.conf` or export in your service unit:

```bash
# MAVLink connection to Pixhawk
MAVLINK_DEVICE=/dev/serial/by-id/usb-<YOUR_PIXHAWK_ID>
MAVLINK_BAUD=115200
MAVLINK_TIMEOUT_S=5.0

# Fence behavior — confirm FENCE_ACTION enum value in your firmware:
# 0=Report, 1=RTL, 2=Land, 4=Brake, 5=SmartRTL. RTL=1 is the team default.
FENCE_ACTION=1

# Camera abstraction — use picamera2 on the Pi
CAMERA_SOURCE=picamera2

# QR payload format — NEED TEAM INPUT once confirmed
SYNTHETIC_QR_PAYLOAD=CACHE_QR_TEST_PAYLOAD_001

# Server bind
GEOFENCE_HOST=0.0.0.0
GEOFENCE_PORT=8000
```

> **Never set** `CAMERA_SOURCE=gazebo` on the Pi. ROS 2 is not installed there
> and `GazeboCameraSource.start()` will raise `ImportError` immediately.

---

## 7. Running the Backend on the Pi

```bash
cd /path/to/m1/backend
source ../.venv/bin/activate
uvicorn app:app --host 0.0.0.0 --port 8000
```

Or as a systemd service (create `/etc/systemd/system/addc-backend.service`):
```ini
[Unit]
Description=ADDC 2027 Geofence + Vision Backend
After=network.target

[Service]
WorkingDirectory=/path/to/m1/backend
ExecStart=/path/to/m1/.venv/bin/uvicorn app:app --host 0.0.0.0 --port 8000
EnvironmentFile=/etc/environment.d/addc.conf
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

---

## 8. Hardware Acceptance Checklist (Phase 9)

Run this before any outdoor flight test:

```bash
# From backend/ directory:
python3 tests/test_phase1_sim_smoke.py
# Expect all PASS — runs the MavlinkBus check against the real Pixhawk
# and the SyntheticQRSource check (no camera hardware needed for this part)
```

Manual checks:
- [ ] `libcamera-hello --list-cameras` shows both cameras
- [ ] `libcamera-still --autofocus-mode=auto` produces a focused image
- [ ] MAVLink heartbeat received: `target_system` and `target_component` both non-zero
- [ ] Fence upload via browser UI succeeds with FC readback match
- [ ] `CAMERA_SOURCE=picamera2` starts without `NotImplementedError` (Phase 9 only)

---

## Differences from Simulation

| Aspect | Simulation (dev machine) | Hardware (Pi) |
|--------|--------------------------|---------------|
| MAVLink device | `udp:127.0.0.1:14550` | `/dev/serial/by-id/...` |
| `CAMERA_SOURCE` | `synthetic` or `gazebo` | `picamera2` |
| ROS 2 | Installed (gz-ros-bridge) | **Not installed** |
| Gazebo | Running | **Not running** |
| Frame source | `GazeboCameraSource` / `SyntheticQRSource` | `Picamera2Source` |
| QR target | 3D model in Gazebo world | Physical printed target |

The `CameraSource` abstraction in `backend/camera_source.py` is the single
point where these differences are isolated. All downstream pipeline code
(`camera_manager.py`, `qr_pipeline.py`) is identical in both environments.
