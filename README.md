# orbbec_camera_rbnx

Robonix package wrapping the **Orbbec Gemini 330-series** (and other Orbbec) RGBD cameras. Owns the `primitive/camera/*` namespace. Exposes the camera's RGB + aligned-depth streams under generic contracts so that mapping, scene, and any vision skill can resolve topic names through atlas (no hardcoded `/camera/...` paths on the consumer side).

Uses the apt-installed `ros-humble-orbbec-camera` / `ros-humble-orbbec-description` packages — no source build needed.

## Capability surface

| Contract                                  | Mode      | Transport | Source / handler                                          |
| ----------------------------------------- | --------- | --------- | --------------------------------------------------------- |
| `robonix/lifecycle/driver`                | rpc       | gRPC      | implicit shared `Driver(CMD_INIT, config_json)` lifecycle |
| `robonix/primitive/camera/rgb`            | topic_out | ROS 2     | `/<cam>/color/image_raw` (sensor_msgs/Image)              |
| `robonix/primitive/camera/depth`          | topic_out | ROS 2     | `/<cam>/depth/image_raw` (sensor_msgs/Image, 16UC1)      |
| `robonix/primitive/camera/extrinsics`     | topic_out | ROS 2     | latched TransformStamped (TODO)                           |
| `robonix/primitive/camera/snapshot`       | rpc       | MCP       | one-shot RGB capture                                      |
| `robonix/primitive/camera/depth_snapshot` | rpc       | MCP       | one-shot depth capture (8-bit JPEG visualization)         |

## Driver-init lifecycle

`start.sh` only brings up the atlas bridge process. The bridge registers the
shared lifecycle Driver automatically; the package manifest must not declare
the deprecated camera-specific Driver contract. It then awaits
`Driver(CMD_INIT, config_json)`.

When `rbnx boot` invokes Init it passes the manifest's `config:` block as JSON. The handler parses config (camera name, model, profiles, depth_registration, IMU on/off), spawns `ros2 launch orbbec_camera orbbec_camera.launch.py …`, waits for the first frame on the configured RGB topic, declares `primitive/camera/{rgb, depth}` on atlas, and returns ok. Atlas only ever advertises endpoints we've confirmed are publishing.

## Layout

```
orbbec_camera_rbnx/
├── package_manifest.yaml
├── orbbec_camera/
│   └── main.py                    driver gRPC + lazy Init
├── scripts/
│   ├── build.sh                   rbnx codegen only (no colcon — apt-installed)
│   └── start.sh                   source ROS, exec atlas_bridge
└── .gitignore
```

## Config (passed via `Driver(CMD_INIT, config_json)`)

```json
{
  "camera_name":          "camera",
  "camera_model":         "gemini330_series",
  "color_profile":        "640x480x30",
  "depth_profile":        "640x480x30",
  "depth_registration":   true,
  "enable_color":         true,
  "enable_depth":         true,
  "enable_point_cloud":   false,
  "enable_imu":           false,
  "device_preset":        "",
  "serial_number":        "",
  "usb_port":             "",
  "sentinel_timeout_s":   30.0
}
```

Leave `device_preset` empty to use the upstream driver's working `Default`
preset. If set, it must be a named string preset rather than a numeric value.

On robots with more than one Orbbec camera, set either `serial_number` or
`usb_port` so the driver cannot bind a different camera due to USB enumeration
order. `usb_port` is the stable topology path reported by the Orbbec driver,
for example `2-3`.

Supported `camera_model` values (from `/opt/ros/humble/share/orbbec_camera/config/`):
- `gemini330_series` — default, covers Gemini 335/336/330/335L/336L/330L/335Lg/335Le
- `gemini2`, `gemini2L`, `gemini210`
- `femto_bolt`, `femto_mega`, `femto`
- `astra2`, `astra`
- `gemini305`, `gemini345`, `gemini345_lg`, `gemini435_le`
- `lidar` — Pulsar ME450 / SL450

## Build / run standalone

```bash
bash scripts/build.sh
bash scripts/start.sh        # registers driver iface, waits for INIT
```

## Prerequisites

```bash
sudo apt install ros-humble-orbbec-camera ros-humble-orbbec-description
# Register udev rules (required for USB camera access)
sudo cp /opt/ros/humble/share/orbbec_camera/udev/99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## License

Apache-2.0 (matches orbbec_camera upstream).
