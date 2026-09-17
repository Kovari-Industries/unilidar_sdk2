# LiDAR IMU quaternion ordering

The SDK bundled with this repository exposes quaternion elements in **w, x, y, z** order for the Unitree L2 on jetson-105. Assign index 0 to ROS `orientation.w` and indices 1–3 to `x`, `y`, `z`. The SDK example's `x, y, z, w` label and upstream ROS message assignment conflict with this measured behavior (upstream's TF assignment uses w-first).

## Evidence collected on 2026-09-17

Two existing upright, in-place robot-turn recordings were read directly with rosbag2 and compared with their previously exported IMU CSVs. Both paths produced matching sample counts and gravity-error means. No new motion or hardware configuration was performed. A live comparison was unavailable: the configured LiDAR address was unreachable and the expected Ethernet link was down.

For each sample, normalize the quaternion and compare the predicted sensor-frame up vector `R(q)^T * [0,0,1]` with normalized measured acceleration. Separately measure the predicted tilt from vertical. Normalization is only for this diagnostic; publication retains the SDK values.

| Recording | Samples | Mean gravity error, SDK wxyz | Mean gravity error, SDK xyzw | Maximum tilt, SDK wxyz | Maximum tilt, SDK xyzw |
| --- | ---: | ---: | ---: | ---: | ---: |
| `20260916_192347` | 13,613 | 1.892° | 78.358° | 1.287° | 179.936° |
| `20260915_203138` | 13,670 | 2.487° | 94.762° | 0.808° | 179.925° |

The wrong interpretation converts a level yaw turn into a near-complete inversion. Stationary first-five-second gravity errors also favor wxyz: 0.892° versus 4.317° in the newer recording and 0.578° versus 38.686° in the older one.

The newer capture manifest identifies evo commit `fc12065a38d340c6fa465a3728be17a433c4eaf0`, whose submodule pin is `b3db7669a0ea31a112470a0a8e2319add0c7fd04`. That driver maps SDK indices `[0,1,2,3]` to ROS `[w,x,y,z]`, allowing the diagnostic to reconstruct both candidate interpretations. The older manifest pins a pre-fix commit despite recording populated IMU values; it corroborates the result but is not the primary source for driver provenance. Manifest provenance does not independently attest the loaded executable.

The bundled aarch64 `libunilidar_sdk2.a` SHA-256 is `4e334b67c1a92152c89363d8014a6e361d7bf590e58484d7d6ddc8541389de28`. Its `getImuData` implementation copies the 56-byte IMU payload directly from the stored packet, without permuting its quaternion elements. This is SDK inspection, not a fresh capture of wire packets.

## Reproduction

The small C++ diagnostic in `imu-review/` reads `/unilidar/imu` only. It assumes the recorded driver mapping above and prints sample counts, mean gravity disagreement, and mean tilt for both interpretations. In a ROS Jazzy development environment with rosbag2 MCAP support:

```sh
cmake -S docs/imu-review -B /tmp/imu-review-build
cmake --build /tmp/imu-review-build
/tmp/imu-review-build/imu_review /path/to/20260916_192347 /path/to/20260915_203138
```

On jetson-105, the recordings are under `/home/benson/evo_bags/`. Existing CSV exports are under `map-comparison/lidar-investigation-20260917/{orbbec,zed}-source-audit/imu.csv`. Raw recordings are not included in Git.

This settles the element-order question for the recorded robot/SDK combination. It does not validate IMU calibration, covariance, firmware behavior on other devices, or integrated yaw accuracy. The previously observed gyro/yaw discrepancy remains unresolved; this change does not enable IMU-based motion control or mapping assistance.
