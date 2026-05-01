# UGV Base ROS - Customization Notes

This file documents the customizations applied in this repository during the current development cycle.

## 1) Architecture and Motion Stack Changes

### IMU usage change
- Base distance is no longer computed by IMU double integration.
- IMU is used for attitude and heading (yaw) reference.
- Wheel odometry is the source of base translation distance.

### Wheel odometry integration
- Added wheel odometry state:
  - `wheel_odom_x`, `wheel_odom_y`, `wheel_odom_dist`, `wheel_odom_path`, `wheel_odom_v`
  - `wheel_odom_last_l`, `wheel_odom_last_r`
- Added helper functions:
  - `resetWheelOdom()`
  - `updateWheelOdom()`
- Main loop now updates wheel odometry continuously.

### Drive-plan state machine (new autonomous base drive feature)
- Added multi-leg drive controller:
  - Turn to requested yaw
  - Drive target distance while holding heading
  - Advance automatically to next leg
- Supports forward and backward legs (negative distance = reverse).
- Supports omitted yaw per leg:
  - If omitted, yaw is inherited from previous leg.
  - If first leg yaw is omitted, current robot yaw is used.
- Drive plan keeps heartbeat alive while active.
- Manual motion commands abort active drive plan.

## 2) New/Updated JSON Commands

### Existing commands kept and repurposed
- `T:149` -> `CMD_ODOM_RESET`
  - Resets base wheel-odometry reference point.

### New commands
- `T:150` -> `CMD_BASE_DRIVE_PLAN`
  - Starts a multi-leg plan.
  - Format:
    ```json
    {
      "T":150,
      "anchor":1,
      "legs":[
        {"yaw":0, "dist":1.0},
        {"dist":0.5},
        {"yaw":180, "dist":-0.3}
      ]
    }
    ```
  - Notes:
    - yaw in degrees
    - dist (or alias `d`) in meters
    - `anchor` optional (default true)

- `T:151` -> `CMD_BASE_SET_ANCHOR`
  - Sets the current base position as the anchor point.

- `T:152` -> `CMD_BASE_DRIVE_ABORT`
  - Aborts active drive plan immediately.

### Existing telemetry/flow commands (still valid)
- `T:130` one-shot base feedback
- `T:131` feedback flow on/off
- `T:142` feedback flow interval

## 3) Telemetry Extensions (T:1001 base feedback)

Added/extended fields:
- `imux`, `imuy`, `imud` -> wheel odometry position/distance (cm)
- `anchd` -> path distance from anchor (cm)
- `anchs` -> straight-line displacement from anchor (cm)
- `dact` -> drive plan active (0/1)
- `dleg` -> current leg index (1-based while active)
- `dlegs` -> total legs in active plan
- `dacc` -> accumulated executed distance in plan (cm)
- `dtgt` -> total target distance for plan (cm)
- `mhz` -> measured motor loop frequency
- `ts` -> telemetry timestamp (ms)

Compatibility mapping retained in IMU feedback:
- Legacy keys `ix`, `iy`, `id`, `ivx`, `ivy` are preserved and now sourced from wheel odometry.

## 4) UART/UDP/Logging Behavior

### UDP mirror behavior
- `Serial` output is mirrored to UDP broadcast on port `14514`.
- Mirror is active when Wi-Fi is connected.

### Serial optimization
- UART writes are skipped when serial host is not connected.
- UDP mirror continues to function independently.

## 5) Web UI Changes

- No new UI panels were added. All new functionality is accessible via JSON API only.

## 6) Build and Environment Changes

### Source filtering
- Main environments now exclude `src/eval/**`.
- Eval environments include only `src/eval/**`.

### Added eval environments
- `esp32-eval`
- `esp32-eval-ota`

### OTA target update
- Default OTA upload target set to `192.168.1.118`.

## 7) New Evaluation Firmware

- Added `src/eval/main.cpp` as standalone IMU distance evaluation firmware.
- Contains dedicated web UI and diagnostic routines separated from main base firmware.

## 8) Additional Fixes

- IMU bias set/get calls corrected to use proper axis-specific APIs for gyro/accel/compass Y/Z setters.
- Removed stale IMU experiment artifacts and aligned flow with wheel odometry architecture.

## 9) Files Changed in This Customization Set

- `platformio.ini`
- `src/IMU_ctrl.h`
- `src/ROS_Driver.cpp`
- `src/json_cmd.h`
- `src/movtion_module.h`
- `src/uart_ctrl.h`
- `src/udp_log.h`
- `src/ugv_advance.h`
- `src/ugv_config.h`
- `src/web_page.h`
- `src/eval/main.cpp` (new)
