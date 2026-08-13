<div align="center">

# KMIDS Veloz
## 2026 WRO – Future Engineers
**Official Engineering Documentation**

> Designing, building, and continuously improving an autonomous vehicle for the World Robot Olympiad Future Engineers challenge.

<!-- IMAGE 1: Robot overview photo (final robot, 3/4 angle, clean background) -->
![Robot overview — TODO: replace with real photo](assets/robot_overview.jpg)

</div>

---

## Team Members

- **Sahas Ninvatchararang (Phonpa)**
- **Olan Sinsuriya (Olan)**
- **Phisit Chuthomsuwan (Champ)**

KMIDS Veloz is a team of students exploring autonomous driving through mechanical design, embedded systems, computer vision, and control theory. This repository documents our full engineering process for the WRO Future Engineers 2026 season: hardware selection, CAD, electronics, software architecture, and the testing/iteration behind our design decisions.

---

## Table of Contents

1. [Overview](#1-overview)
2. [List of Components](#2-list-of-components)
3. [Mechanical Design](#3-mechanical-design)
4. [Power and Sensor Architecture](#4-power-and-sensor-architecture)
5. [Software Architecture](#5-software-architecture)
6. [Obstacle and Parking Strategy](#6-obstacle-and-parking-strategy)
7. [Systems Thinking and Engineering Decisions](#7-systems-thinking-and-engineering-decisions)
8. [Testing and Results](#8-testing-and-results)
9. [Repository Structure](#9-repository-structure)
10. [Build and Upload Instructions](#10-build-and-upload-instructions)
11. [Development Process and Future Improvements](#11-development-process-and-future-improvements)
12. [License](#12-license)

---

## 1. Overview

### 1.1 About the Project

The WRO Future Engineers category requires a vehicle to complete a driving course fully autonomously, first on an open track (three laps, no wall contact) and then on a track with randomly placed obstacle pillars that must be passed on the correct side, ending in a precision parallel-parking maneuver.

Our robot splits its computing across two boards: a **Raspberry Pi 5** handles perception (LIDAR processing, camera-based traffic-light detection, sensor fusion) and high-level decision making, while a **Raspberry Pi Pico 2** runs the real-time control loop for the drive motor and steering servo, communicating with the Pi 5 over I²C. This split keeps hard real-time motor control isolated from the heavier, less deterministic perception workload.

### 1.2 Robot Images

<!-- IMAGE 2-7: Six required views. WRO rules require every one of these. -->
| Front | Rear | Left |
|---|---|---|
| ![Front — TODO](assets/front.jpg) | ![Rear — TODO](assets/rear.jpg) | ![Left — TODO](assets/left.jpg) |

| Right | Top | Bottom |
|---|---|---|
| ![Right — TODO](assets/right.jpg) | ![Top — TODO](assets/top.jpg) | ![Bottom — TODO](assets/bottom.jpg) |

<!-- IMAGE 8: Team photo (required by WRO rules) -->
![Team photo — TODO](assets/team.jpg)

<!-- IMAGE 9: Internal electronics — open chassis or component stack, shows sensor/board placement -->
![Internal electronics — TODO](assets/internals.jpg)

### 1.3 Performance Video

<!-- VIDEO 1: Open Challenge — YouTube link, ≥30s of visible autonomous driving -->
**Open Challenge:** `TODO — add YouTube link`

<!-- VIDEO 2: Obstacle Challenge — YouTube link, ≥30s of visible autonomous driving -->
**Obstacle Challenge:** `TODO — add YouTube link`

---

## 2. List of Components

| Component | Model | Quantity |
|---|---|---:|
| Single-board computer | Raspberry Pi 5 (8 GB) | 1 |
| Microcontroller | Raspberry Pi Pico 2 | 1 |
| PCIe adapter | Raspberry Pi M.2 HAT+ | 1 |
| LIDAR | **Slamtec RPLidar S2** | 1 |
| Camera | Fish-eye Raspberry Pi 5MP IR Camera | 1 |
| IMU | BNO085 9-axis IMU | 1 |
| Drive motor | 20GP-180 DC gearmotor with quadrature encoder | 1 |
| Steering servo | **Surpass Hobby S0009M (9g digital servo)** | 1 |
| Power/UPS module | I²C battery-monitoring UPS (EP-0136-compatible protocol) | 1 |
| Motor driver | DRV8871 | 1 |
| Power switch | N-Channel MOSFET | 1 |
| Camera wire | — | 3 |

> Our CAD part files are still internally named `S0004m.FCStd` and `RPLidarC1.FCStd` from an earlier design pass — the table above reflects the actual parts installed on the robot (S0009M servo, RPLidar S2). <!-- TODO: rename the CAD files to match before final submission, for reproducibility clarity. -->

Each component was selected considering reliability, I²C/PWM compatibility with our Pi 5 + Pico 2 split architecture, availability, and ease of replacement mid-season.

---

## 3. Mechanical Design

All mechanical parts are modeled in FreeCAD before printing. The CAD source is organized as full assemblies (`FreeCAD-Files/Models/`) and individual reusable parts (`FreeCAD-Files/Parts/`), with matching slicer projects (`Slicer-Files/*.3mf`) for printed components.

<!-- IMAGE 10: Annotated CAD assembly (full robot, labeled: chassis, steering, motor, electronics stack) -->
![Annotated CAD assembly — TODO](assets/cad_assembly.png)

### 3.1 Chassis

The finished chassis body measures **244mm (long axis) × 135mm (short axis) × 59mm (height)**, confirmed directly from the FreeCAD model's bounding-box geometry. It's built up from a base plate through a sequence of pads and pockets for mounting cutouts, wire-routing gaps, and standoff holes, with fillets applied on the final edges — this both fits within the WRO 300×200×300mm size limit with margin and gives us room to reposition the electronics stack without redesigning the whole body.

| Dimension | Value |
|---|---|
| Chassis (L × W × H) | 244 × 135 × 59 mm |
| Wheelbase (front-to-rear axle) | 185 mm |
| Track width (left-to-right wheel) | 85 mm |
| Wheel diameter | `TODO — measure and add` |

### 3.2 Steering

<!-- IMAGE 11: Mechanical/steering diagram — linkage geometry, labeled T-bone/transfer/wheel linkages -->
![Steering linkage diagram — TODO](assets/steering_diagram.png)

Our steering geometry (`FreeCAD-Files/Models/SteeringAckermannModel.FCStd`) follows an Ackermann-style linkage, built from printed T-bone and transfer linkage parts (`TBoneLinkageTop/Bottom`, `TransferLinkageLeft/Right`, `WheelLinkageTopLeft/Right`, `WheelLinkageBottomLeft/Right`). Ackermann geometry angles the inner and outer front wheels differently during a turn so both roll rather than scrub, which matters most in the Obstacle Challenge where the robot needs a tight, repeatable turning radius around pillars and into the parking bay.

The physical steering range is constrained in firmware, not just geometry: the Pico 2 maps a `-100%`–`+100%` steering command to a servo pulse between **44° and 136°** (`STEERING_MIN_ANGLE` / `STEERING_MAX_ANGLE` in `main.cpp`), rather than the servo's full mechanical sweep. This lets us clamp the usable range in software once we know the linkage's real mechanical limits, without having to reprint or re-adjust the horn. The S0009M was chosen for its digital, metal-geared construction — a 1.1 kgf·cm-class 9g servo is enough torque to steer the front wheels responsively at this vehicle's weight, while staying small enough to mount directly to the front plate without extra bracketry.

### 3.3 Drivetrain

Drive comes from a single **20GP-180** DC gearmotor with a quadrature encoder, geared through a printed motor gear (`MotorGear.FCStd`) mounted via a motor holder and detachable motor plate (`MotorHolder.FCStd`, `MotorPlate.FCStd`) — the detachable plate is deliberately separated from the main chassis so the motor can be swapped without a chassis reprint.

| Spec | Value | Source |
|---|---|---|
| Rated voltage | 6–12V (we run at 12V, stepped up from the 5V logic rail) | Datasheet |
| Weight | ~80g | Datasheet |
| Gearbox | All-metal planetary | Datasheet |
| Encoder | 28 pulses/rev × 100:1 gear ratio | Our firmware (`ENCODER_PULSE_PER_REV`, `ENCODER_GEAR_RATIO`) |
| No-load RPM / stall torque | `TODO — depends on the specific gear ratio variant; check the label on the gearbox or datasheet page for our exact ratio` | — |

We chose a gearmotor over a plain DC motor specifically for the encoder — it lets the Pico's closed-loop speed controller hold a target wheel speed (RPS) rather than driving PWM duty cycle open-loop, which matters for repeatable lap times when battery voltage sags over a run.

<!-- TODO: describe any gear ratio or motor swap you actually tested, with a before/after result (e.g. lap consistency, stall issues on carpet vs the WRO mat, etc.) — this is the single highest-value addition left for Section 3, since it's the one thing a datasheet can't provide. -->

---

## 4. Power and Sensor Architecture

<!-- IMAGE 12: Wiring/power diagram — battery, UPS, step-up converter, MOSFET, Pi5, Pico2, motor driver, servo, sensors -->
![Wiring diagram — TODO](assets/wiring_diagram.png)

### 4.1 Power Distribution

The system runs its primary logic rail — Raspberry Pi 5, Pico 2, camera, IMU, LIDAR — at **5V**, supplied through a UPS module that reports state of charge over I²C (register `0x17`, matching the EP-0136 protocol used by our `check_battery_status.py` script). Motor power is stepped up separately to 12V, since the 20GP-180 is rated above what the 5V logic rail can supply.

| Component | Rail | Current draw |
|---|---|---|
| Raspberry Pi 5 + M.2 HAT | 5V | `TODO` |
| Pico 2 + sensors (IMU, servo) | 5V | `TODO` |
| RPLidar S2 | 5V | `TODO` |
| Camera | 5V | `TODO` |
| 20GP-180 motor | 12V (stepped up) | `TODO` |

<!-- TODO: fill the current-draw column with a multimeter reading (idle and peak) — even rough numbers here move this section from "listed" to "budgeted," which is what the rubric rewards at Level 4-6. -->

A `set_battery_min.py` / `ups_shutdown.py` pair lets the robot shut down gracefully before the battery is damaged by over-discharge, rather than cutting out mid-run — our answer to "what happens on power failure": the robot shuts down predictably rather than browning out mid-maneuver.

Motor and logic power are switched together through a single **N-channel MOSFET on GPIO 19** of the Pico (`MOSFET_PIN`), which the firmware drives high at boot — this lets one physical switch control power delivery to the drive electronics independently of the Pi 5's own power state.

### 4.2 Sensors

<!-- IMAGE 13: Sensor placement photo/diagram — top-down or side view showing lidar/camera/IMU mounting points -->
![Sensor placement — TODO](assets/sensor_placement.png)

| Sensor | Placement | Why there |
|---|---|---|
| RPLidar S2 | Rear-elevated, above the motor plate, on a standoff-mounted plate | 360° unobstructed view needs it clear of the chassis body and drivetrain height — mounting behind and above the motor plate was the only position with a full clear sweep |
| Fish-eye camera | Front plate, forward-facing | Needs a forward view of pillars and lane lines ahead of the robot; front-mounting keeps its field of view unobstructed by the chassis |
| BNO085 IMU | Center of chassis, near the Pico 2 | Mounting near the physical center of rotation reduces the lever-arm effect of vibration and centripetal acceleration during turns, which otherwise pollutes the accelerometer/gyro reading |

**Calibration:** camera exposure, gain, and white balance are locked in firmware at startup (`cameraOptionCallback` in `obstacle_challenge/main.cpp`) rather than left on auto, so color thresholding for red/green pillars stays consistent instead of drifting frame-to-frame under the competition's fixed lighting. <!-- TODO: describe the IMU calibration step/routine if you have one — the I2C command map already includes an IMU-calibration trigger (see 4.3), worth a sentence on when/how you run it. -->

**Failure handling:** <!-- TODO: describe what actually happens today if the LIDAR, camera, or I2C link drops mid-run — does the robot stop, fall back to dead-reckoning, or is this not yet handled? Even "not yet handled, planned for X" is honest, scoreable content for Criterion 4 (risk identification). -->

### 4.3 Inter-Board Communication

<!-- IMAGE 14: Pi 5 ↔ Pico 2 architecture diagram — the two boards, I2C bus, memory map regions -->
![Pi5-Pico2 architecture — TODO](assets/board_architecture.png)

The Pi 5 and Pico 2 communicate over a **256-byte I²C shared-memory protocol** (`shared/i2c/pico_i2c_mem_addr.h`), with the Pico acting as I²C slave at address `0x39` on a 400 kHz bus. Rather than a simple one-way sensor stream, the memory map is split into fixed regions:

| Region | Direction | Contents |
|---|---|---|
| `COMMAND_ADDR` | Pi 5 → Pico | Restart, IMU calibration mode, skip-calibration |
| `STATUS_ADDR` | Pico → Pi 5 | Bit-packed running/IMU-ready flags |
| `IMU_DATA_ADDR` | Pico → Pi 5 | Fused accelerometer + Euler angles |
| `ENCODER_ANGLE_ADDR` | Pico → Pi 5 | Current wheel encoder angle |
| `MOVEMENT_INFO_ADDR` | Pi 5 → Pico | Target motor speed (RPS) + steering percent |

This two-way memory map means the Pi 5 issues high-level commands (target speed, target steering, calibration triggers) while the Pico independently closes the low-level control loop and reports back sensor state — the Pi 5 never has to wait on a round-trip to know the robot's current heading or encoder position.

---

## 5. Software Architecture

### 5.1 Two-Board Split

| Board | Responsibility | Update rate |
|---|---|---|
| Raspberry Pi 5 | LIDAR processing, camera processing, sensor fusion, high-level state machine | ~60 Hz main loop |
| Raspberry Pi Pico 2 | IMU polling, closed-loop motor speed PID, servo positioning, I²C slave | IMU: 4 ms / Motor PID: 8 ms |

Code is organized into `modules` (hardware interfaces — camera, lidar, pico2 link, i2c), `processors` (pure data-processing logic — lidar, camera, and combined sensor fusion), `utils` (PID controller, logger, ring buffer), and `types` (shared data structures), with a `shared/` directory holding the I²C protocol and structs used by both boards. Every module and processor directory has its own `README.md` documenting its public API.

### 5.2 Perception Pipeline

**LIDAR processing** (`lidar_processor`) converts raw polar scan points into Cartesian coordinates, filters out points too close (<0.005 m) or too far (>3.2 m), then clusters nearby points into `LineSegment`s representing candidate walls. These candidates are bucketed by relative position (front/back/left/right) and resolved into a single wall per side using the robot's current heading estimate.

**Camera processing** (`camera_processor`) converts each frame to HSV and thresholds it against tuned red/green/pink ranges to find traffic-pillar candidates, blacking out the top half of the frame first to suppress ceiling-light noise, then extracts contour centroids above a minimum area.

<!-- IMAGE 15: LIDAR/camera sensor-fusion diagram — how a camera color blob + a LIDAR point become one TrafficLightInfo -->
![Sensor fusion diagram — TODO](assets/sensor_fusion_diagram.png)

**Sensor fusion** (`combined_processor`) does two things: it synchronizes camera frames with LIDAR scans by timestamp, accounting for a configurable camera-to-LIDAR delay offset (since the two sensors don't sample at the same instant or rate), and it matches each camera-detected color blob to the nearest LIDAR point along the same angular ray — giving each detected pillar both a color/type (from the camera) and a precise distance/position (from the LIDAR).

### 5.3 Control Loops

Two independent PID controllers run in the main loop: a **heading PID** that steers the robot to hold a target compass heading, and a **wall-following PID** whose output is added as an offset to the heading target, pulling the robot toward a target distance from the outer wall. The wall PID is explicitly toggled off during turns (`wallPid_.setActive(false)`) so wall-following doesn't fight the turn-in-progress, then re-enabled once the turn completes — a small but deliberate detail that avoids the two control loops fighting each other.

On the Pico 2, a third PID closes the loop on wheel speed (encoder feedback → target RPS), so the Pi 5 only ever needs to say "go at this speed" rather than manage PWM duty cycle directly.

### 5.4 Open Challenge State Machine

<!-- IMAGE 16: Open Challenge state-machine diagram — the 5 states and transitions described below -->
![Open Challenge state machine — TODO](assets/open_challenge_states.png)

The open-challenge robot (`apps/challenges/open_challenge/main.cpp`) runs a five-state machine: `NORMAL` (driving straight, wall-following active) → `PRE_TURN` (front wall detected within 1.2 m, cooldown-gated so it can't re-trigger mid-turn) → `TURNING` (wall PID disabled, heading target snapped 90° in the detected turn direction) → back to `NORMAL` once heading settles within a 20° tolerance, repeating for 12 turns (three laps × four corners) before entering `PRE_STOP` → `STOP`.

---

## 6. Obstacle and Parking Strategy

<!-- IMAGE 17: Obstacle Challenge decision-flow diagram — pillar detection → color → pass-side decision -->
![Obstacle Challenge decision flow — TODO](assets/obstacle_decision_flow.png)

### 6.1 Pillar Pass-Left / Pass-Right Logic

<!-- TODO: this is the most important remaining content gap in the whole document. Write 2-4 sentences in your own words on how the robot decides which side to pass a detected pillar. Likely starting point: once combineTrafficLightInfo() (Section 5.2) returns a TrafficLightInfo with a color (red/green) and a LIDAR position, what does the state machine DO with that — does it set a target lateral offset, switch which wall it's following, insert a temporary waypoint? Whoever wrote combined_processor.cpp or the obstacle_challenge state transitions for pillar handling is the right person to write this paragraph. -->

### 6.2 Direction-Dependent Tuning

The obstacle-challenge state machine is significantly larger than the open-challenge one, because the correct behavior branches on two things the robot only learns at runtime: which direction it's driving (clockwise or counter-clockwise) and where the parking bay ends up relative to its approach. Distinct constants exist for each combination — e.g. `TARGET_OUTER_WALL_DISTANCE_PARKING_CCW` (0.29 m) vs. `TARGET_OUTER_WALL_DISTANCE_PARKING_CW` (0.31 m), and separate turning-radius thresholds for a normal turn vs. a "push" turn (`TURNING_FRONT_WALL_DISTANCE` vs. `..._PUSH`) — which indicates these were tuned independently through testing rather than sharing one generic value.

### 6.3 Parking Sequence

<!-- IMAGE 18: Parking sequence diagram — the CW/CCW approach states and unpark maneuvers described below -->
![Parking sequence diagram — TODO](assets/parking_sequence.png)

Parking itself has direction-specific approach states (`CCW_UNPARK_1..4`, `CW_UNPARK_1..2`, plus dedicated pre-parking and U-turn states), because a counter-clockwise approach and a clockwise approach reach the bay from geometrically different angles and need different sequences of forward/reverse moves to align. Parking-bay detection itself relies on the same LIDAR wall-resolution pipeline described in Section 5.2. <!-- TODO: confirm and describe exactly how the bay boundary is recognized (e.g. a gap of a known width between two wall segments) — this is the second most valuable missing paragraph after 6.1. -->

### 6.4 Edge Cases

<!-- TODO: list 2-3 specific edge cases the state machine actually handles or has been tested against — e.g. what happens if two pillars are close together, if a pillar is only partially visible to the camera but fully visible to LIDAR, or if the robot approaches the parking bay at the very edge of its turning radius. -->

---

## 7. Systems Thinking and Engineering Decisions

- **Splitting perception and control across two boards** was a direct response to a timing constraint: the Pi 5's LIDAR/camera processing pipeline is not hard-real-time, but motor and servo control need to be. Rather than trying to keep the whole loop deterministic on one board, we moved the time-critical motor PID and servo output onto the Pico 2 and let the Pi 5 issue target setpoints over I²C — the Pi 5 can hiccup on a slow perception frame without a wheel losing speed control.
- **Bidirectional shared-memory I²C instead of a simple sensor stream** — the same 256-byte map carries commands, calibration triggers, and status flags in both directions, so the Pi 5 can trigger IMU calibration or a Pico restart without a second communication channel.
- **Decoupling the wall-following and heading PIDs, with an explicit toggle during turns** — an early combined approach let the two loops fight each other mid-turn; disabling the wall PID during `TURNING` and re-enabling it only once heading has settled removed that interaction.
- **Separate tuning constants per direction (CW/CCW) and per maneuver type (normal turn vs. "push" turn)** in the obstacle challenge, rather than one shared threshold — reflecting that the two directions approach walls and the parking bay from different geometry and needed independently tuned targets.
- **Locking camera exposure, gain, and white balance in firmware** rather than relying on auto-exposure, so color thresholding for red/green pillars stays consistent across different competition lighting rather than drifting frame-to-frame.
- **Choosing an encoder-equipped gearmotor over a plain DC motor** specifically to enable closed-loop speed control on the Pico — trading a slightly more complex wiring/firmware setup for repeatable speed as battery voltage sags over a run, rather than open-loop PWM that would drift with charge state.

<!-- TODO: add 1-2 more decisions with a concrete "we chose X over Y because..." — ideally something with a before/after number once Section 8 (Testing) has real data. -->

---

## 8. Testing and Results

<!-- IMAGE 19: Testing-results graph/table — e.g. lap consistency over N runs, or tuning before/after -->
![Testing results — TODO](assets/testing_results.png)

Every run through `open_challenge` and `obstacle_challenge` logs LIDAR, IMU, and encoder data with nanosecond timestamps to disk (`Logger` in `src/utils/logger`), which `log_viewer` and `log_to_video` can replay — this lets us debug a failed run after the fact rather than only observing it live on the field.

| Metric | Result |
|---|---|
| Open Challenge — laps attempted / completed | `TODO` |
| Obstacle Challenge — pillar-pass success rate | `TODO` |
| Parking — success rate over N attempts | `TODO` |
| Before/after result from a specific tuning change | `TODO` |

<!-- TODO: this whole section is the single biggest lever left for Criterion 3 (testing metrics) and Criterion 4 (evidence for tradeoffs) — even a rough "12/15 successful parks over our last test session" beats no number at all. -->

---

## 9. Repository Structure

```text
code/
├─ raspberry-pi-5/
│  ├─ apps/
│  │  ├─ challenges/
│  │  │  ├─ open_challenge/       # Open Challenge executable
│  │  │  └─ obstacle_challenge/   # Obstacle Challenge + parking executable
│  │  ├─ log_viewer/              # Visualize recorded run logs
│  │  ├─ log_to_video/            # Render logs to video
│  │  ├─ scan_map_inner/          # Inner-track mapping tool
│  │  ├─ scan_map_outer/          # Outer-track mapping tool
│  │  ├─ test_camera/             # Camera-only test app
│  │  ├─ test_i2c/                # I2C link test app
│  │  ├─ test_lidar/              # LIDAR-only test app
│  │  └─ test_lidar_cam/          # Combined LIDAR + camera test app
│  ├─ external/                   # lccv, RPLIDAR SDK
│  ├─ scripts/                    # Battery monitoring, shutdown, deploy
│  └─ src/
│     ├─ modules/                 # camera, lidar, i2c_master, pico2
│     ├─ processors/              # camera, lidar, combined (fusion)
│     ├─ types/                   # Shared data structures
│     └─ utils/                   # PID controller, logger, ring buffer
├─ raspberry-pi-pico-2/
│  └─ src/
│     ├─ modules/
│     │  ├─ controllers/          # bno085, motor, encoder, servo
│     │  └─ i2c_slave/            # I2C slave + shared memory protocol
│     └─ utils/pid_controller/
└─ shared/                        # i2c memory map, IMU struct — used by both boards

FreeCAD-Files/
├─ Models/SteeringAckermannModel.FCStd
└─ Parts/                         # Chassis, linkages, axles, mounts, gears

Slicer-Files/                     # .3mf print projects for each printed part
```

Every `modules/`, `processors/`, and `utils/` subdirectory has its own `README.md` documenting the classes and functions it exposes — start there for API-level detail beyond what's summarized above.

---

## 10. Build and Upload Instructions

### 10.1 Dependencies

- **Raspberry Pi 5:** OpenCV, libcamera, the bundled `lccv` and RPLIDAR SDK (in `code/raspberry-pi-5/external/`)
- **Raspberry Pi Pico 2:** the Pico SDK

### 10.2 Raspberry Pi 5

```bash
cd code/raspberry-pi-5
chmod +x build-arm64.sh
./build-arm64.sh
```
Binaries are output to `build/bin/`. We cross-compile using the provided `Dockerfile.cross` / `Dockerfile.compile` rather than building natively on the Pi, to keep build times short and avoid installing the full OpenCV/libcamera toolchain on the robot itself. `upload.sh` then `scp`s the built binaries and scripts to the robot.

### 10.3 Raspberry Pi Pico 2

```bash
cd code/raspberry-pi-pico-2
chmod +x build.sh
./build.sh
```
Flash the resulting `.uf2` file to the Pico in bootloader mode (hold BOOTSEL while connecting via USB, then copy the file to the mounted drive), or with `picotool`:
```bash
sudo picotool load build/<output_name>.uf2 -f
```
<!-- TODO: confirm the exact .uf2 output filename from your Pico CMakeLists.txt (check the `pico_add_extra_outputs` target name) and replace <output_name> above. -->

---

## 11. Development Process and Future Improvements

### 11.1 Mechanical Iteration History

<!-- TODO: describe any version history you have — e.g. "steering v1 used X linkage, we found Y problem, v2 changed Z." Even one iteration with a before/after result is worth more to the rubric than a paragraph of description with no history. -->

### 11.2 Planned Improvements

- Add photos (all six views + internals + team) and Open/Obstacle Challenge demonstration videos
- Fill in wheel diameter, current-draw power budget, and all 14 diagrams marked above
- Write up the pillar pass-left/pass-right decision logic and parking-bay detection method (Section 6.1, 6.3)
- Record lap-consistency and parking-success metrics from testing (Section 8)
- Rename CAD files to match final part numbers (S0004m → S0009M, RPLidarC1 → RPLidar S2)

---

## 12. License

This repository is distributed under the MIT License.

---

**KMIDS Veloz • WRO Future Engineers 2026**
