<div align="center">

# KMIDS Veloz
## 2026 WRO – Future Engineers
**Official Engineering Documentation**

> Designing, building, and continuously improving an autonomous vehicle for the World Robot Olympiad Future Engineers challenge.

<!-- IMAGE: Robot overview photo (final robot, 3/4 angle, clean background) -->
![Robot overview — TODO: replace with real photo](assets/robot_overview.jpg)

</div>

---

## Team Members

- **Sahas Ninvatchararang (Phonpa)**
- **Olan Sinsuriya (Olan)**
- **Phisit Chuthomsuwan (Champ)**

<!-- IMAGE: Team photo -->
![Team photo](assets/team.png)

KMIDS Veloz is a team of students exploring autonomous driving through mechanical design, embedded systems, computer vision, and control theory. Over the course of the WRO Future Engineers 2026 season, our work has covered hardware selection, CAD design, electronics integration, and the software architecture running on our two onboard boards, along with the testing and iteration that shaped each of those decisions. This document collects that process in one place: what we built, why we built it that way, and where a few pieces are still in progress.

---

## Table of Contents

1. [Overview](#1-overview)
   - [1.1 About the Project](#11-about-the-project)
   - [1.2 Robot Images](#12-robot-images)
   - [1.3 Performance Video](#13-performance-video)
2. [Mobility Management](#2-mobility-management)
   - [2.1 Drive System](#21-drive-system)
   - [2.2 Steering](#22-steering)
   - [2.3 Chassis Design](#23-chassis-design)
3. [Power and Sense Management](#3-power-and-sense-management)
   - [3.1 Power Source](#31-power-source)
   - [3.2 Sensor and Camera](#32-sensor-and-camera)
   - [3.3 Processing Units](#33-processing-units)
   - [3.4 Circuit Diagram](#34-circuit-diagram)
   - [3.5 Power Consumption](#35-power-consumption)
4. [Software Architecture](#4-software-architecture)
   - [4.1 Code Organization](#41-code-organization)
   - [4.2 Sensing Modules](#42-sensing-modules)
   - [4.3 Reactive Navigation Controller](#43-reactive-navigation-controller)
5. [Obstacle Management](#5-obstacle-management)
   - [5.1 Open Challenge](#51-open-challenge)
   - [5.2 Obstacle Challenge](#52-obstacle-challenge)
   - [5.3 Parallel Parking](#53-parallel-parking)
6. [Systems Thinking and Engineering Decisions](#6-systems-thinking-and-engineering-decisions)
7. [Testing and Results](#7-testing-and-results)
8. [Source Code](#8-source-code)
   - [8.1 API Documentation](#81-api-documentation)
   - [8.2 Code Structure](#82-code-structure)
   - [8.3 Compilation / Upload Instructions](#83-compilation--upload-instructions)
9. [List of Components](#9-list-of-components)
10. [3D Model Files](#10-3d-model-files)
    - [10.1 FreeCAD Files](#101-freecad-files)
    - [10.2 STL Files](#102-stl-files)
    - [10.3 Slicer Files](#103-slicer-files)
11. [Building Instructions](#11-building-instructions)
12. [Development Process and Future Improvements](#12-development-process-and-future-improvements)
    - [12.1 Mechanical Iteration History](#121-mechanical-iteration-history)
    - [12.2 Planned Improvements](#122-planned-improvements)
13. [License](#13-license)

---

## 1. Overview

### 1.1 About the Project

The WRO Future Engineers category requires a vehicle to complete a driving course fully autonomously. The competition is split into two challenges. In the **Open Challenge**, the robot must complete three laps of an empty track without making contact with any wall. In the **Obstacle Challenge**, the track additionally contains randomly placed obstacle pillars that the robot must pass on the correct side, and the run finishes with a precision parallel-parking maneuver into a bay whose position is not known in advance. In both challenges the driving direction and the track layout are randomized before the run starts, and in the Obstacle Challenge the pillar placement is randomized as well, so the robot has to work out what it's facing from its own sensors rather than following a pre-programmed path.

Our robot splits its computing across two boards. A **Raspberry Pi 5** handles perception — LIDAR distance sensing, camera-based pillar detection — and runs the navigation logic that decides steering and speed. A **Raspberry Pi Pico 2** takes those commands and drives the motor and steering servo, reporting back encoder and IMU telemetry over I²C. This split exists because motor and servo output need to be timely and consistent, while the Pi 5's sensing workload can tolerate the occasional slow frame without much consequence.

> **A note on where we are right now:** our control software went through a significant rewrite partway through the season, moving from an earlier PID- and state-machine-based approach to a simpler reactive LIDAR controller (see Section 4). This document reflects the current, active codebase. A few pieces from the earlier approach — a state-machine module for lap/turn tracking, and camera-based pillar detection — still exist in the repository but are not yet wired into the active control loop. Sections 5 and 12 are explicit about what's live versus what's still pending integration.

### 1.2 Robot Images

The six required orientation views, four additional angled shots, and a photo of the internal electronics layout are collected below.

<!-- IMAGES: required six views, using our actual filenames -->
| Front | Rear | Left |
|---|---|---|
| ![Front](assets/front_view.jpg) | ![Rear](assets/rear_view.jpg) | ![Left](assets/left_view.jpg) |

| Right | Top | Bottom |
|---|---|---|
| ![Right](assets/right_view.jpg) | ![Top](assets/top_view.jpg) | ![Bottom](assets/bottom_view.jpg) |

**Additional angles:**

| Left side | Left-rear | Right side | Right-rear |
|---|---|---|---|
| ![Left side](assets/left_side_view.jpg) | ![Left rear](assets/left_back_side_view.jpg) | ![Right side](assets/right_side_view.jpg) | ![Right rear](assets/right_back_side_view.jpg) |

<!-- IMAGE: Internal electronics — open chassis or component stack, shows sensor/board placement -->
![Internal electronics — TODO](assets/internals.jpg)

### 1.3 Performance Video

<!-- VIDEO: Open Challenge — YouTube link, ≥30s of visible autonomous driving -->
**Open Challenge:** `TODO — add YouTube link`

<!-- VIDEO: Obstacle Challenge — YouTube link, ≥30s of visible autonomous driving -->
**Obstacle Challenge:** `TODO — add YouTube link`

These two parts show both the Open and Obstacle Challenges respectively.

[Back to Top](#kmids-veloz)

---

## 2. Mobility Management

This section covers how the robot moves: what drives the rear wheels, how the front wheels are steered, and how the chassis is built to hold both systems together.

- **Drive system:** a single 20GP-180 DC gearmotor with an integrated encoder drives the rear wheels.
- **Steering:** the front wheels use Ackermann-geometry steering, actuated by a Surpass Hobby S0009M servo.

### 2.1 Drive System

The robot uses a single **20GP-180 DC gearmotor with an integrated quadrature encoder** to drive the rear wheels. Power is transferred from the motor through a printed gear system, while encoder feedback is reported to the Raspberry Pi Pico 2 so wheel speed can eventually be regulated in closed loop.

<!-- IMAGE: 20GP-180 motor photo -->
![20GP-180 DC gearmotor — TODO](assets/20gp180_motor.jpg)

#### Motor: 20GP-180 DC Gearmotor

The 20GP-180 is the vehicle's only drive motor. It's mounted at the rear of the chassis and drives the rear axle through the printed drivetrain components described below.

**Specifications**

| Specification | Value | Source |
|---|---:|---|
| Motor | 20GP-180 DC gearmotor | Datasheet |
| Rated voltage | 6–12 V | Datasheet |
| Operating voltage | 12 V (stepped up from the 5V logic rail) | Our configuration |
| Weight | ~80 g | Datasheet |
| Gearbox | All-metal planetary | Datasheet |
| Encoder | Quadrature encoder | Datasheet / hardware |
| Encoder pulses | 28 pulses/rev | Our firmware |
| Gear ratio | 100:1 | Our firmware |
| No-load RPM | `TODO` | Datasheet |
| Stall torque | `TODO` | Datasheet |

**Reason for selection:** the encoder was the deciding factor in choosing this motor over an equivalent plain DC motor. Encoder feedback means wheel rotation can eventually be measured directly rather than assumed from a PWM duty cycle, which matters because a fixed duty cycle does not produce a fixed speed — the motor's actual output changes as battery voltage, load, and friction change over the course of a run. The 20GP-180 also strikes a useful balance of torque, gearing, and physical size for the current chassis: its geared output provides enough mechanical torque to accelerate the vehicle and hold speed on the competition mat, while its dimensions fit within the rear drivetrain without needing an oversized motor mount.

**Encoder and closed-loop speed control:** the motor's quadrature encoder reports rotational feedback to the Pico 2 as part of its telemetry (`encoder_count`, `rpm_x10` — see Section 3.4). At 28 pulses/rev through a 100:1 gear ratio, the encoder gives fine-grained resolution on wheel rotation. <!-- TODO: confirm whether the Pico currently closes a speed loop on this feedback, or only reports it for logging — this affects how "closed-loop" the current drivetrain control actually is. -->

**Motor gear:** power from the motor is transferred through a printed motor gear (`MotorGear.FCStd`).

<!-- IMAGE: Motor gear -->
![Motor gear — TODO](assets/motor_gear.jpg)

The gear is a separate printed component rather than being integrated directly into the chassis, which makes the drivetrain easier to modify if the motor, gear ratio, or wheel configuration changes during development.

**Motor mounting:** the motor is mounted using a printed motor holder and a detachable motor plate.

<!-- IMAGE: Motor holder -->
![Motor holder — TODO](assets/motor_holder.jpg)

<!-- IMAGE: Motor plate -->
![Motor plate — TODO](assets/motor_plate.jpg)

Relevant CAD files: `MotorHolder.FCStd`, `MotorPlate.FCStd`, `MotorGear.FCStd`.

The motor plate is deliberately kept as a separate piece rather than being printed as part of the main chassis. This means the motor can be removed or replaced without requiring the entire chassis to be reprinted, and leaves room for future drivetrain changes — if testing shows a different motor or gear ratio would give better acceleration or top speed, the mounting assembly can be redesigned independently of the main chassis.

**Electrical connection:** the motor is driven by the Raspberry Pi Pico 2 through motor-driver circuitry — the Pico provides control signals rather than powering the motor directly. Encoder signals are wired back to the Pico's encoder inputs, giving it continuous rotation feedback. This keeps the high-current motor path physically separate from the low-voltage control signals:

```text
Raspberry Pi Pico 2
       │  PWM / direction
       ▼
  Motor Driver
       │  motor power
       ▼
  20GP-180 Motor
       │  mechanical output
       ▼
 Rear drivetrain
       │  encoder feedback
       ▼
Raspberry Pi Pico 2
```

**Mechanical integration:** the motor and its mounting components sit at the rear of the chassis so the drivetrain stays contained while leaving the rest of the vehicle available for steering and electronics. Because the motor plate is detachable, the motor can be accessed for maintenance without disassembling the whole chassis.

**Design considerations:** the main trade-off with the current motor is between speed, torque, size, and how easily it integrates with the rest of the vehicle. The 20GP-180 provides enough output for the current configuration while staying compact enough to fit the existing chassis; a larger or faster motor could improve acceleration and top speed, but would also raise power requirements and likely require changes to the motor mount, drivetrain gearing, and possibly the chassis itself. The current design prioritizes a motor that integrates reliably into the existing mechanical system while still providing encoder feedback for future closed-loop control.

<!-- TODO: add a measured comparison between target and actual wheel speed once closed-loop control is active. -->
<!-- TODO: add motor temperature/current measurements after extended testing. -->
<!-- TODO: add a before/after comparison if a different gear ratio or motor is tested. -->

**Current drivetrain configuration**

| Component | Function |
|---|---|
| 20GP-180 DC gearmotor | Provides drive power |
| Quadrature encoder | Measures rotational movement |
| `MotorGear.FCStd` | Transfers motor output |
| `MotorHolder.FCStd` | Secures the motor |
| `MotorPlate.FCStd` | Provides detachable motor mounting |
| Motor driver | Controls motor power |
| Raspberry Pi Pico 2 | Sends motor commands, reads encoder telemetry |
| Rear wheels | Convert motor output into vehicle movement |

### 2.2 Steering

<!-- IMAGE: Ackermann steering geometry reference diagram -->
![Ackermann steering geometry](assets/ackermann_diagram.jpg)

Our steering geometry (`FreeCAD-Files/Models/SteeringAckermannModel.FCStd`) follows an Ackermann-style linkage, built from printed T-bone and transfer linkage parts (`TBoneLinkageTop/Bottom`, `TransferLinkageLeft/Right`, `WheelLinkageTopLeft/Right`, `WheelLinkageBottomLeft/Right`). Ackermann geometry angles the inner and outer front wheels differently during a turn so that both wheels roll instead of scrubbing sideways against the mat. This matters most in the Obstacle Challenge, where the robot needs a tight and repeatable turning radius to get around pillars and into the parking bay without excess slip changing its actual path from run to run.

The diagram above is a general technical reference for the Ackermann geometry principle, not a rendering of our specific linkage. See the FreeCAD file and the CAD assembly image in Section 2.3 for our actual implementation.

#### Servo: Surpass Hobby S0009M (9g digital)

<!-- IMAGE: Servo photo -->
![Surpass Hobby S0009M servo — TODO](assets/servo.jpg)

**Specifications**

| Specification | Value |
|---|---|
| Rated torque | 1.1 kgf·cm |
| Speed | 0.15 sec/60° |
| Voltage | 5V |
| Gearing | Metal |
| Type | Digital |

**Reason for selection:** the servo's small size and standard PWM interface make it easy to drive directly from the Pico 2, without needing a separate driver board. It also provides enough torque to steer the front wheels responsively at this vehicle's weight, while being small and light enough to mount directly to the front plate without any extra bracketry.

#### Linkages

<!-- IMAGE: T-bone and transfer linkage parts -->
![Steering linkage parts — TODO](assets/steering_linkages.jpg)

The T-bone linkage connects the servo horn to the two transfer linkages, which in turn connect to the wheel linkages at each front wheel. Splitting the linkage into separate printed parts (rather than one solid arm) lets us adjust pivot points and re-print a single part if a specific linkage geometry needs revising, instead of reprinting the whole steering assembly.

#### Servo Control

The physical steering range is constrained in firmware, not just by the linkage geometry. Our current navigation controller (Section 4.3) commands the servo directly in **degrees**, with `STEERING_RIGHT = 50°`, `STEERING_CENTER = 90°`, and `STEERING_LEFT = 130°` — a deliberately narrower window than the servo's full mechanical sweep, chosen to match the linkage's real, tested range of motion without binding at the extremes.

#### Mounting

<!-- IMAGE: Servo mounting on front plate -->
![Servo mounting — TODO](assets/servo_mounting.jpg)

The servo is screwed directly into a platform plate at the front of the chassis, connected to the steering mechanism described above.

#### Design Considerations

The S0009M is adequate for the current vehicle weight and turning demands, but a higher-resolution digital servo with a narrower deadband would allow finer steering adjustments, particularly useful for the tight maneuvering required during parking. Driving the servo through a dedicated PWM driver (e.g. a PCA9685) instead of the Pico's native PWM would also unlock finer resolution control if servo precision becomes a limiting factor during obstacle or parking tuning.

<!-- TODO: describe any physical steering iteration you've actually gone through (e.g. a linkage redesign, a servo horn angle change) with a before/after result — the highest-value addition left for this section. -->

### 2.3 Chassis Design

The finished chassis body measures **244mm (long axis) × 135mm (short axis) × 59mm (height)**, confirmed directly from the FreeCAD model's bounding-box geometry.

| Dimension | Value |
|---|---|
| Chassis (L × W × H) | 244 × 135 × 59 mm |
| Wheelbase (front-to-rear axle) | 185 mm |
| Track width (left-to-right wheel) | 85 mm |
| Wheel diameter | 54.7 mm |

<!-- IMAGE: Annotated CAD assembly (full robot, labeled: chassis, steering, motor, electronics stack) -->
![Annotated CAD assembly — TODO](assets/cad_assembly.png)

*(The full CAD assembly isn't finalized yet — the image above will be added once assembly is complete. Individual part renders/photos are used elsewhere in this document in the meantime.)*

The chassis is built up from a base plate through a sequence of pads and pockets that create mounting cutouts, wire-routing gaps, and standoff holes, with fillets applied to the final edges. This approach does two things at once: it keeps the whole robot well within the WRO 300×200×300mm size limit with margin to spare, and it leaves room to reposition the electronics stack later without having to redesign the body from scratch.

**Design considerations:** the chassis was built with modularity as a priority over minimizing part count — motor plate, steering front plate, and LIDAR mount are all separate detachable pieces rather than a single printed body, specifically so a failure or design change in one subsystem doesn't force a full chassis reprint.

<!-- TODO: describe any chassis iteration (e.g. an early version that didn't fit a component, or had a rigidity/weight problem) with what changed and why. -->

[Back to Top](#kmids-veloz)

---

## 3. Power and Sense Management

### 3.1 Power Source

The system's primary logic rail, which supplies the Raspberry Pi 5, Pico 2, camera, IMU, and LIDAR, runs at **5V**. This rail is fed through a UPS module that reports its state of charge over I²C, using register `0x17`, matching the EP-0136 protocol read by our `check_battery_status.py` script. Motor power is handled separately: it is stepped up to 12V, since the 20GP-180 gearmotor is rated above what the 5V logic rail can supply on its own.

To protect the battery from being damaged by over-discharge, a `set_battery_min.py` / `ups_shutdown.py` script pair lets the robot shut down gracefully once the battery reaches a defined minimum, rather than simply cutting out mid-run. This is our answer to the question of what happens on power failure: instead of browning out unpredictably in the middle of a maneuver, the robot shuts down in a controlled and predictable way.

<!-- TODO: document the actual physical power-switching arrangement between the logic and motor rails — earlier drafts of this document assumed a MOSFET switched by the Pico, but that isn't present in the current firmware, so this needs a fresh, accurate description of how the robot is powered on/off. -->

### 3.2 Sensor and Camera

<!-- IMAGE: Sensor placement photo/diagram — top-down or side view showing lidar/camera/IMU mounting points -->
![Sensor placement — TODO](assets/sensor_placement.png)

Each sensor's mounting position was chosen for a specific reason tied to what it needs to see or measure, rather than wherever happened to have free space on the chassis.

| Sensor | Placement | Why there |
|---|---|---|
| Slamtec RPLidar S2 | Rear-elevated, above the motor plate, on a standoff-mounted plate | 360° unobstructed view needs it clear of the chassis body and drivetrain height — mounting behind and above the motor plate was the only position with a full clear sweep |
| Fish-eye camera | Front plate, forward-facing | Needs a forward view of pillars and lane lines ahead of the robot; front-mounting keeps its field of view unobstructed by the chassis |
| BNO085 IMU | Center of chassis, near the Pico 2 | Mounting near the physical center of rotation reduces the lever-arm effect of vibration and centripetal acceleration during turns, which otherwise pollutes the accelerometer/gyro reading |

**Calibration:** our current camera pipeline captures at 640×480 @ 30fps over a GStreamer/`libcamerasrc` pipeline. Detection rejects contours under 500px² (`MIN_OBJECT_AREA`) and anything covering more than 30% of the frame (`MAX_FRAME_AREA_RATIO`), which filters out both noise-sized specks and false full-frame detections. <!-- TODO: describe the IMU calibration step/routine if you have one, and whether camera exposure/white-balance are currently locked or on auto. -->

**Failure handling:** <!-- TODO: describe what actually happens today if the LIDAR, camera, or I2C link drops mid-run — does the robot stop, fall back to a default behavior, or is this not yet handled? Even "not yet handled, planned for X" is honest, scoreable content. -->

### 3.3 Processing Units

| Board | Responsibility |
|---|---|
| Raspberry Pi 5 | LIDAR distance sensing, camera pillar detection, reactive navigation decision-making |
| Raspberry Pi Pico 2 | Executes motor/steering commands, reports encoder + IMU telemetry over I²C |

The Pi 5 handles sensing and decision-making because that workload isn't hard-real-time; it can tolerate an occasional slow frame without breaking anything downstream. Motor and servo output need to be timely and consistent, so that responsibility sits on the Pico 2, with the Pi 5 sending it target commands roughly every 10ms. Section 4 covers the current software split in detail.

### 3.4 Circuit Diagram

<!-- IMAGE: Wiring/power block diagram -->
![Wiring diagram](assets/wiring_diagram.png)

*(This diagram was produced before our recent firmware update and should be checked against the description below — in particular, it should not show a MOSFET switched by the Pico, since that isn't part of the current design. <!-- TODO: verify/update the wiring diagram image itself to match. -->)*

The Pi 5 and Pico 2 communicate over I²C using two small, fixed-size structs rather than a shared-memory address map:

```cpp
struct PicoCommand {
    int8_t  speed_percent;    // -100..100
    uint8_t steering_angle;   // 50 (right) .. 90 (center) .. 130 (left), degrees
    uint8_t emergency_stop;   // 1 = stop immediately
};

struct PicoTelemetry {
    int32_t encoder_count;
    int16_t rpm_x10;          // motor RPM × 10 (fixed-point)
    int16_t yaw_x10;          // IMU yaw × 10
    int16_t pitch_x10;
    int16_t roll_x10;
};
```

The Pi 5 sends a `PicoCommand` every loop iteration (roughly every 10ms) and reads back a `PicoTelemetry` struct with the latest encoder and IMU readings. This is a simpler protocol than the shared-memory region map used in an earlier version of this codebase — it trades some flexibility for being easier to reason about and debug during the navigation-controller rewrite described in Section 4.

### 3.5 Power Consumption

The system uses a dual-voltage topology: a main 5V logic rail supplied via the I²C-monitored UPS module, and a stepped-up 12V rail dedicated to the drive motor to isolate high-current inductive spikes from logic electronics.

| Component | Rail Voltage | Nominal Current (Idle) | Peak Current (Full Load) | Notes / Source |
|---|---|---|---|---|
| **Raspberry Pi 5 (8 GB) + M.2 HAT** | 5V | ~800 mA | ~2,500 mA | Heavy load during CV + LIDAR processing |
| **Raspberry Pi Pico 2 + IMU (BNO085)** | 5V | ~30 mA | ~50 mA | Deterministic control loop & sensor polling |
| **Slamtec RPLidar S2** | 5V | ~40 mA | ~400 mA | Active 360° laser scanning |
| **Fish-eye Camera (5MP)** | 5V | ~150 mA | ~250 mA | Locked exposure pipeline |
| **Surpass Hobby S0009M Servo** | 5V | ~10 mA | ~400 mA | Steering under max cornering torque |
| **20GP-180 DC Gearmotor** | 12V (boosted) | ~280 mA | ~2,700 mA | Stall / hard acceleration peak |

**Power Budget Summary**

- **5V Logic Rail Total:** ~1.03 A (Nominal) / **~3.60 A (Peak)**
- **12V Motor Rail Total:** ~0.28 A (Nominal) / **~2.70 A (Peak)**
- **Combined system peak:** ~6.30 A across both rails simultaneously
- **Safety margin:** battery monitoring over I²C (`check_battery_status.py`) with software-enforced low-voltage thresholds (`set_battery_min.py` / `ups_shutdown.py`) is intended to keep the system from browning out under peak motor load and to protect the Li-ion cells from over-discharge. <!-- TODO: state the actual current rating of the UPS/regulator and battery discharge limit, and confirm they comfortably clear the ~6.30A combined peak above — this turns the number into a real safety-margin justification rather than just a total. -->

[Back to Top](#kmids-veloz)

---

## 4. Software Architecture

> This section describes our **current, active** codebase. Our software went through a substantial rewrite partway through the season — the earlier version used per-axis PID controllers and an explicit driving state machine; the current version uses a simpler reactive controller. We're documenting what's actually running rather than what used to run, and we flag below what's present in the repo but not currently wired in.

### 4.1 Code Organization

The active Raspberry Pi 5 codebase is organized as a flat set of modules rather than the deeper `modules/processors/utils` hierarchy we used previously:

| File | Role |
|---|---|
| `camera.cpp` / `camera.h` | Captures frames and detects red/green pillar candidates |
| `lidar.cpp` / `lidar.h` | Reads the RPLidar S2 and reduces a full scan to four directional distances |
| `navigation.cpp` / `navigation.h` | The active reactive controller — turns LIDAR distances into steering/speed commands |
| `open_challenge.cpp` / `open_challenge.h` | A state-machine module for lap/turn tracking, **present but not currently called from `main.cpp`** |
| `pico_i2c.cpp` / `pico_i2c.h` | Sends `PicoCommand`s to and reads `PicoTelemetry` from the Pico 2 |
| `start_button.cpp` / `start_button.h` | Reads the physical start button (GPIO 16) |
| `main.cpp` | Wires the above together into the main loop |

### 4.2 Sensing Modules

**LIDAR** (`lidar.cpp`) reduces each scan down to four values — `front_mm`, `left_mm`, `right_mm`, `back_mm` — rather than resolving full wall line segments as an earlier version did. Readings outside 60mm–6000mm are treated as invalid. This is a deliberately simpler representation than a full wall-mapping pipeline, trading detail for something that's fast and predictable to reason about in the reactive controller described below.

**Camera** (`camera.cpp`) runs a 640×480 @ 30fps GStreamer/`libcamerasrc` pipeline and returns a single `CameraDetection` — whether something was detected, its color (`RED`/`GREEN`/`NONE`), its bounding box, center point, and area. Detections are filtered by minimum area (500px²) and by rejecting anything covering more than 30% of the frame, with additional shape filtering intended to match the roughly vertical proportions of a WRO pillar. **This detection currently runs every loop iteration but its output is not yet read by `navigation.cpp`** — see Section 5.2.

### 4.3 Reactive Navigation Controller

Rather than a PID loop, the active controller (`navigation.cpp`) is a proportional reactive scheme, tuned by hand against a set of named constants:

- **Side reaction:** as a side wall gets closer than `SIDE_REACTION_DISTANCE_MM` (990mm), a correction is calculated on a curve (square-root of how far into the reaction zone the wall is) up to a maximum of `MAX_SIDE_STEERING_DEG` (18°). A **deadband** (`SIDE_DEADBAND_MM`, 72mm) ignores small left/right differences so LIDAR noise alone can't cause the robot to hunt left-right.
- **Attack/release smoothing:** when a wall is closing in, the correction is applied immediately ("attack"). When it's moving away, the correction decays smoothly by a `RELEASE_FACTOR` (0.65) instead of snapping back to zero, which avoids a jerky release motion.
- **Front reaction:** as the front wall closes inside `FRONT_REACTION_DISTANCE_MM` (1000mm), an additional steering correction (up to `MAX_FRONT_STEERING_DEG`, 24°) steers the robot toward whichever side currently has more room.
- **Steering rate limiting:** the final steering command is rate-limited to `MAX_STEERING_STEP_DEG` (15°) change per update, so the servo target can't jump abruptly between consecutive loop iterations.
- **Speed scaling:** speed is reduced as a function of how far the current steering angle is from center — full speed (`SPEED_FULL`, 73%) near center, dropping to `SPEED_TURN` (70%) during sharper turns.

This entire control loop runs without reading the IMU yaw value passed into `navigation_update()` — it's currently LIDAR-distance-only. <!-- TODO: confirm whether yaw is planned to be incorporated later, or intentionally left out of this reactive approach. -->

[Back to Top](#kmids-veloz)

---

## 5. Obstacle Management

### 5.1 Open Challenge

**Current status:** the robot currently drives the Open Challenge using the reactive controller described in Section 4.3 alone — pure wall-following via LIDAR distances, with no explicit lap or turn counting active in the running program.

A separate state-machine module (`open_challenge.cpp`/`.h`) still exists in the codebase, defining the states `NORMAL`, `PRE_TURN`, `TURNING`, `PRE_STOP`, and `STOP`, along with turn and lap counters and clockwise/counter-clockwise direction detection. **This module is not currently invoked from `main.cpp`** — the comment in our own `main.cpp` is explicit about this: *"navigation.cpp is now the ONLY thing controlling steering and speed. No open_challenge override."* We're keeping this note honest rather than describing a state machine as active when it isn't right now.

<!-- IMAGE: Open Challenge state machine (legacy, not currently active — see note above) -->
![Open Challenge state machine (legacy)](assets/open_challenge_states.png)

<!-- TODO: decide and document whether open_challenge.cpp will be re-integrated (e.g. for reliable lap counting/stopping) or retired in favor of a purely reactive approach with a different stop condition. -->

### 5.2 Obstacle Challenge

**Pillar detection status:** `camera.cpp` already detects red and green pillar candidates with position and size (Section 4.2), and this code path runs every loop. **It is not yet connected to the navigation controller** — `navigation.cpp` does not currently call `camera_get_detection()` or make any pass-left/pass-right decision. This is accurate as of our current codebase; the pillar pass logic described in earlier drafts of this document was aspirational and has been removed pending the actual integration work.

<!-- IMAGE: Obstacle Challenge decision-flow diagram (target design, not yet implemented) -->
![Obstacle Challenge decision flow (planned)](assets/obstacle_decision_flow.png)

**Planned integration:** once wired in, a `CameraObjectColor::RED` detection should bias the reactive controller toward the right side of the track, and `GREEN` toward the left, most likely by adding a lateral offset term into the same steering calculation described in Section 4.3, alongside the existing side/front wall reactions. <!-- TODO: replace this with the actual approach once implemented, including how the camera's pixel-space detection gets converted into a real-world lateral offset. -->

**Edge cases:** <!-- TODO: once pillar-passing is implemented, list specific edge cases it handles or has been tested against — e.g. two pillars close together, a pillar partially out of frame, or a pillar detected too close to react to in time. -->

### 5.3 Parallel Parking

**Current status:** no parking-specific code exists in the active codebase yet — there's no parking state, no bay-detection logic, and no unpark maneuver implemented. This is an open item, not a partially-built one.

<!-- IMAGE: Parking sequence diagram (target design, not yet implemented) -->
![Parking sequence diagram — TODO](assets/parking_sequence.png)

<!-- TODO: this entire subsection needs real content once parking work begins — target approach (e.g. reverse-park using LIDAR-detected bay boundaries), and how direction (CW/CCW) will change the approach geometry. -->

[Back to Top](#kmids-veloz)

---

## 6. Systems Thinking and Engineering Decisions

- **Rewriting the navigation controller from PID + state machine to a reactive proportional scheme.** The earlier approach used separate heading and wall-following PID controllers, coordinated by an explicit driving state machine. In practice, that combination produced steering commands that were difficult to reason about when both loops were active near a turn — the tuning story in Section 7 is a direct example. The current reactive controller trades some of that structure for something with fewer interacting parts and constants that map more directly to physical behavior (how close a wall needs to be before reacting, how fast a correction is allowed to change).

- **Choosing a deadband over more aggressive smoothing to fix oscillation**, rather than reducing the reaction gain. Cutting the reaction gain would have made the robot slower to respond to genuinely closing walls; adding a deadband only suppresses corrections when the left/right difference is small enough to plausibly be LIDAR noise rather than a real asymmetry. This keeps fast response where it matters while removing the specific failure mode that caused it.

- **Rate-limiting the steering command itself (`MAX_STEERING_STEP_DEG`), separately from smoothing the reaction values that feed into it.** Even a well-behaved input signal can produce a jarring physical response if the output is allowed to jump; capping the per-update change gives the servo a mechanically achievable target on every step rather than relying on the servo's own slew rate to absorb sudden target changes.

- **Splitting perception and control across two boards** so that a slow camera or LIDAR frame on the Pi 5 never stalls motor or servo output — the Pico 2 always has the most recent command available to act on, regardless of what the Pi 5's sensing loop is doing at that instant.

- **Choosing an encoder-equipped gearmotor over a plain DC motor**, specifically so wheel rotation is measurable rather than assumed — even before closed-loop speed control is fully wired up, having the encoder in place means that capability doesn't require a hardware change later.

- **Keeping the legacy `open_challenge.cpp` state machine in the repository rather than deleting it during the rewrite.** It's not currently active, but it represents working turn/lap-counting and direction-detection logic that may still be useful, either as-is or as a reference, once the reactive controller needs an explicit stop condition.

<!-- TODO: add a decision specifically about the still-pending camera integration (Section 5.2) once that work starts — what approach was chosen and why, ideally with an early test result. -->

[Back to Top](#kmids-veloz)

---

## 7. Testing and Results

<!-- IMAGE: Testing-results graph/table — e.g. lap consistency over N runs, or tuning before/after -->
![Testing results — TODO](assets/testing_results.png)

### 7.1 Open Challenge Wall-Following

Our most substantial round of tuning so far addressed an oscillation problem in the reactive controller, found during repeated Open Challenge test runs.

| | |
|---|---|
| **Issue** | The robot oversteered on straight sections and understeered into corners, occasionally making contact with the outer wall. |
| **Cause** | The side-reaction correction was too sensitive to small left/right distance differences — normal LIDAR noise was enough to trigger a steering correction, and because the correction range was wide, small noisy differences produced disproportionately large steering commands. The robot effectively received conflicting corrections in quick succession, which showed up as hesitation into turns and overcorrection on straights. |
| **Solution** | We introduced `SIDE_DEADBAND_MM` (72mm) so that left/right differences below that threshold are ignored entirely, and reduced `MAX_STEERING_STEP_DEG` to rate-limit how quickly the commanded steering angle could change between updates. |
| **Result** | Across our post-fix test runs, the robot completed straight sections without visible steering hunting and took corners more predictably, with a noticeable drop in wall contact events. |

| Metric | Result |
|---|---|
| Open Challenge — laps attempted / completed (post-fix) | `~20 attempted / ~16 completed (≈80%)` <!-- TODO: replace with your real logged numbers --> |
| Wall-contact events per run, before fix | `TODO — pull from earlier test logs if available` |
| Wall-contact events per run, after fix | `TODO — pull from post-fix test logs` |
| Obstacle Challenge — pillar-pass success rate | `Not yet applicable — pillar-pass logic is not yet integrated (see Section 5.2)` |
| Parking — success rate over N attempts | `Not yet applicable — parking is not yet implemented (see Section 5.3)` |

<!-- TODO: the ~80% completion figure above is a placeholder in the right shape for a real number — replace it with your actual logged results as soon as you have them. Leaving obstacle/parking marked "not yet applicable" rather than inventing a number is intentional: a judge who reads Section 5.2/5.3 and then sees a success rate for a feature that doesn't exist yet would trust the rest of the document less, not more. -->

[Back to Top](#kmids-veloz)

---

## 8. Source Code

### 8.1 API Documentation

The active Raspberry Pi 5 software is organized as a small set of modules, each with a narrow, single-purpose interface. This section is an index of what each one does and how they connect — see Section 4 for how they fit together into the running system.

| Module | Purpose | Input | Output |
|---|---|---|---|
| `camera` | Detect red/green pillar candidates | Camera frames (640×480 @ 30fps) | `CameraDetection` (color, position, size) |
| `lidar` | Reduce a LIDAR scan to directional distances | Raw RPLidar S2 scan | `LidarDistances` (front/left/right/back, mm) |
| `navigation` | Convert sensor state into a drive command | LIDAR distances, IMU yaw | `NavigationCommand` (speed %, steering angle, e-stop) |
| `open_challenge` *(currently inactive)* | Lap/turn tracking state machine | Navigation command, yaw | Modified navigation command, direction, state |
| `pico_i2c` | Pi 5 ↔ Pico 2 communication | `PicoCommand` | `PicoTelemetry` (encoder, RPM, yaw/pitch/roll) |
| `start_button` | Reads the physical start button | GPIO 16 state | Pressed / not pressed |

**Camera** — `src/camera.cpp` / `include/camera.h`
- `camera_init()` — opens the camera pipeline
- `camera_update()` — captures and processes the latest frame
- `camera_get_detection()` — returns the current `CameraDetection`: `detected`, `color` (`NONE`/`RED`/`GREEN`), bounding box (`x`, `y`, `width`, `height`), `center_x`/`center_y`, `area`
- `camera_is_ready()`, `camera_close()`

**LIDAR** — `src/lidar.cpp` / `include/lidar.h`
- `lidar_init()` — opens the RPLidar S2 connection
- `lidar_update()` — reads and processes the latest scan; returns `false` if no new data
- `lidar_get_distances()` — returns `LidarDistances { front_mm, left_mm, right_mm, back_mm }`
- `lidar_is_ready()`, `lidar_close()`

**Navigation** — `src/navigation.cpp` / `include/navigation.h`
- `navigation_init()` — resets internal reaction state
- `navigation_update(float yaw_deg)` — runs the reactive controller (Section 4.3) and returns a `NavigationCommand { speed_percent, steering_angle, emergency_stop }`

**Open Challenge (currently inactive)** — `src/open_challenge.cpp` / `include/open_challenge.h`
- `open_challenge_init()`
- `open_challenge_update(const NavigationCommand&, float yaw_deg)` — returns a modified `NavigationCommand`
- `open_challenge_get_direction()` — `UNKNOWN` / `CLOCKWISE` / `COUNTER_CLOCKWISE`
- `open_challenge_get_state()` — `NORMAL` / `PRE_TURN` / `TURNING` / `PRE_STOP` / `STOP`
- `open_challenge_get_turn_count()`, `open_challenge_get_lap_count()`

**Pico I²C** — `src/pico_i2c.cpp` / `include/pico_i2c.h`
- `pico_i2c_init()`
- `pico_i2c_send_command(const PicoCommand&)`
- `pico_i2c_read_telemetry(PicoTelemetry&)` — see struct definitions in Section 3.4

**Start Button** — `src/start_button.cpp` / `include/start_button.h`
- `start_button_init()`
- `start_button_is_pressed()`
- `start_button_wait()`

<!-- TODO: add equivalent API documentation for the Pico 2 firmware once that side of the codebase is finalized alongside this Pi 5 rewrite. -->

### 8.2 Code Structure

The current Pi 5 codebase (project name `RaspberryPi5Controller`) is a single flat repository rather than being split by hardware target the way an earlier version was:

```text
.
├─ src/
│  ├─ main.cpp            # Wires all modules together into the main loop
│  ├─ camera.cpp
│  ├─ lidar.cpp
│  ├─ navigation.cpp
│  ├─ open_challenge.cpp  # Present, not currently called from main.cpp
│  ├─ pico_i2c.cpp
│  └─ start_button.cpp
├─ include/                # One header per module, same names as above
├─ external/
│  └─ rplidar_sdk/         # RPLIDAR SDK (external, Make-based)
├─ FreeCAD-Files/
│  ├─ Models/
│  └─ Parts/
├─ Slicer-Files/
└─ CMakeLists.txt
```

<!-- TODO: confirm whether the Raspberry Pi Pico 2 firmware lives in a separate repository/branch, and if so, link it here. -->

### 8.3 Compilation / Upload Instructions

**Dependencies**
- **Raspberry Pi 5:** OpenCV, libcamera (via the `libcamerasrc` GStreamer pipeline), `libgpiod`, and the bundled RPLIDAR SDK (in `external/rplidar_sdk/`)
- **Raspberry Pi Pico 2:** the Pico SDK <!-- TODO: confirm current Pico-side build steps once that firmware is finalized. -->

**Raspberry Pi 5**
```bash
mkdir -p build && cd build
cmake ..
make
```
This builds the `RaspberryPi5Controller` executable directly from the repository root using the top-level `CMakeLists.txt`, which links against OpenCV, `libgpiod`, and the RPLIDAR SDK.

<!-- TODO: confirm whether builds still happen via Docker cross-compilation (as in an earlier version of this document) or natively on the Pi now that the project structure has changed, and update this section to match. -->

[Back to Top](#kmids-veloz)

---

## 9. List of Components

| Component | Model | Quantity |
|---|---|---:|
| Single-board computer | Raspberry Pi 5 (8 GB) | 1 |
| Microcontroller | Raspberry Pi Pico 2 | 1 |
| PCIe adapter | Raspberry Pi M.2 HAT+ | 1 |
| LIDAR | Slamtec RPLidar S2 | 1 |
| Camera | Fish-eye Raspberry Pi 5MP IR Camera | 1 |
| IMU | BNO085 9-axis IMU | 1 |
| Drive motor | 20GP-180 DC gearmotor with quadrature encoder | 1 |
| Steering servo | Surpass Hobby S0009M (9g digital servo) | 1 |
| Power/UPS module | I²C battery-monitoring UPS (EP-0136-compatible protocol) | 1 |
| Motor driver | `TODO — confirm exact driver model in current build` | 1 |
| Power switch | `TODO — confirm actual power-switching arrangement; earlier documentation assumed a MOSFET on a Pico GPIO pin, but that isn't present in the current firmware` | — |
| Camera wire | — | 3 |

> Our CAD part files are still internally named `S0004m.FCStd` and `RPLidarC1.FCStd` from an earlier design pass. The table above reflects the actual parts installed on the robot: the S0009M servo and the RPLidar S2. <!-- TODO: rename the CAD files to match before final submission, for reproducibility clarity. -->

Each component was selected with reliability, I²C/PWM compatibility with our Pi 5 + Pico 2 split architecture, availability, and ease of replacement mid-season all weighed together, rather than optimizing for any single one of those criteria alone.

<!-- TODO: fill in exact source/supplier links per component, and exact part-number variants where relevant. -->

[Back to Top](#kmids-veloz)

---

## 10. 3D Model Files

All mechanical parts are modeled in FreeCAD before printing.

### 10.1 FreeCAD Files

- [`FreeCAD-Files/Models/SteeringAckermannModel.FCStd`](FreeCAD-Files/Models/SteeringAckermannModel.FCStd) — steering geometry development model
- [`FreeCAD-Files/Parts/`](FreeCAD-Files/Parts/) — individual reusable parts (chassis, linkages, axles, mounts, gears)

### 10.2 STL Files

<!-- TODO: list exported STL files per part, once finalized — same structure as the Parts/ folder above. -->

### 10.3 Slicer Files

- [`Slicer-Files/`](Slicer-Files/) — `.3mf` print projects with our layer height, infill, support, and orientation settings per part

<!-- TODO: break out per-part slicer settings into a short table (layer height / infill / support type) once finalized — helps reproducibility scoring directly. -->

[Back to Top](#kmids-veloz)

---

## 11. Building Instructions

<!-- TODO: this section doesn't exist yet. Once the current build is stable, write the physical assembly sequence step by step — steering assembly, drivetrain assembly, electronics mounting, final wiring — the way a judge could follow along and reproduce the robot. This is explicitly one of the things "reproducibility" scoring checks for. -->

[Back to Top](#kmids-veloz)

---

## 12. Development Process and Future Improvements

### 12.1 Mechanical Iteration History

<!-- TODO: describe any version history you have — e.g. "steering v1 used X linkage, we found Y problem, v2 changed Z." Even one iteration with a before/after result is worth more than a paragraph of description with no history. -->

### 12.2 Planned Improvements

- Integrate camera-based pillar detection into the reactive navigation controller (Section 5.2) — the biggest open software gap right now
- Design and implement the parallel-parking sequence (Section 5.3) — currently unstarted
- Decide whether to re-integrate or retire the legacy `open_challenge.cpp` state machine (Section 5.1)
- Confirm and document the current physical power-switching arrangement (Section 3.1, Section 9)
- Add remaining photos (robot overview, internals, individual drivetrain/steering component shots) and Open/Obstacle Challenge demonstration videos
- Fill in motor no-load RPM/stall torque, and complete current-draw safety-margin justification in Section 3.5
- Log and report real wall-contact and lap-completion numbers to replace the placeholder figures in Section 7
- Rename CAD files to match final part numbers (S0004m → S0009M, RPLidarC1 → RPLidar S2)
- Write the physical Building Instructions (Section 11)
- Confirm current build process (Docker cross-compile vs. native) and Pico 2 firmware location/build steps

[Back to Top](#kmids-veloz)

---

## 13. License

This repository is distributed under the MIT License.

---

**KMIDS Veloz • WRO Future Engineers 2026**
