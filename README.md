<div align="center">

# KMIDS Veloz
## 2026 WRO – Future Engineers
**Official Engineering Documentation**

> Designing, building, and continuously improving an autonomous vehicle for the World Robot Olympiad Future Engineers challenge.

<!-- IMAGE: Robot overview photo (final robot, 3/4 angle, clean background) -->
![Robot overview](assets/robot_overview.png)

</div>

---

## Team Members

- **Sahas Ninvatchararang (Phonpa)**
- **Olan Sinsuriya (Olan)**
- **Phisit Chuthomsuwan (Champ)**

<!-- IMAGE: Team photo -->
![Team photo](assets/team.png)

KMIDS Veloz is a team of students exploring autonomous driving through mechanical design, embedded systems, computer vision, and control theory. Over the course of the WRO Future Engineers 2026 season our work has covered hardware selection, CAD design, electronics integration, and the software architecture running on our two onboard boards, along with the testing and iteration that shaped each of those decisions. This document collects that process in one place: what we built, why we built it that way, and the status of each subsystem.

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
    - [12.1 Mechanical and Software Iteration History](#121-mechanical-and-software-iteration-history)
    - [12.2 Planned Improvements](#122-planned-improvements)
13. [License](#13-license)

---

## 1. Overview

### 1.1 About the Project

The WRO Future Engineers category requires a vehicle to complete a driving course fully autonomously. The competition is split into two challenges. In the **Open Challenge**, the robot must complete three laps of an empty track without making contact with any wall. In the **Obstacle Challenge**, the track additionally contains randomly placed obstacle pillars that the robot must pass on the correct side, and the run finishes with a precision parallel-parking maneuver into a bay whose position is not known in advance. In both challenges the driving direction and track layout are randomized before the run starts, and in the Obstacle Challenge the pillar placement is randomized too, so the robot has to work out what it's facing from its own sensors rather than following a pre-programmed path.

Our robot splits its computing across two boards, connected over I²C with the Pi 5 as master and the Pico 2 as slave. A **Raspberry Pi 5** handles perception — LIDAR distance sensing and camera-based pillar detection — and runs the navigation logic that decides steering and speed. A **Raspberry Pi Pico 2** takes those commands and drives the motor and steering servo, reporting back encoder and IMU telemetry. Because the Pico is a pure I²C slave, it only acts on the most recent command written to it and does not initiate communication itself (Section 3.3, Section 6).

Our active control software is a hand-tuned reactive LIDAR controller: it reacts directly to the four LIDAR distance readings around the robot rather than planning a path or holding a fixed heading. An earlier version of the codebase used per-axis heading control with an explicit lap/turn state machine; that module (`open_challenge.cpp`) is documented in full in Section 5.1 and Section 12.1. Camera-based pillar detection (`camera.cpp`) is documented in Section 4.2 and Section 5.2. Sections 4, 5, and 12 note the integration status of each piece.

### 1.2 Robot Images

The six required orientation views, four additional angled shots, and a photo of the internal electronics layout are collected below.

<!-- IMAGES: required six views, using our actual filenames -->
| Front | Rear | Left |
|---|---|---|
| ![Front](assets/front_view.png) | ![Rear](assets/rear_view.png) | ![Left](assets/left_view.png) |

| Right | Top | Bottom |
|---|---|---|
| ![Right](assets/right_view.png) | ![Top](assets/top_view.png) | ![Bottom](assets/bottom_view.png) |

**Additional angles:**

| Left side | Left-rear | Right side | Right-rear |
|---|---|---|---|
| ![Left side](assets/left_side_view.png) | ![Left rear](assets/left_back_side_view.png) | ![Right side](assets/right_side_view.png) | ![Right rear](assets/right_back_side_view.png) |

**Internal Electronics:**

<img src="assets/internals.png" alt="Internal electronics" width="50%">

Section 3.2 also includes a labeled top-down sensor placement diagram (`assets/sensor_placement.png`) showing LIDAR, camera, and IMU positions schematically.

### 1.3 Performance Video

**Open Challenge:** *link to be added*

**Obstacle Challenge:** *link to be added*

These two parts show both the Open and Obstacle Challenges respectively.

[Back to Top](#kmids-veloz)

---

## 2. Mobility Management

This section covers how the robot moves: what drives the rear wheels, how the front wheels are steered, and how the chassis is built to hold both systems together.

- **Drive system:** a single 20GP-180 DC gearmotor with an integrated encoder drives the rear wheels through a printed gear.
- **Steering:** the front wheels use Ackermann-geometry steering, actuated by a Surpass Hobby S0009M servo through a printed T-bone/transfer-linkage assembly.

### 2.1 Drive System

The robot uses a single **20GP-180 DC gearmotor with an integrated quadrature encoder** to drive the rear wheels. Power is transferred from the motor through a printed gear system (`MotorGear.FCStd`), while encoder feedback is wired back to the Raspberry Pi Pico 2 so wheel rotation can be measured directly instead of assumed.

<!-- IMAGE: 20GP-180 motor photo -->
<img src="assets/20gp180_motor.png" alt="20GP-180 DC gearmotor" width="50%">

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
| Encoder | Quadrature (AB dual-phase Hall) | Datasheet / hardware |
| Encoder pulses | 28 pulses/rev | Our firmware |
| Gear ratio | 100:1 | Our firmware |
| No-load RPM (this ratio) | ~120–150 RPM at 12V | Vendor family curve |
| Stall torque (this ratio) | ~4–6 kgf·cm at 12V | Vendor family curve |

The 20GP-180 family is sold across several gear ratios with no-load speed and stall torque changing accordingly, so the figures above are read from the vendor's general curve for this ratio rather than a bench measurement of our specific unit — next step is putting the wheel on a bench and logging encoder counts over a fixed time at full duty to get a real number.

**Reason for selection:** the encoder was the deciding factor in choosing this motor over an equivalent plain DC motor. Encoder feedback means wheel rotation can be measured directly rather than assumed from a PWM duty cycle, which matters because a fixed duty cycle does not produce a fixed speed — the motor's actual output changes as battery voltage, load, and friction change over the course of a run. The 20GP-180 also strikes a useful balance of torque, gearing, and physical size for the current chassis: its geared output provides enough mechanical torque to accelerate the vehicle and hold speed on the competition mat, while its dimensions fit within the rear drivetrain without needing an oversized motor mount.

**Encoder and closed-loop speed control:** the motor's quadrature encoder reports rotational feedback to the Pico 2 as part of its telemetry (`encoder_count`, `rpm_x10` in the `PicoTelemetry` struct — see Section 3.4). At 28 pulses/rev through a 100:1 gear ratio, the encoder gives fine-grained resolution on wheel rotation — over 2,800 counts per output-shaft revolution. The Pi 5's navigation controller currently commands speed open-loop as a percentage (Section 4.3); telemetry is read every loop but is not yet closed into a speed loop on the Pi 5 side. We're sequencing it this way on purpose — tuning a speed controller against steering behavior that was still unstable (Section 7) would mean chasing a moving target, so getting steering solid came first. Closing the loop on `rpm_x10` to hold a literal target wheel speed is next.

**Motor gear:** power from the motor is transferred through a printed motor gear (`MotorGear.FCStd`).

<!-- IMAGE: Motor gear -->
<img src="assets/motor_gear.png" alt="Motor gear" width="50%">

The gear is a separate printed component rather than being integrated directly into the chassis, which makes the drivetrain easier to modify if the motor, gear ratio, or wheel configuration changes during development. Our CAD parts list also includes `LegoBevelGear.FCStd` and `LegoDifferentialGear.FCStd` — we use off-the-shelf LEGO Technic gear elements paired with a `16GA.FCStd` axle rod inside the rear axle assembly rather than designing custom bevel/differential gearing from scratch, which saved print-and-fit iteration on a part that's easy to get wrong and cheap to buy correct.

**Motor mounting:** the motor is mounted using a printed motor holder and a detachable motor plate.

<!-- IMAGE: Motor holder -->
<img src="assets/motor_holder.png" alt="Motor holder" width="50%">


**Motor plate:** the detachable mounting plate that secures the motor holder to the chassis, allowing the motor to be removed or replaced without reprinting the entire chassis.


<!-- IMAGE: Motor plate -->
<img src="assets/motor_plate.png" alt="Motor plate" width="50%">

Relevant CAD files: `MotorHolder.FCStd`, `MotorPlate.FCStd`, `MotorGear.FCStd`.

The motor plate is deliberately kept as a separate piece rather than being printed as part of the main chassis. This means the motor can be removed or replaced without requiring the entire chassis to be reprinted, and leaves room for future drivetrain changes — if testing shows a different motor or gear ratio would give better acceleration or top speed, the mounting assembly can be redesigned independently of the main chassis.

**Electrical connection:** the motor is driven by the Raspberry Pi Pico 2 through motor-driver circuitry — the Pico provides PWM/direction control signals rather than powering the motor directly. Encoder signals are wired back to the Pico's encoder inputs, giving it continuous rotation feedback. This keeps the high-current motor path physically separate from the low-voltage control signals:

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

**Design considerations — torque, speed and integration trade-off:** the main trade-off with the current motor is between speed, torque, size, and how easily it integrates with the rest of the vehicle. At a 100:1 ratio the 20GP-180 gives up top speed in exchange for torque and low-speed control resolution — we picked this ratio deliberately over a lower one (e.g. 30:1–50:1) because the Obstacle Challenge rewards controlled, repeatable maneuvering around pillars and into the parking bay more than raw straight-line speed, and our reactive controller (Section 4.3) already caps speed to 70–73% duty in normal driving. The current ceiling on lap time is cornering confidence, which is a steering/tuning problem (Section 7), not a motor problem. A shorter gear ratio would raise top speed but reduce the torque margin available for accelerating out of corners, and would need a re-tune of the whole reactive controller's speed table.

**Current drivetrain configuration**

| Component | Function |
|---|---|
| 20GP-180 DC gearmotor | Provides drive power |
| Quadrature encoder | Measures rotational movement |
| `MotorGear.FCStd` | Transfers motor output |
| `LegoBevelGear.FCStd` / `LegoDifferentialGear.FCStd` | Off-the-shelf rear axle gearing |
| `16GA.FCStd` | Rear axle rod |
| `MotorHolder.FCStd` | Secures the motor |
| `MotorPlate.FCStd` | Provides detachable motor mounting |
| Motor driver | Controls motor power |
| Raspberry Pi Pico 2 | Sends motor commands, reads encoder telemetry |
| Rear wheels | Convert motor output into vehicle movement |

### 2.2 Steering

<!-- IMAGE: Ackermann steering geometry reference diagram -->
![Ackermann steering geometry](assets/ackermann_diagram.jpg)

Our steering geometry (`FreeCAD-Files/Models/SteeringAckermannModel.FCStd`) follows an Ackermann-style linkage, built from printed T-bone and transfer linkage parts (`TBoneLinkageTop/Bottom`, `TransferLinkageLeft/Right`, `WheelLinkageTopLeft/Right`, `WheelLinkageBottomLeft/Right`, `AxleHolder`, `FrontWheelAxleLeft/Right`, `FrontWheelStopper`). Ackermann geometry angles the inner and outer front wheels differently during a turn so that both wheels roll instead of scrubbing sideways against the mat. This matters most in the Obstacle Challenge, where the robot needs a tight and repeatable turning radius to get around pillars and into the parking bay without excess slip changing its actual path from run to run.

The diagram above is a general technical reference for the Ackermann geometry principle, not a rendering of our specific linkage. See the FreeCAD file and the CAD assembly image in Section 2.3 for our actual implementation.

#### Servo: Surpass Hobby S0009M (9g digital)

<!-- IMAGE: Servo photo -->
<img src="assets/servo.png" alt="Surpass Hobby S0009M servo" width="50%">

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
<img src="assets/steering_linkages.png" alt="Steering linkage parts" width="60%">

The T-bone linkage connects the servo horn to the two transfer linkages, which in turn connect to the wheel linkages at each front wheel. Splitting the linkage into separate printed parts (rather than one solid arm) lets us adjust pivot points and re-print a single part if a specific linkage geometry needs revising, instead of reprinting the whole steering assembly.

#### Servo Control

The physical steering range is constrained in firmware, not just by the linkage geometry. Our active navigation controller (`navigation.cpp`, Section 4.3) commands the servo directly in **degrees**, with `STEERING_RIGHT = 50°`, `STEERING_CENTER = 90°`, and `STEERING_LEFT = 130°` — a deliberately narrower window than the servo's full mechanical sweep, chosen to match the linkage's real, tested range of motion without binding at the extremes. The `open_challenge.cpp` module (Section 5.1) uses a slightly narrower range (`60°`–`90°`–`120°`) from an earlier tuning pass. Next step: reconcile the two ranges before `open_challenge.cpp` is ever reactivated, so it can't command an angle outside what the current linkage was last verified against.

#### Mounting

<!-- IMAGE: Servo mounting on front plate -->
![Servo mounting](assets/servo_mounting.png)

The servo is screwed directly into a platform plate at the front of the chassis, connected to the steering mechanism described above.

#### Design Considerations

The S0009M is adequate for the current vehicle weight and turning demands, but a higher-resolution digital servo with a narrower deadband would allow finer steering adjustments, particularly useful for the tight maneuvering required during parking. Driving the servo through a dedicated PWM driver (e.g. a PCA9685) instead of the Pico's native PWM would also unlock finer resolution control if servo precision becomes a limiting factor during obstacle or parking tuning.

Next step: document a physical steering iteration (e.g. a linkage dimension change or a servo horn angle change) with a measured before/after result, once one has actually been tested.

### 2.3 Chassis Design

The finished chassis body measures **244mm (long axis) × 135mm (short axis) × 59mm (height)**, confirmed directly from the FreeCAD model's bounding-box geometry.

| Dimension | Value |
|---|---|
| Chassis (L × W × H) | 244 × 135 × 59 mm |
| Wheelbase (front-to-rear axle) | 185 mm |
| Track width (left-to-right wheel) | 85 mm |
| Wheel diameter | 54.7 mm |

<!-- IMAGE: Annotated CAD assembly (full robot, labeled: chassis, steering, motor, electronics stack) -->
![Annotated CAD assembly — TODO: add once assembly is finalized](assets/cad_assembly.png)

The full annotated CAD assembly render will be added once the complete assembly is finalized in FreeCAD; individual part renders/photos are used elsewhere in this document in the meantime.

The chassis is built up from a base plate through a sequence of pads and pockets that create mounting cutouts, wire-routing gaps, and standoff holes, with fillets applied to the final edges. This approach does two things at once: it keeps the whole robot well within the WRO 300×200×300mm size limit with margin to spare, and it leaves room to reposition the electronics stack later without having to redesign the body from scratch.

**Design considerations:** the chassis was built with modularity as a priority over minimizing part count — motor plate, steering front plate, and LIDAR mount are all separate detachable pieces rather than a single printed body, specifically so a subsystem redesign doesn't force a full chassis reprint. A `FrontCover.FCStd` part closes off the front electronics/servo area separately from the main chassis shell for the same reason.

Our CAD files for the servo and LIDAR mount are still internally named `S0004m.FCStd` and `RPLidarC1.FCStd`, left over from an earlier hardware evaluation pass before we settled on the S0009M and RPLidar S2. The mounts themselves already fit the parts we use — only the file names need renaming (Section 9, Section 12.2).

[Back to Top](#kmids-veloz)

---

## 3. Power and Sense Management

### 3.1 Power Source

The system's primary logic rail, which supplies the Raspberry Pi 5, Pico 2, camera, IMU, and LIDAR, runs at **5V**. This rail is fed through a UPS module that reports its state of charge over I²C, using register `0x17`, matching the EP-0136 protocol read by our `check_battery_status.py` script. Motor power is handled separately: it is stepped up to 12V, since the 20GP-180 gearmotor is rated above what the 5V logic rail can supply on its own.

To protect the battery from being damaged by over-discharge, a `set_battery_min.py` / `ups_shutdown.py` script pair lets the robot's software shut down gracefully once the battery reaches a defined minimum, rather than simply cutting out mid-run.

**Physical power switching:** the robot is switched on and off at the battery/UPS pack itself via its physical switch, not through a software-controlled line. The Raspberry Pi OS shutdown sequence (`ups_shutdown.py`) is a separate mechanism from motor-rail power, which stays live whenever the pack is on. The only software-side stop mechanism right now is the `emergency_stop` flag in `PicoCommand` being honored by the Pico firmware — there's no relay or MOSFET gating the 12V motor rail independently. A software-triggered cutoff on that rail is on our list (Section 12.2); in the meantime a physical, easily reachable kill switch on the battery pack is standard practice at competition regardless of what the software does.

### 3.2 Sensor and Camera

<!-- IMAGE: Sensor placement photo/diagram — top-down or side view showing lidar/camera/IMU mounting points -->
<img src="assets/sensor_placement.png" alt="Sensor placement" width="60%">

Each sensor's mounting position was chosen for a specific reason tied to what it needs to see or measure, rather than wherever happened to have free space on the chassis.

| Sensor | Placement | Why there |
|---|---|---|
| Slamtec RPLidar S2 | Rear-elevated, above the motor plate, on a standoff-mounted plate | 360° unobstructed view needs it clear of the chassis body and drivetrain height — mounting behind and above the motor plate was the only position with a full clear sweep |
| Fish-eye camera | Front plate, forward-facing | Needs a forward view of pillars and lane lines ahead of the robot; front-mounting keeps its field of view unobstructed by the chassis |
| BNO085 IMU | Center of chassis, near the Pico 2 | Mounting near the physical center of rotation reduces the lever-arm effect of vibration and centripetal acceleration during turns, which otherwise pollutes the accelerometer/gyro reading |

**Camera calibration and pipeline:** our camera pipeline captures at 640×480 @ 30fps over a GStreamer/`libcamerasrc` pipeline (`libcamerasrc ! video/x-raw,format=NV12,colorimetry=bt709,width=640,height=480,framerate=30/1 ! queue ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false`). The `drop=true max-buffers=1 sync=false` appsink configuration always hands the processing loop the newest available frame and discards anything older, rather than letting frames queue up if a loop iteration runs slow — for a reactive controller a stale frame is worse than a dropped one. The camera is physically mounted upside-down on the front plate, so every captured frame is rotated 180° in software before any detection runs. Detection converts to HSV and thresholds two ranges — green (`H 35–90, S 80–255, V 50–255`) and red, which wraps around hue 0 so it's built from two ranges (`H 0–10` and `H 170–179`, both `S 100–255, V 60–255`) combined with a bitwise OR. Both masks go through a morphological open then close (3×3 kernel) to remove speckle noise and close small gaps before contour detection. Detected contours are rejected if they're under 500px² (`MIN_OBJECT_AREA`) or cover more than 30% of the frame (`MAX_FRAME_AREA_RATIO`). A further shape filter requires `height > width * 1.15` and a minimum bounding box of 40×15px, since a WRO pillar is reliably taller than it is wide. Exposure/white balance are still on the pipeline's defaults rather than explicitly locked — that's next once we're tuning thresholds against actual competition-venue lighting.

<!-- IMAGE: Camera detection pipeline -->
![Camera detection pipeline](assets/camera_pipeline.png)

**LIDAR filtering pipeline:** raw scan points outside 40mm–9000mm are discarded before any further processing (`lidar.cpp`). Valid points are grouped into four angular sectors — front (±6°), left/right (±8° each), back (±8°). Each sector's distance is the **median** of all points that landed in it, and that raw median is then passed through a temporal filter: a normal frame-to-frame change is smoothed with an exponential moving average (`FILTER_ALPHA = 0.40`), but a jump larger than 1200mm is only accepted once it's been seen for 3 consecutive frames (`JUMP_CONFIRM_FRAMES`) — this stops a single bad LIDAR return from producing a one-frame phantom wall or opening. The navigation controller (Section 4.3) then applies its own, separate validity window (60mm–6000mm) on top of this already-filtered value. For debugging, `lidar_update()` also renders a top-down occupancy image (800×800px, 0.08 px/mm) with the raw point cloud, Hough-transform-detected wall line segments, and range rings, saved to disk every scan — useful for bench tuning; disabling that continuous write for the competition build is on our list (Section 12.2).

<!-- IMAGE: LIDAR angular sector diagram -->
<img src="assets/lidar_sectors.png" alt="LIDAR angular sectors" width="70%">

The diagram above shows the four angular sectors (front ±6°, left/right ±8°, back ±8°) the raw point cloud is grouped into before the median + EMA filtering described above is applied — narrower sectors than an earlier ±15° version, specifically to keep an adjacent wall or opening from bleeding into the wrong sector's reading.

**Failure handling:** if the LIDAR read fails for a cycle (`lidar_update()` returns `false`), the main loop skips sending a new command that iteration and retries after a short sleep rather than acting on stale or zeroed data. If the camera fails to open, `main()` exits before the robot is allowed to start. If the I²C link to the Pico drops, `pico_i2c_send_command()` returns `false` and the failure is logged; the Pico, as a pure I²C slave, simply stops receiving new targets until the link returns. An explicit retry/reconnect path for the I²C link, and a defined Pico-side behavior if it stops receiving commands for too many cycles, are both still open items (Section 6, Section 12.2).

### 3.3 Processing Units

| Board | Role | Responsibility |
|---|---|---|
| Raspberry Pi 5 | I²C master | LIDAR distance sensing, camera pillar detection, reactive navigation decision-making |
| Raspberry Pi Pico 2 | I²C slave (address `0x39`) | Executes motor/steering commands, reports encoder + IMU telemetry |

<!-- IMAGE: Pi 5 / Pico 2 I2C master-slave relationship -->
![I2C master/slave relationship](assets/i2c_master_slave.png)

The Pi 5 handles sensing and decision-making because that workload isn't hard-real-time; it can tolerate an occasional slow frame without breaking anything downstream. Motor and servo output need to be timely and consistent, so that responsibility sits on the Pico 2, with the Pi 5 sending it target commands roughly every 10ms. Because the Pico is purely a slave on this bus, it never initiates communication — it only acts on the most recent command the Pi 5 wrote to it. Section 4 covers the current software split in detail.

### 3.4 Circuit Diagram

<!-- IMAGE: Wiring/power block diagram -->
![Wiring diagram](assets/wiring_diagram.png)

The Pi 5 and Pico 2 communicate over I²C at address `0x39`. Commands are sent as three raw bytes — speed percent, steering angle, and an emergency-stop flag — packed directly from a `PicoCommand` struct; telemetry is read back as a fixed-size byte block and reinterpreted directly as a `PicoTelemetry` struct via `memcpy`:

```cpp
struct PicoCommand {
    int8_t  speed_percent;    // -100..100
    uint8_t steering_angle;   // 50 (right) .. 90 (center) .. 130 (left), degrees
    uint8_t emergency_stop;   // 1 = stop immediately
};

struct PicoTelemetry {
    int32_t encoder_count;
    int16_t rpm_x10;          // motor RPM x 10 (fixed-point)
    int16_t yaw_x10;          // IMU yaw x 10
    int16_t pitch_x10;
    int16_t roll_x10;
};
```

The Pi 5 sends a `PicoCommand` every loop iteration (roughly every 10ms) and reads back a `PicoTelemetry` struct with the latest encoder and IMU readings. This is a simpler protocol than a shared-memory address map — it trades some flexibility for being easier to reason about and debug. One thing worth flagging: reading `PicoTelemetry` via raw `memcpy` assumes identical struct packing/alignment between the Pi 5 (ARM Cortex-A76) and Pico 2 (RP2350) toolchains, which hasn't been formally verified — it works today, but a future struct or compiler change on either side could silently corrupt telemetry. Moving to an explicit fixed-layout serialization (or at minimum a version byte and checksum) is on our list (Section 12.2).

### 3.5 Power Consumption

The system uses a dual-voltage topology: a main 5V logic rail supplied via the I²C-monitored UPS module, and a stepped-up 12V rail dedicated to the drive motor to isolate high-current inductive spikes from logic electronics.

![Power budget by component](assets/power_budget.png)

| Component | Rail Voltage | Nominal Current (Idle) | Peak Current (Full Load) | Notes / Source |
|---|---|---|---|---|
| **Raspberry Pi 5 (8 GB) + M.2 HAT** | 5V | ~800 mA | ~2,500 mA | Heavy load during CV + LIDAR processing |
| **Raspberry Pi Pico 2 + IMU (BNO085)** | 5V | ~30 mA | ~50 mA | Deterministic control loop & sensor polling |
| **Slamtec RPLidar S2** | 5V | ~40 mA | ~400 mA | Active 360° laser scanning |
| **Fish-eye Camera (5MP)** | 5V | ~150 mA | ~250 mA | See Section 3.2 for exposure lock status |
| **Surpass Hobby S0009M Servo** | 5V | ~10 mA | ~400 mA | Steering under max cornering torque |
| **20GP-180 DC Gearmotor** | 12V (boosted) | ~280 mA | ~2,700 mA | Stall / hard acceleration peak |

**Power Budget Summary**

- **5V Logic Rail Total:** ~1.03 A (Nominal) / **~3.60 A (Peak)**
- **12V Motor Rail Total:** ~0.28 A (Nominal) / **~2.70 A (Peak)**
- **Combined system peak:** ~6.30 A across both rails simultaneously
<<<<<<< Updated upstream
- **Safety margin:** battery monitoring over I²C (`check_battery_status.py`) with software-enforced low-voltage thresholds (`set_battery_min.py` / `ups_shutdown.py`) is intended to keep the system from browning out under peak motor load and to protect the Li-ion cells from over-discharge. Confirming the exact current rating printed on the UPS/regulator module against the ~6.30A combined peak above — to quantify real headroom rather than just estimate it — is a five-minute check tracked in Section 12.2.
=======
- **Safety margin:** battery monitoring over I²C (`check_battery_status.py`) with software-enforced low-voltage thresholds (`set_battery_min.py` / `ups_shutdown.py`) is intended to keep the system from browning out under peak motor load and to protect the Li-ion cells from over-discharge.
>>>>>>> Stashed changes

[Back to Top](#kmids-veloz)

---

## 4. Software Architecture

### 4.1 Code Organization

The active Raspberry Pi 5 codebase (project `RaspberryPi5Controller`, built executable `rpi5_controller`) is organized as a flat set of modules under `src/`/`include/` rather than a deeper `modules/processors/utils` hierarchy:

| File | Role |
|---|---|
| `camera.cpp` / `camera.h` | Captures frames and detects red/green pillar candidates |
| `lidar.cpp` / `lidar.h` | Reads the RPLidar S2 and reduces a full scan to four filtered directional distances |
| `navigation.cpp` / `navigation.h` | The active reactive controller — turns LIDAR distances into steering/speed commands |
| `open_challenge.cpp` / `open_challenge.h` | A heading-hold state machine for lap/turn tracking (Section 5.1) |
| `pico_i2c.cpp` / `pico_i2c.h` | Sends `PicoCommand`s to and reads `PicoTelemetry` from the Pico 2 |
| `start_button.cpp` / `start_button.h` | Reads the physical start button (GPIO 16, libgpiod v2, active-low with a 30ms hardware debounce) |
| `main.cpp` | Wires the above together into the main loop |

<!-- IMAGE: Pi 5 software module / data-flow diagram -->
![Pi 5 software module and data flow](assets/software_architecture.png)

The diagram above shows how `main.cpp` actually wires these modules together each loop iteration — sensing modules feeding into the active navigation controller, which feeds `pico_i2c` out to the Pico 2, with `open_challenge` sitting off to the side as a module that compiles but isn't in the active call path (Section 5.1).

### 4.2 Sensing Modules

**LIDAR** (`lidar.cpp`) reduces each scan down to four values — `front_mm`, `left_mm`, `right_mm`, `back_mm` — through the median + temporal-filter pipeline described in Section 3.2, rather than resolving full wall line segments the way an earlier version did. This is a deliberately simpler representation than a full wall-mapping pipeline, trading detail for something that's fast and predictable to reason about in the reactive controller below.

**Camera** (`camera.cpp`) runs the 640×480 @ 30fps GStreamer/`libcamerasrc` pipeline described in Section 3.2 and returns a single `CameraDetection` — whether something was detected, its color (`RED`/`GREEN`/`NONE`), its bounding box, center point, and area. Detection runs every loop iteration and is validated against the shape/area filters in Section 3.2. Wiring `camera_get_detection()`'s output into `navigation.cpp`'s steering decision is the next step — see Section 5.2 for the target design.

### 4.3 Reactive Navigation Controller

<img src="assets/navigation_control_loop.png" alt="Reactive navigation control loop" width="70%">

Rather than planning a path or holding a fixed heading, the active controller (`navigation.cpp`) is a proportional reactive scheme: every ~10ms it looks at the four current LIDAR distances and reacts directly, tuned by hand against a set of named constants.

- **Side reaction:** as a side wall gets closer than `SIDE_REACTION_DISTANCE_MM` (990mm), a correction is calculated on a square-root curve — `sqrt(ratio) * MAX_SIDE_STEERING_DEG` where `ratio` is how far into the 990mm→120mm reaction zone the wall has intruded — up to a maximum of `MAX_SIDE_STEERING_DEG` (18°). The square-root shape gives a fast initial response as a wall first enters the reaction zone, then tapers as the wall gets very close, rather than responding linearly. A **deadband** (`SIDE_DEADBAND_MM`, 72mm) ignores small left/right differences so LIDAR noise alone can't trigger a steering correction — this constant is the direct result of the tuning work described in Section 7.
- **Attack/release smoothing:** when a wall is closing in, the correction is applied immediately ("attack"). When it's moving away, the correction decays smoothly by a `RELEASE_FACTOR` (0.65) each update instead of snapping back to zero, which avoids a jerky release motion as the robot straightens out after a correction.
- **Front reaction:** as the front wall closes inside `FRONT_REACTION_DISTANCE_MM` (1000mm), an additional steering correction (same square-root shape, up to `MAX_FRONT_STEERING_DEG`, 24°) steers the robot toward whichever side currently has more room — the side with the larger LIDAR reading.
- **Steering rate limiting:** the final steering command is rate-limited to `MAX_STEERING_STEP_DEG` (15°) change per update, so the servo target can't jump abruptly between consecutive loop iterations, and is snapped exactly to center once it's within 1° to avoid perpetual small jitter around 90°.
- **Speed scaling:** speed is reduced as a function of how far the current steering angle is from center — `SPEED_FULL` (73%) within 8° of center, `SPEED_MID` (71%) between 8°–20°, dropping to `SPEED_TURN` (70%) beyond 20° of correction.

The controller accepts `yaw_deg` in its function signature but the current implementation is LIDAR-distance-only — we're holding off on adding IMU yaw until we've evaluated whether it actually improves the reactive scheme, since heading drift would need its own tuning pass to integrate cleanly rather than being a drop-in addition (Section 12.2).

[Back to Top](#kmids-veloz)

---

## 5. Obstacle Management

### 5.1 Open Challenge

**Status:** the robot currently drives the Open Challenge using the reactive controller described in Section 4.3 — pure wall-following via LIDAR distances. `main.cpp` documents this directly in its own comments: *"navigation.cpp is now the ONLY thing controlling steering and speed."*

<!-- IMAGE: Open Challenge state machine -->
![Open Challenge state machine](assets/open_challenge_states.png)

A second module, `open_challenge.cpp`/`.h`, implements a more structured heading-hold approach and is documented here in full since it represents working, tested design logic:

- **Direction detection:** on startup the robot doesn't know whether the course runs clockwise or counter-clockwise. It compares left vs. right LIDAR distance once the front wall is within 1600mm (`DIRECTION_DETECT_FRONT_MM`); a persistent 220mm+ difference (`DIRECTION_SIDE_DIFF_MM`) in one direction, confirmed over 4 consecutive frames (`DIRECTION_CONFIRM_COUNT`) to reject a single noisy reading, locks in `CLOCKWISE` or `COUNTER_CLOCKWISE`.
- **Heading-hold wall following:** `calculate_normal_control()` holds a target IMU heading (`target_heading`) and nudges that target by a proportional term derived from the outer wall's distance from a fixed 300mm setpoint (`WALL_HEADING_KP = 0.028`, clamped to ±14°), then steers to close the gap between the robot's actual yaw and that adjusted target (`HEADING_STEERING_KP = 0.55`).
- **Turn state machine:** `NORMAL → PRE_TURN` triggers when the front wall closes inside 1200mm and a 1.5-second cooldown since the last turn has elapsed; `PRE_TURN → TURNING` triggers at 650mm, committing to a fixed steering angle and a ±90° yaw target change based on the locked direction; `TURNING → NORMAL` resolves once heading error is within 20°, incrementing a turn counter. After 12 tracked turns (3 laps × 4 corners) it moves to `PRE_STOP`, holds for a 1.2-second finish delay, then `STOP`s with `emergency_stop` set.

Whether to re-integrate this module alongside the reactive controller (e.g. for reliable lap counting/stopping) or retire it is still an open decision — see Section 12.1 and 12.2. Its steering range needs reconciling with `navigation.cpp`'s before it's reactivated (Section 2.2).

### 5.2 Obstacle Challenge

`camera.cpp` detects red and green pillar candidates with position and size (Section 4.2), running every loop against real camera frames.

<!-- IMAGE: Obstacle Challenge decision-flow diagram -->
![Obstacle Challenge decision flow](assets/obstacle_decision_flow.png)

**Target integration design:** a `CameraObjectColor::RED` detection should bias the reactive controller toward the right side of the track, and `GREEN` toward the left — matching the WRO rule that a robot passes red pillars on their right and green pillars on their left. The plan is to convert the pillar's `center_x` pixel position (and its `area`, as a rough proxy for distance) into a lateral offset term in millimeters, and add that directly into the same steering sum that Section 4.3's side/front wall reactions already feed into, rather than a second, separate control path: detect color → confirm via the existing area/shape filters → compute how far off-center it is in frame → bias steering proportionally, capped the same way side/front reactions are already capped. Wiring `camera_get_detection()` into `navigation.cpp`'s steering calculation per this design is the highest-priority remaining software item (Section 12.2).

**Edge cases to handle during integration:**
- Two pillars visible and close together in frame — decide which governs the bias (nearest by area, or first-detected).
- A pillar only partially in frame at the edge — likely ignore below a minimum visible width rather than acting on it.
- A pillar detected too close to react to in time — LIDAR front-reaction should take priority over a color bias in this case, since colliding is worse than misjudging which side to pass on.

### 5.3 Parallel Parking

<!-- IMAGE: Parking sequence diagram -->
![Parking sequence diagram](assets/parking_sequence.png)

**Target approach:** a LIDAR-only reverse-park using the same four-direction distance read the reactive controller already relies on, extended to watch the side and back distances specifically for the bay's opening once the robot is past the last pillar. Planned sequence: drive past the parking zone at reduced speed while watching the side LIDAR reading for the gap where the bay wall recesses; confirm the gap over multiple frames using the same multi-frame confirmation pattern `open_challenge.cpp`'s direction detection already uses (Section 5.1); steer into a reverse arc using the back and side distances to judge depth and squareness. Driving direction (CW/CCW) changes which side the bay opening appears on and which way the reverse arc curves, the same way it changes the outer-wall side in `open_challenge.cpp`. Implementing parking-bay detection and the reverse-park maneuver per this approach is the second-highest-priority software item, right after Section 5.2 (Section 12.2).

[Back to Top](#kmids-veloz)

---

## 6. Systems Thinking and Engineering Decisions

- **Choosing a reactive proportional controller over the heading-hold state machine for the current build.** `open_challenge.cpp` (Section 5.1) uses explicit states, a held heading target, and a wall-offset correction. In bench testing, the combination of a heading-hold loop and a wall-following correction produced steering commands that were harder to predict near a turn, since both terms could push in different directions depending on exactly when a turn was detected relative to the heading error at that instant. The reactive controller trades that structure for a smaller, more predictable set of interacting parts — constants that map directly to physical behavior (how close a wall needs to be before reacting, how fast a correction is allowed to change) rather than to two coupled control loops.

- **Choosing a deadband over a lower reaction gain to fix oscillation** (see Section 7 for the full story). Cutting the reaction gain would have made the robot slower to respond to genuinely closing walls; adding a 72mm deadband only suppresses corrections when the left/right difference is small enough to plausibly be LIDAR noise rather than a real asymmetry.

- **Rate-limiting the steering command itself (`MAX_STEERING_STEP_DEG`), separately from smoothing the reaction values that feed into it.** Even a well-behaved input signal can produce a jarring physical response if the output is allowed to jump; capping the per-update change gives the servo a mechanically achievable target on every step rather than relying on the servo's own slew rate to absorb sudden target changes.

- **Splitting perception and control across two boards, with the Pi 5 as I²C master and the Pico 2 as a pure slave**, so that a slow camera or LIDAR frame on the Pi 5 never stalls motor or servo output. The trade-off: the Pico has no way to act independently if the link drops (Section 3.3) — see the risk item below.

- **Choosing an encoder-equipped gearmotor over a plain DC motor**, specifically so wheel rotation is measurable rather than assumed — even before closed-loop speed control is fully wired up (Section 2.1), having the encoder in place means that capability doesn't require a hardware change later.

- **Keeping the `open_challenge.cpp` state machine in the repository as a maintained, documented module rather than deleting it.** It contains working turn/lap-counting, direction-detection, and heading-hold logic that may be reactivated once the reactive controller needs an explicit stop condition.

- **Risk — no hardware interlock on the motor rail.** There's no MOSFET or relay gating the 12V motor rail from software (Section 3.1); the only stop mechanism is the `emergency_stop` flag inside `PicoCommand` being honored by the Pico firmware. Mitigation: a physical, easily reachable kill switch on the battery pack at competition, independent of software state.

- **Risk — I²C struct layout assumed, not formally verified, between two different architectures.** Covered in Section 3.4: reading `PicoTelemetry` via raw `memcpy` assumes identical struct packing between the Pi 5 (ARM Cortex-A76) and Pico 2 (RP2350) toolchains. Mitigation planned: explicit fixed-layout serialization (Section 12.2).

- **Constraint — pillar integration and parking share the same underlying LIDAR/camera data pipeline and are both required for full Obstacle Challenge scoring.** Both are scoped and sequenced in Section 12.2, with pillar integration prioritized first since it reuses more of the already-working reactive controller.

[Back to Top](#kmids-veloz)

---

## 7. Testing and Results

![Testing results — wall contact and completion rate](assets/testing_results.png)

### 7.1 Open Challenge Wall-Following

Our most substantial round of tuning so far addressed an oscillation problem in the reactive controller, found during repeated Open Challenge test runs on our practice mat.

| | |
|---|---|
| **Issue** | The robot oversteered on straight sections and occasionally understeered into corners, making contact with the outer wall. |
| **Cause** | The side-reaction correction had no deadband and a wide reaction range, so ordinary LIDAR measurement noise — a few millimeters of frame-to-frame difference between the left and right readings — was enough to trigger a real steering correction. Because the reaction curve mapped even a small L/R difference to a non-trivial steering angle, the robot was effectively receiving small, conflicting corrections every ~10ms. On a straight, this showed up as constant small left-right hunting instead of a stable centered path; going into a turn, the same hunting behavior delayed the point at which a real, large asymmetry (an actual approaching corner wall) was allowed to dominate the steering command, so the turn-in itself was late and understeered. |
| **Solution** | We introduced `SIDE_DEADBAND_MM` (72mm) so that left/right differences below that threshold are ignored entirely, and separately added `MAX_STEERING_STEP_DEG` (15°) to rate-limit how quickly the commanded steering angle is allowed to change between consecutive updates. |
| **Result** | Across our post-fix test runs, the robot held a visibly straighter line on straight sections and turned into corners more decisively once a real wall asymmetry appeared, with a clear drop in wall-contact events per run (see chart above). |

**Logged test results (5-session sample):**

| Metric | Before fix | After fix |
|---|---:|---:|
| Wall-contact events per run (avg. of 5 runs) | 4.4 | 0.6 |
| Open Challenge laps attempted | — | 20 |
| Open Challenge laps completed without a wall-contact ending the run | — | 16 (≈80%) |
| Obstacle Challenge — pillar-pass success rate | Not yet measurable — pillar-pass integration (Section 5.2) isn't wired in yet |
| Parking — success rate over N attempts | Not yet measurable — parking (Section 5.3) isn't implemented yet |

### 7.2 What we'd still like to measure

- Wall-contact events and completion rate broken out separately for clockwise vs. counter-clockwise runs.
- Actual motor current draw during hard acceleration and stall, to replace the estimated figures in Section 3.5 with bench-measured ones.
- Pillar-pass and parking success rate, once Section 5.2 and 5.3 are implemented — this test procedure repeats as-is once there's a real behavior to measure.

[Back to Top](#kmids-veloz)

---

## 8. Source Code

### 8.1 API Documentation

The software is organized into hardware interface modules (LIDAR, camera, Pico I²C, start button), one active control module (navigation), and one heading-hold control module (open_challenge) that shares the same command structure. Each module exposes a small, defined interface so the rest of the system can use it without knowing its internals.

| Module | Purpose | Input | Output | Hardware |
|---|---|---|---|---|
| `camera` | Detect red/green pillar candidates | Camera frames (640×480 @ 30fps, GStreamer/libcamera) | `CameraDetection` (color, bbox, center, area) | Fish-eye 5MP CSI camera |
| `lidar` | Reduce a scan to filtered directional distances | Raw RPLidar S2 scan | `LidarDistances` (front/left/right/back, mm) | Slamtec RPLidar S2 (USB) |
| `navigation` | Active — convert LIDAR distances into a drive command | `LidarDistances`, IMU yaw (accepted, not yet used) | `NavigationCommand` (speed %, steering angle, e-stop) | — (pure logic) |
| `open_challenge` | Heading-hold wall following + lap/turn state machine | `NavigationCommand`, yaw | Modified `NavigationCommand`, direction, state, turn/lap count | — (pure logic) |
| `pico_i2c` | Pi 5 ↔ Pico 2 communication | `PicoCommand` (3 bytes) | `PicoTelemetry` (encoder, RPM, yaw/pitch/roll) | I²C, address `0x39` |
| `start_button` | Reads the physical start button | GPIO 16 state (libgpiod v2) | Pressed / not pressed | Momentary button, GPIO16, active-low |

**Camera** — `src/camera.cpp` / `include/camera.h`
```cpp
bool camera_init();                          // opens the libcamera/GStreamer pipeline
void camera_update();                        // captures + processes the latest frame (HSV threshold, contour, shape filter)
void camera_close();
bool camera_is_ready();
CameraDetection camera_get_detection();       // returns: detected, color (NONE/RED/GREEN),
                                               // x, y, width, height, center_x, center_y, area
```

**LIDAR** — `src/lidar.cpp` / `include/lidar.h`
```cpp
bool lidar_init();                            // opens the RPLidar S2 serial connection
bool lidar_update();                          // reads + filters the latest scan; false = no new/valid data this cycle
void lidar_close();
bool lidar_is_ready();
LidarDistances lidar_get_distances();         // returns: front_mm, left_mm, right_mm, back_mm (median + EMA filtered)
```

**Navigation (active controller)** — `src/navigation.cpp` / `include/navigation.h`
```cpp
void navigation_init();                       // resets internal reaction/smoothing state
NavigationCommand navigation_update(float yaw_deg);
                                               // runs the reactive controller (Section 4.3)
                                               // returns: speed_percent, steering_angle, emergency_stop
```

**Open Challenge** — `src/open_challenge.cpp` / `include/open_challenge.h`
```cpp
void open_challenge_init();
NavigationCommand open_challenge_update(const NavigationCommand&, float yaw_deg);
Direction open_challenge_get_direction();     // UNKNOWN / CLOCKWISE / COUNTER_CLOCKWISE
State open_challenge_get_state();             // NORMAL / PRE_TURN / TURNING / PRE_STOP / STOP
int open_challenge_get_turn_count();
int open_challenge_get_lap_count();
```

**Pico I²C** — `src/pico_i2c.cpp` / `include/pico_i2c.h`
```cpp
bool pico_i2c_init();                                  // opens /dev/i2c-1, targets address 0x39
bool pico_i2c_send_command(const PicoCommand&);         // writes 3 raw bytes: speed, steering, e-stop
bool pico_i2c_read_telemetry(PicoTelemetry&);           // reads sizeof(PicoTelemetry) bytes, memcpy into struct
```
See Section 3.4 for the exact struct layouts and the struct-packing assumption this relies on.

**Start Button** — `src/start_button.cpp` / `include/start_button.h`
```cpp
bool start_button_init();                     // configures GPIO16 via libgpiod v2, active-low, pull-up
bool start_button_is_pressed();                // debounced (30ms hardware debounce)
void start_button_wait();                      // blocks until pressed
```

Equivalent API documentation for the Pico 2 firmware will be added once its source is added to this repository (Section 8.2).

### 8.2 Code Structure

The current Pi 5 codebase (project `RaspberryPi5Controller`, executable target `rpi5_controller`) is organized as a flat repository:

```text
.
├── src/
│   ├── main.cpp            # Wires all modules together into the main loop
│   ├── camera.cpp
│   ├── lidar.cpp
│   ├── navigation.cpp
│   ├── open_challenge.cpp
│   ├── pico_i2c.cpp
│   └── start_button.cpp
├── include/                 # One header per module, same names as above
├── external/
│   └── rplidar_sdk/         # RPLIDAR SDK (external, Make-based, built as a static lib)
├── FreeCAD-Files/
│   ├── Models/
│   └── Parts/
├── Slicer-Files/
├── .github/                 # CI workflow (see Section 8.3)
├── .gitignore
├── LICENSE
└── CMakeLists.txt
```

<<<<<<< Updated upstream
**Repository cleanup still to do:** our working copy currently has a checked-in `build/` directory and a few loose camera test images at the repository root (`test.jpg`, `camera_test.jpg`, `camera_opencv_test.jpg`, an empty `lidar_wall.png`), along with a duplicate top-level `camera.cpp`/`camera.h` outside `src/`/`include/`. None of these are referenced by `CMakeLists.txt`. Before final submission: delete the stray root-level files, add `build/` to `.gitignore` (or `git rm -r --cached build/`), and confirm `git status` is clean against the structure above.

=======
>>>>>>> Stashed changes
**Pico 2 firmware:** the Pico 2's own firmware — the code implementing `PicoCommand`/`PicoTelemetry` handling, PWM output, and encoder/IMU reading on the RP2350 side — is not included in either zip backing this document. It exists as a separate Pico SDK build that we flash to the Pico 2 independently of the Pi 5 build in Section 8.3. Adding that firmware source to this repository (or linking to it) is the highest-priority reproducibility item — without it, the Pi 5 side is reproducible from this repo but the Pico 2 side isn't (Section 12.2).

### 8.3 Compilation / Upload Instructions

**Dependencies**
- **Raspberry Pi 5:** OpenCV, libcamera (via the `libcamerasrc` GStreamer pipeline), `libgpiod` (v2 API), `pthread`, and the bundled RPLIDAR SDK (built from `external/rplidar_sdk/`, linked as a static library)
- **Raspberry Pi Pico 2:** Pico SDK — firmware source not yet in this repository (see Section 8.2)

**Building on the Raspberry Pi 5**
```bash
git clone <repository-url>
cd KMIDS-Veloz-FutureEngineer2026
mkdir -p build && cd build
cmake ..
make
```
This builds the `rpi5_controller` executable directly from the repository root using the top-level `CMakeLists.txt`, which links against OpenCV, `libgpiod`, `pthread`, and the RPLIDAR SDK static library. The build happens natively on the Pi 5 itself — no cross-compilation or Docker step, which avoids cross-compile/ABI mismatch issues at the cost of a slower build than cross-compiling on a desktop would be.

**Running the robot**
```bash
cd build
sudo ./rpi5_controller
```
`sudo` (or equivalent GPIO/I²C group permissions) is required for GPIO and I²C device access. On startup the program initializes the camera, LIDAR, and Pico I²C link, then waits on the physical start button (Section 8.1) before entering the main control loop.

Pico 2 firmware build and flashing steps (`pico-sdk` build, `picotool`/BOOTSEL procedure) will be documented once the firmware source is added to this repository.

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
| Drive motor | 20GP-180 DC gearmotor with quadrature encoder (100:1, 28 ppr) | 1 |
| Steering servo | Surpass Hobby S0009M (9g digital servo) | 1 |
| Power/UPS module | I²C battery-monitoring UPS (EP-0136-compatible protocol, register `0x17`) | 1 |
| Drivetrain gearing | LEGO Technic bevel gear + differential gear (`LegoBevelGear`, `LegoDifferentialGear`) | 1 set |
| Rear axle rod | 16-gauge steel rod (`16GA.FCStd`) | 1 |
| Motor driver | H-bridge driver board | 1 |
| Power switch | Physical switch on the battery/UPS pack | 1 |
| Start button | Momentary push button, GPIO16 | 1 |
| Camera wire | CSI ribbon cable | 1 |

Two small cleanup items before final submission: confirm the exact motor driver model against the physical board (listed generically above), and rename `S0004m.FCStd` / `RPLidarC1.FCStd` in the CAD folder to match the S0009M servo and RPLidar S2 they actually represent (Section 2.3, Section 12.2). Exact supplier links per component are still to be filled in.

Each component was selected with reliability, I²C/PWM compatibility with our Pi 5 + Pico 2 split architecture, availability, and ease of replacement mid-season all weighed together (Section 6 covers several of these choices in more depth).

[Back to Top](#kmids-veloz)

---

## 10. 3D Model Files

All mechanical parts are modeled in FreeCAD before printing.

### 10.1 FreeCAD Files

- [`FreeCAD-Files/Models/SteeringAckermannModel.FCStd`](FreeCAD-Files/Models/SteeringAckermannModel.FCStd) — steering geometry development model
- [`FreeCAD-Files/Parts/`](FreeCAD-Files/Parts/) — individual reusable parts, listed below by subsystem

| Subsystem | Part files |
|---|---|
| Chassis / covers | `Chassis.FCStd`, `FrontCover.FCStd` |
| Drivetrain (rear) | `MotorGear.FCStd`, `MotorHolder.FCStd`, `MotorPlate.FCStd`, `LegoBevelGear.FCStd`, `LegoDifferentialGear.FCStd`, `16GA.FCStd`, `BackWheelConnector.FCStd`, `BackWheelStopper.FCStd`, `BackWheelAxle/BackWheelAxleLeft.FCStd`, `BackWheelAxle/BackWheelAxleRight.FCStd` |
| Steering (front) | `AxleHolder.FCStd`, `FrontWheelStopper.FCStd`, `FrontWheelAxle/FrontWheelAxleLeft.FCStd`, `FrontWheelAxle/FrontWheelAxleRight.FCStd`, `TBoneLinkage/TBoneLinkageTop.FCStd`, `TBoneLinkage/TBoneLinkageBottom.FCStd`, `TransferLinkage/TransferLinkageLeft.FCStd`, `TransferLinkage/TransferLinkageRight.FCStd`, `WheelLinkage/WheelLinkageTopLeft.FCStd`, `WheelLinkage/WheelLinkageTopRight.FCStd`, `WheelLinkage/WheelLinkageBottomLeft.FCStd`, `WheelLinkage/WheelLinkageBottomRight.FCStd` |
| Wheels | `Wheel.FCStd` (×4, printed once and reused) |
| Electronics mounts | `RaspberryPi5.FCStd`, `RaspberryPi5M2Hat.FCStd`, `RpiCamera.FCStd`, `RPLidarC1.FCStd` *(RPLidar S2 mount — stale filename, see Section 9)*, `S0004m.FCStd` *(S0009M servo mount — stale filename, see Section 9)* |

### 10.2 STL Files

Only `.FCStd` sources and the `.3mf` slicer projects below currently exist. Since a `.3mf` file already contains sliced geometry, printable STLs can be re-exported from either the FreeCAD parts or the existing `.3mf` files at any time — exporting them into a dedicated `STL-Files/` folder, named to match the table in Section 10.1, is a same-day task that costs us nothing we don't already have (Section 12.2).

### 10.3 Slicer Files

- [`Slicer-Files/`](Slicer-Files/) — `.3mf` print projects with our layer height, infill, support, and orientation settings per part

| Part | Slicer file present? |
|---|---|
| Chassis | ✅ `Chassis.3mf` |
| Front cover | ✅ `FrontCover.3mf` |
| Motor plate | ✅ `MotorPlate.3mf` |
| Motor gear | ✅ `MotorGear.3mf` |
| Axle holder | ✅ `AxleHolder.3mf` |
| Front wheel axles (L+R) | ✅ `FrontWheelAxleboth.3mf` |
| Back wheel axles (L+R) | ✅ `BackWheelAxleBoth.3mf` |
| Motor holder, front/back wheel stoppers, back wheel connector, steering linkages (T-bone/transfer/wheel), wheels, electronics mounts | ❌ not yet exported |

Roughly a quarter of our printed parts have a finished slicer project right now; the rest were printed from FreeCAD exports with settings chosen at print time. Exporting the remaining parts' settings into their own `.3mf` files is tracked in Section 12.2.

[Back to Top](#kmids-veloz)

---

## 11. Building Instructions

This section walks through physical assembly in the order we build the robot, referencing the CAD parts from Section 10. It assumes all parts are already printed per Section 10.3 and electronics are on hand per Section 9.

**1. Print and prep parts.** Print every part listed in Section 10.1 at the settings captured in the `.3mf` files where available (Section 10.3); for parts without a saved slicer profile, use 0.2mm layer height, ≥20% infill, and supports on any steep overhang. Remove supports and test-fit mating parts (linkage pivots especially) before moving on — a linkage that binds is much easier to fix by re-printing one part now than after the whole steering assembly is together.

**2. Assemble the rear drivetrain.** Mount the 20GP-180 motor into `MotorHolder.FCStd`, and attach `MotorGear.FCStd` to the motor's output shaft. Build the rear axle using `BackWheelAxleLeft/Right.FCStd`, the `16GA.FCStd` steel rod, `BackWheelConnector.FCStd`, and the LEGO `LegoBevelGear.FCStd`/`LegoDifferentialGear.FCStd` pair, securing both ends with `BackWheelStopper.FCStd`. Press the two rear `Wheel.FCStd` wheels onto the axle ends.

**3. Mount the motor to the chassis.** Attach `MotorHolder.FCStd` (with motor installed) to `MotorPlate.FCStd`, then screw the motor plate into the rear of `Chassis.FCStd` as a detachable connection (Section 2.1). Mesh `MotorGear.FCStd` with the rear axle gearing and confirm the rear wheels spin freely by hand before continuing.

**4. Assemble the front steering linkage.** Build the Ackermann linkage per `FreeCAD-Files/Models/SteeringAckermannModel.FCStd`: mount `AxleHolder.FCStd` to the front of the chassis, install `FrontWheelAxleLeft/Right.FCStd` through it and secure with `FrontWheelStopper.FCStd`, then connect `TBoneLinkageTop.FCStd`/`TBoneLinkageBottom.FCStd` to `TransferLinkageLeft.FCStd`/`TransferLinkageRight.FCStd`, and those in turn to `WheelLinkageTopLeft/Right.FCStd` and `WheelLinkageBottomLeft/Right.FCStd` at each front wheel hub. Press the front `Wheel.FCStd` wheels onto the front axles once the linkage moves freely by hand at both steering extremes.

**5. Install the steering servo.** Mount the S0009M servo to the front plate, then connect its horn to the T-bone linkage from step 4. With the servo centered (90°), the linkage should sit visually straight — adjust the horn's mounting spline position by one tooth if it doesn't, since mechanical offset here becomes real steering bias once `STEERING_CENTER = 90°` is commanded in software (Section 2.2, Section 4.3).

**6. Close up the front.** Install `FrontCover.FCStd` over the front electronics/servo area.

**7. Mount the electronics.** Install the Raspberry Pi 5 (`RaspberryPi5.FCStd` mount) with the M.2 HAT (`RaspberryPi5M2Hat.FCStd`) attached underneath, the Pico 2 nearby, and the BNO085 IMU as close to chassis center as the mount allows (Section 3.2). Mount the RPLidar S2 on its standoff plate at the rear, elevated above the motor plate (`RPLidarC1.FCStd` mount — stale filename, see Section 9). Mount the fish-eye camera to the front plate using `RpiCamera.FCStd` — it's installed upside-down on purpose and corrected with a 180° frame rotation in software (Section 3.2).

**8. Wire power.** Connect the battery pack to the UPS module, the UPS module's 5V output to the Pi 5/Pico 2/camera/IMU/LIDAR, and its boosted 12V output to the motor driver, following Section 3.4's wiring diagram. Confirm the physical power switch on the battery/UPS pack (Section 3.1) cuts all power before doing any further wiring.

**9. Wire data connections.** Wire I²C between the Pi 5 and Pico 2 (Pico at address `0x39`), I²C between the Pico 2 and the BNO085 IMU, the RPLidar S2 over USB, the camera over CSI, the motor driver's PWM/direction lines and encoder lines to the Pico 2, the servo's PWM line to the Pico 2, and the start button to Pi 5 GPIO16. See Section 3.4 for the full block diagram.

**10. Flash the Pico 2.** Build and flash the Pico 2 firmware (Section 8.2, Section 8.3).

**11. Build the Pi 5 software.** Follow Section 8.3 to install dependencies, clone the repository, and build `rpi5_controller`.

**12. Power-on check.** With wheels off the ground, power on, and confirm: the start button is read correctly (Section 8.1), the LIDAR reports plausible distances (Section 3.2), the camera opens and detects a held-up red or green object (Section 4.2), and the Pico 2 responds to a centered/neutral `PicoCommand` without the motor spinning unexpectedly.

**13. First driving test.** Set the robot down on an open mat, start the program, and confirm the reactive controller (Section 4.3) holds a roughly straight line and reacts sensibly as a wall is brought closer on one side — this is the same manual bench test that led to the tuning work in Section 7.

[Back to Top](#kmids-veloz)

---

## 12. Development Process and Future Improvements

### 12.1 Mechanical and Software Iteration History

**Hardware selection pass (mechanical).** Before settling on the S0009M servo and RPLidar S2, we evaluated at least one other servo and LIDAR option — visible in our CAD folder, where the servo mount is still named `S0004m.FCStd` and the LIDAR mount `RPLidarC1.FCStd` (Section 9, Section 10.1). The current Ackermann linkage (Section 2.2) is on its first built revision; a dimensioned before/after redesign will be documented here once one has actually been tested against a specific mechanical limit (binding, angle range, or slop).

**Control software rewrite.** The season started with `open_challenge.cpp`'s approach: a held IMU heading target, a proportional wall-offset correction against a 300mm outer-wall setpoint, and an explicit five-state machine (`NORMAL`/`PRE_TURN`/`TURNING`/`PRE_STOP`/`STOP`) for lap and turn counting (Section 5.1). Tuning this combination was difficult because the heading-hold term and the turn-detection state machine could disagree about robot behavior near a corner — one reasoning about absolute heading, the other about instantaneous front-wall distance. We moved to the simpler reactive scheme now active in `navigation.cpp` (Section 4.3) to remove that source of disagreement: a single, smaller set of directly-physical constants (deadband, reaction distance, max steering step) proved faster to tune by hand than the two-loop system it replaced. The `open_challenge.cpp` module remains in the repository as documented, maintained code (Section 6) rather than being deleted.

**Reactive controller tuning (measured — see Section 7).** The deadband/rate-limit fix described in Section 7.1: oversteering and wall contact on straights (problem) → added `SIDE_DEADBAND_MM` and `MAX_STEERING_STEP_DEG` (change) → wall-contact events dropped from an average of 4.4 to 0.6 per run across our 5-session sample (result).

### 12.2 Planned Improvements

Ordered by priority:

1. **Wire the camera → navigation pillar-pass integration (Section 5.2).** Detection is implemented and validated; the steering-bias integration is the remaining piece.
2. **Design and implement the parallel-parking sequence (Section 5.3).** Target approach documented; implementation pending.
3. **Add the Pico 2 firmware source to this repository** (Section 8.2, Section 8.3).
4. **Decide on `open_challenge.cpp` reactivation** alongside or instead of the reactive controller (Section 5.1, Section 6).
5. **Resolve the `PicoTelemetry` struct-packing assumption** with explicit fixed-layout serialization (Section 3.4).
6. **Export STL files and finish remaining `.3mf` slicer profiles** (Section 10.2, 10.3).
7. **Add remaining photos and videos** — the Open/Obstacle Challenge demonstration videos (Section 1.2, 1.3).
8. **Lock camera exposure/white balance** against actual competition-venue lighting (Section 3.2).

[Back to Top](#kmids-veloz)

---

## 13. License

This repository is distributed under the MIT License.

---

**KMIDS Veloz • WRO Future Engineers 2026**
