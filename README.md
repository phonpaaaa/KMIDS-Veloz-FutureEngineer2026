
# KMIDS Veloz

<div align="center">

# 2026 WRO – Future Engineers

**Official Project Documentation**

> Designing, building, and continuously improving an autonomous vehicle for the World Robot Olympiad Future Engineers challenge.

![Robot](assets/testimage.png)

</div>

---

# Our Team name is KMIDS Veloz. Hello

Welcome to the official documentation repository for **KMIDS Veloz**. This repository documents our design process, hardware, CAD models, electronics, software, testing, and engineering decisions throughout the 2026 WRO Future Engineers season.

> **Note:** This README is intentionally structured as living documentation. As the season progresses, additional CAD models, source code, testing videos, wiring diagrams, and development logs will be added.

# 2026 WRO - Future Engineers - Project Documentation

## Team Members

- **Sahas Ninvatchararang (Phonpa)**
- **Olan Sinsuriya (Olan)**
- **Phisit Chuthomsuwan (Champ)**

KMIDS Veloz is a team of students interested in robotics and engineering. Through this project, we explore autonomous driving technologies while developing practical experience in mechanical engineering, embedded systems, programming, electronics, CAD design, rapid prototyping, and collaborative problem solving.

---

# Table of Contents

1. Overview
2. List of Components
3. Mechanical Design
4. Electronics
5. Software
6. Development Process
7. Repository Structure
8. Future Improvements
9. License

---

# 1. Overview

## 1.1 About the Project

The World Robot Olympiad Future Engineers category challenges teams to build an autonomous vehicle capable of completing a driving course without human intervention.

Our project combines multiple engineering disciplines into one system. Every subsystem—mechanical, electrical, and software—is designed to work together to produce a reliable autonomous platform.

Throughout development we continually prototype, test, redesign, manufacture, and evaluate components. Every iteration helps improve reliability, consistency, serviceability, and overall vehicle performance.

Our objective is not only to perform well during competition but also to understand the engineering principles behind autonomous vehicles.

## 1.2 Images of Robot

Place robot photographs here.

Recommended images:

- Front View
- Rear View
- Left Side
- Right Side
- Top View
- Internal Electronics

## 1.3 Performance Video

Add your YouTube demonstration here.

---

# 2. List of Components

| Component | Quantity |
|-----------|---------:|
| Tamiya Wheels | 2 |
| Surpass Hobby S0009M Servo | 2 |
| Slamtec RPLidar | 1 |
| M.2 HAT | 2 |
| Camera Wire | 3 |
| Fish-eye Camera | 1 |
| MOSFET Module | 5 |
| 20GP-180 Motor with Encoder | 3 |
| BNO080 Module | 1 |
| UPS GenG Power Module | 1 |
| Raspberry Pi 5 (8 GB) | 1 |
| Raspberry Pi Pico | 1 |
| Step-Down Converter | 1 |
| DRV8871 Driver | 1 |

Each component was selected after considering reliability, compatibility, availability, ease of replacement, and suitability for an autonomous robotics platform.

---

# 3. Mechanical Design

We mainly designed in FreeCAD first, then printed for the real thing.

The repository currently contains a FreeCAD steering assembly model located at:

`FreeCAD-Files/Models/SteeringAckermannModel.FCStd`

Using CAD before manufacturing allows us to verify dimensions, improve packaging, reduce unnecessary material, and identify interference before printing.

## 3.1 Chassis

The chassis acts as the structural foundation of the robot. It supports the drivetrain, steering assembly, sensors, electronics, batteries, and mounting hardware while maintaining rigidity and allowing easy maintenance.

Future documentation will include exploded views, measurements, and weight distribution analysis.

## 3.2 Steering Mechanism

The included FreeCAD steering model documents our steering geometry development. CAD modelling allows steering linkage adjustments before manufacturing and reduces trial-and-error during assembly.

Future versions of this repository will include additional steering revisions and assembly drawings.

## 3.3 Drive System

The drivetrain transfers power from the motors to the wheels while maintaining smooth vehicle movement. Design considerations include alignment, durability, accessibility, and ease of maintenance.

---

# 4. Electronics

## 4.1 Main Controller

The Raspberry Pi 5 serves as the primary onboard computer responsible for executing high-level software and coordinating autonomous operation.

## 4.2 Sensors

Our robot integrates multiple sensors to provide environmental awareness and vehicle feedback.

Current hardware includes:

- Fish-eye Camera
- Slamtec RPLidar
- BNO080 IMU

## 4.3 Motor Drivers

Motor drivers provide controlled power delivery from the battery to the motors while isolating control electronics.

## 4.4 Power Distribution

The power system distributes energy safely between computing hardware, sensors, motor drivers, and auxiliary electronics.

Future revisions will include complete wiring diagrams.

---

# 5. Software

## 5.1 Programming Language

We used C++ Programming language and other git stuff.

The project is intended to remain modular and maintainable with reusable components and clear separation between hardware interfaces and autonomous logic.

## 5.2 Computer Vision

This section will document the complete perception pipeline including image acquisition, preprocessing, feature extraction, and autonomous decision making.

## 5.3 Vehicle Control

Future documentation will describe steering logic, speed control, and decision making.

## 5.4 Communication

This section will describe communication between onboard computing hardware and embedded controllers.

---

# 6. Development Process

## 6.1 Prototype

Like most engineering projects, development began with early prototypes. Every revision provided opportunities to improve manufacturability, assembly, reliability, and performance.

## 6.2 Testing

Testing is performed continuously throughout development. Mechanical, electrical, and software subsystems are evaluated independently before full system integration.

## 6.3 Improvements

Examples of future improvements include:

- Better cable management
- Improved component mounting
- Additional CAD optimization
- Software optimization
- Reduced vehicle weight
- Improved serviceability

## 6.4 Challenges

Building an autonomous robot requires balancing mechanical constraints, electronics packaging, software reliability, manufacturing tolerances, and time management.

Every challenge encountered becomes an opportunity to improve future iterations.

---

# 7. Repository Structure

```text
.github/
assets/
FreeCAD-Files/
└── Models/
    └── SteeringAckermannModel.FCStd

LICENSE
README.md
```

Repository contents will expand as development continues.

---

# 8. Future Improvements

Planned additions include:

- More CAD assemblies
- Wiring diagrams
- Source code
- Build instructions
- Testing data
- Performance analysis
- Autonomous software architecture
- Mechanical revision history
- Competition media
- Manufacturing documentation

---

# License

This repository is distributed under the MIT License.

---

**KMIDS Veloz • WRO Future Engineers 2026**
