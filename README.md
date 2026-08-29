# Project Proposal: DIY Smart Walker-Bot from Salvaged Robot Vacuums

*(Phase 1 scope: Companion & Monitoring Robot — see §4)*

**Author:** [Your name]
**Date:** 29 August 2026
**Location:** Cygnet, Tasmania

## 1. Overview

This project proposes building a low-cost, "poor man's" robot by repurposing components from two retired robot vacuum cleaners, combined with a Raspberry Pi-based navigation stack and a locally-hosted LLM for conversational interaction. It is scoped in two explicit phases (see §4): **Phase 1** is a mobile companion/monitoring robot — it navigates autonomously, converses via a local LLM, and watches for fall/anomaly events, but never bears a person's physical weight. **Phase 2** — an actual physical mobility-assist walker frame a person leans on for balance — is deliberately treated as out-of-scope future work requiring purpose-built, load-rated hardware, not an assumed extension of the same salvaged platform. Splitting scope this way keeps the fall-prevention and socially-assistive-robot research below honestly matched to what salvaged vacuum-cleaner components can actually deliver.

## 2. Motivation

Commercial smart walkers and socially assistive robots (SARs) show strong evidence of benefit for mobility support, fall prevention, and companionship in older adults, but remain expensive and often inaccessible for home hobbyist experimentation. Robot vacuums are widely available, contain high-quality reusable components (geared drive motors with encoders, LiDAR units, IMUs, bump/cliff sensors), and are frequently discarded once they fail or are superseded. This project aims to combine salvaged hardware with an accessible open-source software stack (ROS2 + local LLM) to build a functional Phase 1 prototype, with Phase 2 physical mobility assistance treated as a distinct, later engineering problem rather than an assumed extension of the same hardware.

## 3. Background Research

### 3.1 Smart Walkers and Robotic Mobility Aids (informs the Phase 2 aspiration, not the Phase 1 deliverable)
- Smart walkers already demonstrate fall prevention, adaptive power assistance, and rehabilitation benefits in clinical trials, including in patients with a mean age of 85 (SafeWalker study).
- Research prototypes (MIT/GIST Adaptive Walker, ROS-based fuzzy-controlled walkers) show that combining IMUs, force sensors, and simple control loops on Raspberry Pi/PIC hardware is a well-trodden and reproducible path — for a purpose-built frame, not a repurposed vacuum drivetrain (see §4.2 and §7 on why that distinction matters).

### 3.2 Socially Assistive Robots (SARs) (directly informs the Phase 1 conversational design)
- SARs are increasingly used for companionship, reminders, and engagement in geriatric care settings, with evidence of improved mood and reduced loneliness, though effects on cognition remain unproven.
- The ARI robot trial (Paris geriatric day hospital) showed that adding a large language model to a physical robot produced statistically significant jumps in both usability and acceptability as interactions became more natural — directly supporting the "friendly LLM" component of this proposal.

### 3.3 Vacuum-to-Robot Conversion Projects (directly informs the Phase 1 hardware approach)
- The PlatypusBot project (element14/Arduino) salvaged drive motors, encoders, and a pump from a dead robot vacuum, adding a 360° LiDAR and Raspberry Pi for future SLAM work — a near-identical starting point to this proposal.
- A Roomba 581 revival project added a Raspberry Pi Zero 2W, YDLiDAR X4, IMU, and magnetometer, running ROS for full autonomous navigation.
- Hobbyist discussion threads confirm that LiDAR units from broken robot vacuums (particularly Neato models) are commonly repurposed for Arduino/Raspberry Pi robotics projects.

## 4. Scope: Two-Phase Plan

To keep the ambitious framing in §1–3 honestly matched to what salvaged vacuum-cleaner hardware can deliver, the project is split into two phases with a hard boundary between them.

### 4.1 Phase 1 — Companion/Monitoring Robot (the actual deliverable)
A robot that navigates a home autonomously, converses via a local LLM, and monitors for fall/anomaly events — but is never leaned on or physically depended on for balance. This is what §5–7 below describe and schedule.

**Explicit non-goals for Phase 1:**
- Not a mobility aid a person holds or leans on.
- Not relied upon to physically arrest a fall in progress.
- Not a substitute for a real assistive walker if genuine balance support is needed.

### 4.2 Phase 2 — Physical Mobility-Assist Frame (future work, not scheduled)
A genuine weight-bearing walker would need purpose-built, load-rated hardware: a rigid frame engineered for a person's weight, casters rated for that load, a mechanical (not just software) brake, and drive motors/gearboxes sized for continuous torque under load — none of which a salvaged robot-vacuum drivetrain provides. If pursued at all, this is a separate mechanical-engineering project layered onto a proven Phase 1 platform, not an assumed next step. It's documented here only as the aspirational direction that motivated the background research in §3.1.

## 5. Proposed Architecture

```
[Salvaged Motors + Wheels] --> [Motor Driver: L298N/BTS7960] --> [Motion Controller]
[Salvaged LiDAR + IMU + Bump Sensors] --> [Sensor Fusion: Raspberry Pi 5] --> [ROS2 Nav Stack: SLAM + Obstacle Avoidance]
[ROS2 Nav Stack] --> [Motion Controller] --> [Motor Driver]
[Hardware E-Stop] --> [Motor Driver]  (cuts power directly; bypasses Pi, ROS2, and LLM entirely)
[Software Watchdog] --> [Motor Driver]  (independent of ROS2 nav loop; halts on lost heartbeat)
[Local LLM: Ollama on 5060 Ti server] <--> [Voice I/O: Mic/Speaker + STT/TTS]
[Local LLM] <--> [Sensor Fusion / Nav Stack]
[Nav Stack + LLM] --> [Companion App: Status, Location, Fall Alerts]
[Sensor Fusion] --> [Fall/Anomaly Detection] --> [Companion App]
```

### 5.1 Hardware Layer
- **Drive**: Salvaged Roomba/Neato geared motors and wheels (with encoders where present)
- **Sensing**: Salvaged LiDAR unit (if present, e.g. Neato), bump/cliff IR sensors, added IMU for tilt/balance
- **Compute**: Raspberry Pi 5 as onboard controller; existing home server (Nvidia 5060 Ti) for LLM inference over local network
- **Safety**: physical E-stop switch wired directly into the motor driver's power line (see §5.4) — independent of the Pi, ROS2, and the LLM

### 5.2 Software Layer
- **Motor control**: L298N or BTS7960 driver interfacing salvaged motors with the Pi
- **Navigation**: ROS2 stack handling SLAM mapping, obstacle avoidance, and path planning from fused LiDAR/IMU data
- **Motion controller**: Translates navigation commands into wheel speeds; includes companion-robot behaviours such as pace matching (see §4.1 — deliberately not fall-arrest, which would require Phase 2 hardware)
- **Fall/anomaly detection**: Lightweight monitor on IMU tilt/deceleration data, triggering companion app alerts

### 5.3 Conversational Layer
- **Local LLM** (Ollama) provides voice interaction, status queries, and conversational companionship
- **Voice I/O**: Onboard mic/speaker with lightweight STT/TTS, relaying to the LLM server
- **Bidirectional nav link**: Natural-language commands (e.g. "take me back to the shed") translated into ROS2 navigation goals
- Voice-based "stop" commands are a convenience layer on top of, never a substitute for, the hardware E-stop in §5.4 — LLM/network round-trip latency makes a spoken command an unacceptably slow sole safety mechanism

### 5.4 Safety Layer
- **Hardware E-stop**: a physical switch/button wired directly into the motor driver's power supply, cutting drive-motor power regardless of what the Pi, ROS2 stack, or LLM are doing. Must exist before any motor is under program control — implemented at build-order step 2 (§6), before teleoperation even starts.
- **Software watchdog**: a lightweight process independent of the ROS2 nav stack that halts motors if it stops receiving heartbeat signals — covers software hangs/crashes, complementing (not replacing) the hardware E-stop.
- Consistent with the Phase 1 non-goals (§4.1): this safety layer exists to stop the robot safely, not to arrest a person's fall — it remains a mobile robot, not a load-bearing device.

### 5.5 Companion App
- A lightweight, local-network-only phone dashboard surfacing current location/status, fall/anomaly alerts, and a basic conversation log.
- Minimal viable build: a simple web page served from the Pi, polling status over the home network — no cloud dependency needed for a home-lab deployment.
- Not required for the first working prototype (last in the build order, §6); a plain local log file of anomaly events is a sufficient fallback until this is built.

### 5.6 Power Budget
- Rough sizing before finalizing hardware: drive motors (salvaged, exact draw depends on donor model — see §7 risk), Raspberry Pi 5 (~5–8W typical), LiDAR unit (~2–5W typical for hobby units), IMU/sensors (negligible), plus mic/speaker and any onboard STT/TTS processing.
- LLM inference itself runs on the existing 5060 Ti home server, not onboard — only voice I/O and network transmission need to be budgeted on the robot itself.
- Target enough runtime for a single monitoring session (a few hours) with margin for recharge cycles; exact figures depend on which vacuum models are salvaged.

## 6. Suggested Build Order

1. Strip both vacuums; bench-test motors, encoders, and LiDAR independently
2. Wire the hardware E-stop (§5.4) into the motor driver's power line, **then** implement basic teleoperated differential-drive motor control from the Pi — the E-stop must exist before any motor is under program control, teleoperated or autonomous
3. Integrate LiDAR/IMU; achieve a minimal ROS2 SLAM map
4. Add obstacle avoidance, the software watchdog (§5.4), and fall-style anomaly detection
5. Integrate voice I/O and connect to local LLM once the physical platform is stable
6. (Optional, once 1–5 are stable) Build the Companion App (§5.5)

Phase 2 (§4.2) is not part of this build order — it would only begin, as a separate project, once Phase 1 is complete and only if pursued at all.

## 7. Risks and Considerations

- Salvaged motor/battery voltage and current ratings must be verified before pairing with a new driver board
- LiDAR availability depends on vacuum model; not all units include a spinning LiDAR (some use simpler bump/random-walk navigation)
- **Weight-bearing capacity**: salvaged robot-vacuum drivetrains are sized to move a ~3–4kg vacuum, not to resist or support a person's weight — this is why Phase 1 explicitly excludes any weight-bearing function (§4.1). Do not repurpose this platform as a leaned-on mobility aid without the purpose-built frame described in §4.2.
- **Safety-critical latency**: LLM inference and STT/TTS happen over the local network, which is too slow to be the primary stop mechanism — the hardware E-stop (§5.4) must be independent of the LLM and, ideally, of the Pi/ROS2 stack entirely.
- **Phase conflation**: because this project deliberately sits "between" a walker and a companion robot in framing, there's a risk of treating Phase 1 hardware as load-bearing once it visually resembles a walker — the non-goals in §4.1 exist specifically to guard against that.
- This is a multi-month hobby project rather than a weekend build; scope should be managed incrementally per the build order above

## 8. References and Similar Projects

1. Smart Robotic Walker with Intelligent Close-Proximity Interaction — https://pmc.ncbi.nlm.nih.gov/articles/PMC7642877/
2. Development and Control of a Robotic Assistant Walking Aid — https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2025.1646803/full
3. GIST and MIT AI Walking Assist Robot — https://biz.chosun.com/en/en-science/2025/06/11/2KQCBWVVIRDW5G3VSEATI6T37Y/
4. ROS-Based Smart Walker with Fuzzy Posture Judgement — https://www.mdpi.com/1424-8220/21/7/2371
5. Robotic Walking Aid Rehabilitation Study (SafeWalker) — https://www.sciencedirect.com/science/article/abs/pii/S0378512218306480
6. Robotically-Augmented Walker for Older Adults (CMU) — https://www.cs.cmu.edu/~kiesler/publications/2003pdfs/2003_robotically-augmented-walker.pdf
7. Interactive Robotic Walker for Elderly Mobility (MIT) — https://people.csail.mit.edu/peterkty/pub/KuanTing_arso2010.pdf
8. Socially Assistive Walker for Daily Living Assistance — https://pmc.ncbi.nlm.nih.gov/articles/PMC11362817/
9. Exploring Opportunities and Challenges of Smart Walkers (Survey) — https://www.computer.org/csdl/magazine/pc/2025/03/10857653/23VC5gT5Xzi
10. Development of a Smart Walker for Clinical Settings — https://pmc.ncbi.nlm.nih.gov/articles/PMC12766821/
11. Wearable Lower-Limb Exoskeleton for Fall Prevention — https://pmc.ncbi.nlm.nih.gov/articles/PMC13417239/
12. A Robotic Walker That Provides Guidance (CMU/Thrun) — https://www.cs.cmu.edu/~thrun/papers/thrun.robo-walker.pdf
13. Acceptability and Usability of ARI Socially Assistive Robot (LLM Integration) — https://humanfactors.jmir.org/2025/1/e76496/
14. Implementation of Socially Assistive Robots in Geriatric Care Institutions — https://pmc.ncbi.nlm.nih.gov/articles/PMC11423382/
15. Robots for Elderly Care: Review and Multi-Criteria Optimization — https://pmc.ncbi.nlm.nih.gov/articles/PMC10178192/
16. PlatypusBot — Robot Built from Vacuum Cleaner Parts — https://blog.arduino.cc/2025/06/27/this-platypus-shaped-robot-is-built-from-vacuum-cleaner-parts/
17. Reddit: Repurposing Broken Robot Vacuum LiDAR — https://www.reddit.com/r/arduino/comments/1ilfqmr/could_the_lidar_from_broken_robot_vacuum_cleaners/
18. Turning a Broken Robot Vacuum into a Network-Controlled Platform (element14) — https://community.element14.com/challenges-projects/element14-presents/project-videos/w/documents/72070/turning-a-broken-robot-v
19. DIY Roomba 581 Revival with ROS and LiDAR — https://www.mirkosertic.de/blog/2022/02/roomba-series/
20. DIY ROS Differential-Drive Robot (Raspberry Pi) — https://github.com/dblanding/diy-ROS-robot
21. Upgrading LeKiwi Robot with LiDAR (Open-Source Autonomous Platform) — https://foxglove.dev/blog/upgrading-the-lekiwi-into-a-lidar-equipped-explorer

---
*Prepared as a personal hobby project proposal for home-lab robotics experimentation.*
