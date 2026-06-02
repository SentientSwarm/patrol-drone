# Autonomous Drone Patrol Project (Revised)

## Objectives

Design and build an autonomous drone system for:

- Indoor patrol and navigation (no GPS)
- Outdoor perimeter patrol on private property
- **Forest and trail navigation under tree canopy** (degraded or unavailable GPS)
- Waypoint-based and learned route execution
- Checkpoint inspection and image capture
- Anomaly detection from patrol imagery
- Embodied-AI and world-model experimentation (sim-to-real, learned trajectories)

Primary priorities:

- Open and programmable stack (PX4 / ROS 2 / Pixhawk)
- Single shared flight stack across indoor and outdoor platforms
- Integration with local AI infrastructure (DGX-class compute)
- Simulation-first development with rosbag replay from day one
- Visual-inertial odometry as the primary localization method everywhere; GPS as augmentation when available
- Incremental capability growth, classical-system-first then learned components

---

## High-Level Architecture

```text
                  ┌─────────────────────────┐
                  │ DGX / GPU Cluster       │
                  │-------------------------│
                  │ Simulation (Gazebo,     │
                  │   Isaac Sim)            │
                  │ Model training          │
                  │ Rosbag replay & analysis│
                  │ Heavy/offline inference │
                  │ World models            │
                  │ Mission planning        │
                  └────────────┬────────────┘
                               │
                       WiFi / 4G / Telemetry
                       (enrichment only — never
                        in the safety loop)
                               │
        ┌──────────────────────┴──────────────────────┐
        │                                             │
┌───────▼─────────────────┐               ┌───────────▼─────────────┐
│ Indoor Drone (250mm)    │               │ Outdoor Drone (X500)    │
│-------------------------│               │-------------------------│
│ Pixhawk 6C Mini (PX4)   │               │ Pixhawk 6X (PX4)        │
│ Jetson Orin Nano        │               │ Jetson Orin NX 16GB     │
│ RealSense D456          │               │ RealSense D456          │
│ Optical flow + LRF      │               │ GPS + (optional RTK)    │
│ ORB-SLAM3 / Isaac VSLAM │               │ ORB-SLAM3 / Isaac VSLAM │
│ AprilTag relocalization │               │ VIO+GPS fused via EKF2  │
│ Prop guards             │               │ Trail-following policy  │
└─────────────────────────┘               └─────────────────────────┘
```

The architectural commitment: **same flight stack, same companion compute family, same perception pipeline on both platforms.** Code, models, and tooling transfer directly. Sensors and frame size differ; everything above the driver layer does not.

---

## Development Philosophy

### Shared platform principles

- **PX4 + ROS 2 (Jazzy) + uXRCE-DDS** as the universal flight + middleware stack. (Note: we deliberately moved off Humble — see "Distro and OS decision" below.)
- **Visual-inertial odometry as the universal localizer.** GPS is an augmentation when available, not the primary mechanism. This is what makes forest, indoor, and open-field flight architecturally identical.
- **Safety-critical loop is always onboard.** VIO, obstacle avoidance, basic detection, failsafes — none of these depend on the link.
- **Logging and replay from day one.** Every flight produces a rosbag with full sensor streams. The rosbag is the regression test, the training corpus, and the debugging tool.

### Indoor Platform (250mm class)

Purpose:
- Indoor autonomous patrol with VIO-based localization
- Repeatable checkpoint inspection and image capture
- AprilTag-aided relocalization for drift correction
- Embodied AI experimentation in a controlled environment

Why not Crazyflie: the Crazyflie + Flow Deck + Multi-ranger combination is optical-flow position hold plus 5-direction rangefinding. It does not produce a metric map or a pose estimate good enough for repeatable checkpoint patrol, and the AI Deck's GAP8 (~22 GOPS) is roughly 1,800× less compute than a Jetson Orin Nano (40 TOPS). For "learn to fly indoors and crash cheaply" it is excellent; for "real indoor patrol that captures useful imagery and feeds an anomaly detector" it is the wrong tool. Choosing a 250mm Pixhawk platform also keeps the flight stack identical to outdoor.

### Outdoor Platform (Holybro X500 v2)

Purpose:
- Perimeter patrol on private property
- **Forest and trail navigation** under tree canopy where GPS is unreliable
- Larger payload for richer sensing (depth camera, future LiDAR)
- Mature PX4 reference platform for autopilot development

---

## Forest and Trail Navigation: Architectural Implications

Tree canopy makes GPS unreliable due to multipath, attenuation, and intermittent fix loss. This isn't a special case to bolt on later — it shapes the outdoor architecture.

**The unified answer is VIO everywhere, GPS when available.** PX4's EKF2 can fuse visual-inertial odometry with GPS, falling back to VIO-only when GPS quality degrades. This means:

1. The outdoor platform must carry the same VIO sensor suite as the indoor platform (RealSense D456 + IMU).
2. ORB-SLAM3 or Isaac ROS Visual SLAM publishes pose to PX4 via `vehicle_visual_odometry` regardless of GPS state.
3. Forest patrol routes use waypoints in a VIO-relative frame anchored by occasional landmarks (AprilTags at trailheads, GPS fixes in clearings).
4. Trail-following is a separate perception problem on top of localization — segment the trail visually, generate velocity commands toward the trail centerline. This is well-studied (IDSIA Forest Trail Dataset, more recent work on learned trail policies).

**Optional but worth considering for forest: a small 3D LiDAR.** The Livox Mid-360 (~$900) or a Unitree L1-class unit gives 360° obstacle awareness regardless of lighting and fills in the gaps where stereo struggles (low light under canopy, low-texture surfaces, motion blur). It's a meaningful payload and power addition; defer to Phase 5 once VIO-based forest flight is working, then evaluate whether stereo alone is enough.

**RTK GPS (~$300–500 for a base + rover)** gives centimeter-level accuracy when sky is visible. Worth adding for the open-field portions of patrol routes — it makes route repeatability much better at the cost of a base station setup.

---

## Hardware Architecture

### Indoor Platform — 250mm Pixhawk Quad

| Component | Choice | Notes |
|---|---|---|
| Frame | 250mm class (e.g., Holybro QAV250 or similar) | Prop guards mandatory |
| Flight controller | Pixhawk 6C Mini | PX4 reference, smaller form factor |
| Companion computer | Jetson Orin Nano 8GB | 40 TOPS, low weight, sufficient for VIO + lightweight detection |
| Primary sensor | Intel RealSense D456 | Active stereo + IR projector, IMU, well-supported by ORB-SLAM3 |
| Optical flow | PMW3901 + TFMini-S laser rangefinder | Fallback when VIO struggles; low-latency hover |
| Telemetry | WiFi (802.11ac) | Indoor environment, low latency |
| Power | 4S 1300mAh LiPo | ~10–12 min flight with payload |

### Outdoor Platform — Holybro X500 v2

| Component | Choice | Notes |
|---|---|---|
| Frame | Holybro X500 v2 (or S500 v2 / X650 — see availability note) | 500mm, ~1.7kg AUW with sensors |
| Flight controller + companion | **Pixhawk 6X on Holybro Pixhawk Jetson Baseboard** | Integrated FMU + Jetson carrier, see rationale below |
| Companion computer | Jetson Orin NX 16GB | ~100 TOPS, supports onboard YOLO + VIO concurrently |
| Onboard storage | M.2 NVMe SSD (via baseboard) | Required for high-rate rosbag capture |
| Primary sensor | Intel RealSense D456 | Same as indoor — pipeline portability |
| GPS | u-blox M9N (kit) + optional RTK base/rover | RTK for centimeter accuracy in clear sky |
| Optional 3D sensor | Livox Mid-360 LiDAR | Phase 5+; for forest obstacle awareness |
| Telemetry | SiK radio (control) + WiFi or 4G (data) | Control link must be independent of WiFi |
| Power | 4S 5000mAh LiPo | ~15–18 min flight; baseboard BEC rated 3S–4S |

### Frame availability note

The X500 v2 is frequently out of stock at Holybro direct but often available at distributors (Flying Tech, DrUAV, GetFPV, Pixhawk.Store, Amazon). If unavailable, the **S500 v2 Development Kit** is the no-think substitute — same Pixhawk/GPS/telemetry ecosystem, nylon-carbon arms instead of full carbon tube, ~90 min assembly vs ~30. The **X650** is the upgrade worth considering for this project specifically: foldable arms (better for trailhead transport), larger payload margin (future LiDAR), and longer battery options. All three use the same flight stack, so the plan is unchanged regardless of which you pick.

### Why RealSense D456 over OAK-D Lite

OAK-D Lite is cheaper and has on-camera VPU inference, which is appealing. But for a VIO-driven architecture, depth quality matters more than on-camera compute (your inference lives on the Jetson, not the camera). The D456 has active stereo with an IR projector — substantially better depth in low-texture environments, which matters indoors and under canopy. Picking one camera that works across both platforms also simplifies the pipeline.

### Why the Pixhawk 6X + Jetson Baseboard (not discrete boards) outdoor

Two reasons specific to this architecture, both on the critical path:

**Ethernet between FC and Jetson.** The baseboard connects the Pixhawk and Jetson over an onboard UART/CAN/Ethernet switch. uXRCE-DDS over Ethernet is what makes high-rate VIO pose feedback into EKF2 reliable — getting clean Ethernet between two separate boards on a vibrating airframe is exactly the integration work that fails intermittently six months in. Here it's a PCB trace. A known reference build (see Reference Implementations) hit a noisy-data-link problem talking to the FC over GPIO and had to fall back to a USB-UART adapter; the baseboard is engineered to avoid that.

**NVMe slot for logging.** The baseboard's M.2 NVMe slot solves the rosbag storage-bandwidth problem directly. High-rate sensor capture on a USB SSD is a known pain point; NVMe makes it a non-issue. Logging-from-day-one (a Phase 1 principle) needs this.

The cost is ~$200–300 over discrete 6C + Orin, plus fixed mechanical layout and Jetson-form-factor lock-in. On the X500/X650 platform plate those costs don't bite. The 6X also brings triple-IMU redundancy, which matters when flying over your own property under canopy.

**Indoor stays discrete:** Pixhawk 6C Mini + Orin Nano connected by short ribbon/USB-UART. The baseboard is too large and assumes an NX/Nano-class load that's heavy for a 250mm frame; indoor is VIO-only at lower fusion rates, so the Ethernet argument is weaker. Expect to need the USB-UART adapter on this build (see Reference Implementations).

---

## Software Stack

### Core flight + middleware

| Component | Purpose |
|---|---|
| PX4 Autopilot | Flight control, EKF2 sensor fusion, failsafes |
| uXRCE-DDS | Native ROS 2 ↔ PX4 bridge (replaces MAVROS) |
| ROS 2 Jazzy | Robotics middleware, all perception nodes |
| MAVSDK-Python | High-level mission orchestration |
| QGroundControl | Mission planning, parameter tuning, log analysis |

### Perception

| Component | Purpose |
|---|---|
| ORB-SLAM3 *or* Isaac ROS Visual SLAM | VIO/SLAM, publishes to `vehicle_visual_odometry` |
| RealSense ROS 2 driver | Depth + RGB + IMU streams |
| AprilTag ROS 2 (`apriltag_ros`) | Fiducial relocalization for drift correction |
| YOLOv8/v11 + TensorRT (INT8) | Object detection, checkpoint classification |
| Custom anomaly detector (autoencoder/diffusion-based) | Trained on DGX, deployed quantized to Jetson |
| Trail segmentation model (Phase 5) | Trail-following for forest navigation |

### Simulation

| Component | Purpose |
|---|---|
| PX4 SITL + Gazebo Harmonic | Flight dynamics, control development, integration testing |
| Isaac Sim (later) | High-fidelity rendering, synthetic data generation, RL training environments |

Start with Gazebo. Add Isaac Sim only when you need photorealistic rendering for training perception models or running large-scale RL.

### Logging and replay (foundational, not optional)

| Component | Purpose |
|---|---|
| `rosbag2` (mcap format) | Full sensor capture per flight |
| PX4 ULog | Flight controller telemetry |
| Custom telemetry pipeline → DGX | Automated upload, indexing, and replay |
| Foxglove Studio | Visualization of bags, sensor streams, flight paths |

Build this in Phase 1. Every flight gets logged. Every regression gets a replayable bag. This is the single highest-leverage piece of infrastructure in the entire project.

---

## Distro and OS decision

**Settled on Ubuntu 24.04 + ROS 2 Jazzy + JetPack 7.2.** This is a deliberate move off the earlier Humble + 22.04 + JetPack 6.x stack. The reasoning is worth capturing explicitly because the call is not obvious — PX4's official docs as of mid-2026 still recommend Humble + 22.04, and the most prominent reference implementations (including Bernas) target the older stack.

**Why we moved:**

- **Humble EOLs May 2027.** This project's Phase 5–8 work runs well past that. Starting on Humble means scheduling a forced migration mid-project, with the worst possible timing — after we have working VIO, trained models, and accumulated parameter tuning that has to come along.
- **Isaac ROS moved.** As of Isaac ROS 4.0 (current), NVIDIA's recommended platform is ROS 2 Jazzy on Ubuntu 24.04. Since Isaac ROS is the gating dependency for Phases 3–5 (Isaac ROS Visual SLAM, perception acceleration, NITROS), aligning with what NVIDIA actively recommends matters more than aligning with what PX4 docs haven't been updated to recommend yet.
- **JetPack 7.2 brought the Orin modules forward.** JetPack 7.2 (released June 2026) finally brought Jetson Orin NX/AGX to Ubuntu 24.04 with Linux 6.8 kernel and real-time support, putting them on the same software generation as Jetson Thor. JetPack 6.x is now the legacy track.
- **Migration cost is lowest at Phase 1.** Pure simulation, no hardware, no field data, no parameter tuning to preserve. Every later phase makes the migration more expensive.

**What we accept as the cost:**

- **PX4 doesn't officially support this combination yet.** PX4's main-branch docs still recommend Humble + 22.04. The community has demonstrated PX4 + Jazzy + Gazebo Harmonic + 24.04 working end-to-end (working community guides exist), but we should expect ~1 week of integration friction in Phase 1 that wouldn't exist on Humble.
- **The Bernas reference can't be forked directly.** Their stack is JetPack 6.2 + Humble + Isaac ROS 3.2. We adopt their architectural pattern, EKF2 parameter set, and IMU calibration approach — but we rebuild on Jazzy/24.04/Isaac ROS 4.x. This is more work than a clean fork but less than greenfield.
- **The Lyrical Luth question.** ROS 2 Lyrical Luth was released May 2026 (LTS through 2031). We're explicitly *not* using it — too new, ecosystem hasn't caught up, Isaac ROS doesn't target it. Jazzy is the right "modern but supported" choice; Lyrical Luth is a future migration we'll evaluate when the ecosystem matures.

**The hedge we considered and rejected:** stay on Humble through Phase 4 (matching the Bernas reference), migrate to Jazzy before Phase 5. Defensible but adds a planned migration to a phase that already introduces forest navigation and synthetic data — two new hard problems at once. Cleaner to absorb the Jazzy friction in Phase 1 while there's nothing else competing for attention.

This decision should be the project's first ADR. Concrete artifact for whoever picks up the codebase later.

---

## Reference Implementations

### GPS-Denied Drone with NVIDIA Jetson Orin Nano (Bernas)

A complete, working, MIT-licensed build of the hardest single piece of this project — Isaac ROS VSLAM on a Jetson Orin Nano fused into PX4's EKF2, flying autonomous patterns with no GPS and no motion capture in the control loop. This is the recommended architectural reference for **Phase 4** and de-risks the localization substrate the entire indoor system rests on.

- Project: https://www.hackster.io/bandofpv/gps-denied-drone-with-nvidia-jetson-orin-nano-9f3417
- Repo: `bandofpv/VSLAM-UAV` (MIT)
- Stack: Jetson Orin Nano (JetPack 6.2), Intel D435i, PX4 v1.15.4, Isaac ROS VSLAM (release-3.2), ROS 2 Humble, Docker
- **Our adaptation:** rebuild on JetPack 7.2 + ROS 2 Jazzy + Isaac ROS 4.x. Architecture and parameter values transfer; specific package versions and container definitions do not.

**Adopt directly:**

- **The EKF2 vision-fusion parameter set.** Getting PX4 to trust vision and ignore GPS is fiddly and under-documented; this build publishes the exact working values (see "EKF2 Vision Parameters" below). Days of trial-and-error saved.
- **The IMU rigor.** They calibrate the RealSense IMU, then run Allan Variance parameter estimation on 3+ hours of static IMU data and feed the noise/bias values back into the VSLAM pipeline. This is the difference between manageable drift and a VIO that wanders. Don't skip it for a repeatable-checkpoint use case.
- **SSD-for-rosbags.** They confirm an SSD is required on the Jetson for container images and rosbag storage — the same conclusion behind our NVMe-on-baseboard call.
- **The containerized toolchain pattern.** Docker-based isolation with NVIDIA Container Runtime is the right shape; we just rebuild on JetPack 7.2 + Jazzy + Isaac ROS 4.x rather than forking their 6.2/Humble/3.2 image.

**Three deltas to apply:**

1. **MAVROS → uXRCE-DDS.** This build uses MAVROS and explicitly disables the DDS bridge (`UXRCE_DDS_CFG = 0`), talking to PX4 over MAVLink. MAVROS is the legacy translation layer; we chose native uXRCE-DDS. Use their MAVROS path as the fast route to first flight, then migrate. Deliberate decision, not inherited.
2. **Add AprilTag drift correction.** This build is pure VIO with no loop closure or relocalization — fine for short pattern demos, insufficient for repeatable multi-checkpoint patrol over a 10-minute flight. AprilTag relocalization at known positions is our Phase 4 answer and the most important gap this reference leaves open.
3. **Outdoor: GPS+VIO fused, not GPS-off.** Their config disables GPS entirely (`EKF2_GPS_CTRL = 0`). For the outdoor/forest platform we want EKF2 fusing VIO *and* GPS, falling back to VIO-only when GPS quality degrades under canopy — not GPS permanently off. The indoor build inherits their GPS-off config as-is.

**Also note — the GPIO noise problem.** They found the Orin Nano struggled to talk to the FC over GPIO ("noisy data link") and fell back to a USB-UART adapter. This is direct evidence for the Jetson baseboard on the outdoor platform (engineered Ethernet/UART switch avoids it) and a flag that the discrete indoor build will want the USB-UART adapter from the start.

**What it does NOT cover (additive work, by phase):**

- No patrol mission logic — flies predefined shapes (square, circle, figure8, spiral) in OFFBOARD mode, not waypoint missions with checkpoint hover-and-capture (our Phases 3–4).
- No perception payload — camera is a VIO sensor only; no detection, image capture, or anomaly pipeline (our Phases 3, 6).
- Indoor-only, F450 frame, older D435i camera — no outdoor GPS+VIO fusion, no forest validation (our Phases 3, 5).

### EKF2 Vision Parameters (from the Bernas reference, for GPS-denied/VIO-only)

These are the indoor (Phase 4) starting values. For outdoor (Phase 3/5), re-enable GPS control rather than forcing vision-only height/position.

```text
# Companion link (their MAVROS path; we migrate to uXRCE-DDS)
MAV_1_CONFIG    = TELEM2
UXRCE_DDS_CFG   = 0          # disabled in their MAVROS setup; we set to TELEM port for DDS
SER_TEL2_BAUD   = 921600

# Vision fusion, GPS-denied
EKF2_HGT_REF    = Vision
EKF2_EV_DELAY   = 50.0 ms
EKF2_GPS_CTRL   = 0          # outdoor: re-enable for GPS+VIO fusion
EKF2_BARO_CTRL  = Disabled
EKF2_RNG_CTRL   = Disable range fusion
EKF2_REQ_NSATS  = 5
MAV_USEHILGPS   = Enabled
EKF2_MAG_TYPE   = None
```

---

## Project Phases

The phase order is reversed from the original plan: **outdoor before indoor.** With a Pixhawk-based indoor platform, indoor flight is *harder* than outdoor (VIO is harder than GPS, obstacles are closer, failure cost is higher). Validate the toolchain in the easier environment first, then bring the working stack inside.

### Phase 1 — Simulation Foundation and Logging Pipeline (Weeks 1–3)

**Goals:** end-to-end software stack working in simulation, replay infrastructure online before any real hardware.

- PX4 SITL + Gazebo Harmonic + ROS 2 Jazzy installed and integrated
- Simulated X500 flying a multi-waypoint mission via MAVSDK-Python
- `rosbag2` capture and replay working, with telemetry uploading to DGX
- Foxglove Studio configured for visualization
- Basic ROS 2 mission node skeleton (waypoint navigation, image capture stub, return-to-home)

**Exit criterion:** simulated drone completes a 5-waypoint patrol with image capture at each checkpoint, full bag captured, replayable on DGX, visualizable in Foxglove.

### Phase 2 — Outdoor First Flights (Weeks 4–5)

**Goals:** validate hardware build and basic autonomy in the easiest real environment (open field, GPS available, no perception stack yet).

- Assemble X500 v2 + Pixhawk 6X + GPS, no Jetson yet
- Manual flight, then GPS-waypoint autonomous missions
- Geofence and failsafe configuration
- Remote ID compliance setup
- ULog and rosbag capture from real flights, replayed on DGX

**Exit criterion:** outdoor drone autonomously flies a GPS-waypoint perimeter loop and returns home, with full telemetry captured and reviewed.

### Phase 3 — Onboard Perception, Outdoor (Weeks 6–10)

**Goals:** add the Jetson, perception stack, and image capture. Validate VIO outdoors before relying on it indoors.

- Mount Jetson Orin NX + RealSense D456 on X500
- ORB-SLAM3 / Isaac VSLAM publishing pose; verify EKF2 fusion of VIO + GPS
- Obstacle avoidance via depth-based costmap
- Checkpoint image capture and onboard storage
- YOLOv8 INT8 detection running concurrently with VIO at acceptable framerate (>15 FPS)

**Exit criterion:** outdoor patrol with image capture at checkpoints, obstacle avoidance active, VIO pose visibly tracking GPS within tolerance, all streams captured to bag.

### Phase 4 — Indoor VIO Patrol (Weeks 11–16)

**Goals:** the hardest core capability — repeatable indoor patrol using the same software stack, just in a smaller frame and without GPS.

- Assemble 250mm indoor platform
- Test VIO performance in a controlled space (garage or warehouse) before tighter environments
- Place AprilTags at known positions along the patrol route for relocalization
- Tune EKF2 for VIO-only operation
- Port patrol mission logic from outdoor; only the localization source differs

**Exit criterion:** indoor drone completes a repeatable patrol route across 3+ checkpoints with <0.5m position accuracy at each checkpoint, AprilTag relocalization correcting drift, full bag captured.

**Reference implementation:** Andrew Bernas' "GPS-Denied Drone with NVIDIA Jetson Orin Nano" (Hackster, MIT-licensed, repo `bandofpv/VSLAM-UAV`) is a near-exact match for this phase and the recommended starting point. See the Reference Implementations section for what to adopt and the three deltas to apply.

### Phase 5 — Forest and Trail Navigation (Weeks 17–22)

**Goals:** extend outdoor capability to GPS-degraded environments under canopy.

- Validate VIO performance under tree canopy (lighting variation, motion blur, low-texture ground)
- Trail segmentation model trained on forest trail datasets (IDSIA + your own captured data)
- Trail-following behavior: blend trail-centerline visual servoing with waypoint navigation
- Optional: integrate Livox Mid-360 LiDAR if stereo proves insufficient
- AprilTag landmarks at trailheads and key decision points

**Exit criterion:** drone follows a marked trail loop on your property under canopy, returns to launch, captures imagery throughout. VIO holds without catastrophic drift; LiDAR decision made on data.

### Phase 6 — Anomaly Detection (Weeks 23–28)

**Goals:** turn captured patrol imagery into actionable detections.

- Baseline anomaly detection: SSIM or learned perceptual diff against reference images per checkpoint
- Train reconstruction-based anomaly detector (autoencoder or diffusion model) on DGX using collected patrol data
- Deploy quantized detector to Jetson for real-time flagging
- End-to-end alerting: anomaly detected → image + metadata + pose → DGX → notification

**Exit criterion:** system flags meaningful changes during patrol (new object, opened door, vehicle present, debris on trail) with acceptable false-positive rate.

### Phase 7 — Operational Concept (Weeks 29+, ongoing)

**Goals:** turn the prototype into something that runs on a schedule.

- Automated landing pad with charging contacts, *or* multi-battery operational workflow
- Mission scheduler (cron or richer) on DGX dispatching patrol routes
- Pre/post-flight automated checks (battery health, sensor calibration)
- Long-term log retention and analytics

This phase is unglamorous and easy to defer — but a "patrol system" that requires manual battery swaps every 15 minutes is a demo, not a system.

### Phase 8 — Learned Navigation and World Models (ongoing research)

**Goals:** with a working classical baseline, layer in the embodied AI work.

- Digital twin of patrol environment in Isaac Sim
- RL or imitation learning policies for adaptive navigation (dynamic obstacles, adaptive trail-following)
- World-model experimentation (Dreamer-class architectures, learned trajectory prediction)
- Sim-to-real transfer evaluated against the classical baseline as the regression test

This phase is open-ended. Phases 1–6 give you a working system; this is where the research-grade work happens, with the classical pipeline as the safety net and evaluation baseline.

---

## Indoor BOM (revised)

| Item | Notes | Approx Cost |
|---|---|---|
| 250mm frame (QAV250 or similar) | With prop guards | $80–120 |
| Pixhawk 6C Mini | PX4 flight controller | $200 |
| Jetson Orin Nano 8GB Dev Kit | Companion computer | $500 |
| RealSense D456 | Active stereo + IMU | $400 |
| PMW3901 optical flow + TFMini-S | Fallback hover | $50 |
| Motors + ESCs (4x) | 2306-class, 30A ESCs | $120 |
| Props (5–6", with guards) | Plus spares | $30 |
| 4S 1300mAh LiPo (3x) | Flight power | $90 |
| LiPo charger + storage bag | Safety | $80 |
| AprilTag printed markers | Fiducials for relocalization | $20 |
| Cabling, mounting, misc | | $50 |
| **Indoor total** | | **~$1,620–1,660** |

---

## Outdoor BOM (revised)

### Core flight system

| Item | Notes | Approx Cost |
|---|---|---|
| Holybro X500 v2 PX4 kit | Frame + Pixhawk 6X + GPS + ESCs + motors | $700 |
| RadioMaster Boxer TX + RX | Control link | $180 |
| SiK telemetry radios (915MHz) | Ground station telemetry | $80 |
| 4S 5000mAh LiPo (3x) | Flight power | $150 |
| LiPo charger + storage bag | Safety | $80 |

### Compliance

| Item | Notes | Approx Cost |
|---|---|---|
| Remote ID module | FAA Part 89 compliance | $40 |
| Landing pad | Operations | $20 |

### Vision and autonomy (committed, not optional)

| Item | Notes | Approx Cost |
|---|---|---|
| Pixhawk Jetson Baseboard (with Pixhawk 6X) | Integrated FMU + Orin carrier, Ethernet/CAN/UART, NVMe slot | $400–500 |
| Jetson Orin NX 16GB module | Onboard inference + VIO | $500 |
| M.2 NVMe SSD (500GB+) | Rosbag + container storage | $60 |
| RealSense D456 | Same camera as indoor platform | $400 |
| External UBEC 12A (optional, for >4S) | BEC redundancy / higher voltage | $30 |
| Mounting hardware, cabling | | $60 |

Note: the baseboard bundles the Pixhawk 6X, so the 6X is no longer a separate line item in the core flight system if you buy the bundle. Adjust the core-system cost down by ~$300 (the standalone 6X) if bundling.

### Optional augmentations

| Item | Notes | Approx Cost |
|---|---|---|
| RTK GPS base + rover | Centimeter accuracy in open sky | $500 |
| Livox Mid-360 LiDAR (Phase 5+) | 3D obstacle awareness under canopy | $900 |

| Configuration | Approx Cost |
|---|---|
| Outdoor base (no Jetson) | ~$1,250 |
| Outdoor with Jetson + perception | ~$2,450 |
| Outdoor with RTK + LiDAR | ~$3,850 |

---

## Compute Architecture

### Onboard (Jetson, both platforms)

Always runs locally, never depends on link state:

- VIO/SLAM (must be <20ms loop time to feed EKF2)
- Obstacle avoidance / local costmap (<50ms)
- Real-time object detection (YOLO INT8)
- AprilTag detection and relocalization
- Mission state machine
- Trail segmentation (outdoor, Phase 5+)
- Anomaly detection inference (Phase 6+, quantized)
- Failsafes, return-to-home, geofence enforcement

Power budget: ~15W (Orin NX) / ~7W (Orin Nano). Both fit comfortably in their respective frames.

### Offboard (DGX cluster)

Runs centrally, tolerant of 100ms+ latency, never in the safety loop:

- Simulation environments (Gazebo, Isaac Sim)
- Model training (perception, anomaly, RL policies)
- Heavy/offline inference (detailed scene understanding, multi-frame anomaly analysis)
- Rosbag ingestion, indexing, replay infrastructure
- Mission planning and scheduling
- Telemetry storage and analytics
- Ground station UI (Foxglove, QGroundControl)

### Communication

- **Indoor:** WiFi 802.11ac, low-latency, reliable
- **Outdoor open field:** WiFi with directional antenna *or* 4G modem
- **Outdoor under canopy / forest:** 4G modem if cell coverage, otherwise store-and-forward (drone returns to WiFi range and uploads)
- **Control link is always independent** of the data link (SiK 915MHz radio for outdoor, dedicated WiFi channel indoors)

---

## Key Risks and Failure Modes

### Technical

| Risk | Mitigation |
|---|---|
| **VIO drift accumulation** (the biggest indoor risk and a real outdoor risk under canopy) | AprilTag relocalization, optical flow fallback, periodic GPS fix when available outdoor |
| **GPS unreliability under canopy** | VIO-first architecture; GPS as augmentation only |
| **Sim-to-real perception gap** | Domain randomization in sim, fine-tune on real data, Isaac Sim for high-fidelity training data |
| **Battery limits patrol scope** | Plan routes around 12–18 min flight time; Phase 7 charging dock |
| **Single-motor failure on quadrotor** (no redundancy) | Prop guards indoor, conservative airspeed (2–3 m/s indoor), consider hexacopter if risk profile requires it |
| **Onboard inference overload** | Profile early, INT8 quantization, separate threads/cores for VIO vs detection |
| **Link latency on offboard inference** | Keep safety-critical inference onboard; offboard for enrichment only |
| **Forest motion blur and low light** | Global-shutter consideration if stereo proves insufficient; Livox LiDAR as Phase 5 backstop |

### Operational

| Risk | Mitigation |
|---|---|
| **FAA compliance** (Remote ID, Part 107 if applicable) | Remote ID module from day one; understand recreational vs commercial rules for your operations |
| **Loss-of-link** | Pre-programmed RTH or land-in-place; control link independent of data link |
| **Tree/wire collisions outdoor** | Conservative altitude floors, depth-based avoidance, learned trail policies in Phase 5 |
| **LiPo fire risk** | Storage bag, balance charger, never charge unattended |
| **Weather** | Defined operational envelope (no rain, wind <15 mph), abort criteria in mission logic |

---

## Strategic Principles

### Do

- **Simulation first, real flight second, every phase.** New behavior gets validated in Gazebo before it touches hardware.
- **Same flight stack everywhere.** PX4 + ROS 2 + Pixhawk + Jetson on both platforms.
- **VIO as the universal localizer.** GPS augments, doesn't lead.
- **Log every flight, replay liberally.** The bag is your training data, your test suite, and your debugger.
- **Keep the safety loop onboard.** No exceptions.
- **Classical system before learned components.** The classical pipeline is the baseline you evaluate learning against.

### Avoid

- Closed consumer drone ecosystems
- Mixed flight stacks (Crazyflie + PX4) that fragment your tooling
- WiFi-only outdoor control
- GPS-only outdoor localization (forest will break it)
- Heavy onboard compute without profiling first
- Complex autonomy before robust failsafes
- BVLOS ambitions initially
- Skipping AprilTag/fiducial strategy and "hoping VIO is enough"
- Treating logging/replay as a Phase 4 deliverable (it's a Phase 1 deliverable)

---

## Recommended First Milestone

A realistic first success target — achievable end of Phase 4:

```text
Outdoor:
- X500 + Jetson + RealSense flying a GPS+VIO fused waypoint loop
- Image capture at 3+ checkpoints
- Obstacle avoidance active
- Full rosbag captured, uploaded to DGX, replayable

Indoor:
- 250mm + Jetson Nano + RealSense flying VIO-only patrol
- 3+ checkpoints with AprilTag relocalization
- <0.5m position accuracy at each checkpoint
- Image capture and onboard storage

Infrastructure:
- Simulation pipeline (Gazebo + PX4 SITL + ROS 2)
- Logging/replay pipeline (rosbag2 + DGX ingestion + Foxglove)
- Telemetry to DGX in flight, post-flight analytics offline
```

Achieving that milestone establishes the full autonomy pipeline, the simulation workflow, the AI integration path, and the safety architecture — without committing to any specific learned component and without locking in forest navigation or anomaly detection until the foundation is solid.

Forest navigation (Phase 5), anomaly detection (Phase 6), operational concept (Phase 7), and learned navigation (Phase 8) all build on this foundation rather than competing with it.
