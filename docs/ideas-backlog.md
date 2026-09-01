# Ideas Backlog

Raw feature ideas being kicked around, not yet speced or planned. An entry here is not a
commitment — just a place to keep track of things worth revisiting. When one is ready to move
forward, it gets a proper design pass (`docs/superpowers/specs/`) before a plan
(`docs/superpowers/plans/`).

- **Home Assistant integration for `walker_llm_bridge`** — have the robot act as a
  conversational client of an existing Home Assistant instance (not a hub of its own), so
  questions like "who's at the door" get routed as a new intent to Home Assistant's existing
  doorbell/camera integration. Fits the project's existing pattern of composing upstream
  systems (Nav2/slam_toolbox) rather than reimplementing them. Not yet speced.

- **Salvaged bump/cliff sensors from the donor Roombas** — already implied by the README's
  own data-flow diagram ("salvaged bump/cliff sensors → sensor fusion → nav stack"), just not
  yet built. Bump sensors are a straightforward Nav2 costmap contact-obstacle layer. Cliff
  sensors are safety-critical and should likely route through `walker_safety`'s
  hardware/watchdog layer rather than only through the ROS2 nav loop, consistent with this
  project's existing safety-layer invariants. Not yet speced.

- **Door sill / small obstacle traversal** — mostly a mechanical concern, not a software one.
  Salvage the donor Roombas' whole wheel modules (motor + wheel + spring suspension), not just
  bare motors, since that suspension travel is what lets a vacuum climb a low threshold in the
  first place — easy to lose if a motor gets remounted rigidly on a new frame. Verify against
  real door sill heights during hardware bring-up/real-world testing, not simulation
  (`room_map.py`'s floor is flat and won't surface this). Not yet speced.

- **Kinect-based gait analysis / fitness tracking of the walker's user** — well supported by
  the research (Kinect-based gait analysis, fall-risk assessment, and home-based physical
  function assessment for older adults are all established literature, added to the UTAS MCP
  knowledge base). The open design question: the literature assumes a stationary Kinect
  watching the subject walk through its view (a gait-lab setup), while this project's Kinect is
  forward-facing and mounted on the moving robot itself — so it mostly sees the room, not a
  consistently-framed view of the person using the walker. May need a second, differently
  mounted sensor rather than reusing the nav Kinect. Not yet speced.
- **Grip strength via the handle's resistive start/stop sensor** — dual-purpose reuse of a
  sensor the walker needs anyway for motor control, no extra hardware. Precedented by an
  Arduino + FSR strain-gauge fatigue-assessment paper in the same knowledge base. Keep this
  wellness-metric use logically separate from the handle's role as a (convenience-layer, not
  safety-critical) motor control input. Not yet speced.
- **Step count via the existing ESP32 IMU** — likely needs zero new hardware:
  `walker_anomaly_detection`'s already-planned 9-axis IMU is the same class of sensor used in
  the wearable-accelerometer gait-speed literature for older adults. A software/algorithm
  addition to an existing package, not a new sensor. Not yet speced.
- **Step length, derived from odometry ÷ step count** — pure IMU-based step-length estimation
  (double-integrating acceleration) is drift-prone, which is why the wearable-IMU literature
  mostly reports speed/cadence rather than step length. This project can sidestep that: divide
  `walker_motor_driver`'s wheel-odometry distance by the IMU-derived step count over the same
  window. Assumes the person's pace is coupled to the walker's motion (not lagging/leading it,
  or working the frame back and forth while stationary). Not yet speced.
