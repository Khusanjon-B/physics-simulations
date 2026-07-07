# Double Pendulum — Real-Time Chaotic Simulation on Raspberry Pi

A C++/[raylib](https://www.raylib.com/) double pendulum, built from scratch on a Raspberry Pi 5 driving a DSI touchscreen. This repo captures not just the finished simulation but the **progression of physics models** used to reach it — from a naive uncoupled approximation, through Euler–Cromer integration of the true coupled equations, to a Runge–Kutta 4 (RK4) integrator that conserves energy to a fraction of a percent.

The project was as much about *numerical methods* as graphics: each iteration is kept as a separate file so the effect of the integrator on physical accuracy is visible and measurable.

---

## What it does

- Simulates a double pendulum — two rigid arms, the second hinged off the tip of the first — swinging under gravity.
- Renders in real time at 60 FPS with a fading motion trail on the lower bob.
- Displays a live **energy readout** (kinetic, potential, total) and an **energy-drift percentage** relative to the launch state — used as an empirical measure of integrator quality.
- Runs fullscreen-capable on a Raspberry Pi 5 with a DSI touchscreen; the fixed pivot is draggable via touch (treated as mouse input).

---

## The physics

A double pendulum is a canonical **chaotic system**: the two arms are rigidly coupled, so each arm's motion depends on the other's angle, angular velocity, and angular acceleration simultaneously. This coupling is why the motion is famously sensitive to initial conditions — arbitrarily small differences in starting angle diverge into completely different trajectories.

The equations of motion are derived via **Lagrangian mechanics** (not reproduced here — see any classical mechanics text). This project takes the standard published closed-form angular accelerations and focuses on integrating them correctly.

State is represented by four numbers:

- `theta1`, `omega1` — angle and angular velocity of the upper arm
- `theta2`, `omega2` — angle and angular velocity of the lower arm

Angles in the physics are measured **from vertical** (0 = hanging straight down). The rendering uses a separate **from-horizontal** convention that suits the drawing math, with the ±90° conversion handled at the boundary between physics and display.

### A note on units

The simulation works in **pixel space**, so `g` is a tunable constant in pixels/s² (≈ 800), not the physical 9.8 m/s². Energy magnitudes are correspondingly large and are scaled for display; only *relative* energy change (drift) is physically meaningful, since potential energy's zero point is arbitrary.

---

## The iterations

The heart of the project. Each file is a self-contained stage.

### 1. Uncoupled (`pendulum_uncoupled.cpp`)
Two arms, but each treated as an **independent simple pendulum** — arm 2 swings about arm 1's moving tip using the lone-pendulum equation, ignoring the coupling forces. Structurally correct (chaining, two independent states, drawing) but physically wrong: no energy exchange between the arms, so the motion looks plausible but lifeless, with none of the characteristic chaotic whip. Built first to isolate the *structure* before adding the hard math.

### 2. Euler–Cromer (`pendulum_euler.cpp`)
The **true coupled equations**, integrated with semi-implicit (Euler–Cromer) integration — velocity updated first, then position from the new velocity. Produces genuine chaotic motion. However, the integrator is not energy-conserving: during the violent initial fall it *injects* energy (drift spiking to 200%+), then during calmer phases it *dissipates* energy (drift going negative). Bounded but visibly inaccurate over time.

### 3. RK4 (`pendulum_rk4.cpp`)
The same coupled equations, integrated with **4th-order Runge–Kutta**. The physics is refactored into a pure `derivatives(State) -> State` function that RK4 samples at four points per timestep (start, two midpoints, end) and blends with 1-2-2-1 weighting. Energy drift collapses to a fraction of a percent — the integrator earns its complexity. This is the definitive version.

**Measured result:** switching from Euler–Cromer to RK4 took energy drift from swings of **±200%+** down to **≈0%** over the same run, confirming the accuracy gain empirically rather than by assumption.

---

## Key implementation details

- **Derived geometry, not stored.** Each arm stores `angle` + `length` and computes its endpoint on demand (`end()`), so the bob position can never fall out of sync with the angle. Arm 2's start is re-derived from arm 1's end each frame.
- **State/parameter separation.** The RK4 `derivatives` function reads angles and velocities only from the passed-in `State`, and masses/lengths only from the (constant) arm parameters — the discipline that makes RK4's hypothetical-state sampling valid.
- **Frame-rate independence.** Physics is stepped by `dt = GetFrameTime()`, so motion runs at a consistent real-world speed regardless of frame rate.
- **Fixed-length trail.** The lower bob's recent positions are stored in a capped buffer and drawn as fading line segments (newer = brighter).
- **Throttled UI.** The energy/drift text refreshes a few times per second rather than every frame, decoupling display rate from simulation rate for readability.

---

## Building

Requires raylib (built for the Pi with the OpenGL ES 2.0 backend) and a C++17 compiler. Built with CMake.

```bash
mkdir build && cd build
cmake ..
make
```

Run each iteration from a terminal on the display (raylib needs a real display server):

```bash
./pendulum_rk4
```

---

## Controls

- **Drag the pivot** (touch or mouse) to reposition the anchor while it swings.

---

## Hardware / environment

- Raspberry Pi 5 (8 GB)
- Freenove 5" DSI touchscreen (800×480)
- Raspberry Pi OS (Bookworm, 64-bit), Wayland desktop
- raylib compiled from source with `GRAPHICS_API_OPENGL_ES2`

---

## Possible extensions

- Pivot forcing — drive the anchor's motion to whip the pendulum (accelerating reference frame).
- Live sliders for arm length / mass / gravity.
- Multiple pendulums with near-identical initial conditions, to visualize divergence (chaos).
- Symplectic integrator (Verlet) for guaranteed long-term energy bounds.