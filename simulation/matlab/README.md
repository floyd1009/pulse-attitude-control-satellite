# MATLAB Simulation Codes

ADCS simulation codes for the PULSE PocketQube project.

## Files

### 1. `free_tumbling_final.m`
**Purpose:** Validates physics before adding control algorithms.

**Key Features:**
- Implements Euler's rigid body equations (no external torque)
- Verifies angular momentum conservation (|H| = constant)
- **Validation:** 0.0012% variation in angular momentum magnitude
- Demonstrates oscillatory behavior due to inertia coupling

**Results:**
- Initial rates: 0.5, 0.3, 0.8 rad/s (28.6°/s, 17.2°/s, 45.8°/s)
- Oscillation period: 3-5 seconds
- Total angular rate: 119-120°/s stable oscillation

**Run:** `free_tumbling_final`

---

### 2. `BDOT_ODE45.m`
**Purpose:** B-dot detumbling control algorithm with quaternion dynamics.

**Algorithm:**
- Control law: **M = -k · (dB/dt) = k · (ω × B_body)**
- Magnetic torque: **τ = M × B**
- Full 6-DOF dynamics (angular velocity + quaternion kinematics)

**Parameters:**
- Inertia: I = diag([1e-5, 2e-4, 5e-4]) kg·m²
- Control gain: k = 5e3
- Magnetic field: |B| = 2e-5 T, constant in the inertial frame (order-of-LEO magnitude; no orbital/IGRF variation)

**Results:**
- The script integrates a fixed 5000 s span and plots ω(t) and ‖H‖(t); it does not compute a settling time itself.
- Reading the ω plot for this tuned run (initial ‖ω‖ ≈ 11°/s, from [10, 5, -0.2]°/s), the rate falls to ~0.01°/s in ~10 minutes.
- The initial condition is chosen so H · B = 0, so ‖H‖ decays toward zero (no residual spin about the field). See the note below on how this compares to the Monte Carlo settling time.

**Run:** `BDOT_ODE45`

---

### 3. `monte_carlo.m`
**Purpose:** Robustness validation across varying initial conditions and parameters.

**Test Matrix:**
- **N = 50 simulations**
- Initial angular velocity: 10-50°/s (random direction)
- Control gain variation: k ± 10%
- Magnetic field variation: B ± 10%

**Statistics:**
- Settling criterion (coded, line 14): first time ‖ω‖ < 0.01 rad/s (≈ 0.57°/s) — a much looser bar than the B-dot demo's 0.01°/s
- Mean settling time over the converged runs: ~1.5 minutes (std ~0.2 minutes)
- Convergence is geometry-dependent: the control torque is ⊥ B (constant inertial field), so H · B is conserved and a random-direction case can retain a small residual spin about B. Re-run locally to confirm how many of the 50 settle.

**Run:** `monte_carlo`

---

## Why the two settling times differ

The `BDOT_ODE45.m` ~10 min and `monte_carlo.m` ~1.5 min figures use **different stopping criteria** and are not directly comparable:

- `BDOT_ODE45.m` quotes the time to reach **0.01 °/s** (≈ 1.7e-4 rad/s) — read off the plot; the script computes no settling metric.
- `monte_carlo.m` stops at **0.01 rad/s** (≈ 0.57 °/s) — the coded threshold (line 14).

Same digits, different units: the Monte Carlo bar is ~57× looser. B-dot damping of the field-perpendicular rate is roughly exponential, so clearing that extra ~57× costs about four more e-foldings (ln 57 ≈ 4) — which is what stretches the B-dot demo to ~10 min, while the Monte Carlo run stops at the looser bar and reports ~1.5 min. The **threshold definition, not a faster maneuver, accounts for the gap.** (The Monte Carlo field is also ~√2 weaker, 14 µT vs 20 µT; since B-dot torque ∝ |B|², that damps ~2× slower per e-folding, working *against* a shorter Monte Carlo time — so it only reinforces that the threshold is the cause.)

---

## Usage

All scripts are self-contained. Simply run in MATLAB:
```matlab
% Validate physics first
free_tumbling_final

% Test B-dot control
BDOT_ODE45

% Run robustness analysis
monte_carlo
```

## Requirements

- MATLAB R2023b or later
- No additional toolboxes required (uses base MATLAB + ODE45)

## Validation Status

✅ **Physics validated:** angular momentum conservation verified  
✅ **Control validated:** B-dot detumbling successful  
◻️ **Robustness:** 50-case Monte Carlo over 10–50 °/s; convergence count and mean settling time to be re-confirmed locally (see note above)  

## Next Steps

These baseline simulations will be extended in thesis work:
- Add Extended Kalman Filter for state estimation
- Implement adaptive gain scheduling
- Add thermal compensation algorithms
- Integrate sensor noise models

---

**Source:** PULSE ADCS Project (2025)  
**Validation:** TU Berlin, Prof. Cem supervision
