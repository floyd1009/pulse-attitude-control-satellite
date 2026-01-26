# MATLAB Simulation Codes

Validated ADCS simulation codes from PULSE PocketQube project.

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
- Inertia: I = d 2e-4, 5e-4]) kg·m²
- Control gain: k = 5e3
- Magnetic field: 2e-5 T constant (LEO-representative)

**Results:**
- Detumbling time: ~10 minutes (10°/s → 0.01°/s)
- Angular momentum decays to zero when H · B = 0

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
- Mean settling time: ~1.5 minutes
- Standard deviation: ~0.2 minutes
- All scenarios converge successfully

**Run:** `monte_carlo`

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

✅ **Physics valgular momentum conservation verified  
✅ **Control validated:** B-dot detumbling successful  
✅ **Robustness validated:** 50/50 Monte Carlo scenarios converge  

## Next Steps

These baseline simulations will be extended in thesis work:
- Add Extended Kalman Filter for state estimation
- Implement adaptive gain scheduling
- Add thermal compensation algorithms
- Integrate sensor noise models

---

**Source:** PULSE ADCS Project (2025)  
**Validation:** TU Berlin, Prof. Cem supervision
