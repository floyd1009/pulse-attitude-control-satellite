# PULSE ADCS Thesis: Adaptive Magnetic Attitude Control

**Hardware-Validated Algorithms for Autonomous Maneuver Execution**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Status: Active Development](https://img.shields.io/badge/Status-Active%20Development-green.svg)]()

## Overview

This repository contains the complete implementation and validation of adaptive magnetic attitude determination and control algorithms for the PULSE PocketQube satellite. The thesis advances small satellite ADCS from simulation (TRL 3-4) to flight-ready validation (TRL 6-7) through extensive hardware-in-loop testing.

**Author:** Floyd D'Souza  
**Institution:** TU Berlin  
**Supervisor:** Prof. Cem  
**Timeline:** February - October 2026  
**Defense:** October 2, 2026

## Research Objectives

- Develop Extended Kalman Filter for attitude estimation with magnetic field error correction
- Implement adaptive B-dot controller with real-time gain scheduling
- Validate through 50+ test scenarios in Helmholtz cage and thermal vacuum chamber
- Characterize commercial ADCS sensor performance across operational thermal range
- Demonstrate algorithm transferability to reusable rocket GNC applications

## Repository Structure
```
pulse-adcs-thesis/
├── simulation/           # Algorithm development and validation
│   ├── matlab/          # MATLAB/Simulink models
│   ├── python/          # Python analysis scripts
│   └── results/         # Simulation outputs
├── firmware/            # Embedded implementation
│   ├── stm32/          # STM32 flight code
│   └── tests/          # Unit tests
├── hardware/            # Hardware integration
│   ├── testbed/        # HIL setup documentation
│   └── calibration/    # Sensor calibration procedures
├── data/                # Experimental results
│   ├── helmholtz/      # Helmholtz cage test data
│   ├── thermal/        # Utilities
│   ├── data-processing/
│   └── visualization/
└── docs/                # Documentation
    ├── thesis/         # LaTeX thesis source
    ├── presentations/  # Meeting slides
    └── datasheets/     # Hardware specifications
```

## Development Phases

### Phase 1: Remote Algorithm Development (Feb-May 2026)
- Extended Kalman Filter implementation
- Adaptive B-dot controller
- Software-in-loop validation
- Monte Carlo simulation campaign

### Phase 2: Hardware Validation (May-Aug 2026)
- Helmholtz cage integration and testing
- Thermal vacuum chamber characterization
- Performance metrics compilation
- Results analysis and documentation

## Key Technologies

- **Control Algorithms:** EKF, Adaptive B-dot, PID
- **Hardware:** STM32F4, BNO055 IMU, Custom magnetorquers
- **Testing:** 3-axis Helmholtz cage, Thermal vacuum chamber
- **Languages:** MATLAB/Simulink, C/C++, Python
- **Version Control:** Git with GitFlow methodology

## License

MIT License -.

## Contact

**Floyd D'Souza**  
GitHub: [@floyd1009](https://github.com/floyd1009)

---

**Status:** Active development (Feb-Oct 2026)
