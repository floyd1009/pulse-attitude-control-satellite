# PULSE ADCS: Adaptive Magnetic Attitude Control

**Hardware-Validated Algorithms for Autonomous Maneuver Execution**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Status: Active Development](https://img.shields.io/badge/Status-Active%20Development-green.svg)]()

## Overview

This repository contains the complete implementation and validation of adaptive magnetic attitude determination and control algorithms for the PULSE PocketQube satellite. The thesis advances small satellite ADCS from simulation (TRL 3-4) to flight-ready validation (TRL 6-7) through extensive hardware-in-loop testing.



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



## License

MIT License -.

## Contact

**Floyd D'Souza**  
GitHub: [@floyd1009](https://github.com/floyd1009)

---

**Status:** Active development (Feb-Oct 2026)
