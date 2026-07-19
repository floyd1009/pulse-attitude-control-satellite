# PULSE ADCS — B-dot Detumbling Simulation

MATLAB simulation environment for the magnetic attitude control (detumbling) of TU Berlin's **PULSE 3P PocketQube** satellite.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## What's here

All scripts live in [`simulation/matlab/`](simulation/matlab):

| Script | Purpose |
|---|---|
| `free_tumbling_final.m` | Rigid-body tumbling dynamics of the 3P PocketQube |
| `BDOT_ODE45.m` | Closed-loop B-dot detumbling with magnetorquer actuation (ODE45) |
| `monte_carlo.m` | Monte Carlo dispersion analysis over initial tumble rates |

## Context

Developed as part of the ADCS work for the PULSE PocketQube at TU Berlin. This repository contains the simulation side of that work: the tumbling dynamics, the B-dot control law, and the dispersion analysis used to size the detumbling performance.

The flight firmware (C, STM32) and the hardware-in-the-loop test campaign were developed within the PULSE team and live in the team's repository. Results are available on request.

## Requirements

MATLAB R2023b or later.

## License

MIT

## Contact

**Floyd D'Souza** — [@floyd1009](https://github.com/floyd1009)
