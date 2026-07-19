# Results

Figures for the PULSE B-dot detumbling study are **generated locally** from the
MATLAB scripts in [`../matlab/`](../matlab) — they are not committed to the
repository. Run the scripts in MATLAB R2023b+ (base MATLAB + ODE45, no extra
toolboxes) and export each figure at 150 dpi.

| Expected figure | Source script | Content |
|---|---|---|
| `free_tumbling_momentum.png` | `free_tumbling_final.m` | Angular-momentum magnitude vs time (conservation check) |
| `bdot_detumbling_rates.png` | `BDOT_ODE45.m` | Body angular rates vs time under B-dot control |
| `monte_carlo_settling_hist.png` | `monte_carlo.m` | Settling-time distribution (N = 50) |

Example export (per figure, after running its script):

```matlab
BDOT_ODE45
exportgraphics(gcf, 'simulation/results/bdot_detumbling_rates.png', 'Resolution', 150)
```
