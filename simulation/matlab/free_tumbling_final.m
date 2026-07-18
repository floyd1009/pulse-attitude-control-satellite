% Free Tumbling Simulation: Euler's Equations with Conserved Angular Momentum (Refined)
% =========================================================================
% EQUATIONS AND REASONING FOR FREE TUMBLING SIMULATION
% =========================================================================
% 1. Euler's Rigid Body Equations (No External Torque):
%    J * dω/dt + ω × (J * ω) = 0
%    - Where J is the inertia tensor (kg·m²), ω is angular velocity (rad/s)
%    - Solution: dω/dt = J^(-1) * (-ω × (J * ω))
%    - Reason: Describes internal coupling due to inertia differences, causing oscillations
%             in ω_x, ω_y, ω_z without external torque (τ = 0).

% 2. Angular Momentum Conservation:
%    dH/dt = τ_ext = 0 (in inertial frame)
%    - H = J * ω (angular momentum vector)
%    - |H| = constant (magnitude conserved in free tumbling)
%    - Reason: No external torque implies conserved H, validated by numerical precision.

% 3. Coupling Effect:
%    dω_x/dt ∝ (J_yy - J_zz) * ω_y * ω_z / J_xx
%    dω_y/dt ∝ (J_zz - J_xx) * ω_x * ω_z / J_yy
%    dω_z/dt ∝ (J_xx - J_yy) * ω_x * ω_y / J_zz
%    - Reason: Asymmetry in J (J_zz < J_xx = J_yy) drives oscillatory exchange
%             between axes, producing sinusoidal-like motion.

% =========================================================================
% SIMULATION PARAMETERS AND IMPLEMENTATION
% =========================================================================

clear all; close all; clc;

% Parameters (from PULSE project: approximate 1U PocketQube inertia)
J = diag([0.001, 0.001, 0.0005]);  % Inertia tensor (kg·m²), slightly asymmetric

% Initial conditions (high tumbling rates, in rad/s)
omega0 = [0.5; 0.3; 0.8];

% Time span (30 seconds to show oscillations)
tspan = [0 30];

% High precision
options = odeset('RelTol',1e-8, 'AbsTol',1e-10);
[t, omega] = ode45(@(t, omega) free_dynamics(t, omega, J), tspan, omega0, options);

% Calculate angular momentum magnitude
H_mag = zeros(length(t), 1);
for i = 1:length(t)
    H_mag(i) = norm(J * omega(i, :)');
end

% Calculate conservation metric
H_variation_percent = (max(H_mag) - min(H_mag)) / mean(H_mag) * 100;

% Calculate kinetic energy (should be constant)
E_kin = zeros(length(t), 1);
for i = 1:length(t)
    E_kin(i) = 0.5 * omega(i, :) * J * omega(i, :)';
end
E_variation_percent = (max(E_kin) - min(E_kin)) / mean(E_kin) * 100;

% Visualization
figure('Position', [100 100 1200 800]);

% Plot 1: Angular velocities
subplot(2,1,1);
plot(t, omega * (180/pi), 'LineWidth', 2);  % Convert to deg/s
legend('\omega_x', '\omega_y', '\omega_z', 'Location', 'best');
xlabel('Time [seconds]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Angular Rate [deg/s]', 'FontSize', 12, 'FontWeight', 'bold');
title('Free Tumbling: Angular Velocity Components (Oscillations)', ...
      'FontSize', 14, 'FontWeight', 'bold');
grid on;
xlim([0 max(t)]);
ylim([min(min(omega))*180/pi - 0.1, max(max(omega))*180/pi + 0.1]);

% Plot 2: Angular momentum magnitude
subplot(2,1,2);
plot(t, H_mag, 'g-', 'LineWidth', 3);
xlabel('Time [seconds]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Angular Momentum Magnitude |H| [kg·m²/s]', 'FontSize', 12, 'FontWeight', 'bold');
title(sprintf('Angular Momentum Conservation (Variation: %.4f%%)', H_variation_percent), ...
      'FontSize', 14, 'FontWeight', 'bold');
grid on;
xlim([0 max(t)]);
ylim([min(H_mag)-1e-7, max(H_mag)+1e-7]);  % Tighter y-axis

% Console output
fprintf('\n=== FREE TUMBLING RESULTS ===\n');
fprintf('Initial angular rates: [%.2f, %.2f, %.2f] rad/s\n', omega0(1), omega0(2), omega0(3));
fprintf('Final angular rates: [%.2f, %.2f, %.2f] rad/s\n', omega(end,1), omega(end,2), omega(end,3));
fprintf('Mean |H|: %.6f kg·m²/s\n', mean(H_mag));
fprintf('|H| variation: %.4f%%\n', H_variation_percent);
fprintf('Kinetic energy variation: %.4f%%\n', E_variation_percent);
fprintf('No torque applied; variations due to numerical precision only.\n');

%% Local Function
% Dynamics function: Euler's equations for free tumbling (tau = 0)
function dot_omega = free_dynamics(~, omega, J)
    dot_omega = J \ (-cross(omega, J * omega));  % No external torque
end
