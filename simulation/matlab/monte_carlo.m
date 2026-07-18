% Monte Carlo Simulation for B-Dot Control Robustness (Integrated with Full Dynamics)

clear all; close all; clc;

% Parameters (from your B-Dot code)
I = diag([1e-5, 2e-4, 5e-4]); % Anisotropic inertia [kg m^2]
invI = inv(I);
k_mean = 5e3; % Mean control gain
B_mag_mean = 2e-5 / sqrt(2); % Mean B magnitude [T] (adjusted for your B_inertial norm)
tspan = [0 5000]; % Simulation time [s]

N_mc = 50; % Number of runs
settling_times = zeros(N_mc, 1);
settling_threshold = 0.01; % rad/s (~0.57 deg/s)

for i = 1:N_mc
    % Randomize parameters
    k = k_mean * (1 + 0.2*(rand - 0.5)); % +/-10% variation
    B_mag_rand = B_mag_mean * (1 + 0.2*(rand - 0.5)); % +/-10% variation
    B_inertial = B_mag_rand * randn(3,1); % Random direction, norm ~ B_mag_rand (use randn for variety)
    B_inertial = B_inertial / norm(B_inertial) * B_mag_rand; % Normalize to exact magnitude
    
    % Random initial omega: total norm 10-50 deg/s, random direction
    omega_mag_deg = 10 + rand*40; % 10-50 deg/s
    omega_dir = randn(3,1); omega_dir = omega_dir / norm(omega_dir);
    omega0 = deg2rad(omega_mag_deg) * omega_dir; % [rad/s]
    
    q0 = [1; 0; 0; 0]; % Initial quaternion
    state0 = [omega0; q0];
    
    % Run ODE
    [t_sim, states] = ode45(@(t, state) odefun(t, state, I, invI, k, B_inertial), tspan, state0);
    
    omega_sim = states(:, 1:3);
    omega_norm = vecnorm(omega_sim, 2, 2); % ||omega|| at each time
    
    % Find settling time (first time ||omega|| < threshold)
    idx = find(omega_norm < settling_threshold, 1);
    if ~isempty(idx)
        settling_times(i) = t_sim(idx) / 60; % min
    else
        settling_times(i) = NaN;
    end
end

% Statistics (ignore NaN)
valid_times = settling_times(~isnan(settling_times));
min_time = min(valid_times);
max_time = max(valid_times);
mean_time = mean(valid_times);
median_time = median(valid_times);
std_time = std(valid_times);

% Display
disp('=== Monte Carlo Results ===');
disp(['Mean settling time: ' num2str(mean_time, '%.1f') ' min']);
disp(['Std Dev: ' num2str(std_time, '%.1f') ' min']);
disp(['Min time: ' num2str(min_time, '%.1f') ' min']);
disp(['Max time: ' num2str(max_time, '%.1f') ' min']);
disp(['Median time: ' num2str(median_time, '%.1f') ' min']);

% Histogram
figure;
histogram(valid_times, 10);
title('Settling Time Distribution (min)');
xlabel('Time (min)'); ylabel('Frequency');
grid on;

% ODE Function
function dstate = odefun(~, state, I, invI, k, B_inertial)
    % Extract state
    omega = state(1:3);
    q = state(4:7);
    
    % Normalize quaternion (to prevent drift)
    if norm(q) == 0  % Rare edge case safeguard
        q = [1; 0; 0; 0];
    else
        q = q / norm(q);
    end
    
    % Compute DCM (inertial to body frame)
    dcm = quat_to_dcm(q);
    
    % Magnetic field in body frame
    B_body = dcm * B_inertial;
    
    % dB/dt in body frame (analytical: -omega × B_body)
    dB_dt = -cross(omega, B_body);
    
    % B-Dot control: M = -k * dB_dt
    M = -k * dB_dt; % This simplifies to k *(omega × B_body)
    
    % Control torque: tau = M × B_body
    tau = cross(M, B_body);
    
    % Disturbance torque (none for simplicity)
    tau_dist = [0; 0; 0];
    
    % Euler's equation: dot_omega = I^{-1} (tau_control + tau_dist - omega × (I omega))
    h = I * omega; % Angular momentum
    dot_omega = invI * (tau + tau_dist - cross(omega, h));
    
    % Quaternion kinematics: dot_q = 0.5 * [ -qv' omega; w omega + qv × omega ]
    w = q(1);
    qv = q(2:4);
    dot_q_scalar = -0.5 * qv' * omega;
    dot_q_vector = 0.5 * (w * omega + cross(qv, omega));
    dot_q = [dot_q_scalar; dot_q_vector];
    
    % Combined derivative
    dstate = [dot_omega; dot_q];
end

% Helper Function: Quaternion to DCM
function dcm = quat_to_dcm(q)
    % Assumes q = [w; x; y; z], normalized
    w = q(1); x = q(2); y = q(3); z = q(4);
    dcm = [w^2 + x^2 - y^2 - z^2, 2*(x*y + w*z), 2*(x*z - w*y);
           2*(x*y - w*z), w^2 - x^2 + y^2 - z^2, 2*(y*z + w*x);
           2*(x*z + w*y), 2*(y*z - w*x), w^2 - x^2 - y^2 + z^2];
end
