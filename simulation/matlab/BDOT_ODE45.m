% MATLAB Simulation for B-Dot Attitude Control Algorithm
% 
% This script simulates the detumbling of a satellite using the B-Dot control law.
% The dynamics include Euler's rotational equations and quaternion kinematics.
% A constant magnetic field is assumed in the inertial frame for simplicity.
% The B-Dot law is implemented as M = -k * (dB/dt), where dB/dt = -omega × B_body.
% Thus, M = k * (omega × B_body) to ensure damping.
%
% Parameters are tuned for a PocketQube: more anisotropic I for stronger oscillations,
% reduced k for slower damping with more cycles, initial H · B = 0 for full decay to zero.

%% Parameters
I = diag([1e-5, 2e-4, 5e-4]);  % More anisotropic moment of inertia [kg m^2] for oscillations
invI = inv(I);                 % Inverse inertia
k = 5e3;                       % Reduced control gain for slower damping, more oscillation cycles
B_inertial = 2e-5 * [1; 0; 1] / sqrt(2);  % Constant inertial magnetic field [T]

% Initial condins (adjusted so that H · B = 0 for full damping to zero)
% omega_z = - (I_x / I_z) * omega_x , with omega_x=10 deg/s, omega_y=5 deg/s
omega_x_deg = 10;
omega_y_deg = 5;
omega_z_deg = - (1e-5 / 5e-4) * omega_x_deg;  % = -0.2 deg/s
omega0 = deg2rad([omega_x_deg; omega_y_deg; omega_z_deg]);  % Initial angular velocity [rad/s]
q0 = [1; 0; 0; 0];               % Initial quaternion (scalar first)
state0 = [omega0; q0];           % Initial state vector [omega; q]

% Simulation time
tspan = [0 5000];                % Time span [s]

%% ODE Solver
% Use ode45 as the system is not stiff; efficient for this smooth dynamics.
[t, states] = ode45(@(t, state) odefun(t, state, I, invI, k, B_inertial), tspan, state0);

%% Post-processing
omega = states(:, 1:3);          % Angular velocity components
q = states(:, 4:7);              % Quaternions (not used further here)

% Compute norm of angular momentum H = ||I * omega||
H_norm = zeros(length(t), 1);
for i = 1:length(t)
    H_norm(i) = norm(I * omega(i, :)', 'fro');  % Nor of H vector
end

%% Plots
% Plot 1: Angular velocity components vs. time (in minutes)
figure(1);
plot(t/60, rad2deg(omega(:,1)), 'b-', t/60, rad2deg(omega(:,2)), 'r-', t/60, rad2deg(omega(:,3)), 'g-');
grid on;
xlabel('Time [min]');
ylabel('Angular Velocity [deg/s]');
legend('\omega_x', '\omega_y', '\omega_z');
title('Angular Velocity Components');

% Plot 2: Total angular momentum norm vs. time (in minutes)
figure(2);
plot(t/60, H_norm);
grid on;
xlabel('Time [min]');
ylabel('||H|| [kg m^2 / s]');
title('Total Angular Momentum Norm');

%% ODE Function
function dstate = odefun(~, state, I, invI, k, B_inertial)
    % Extract state
    omega = state(1:3);
    q = state(4:7);
    
    % Normalize quaternion (to prevent drift)
    q = q / norm(q);
    
    % Compute DCM (inertial to body frame)
    dcm = quat_to_dcm(q);
    
    % Magnetic field in body frame
    B_body = dcm * B_inertial;
    
    % dB/dt in body frame (analytical: -omega × B_body)
    dB_dt = -cross(omega, B_body);
    
    % B-Dot control:M = -k * dB/dt
    M = -k * dB_dt;  % This simplifies to k * (omega × B_body)
    
    % Control torque: tau = M × B_body
    tau = cross(M, B_body);
    
    % Disturbance torque (none for simplicity)
    tau_dist = [0; 0; 0];
    
    % Euler's equation: dot_omega = I^{-1} (tau_control + tau_dist - omega × (I omega))
    h = I * omega;  % Angular momentum
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

%% Helper Function: Quaternion to DCM
function dcm = quat_to_dcm(q)
    % Assumes q = [w; x; y; z], normalized
    w = q(1); x = q(2); y = q(3); z = q(4);
    dcm = [w^2 + x^2 - y^2 - z^2, 2*(x*y + w*z), 2*(x*z - w*y);
           2*(x*y - w*z), w^2 - x^2 + y^2 - z^2, 2*(y*z +);
           2*(x*z + w*y), 2*(y*z - w*x), w^2 - x^2 - y^2 + z^2];
end
