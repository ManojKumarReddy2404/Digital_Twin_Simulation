%% ===============================
%  Digital Twin – Joint Parameters
% ===============================

%% Mechanical Parameters
J = 0.01;        % kg.m^2 (joint inertia)
b = 0.1;         % N.m.s/rad (viscous damping)
tau_c = 0.05;    % N.m (Coulomb friction)
tau_max = 5;     % N.m (maximum motor torque)

%% Simulation Parameters
T_sim = 5;       % total simulation time (s)
Ts = 0.001;      % sampling time (s)

theta_0 = 0;     % initial position (rad)
omega_0 = 0;     % initial velocity (rad/s)

%% Sensor Parameters
encoder_resolution = 4096;   % counts per revolution
vel_noise_std = 0.01;        % rad/s (velocity sensor noise)
torque_noise_std = 0.02;     % N.m (torque sensor noise)
