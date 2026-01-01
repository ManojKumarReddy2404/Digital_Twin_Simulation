clc;
clear;
close all;

%% =========================================================
%  SET PROJECT ROOT (VERY IMPORTANT)
% =========================================================
thisFile = mfilename('fullpath');
projectRoot = fileparts(fileparts(thisFile));   % goes up from matlab/
addpath(genpath(projectRoot));

resultsDir = fullfile(projectRoot, 'results');
plotsDir   = fullfile(resultsDir, 'plots');
dataDir    = fullfile(resultsDir, 'data');

if ~exist(plotsDir,'dir'), mkdir(plotsDir); end
if ~exist(dataDir,'dir'), mkdir(dataDir); end

%% =========================================================
%  LOAD PARAMETERS
% =========================================================
run(fullfile(projectRoot,'matlab','params.m'));

%% =========================================================
%  INITIAL CONDITIONS
% =========================================================
x0 = [theta_0; omega_0];

%% =========================================================
%  DYNAMIC SIMULATION
% =========================================================

% Step torque input (50% of max torque)
tau_val = 0.5 * tau_max;

% Joint dynamics ODE
joint_ode = @(t, x) [
    x(2);
    (tau_val - b*x(2) - tau_c*sign(x(2))) / J
];

% Solve ODE
[t_out, x_out] = ode45(joint_ode, [0 T_sim], x0);

theta = x_out(:,1);
omega = x_out(:,2);

%% =========================================================
%  PLOTS – JOINT RESPONSE
% =========================================================

% Joint Angle
figure;
plot(t_out, theta, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Joint Angle \theta (rad)');
title('Joint Angle Response');
grid on;
saveas(gcf, fullfile(plotsDir,'joint_angle_response.png'));

% Joint Velocity
figure;
plot(t_out, omega, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Angular Velocity \omega (rad/s)');
title('Joint Velocity Response');
grid on;
saveas(gcf, fullfile(plotsDir,'joint_velocity_response.png'));

%% =========================================================
%  SENSOR SIMULATION
% =========================================================

% Encoder quantization
counts_per_rad = encoder_resolution / (2*pi);
encoder_counts = round(theta * counts_per_rad);

% Velocity sensor with noise
omega_sensor = omega + vel_noise_std * randn(size(omega));

% Torque sensor with noise
tau_applied = tau_val * ones(size(t_out));
tau_sensor = tau_applied + torque_noise_std * randn(size(t_out));

% Sampling
sample_idx = 1:10:length(t_out);

t_s       = t_out(sample_idx);
encoder_s = encoder_counts(sample_idx);
omega_s   = omega_sensor(sample_idx);
tau_s     = tau_sensor(sample_idx);

%% =========================================================
%  SENSOR PLOTS
% =========================================================

% Encoder
figure;
stairs(t_s, encoder_s, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Encoder Counts');
title('Encoder Output');
grid on;
saveas(gcf, fullfile(plotsDir,'encoder_output.png'));

% Velocity sensor
figure;
plot(t_s, omega_s, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Velocity Sensor Output');
grid on;
saveas(gcf, fullfile(plotsDir,'velocity_sensor_output.png'));

% Torque sensor
figure;
plot(t_s, tau_s, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Torque (N·m)');
title('Torque Sensor Output');
grid on;
saveas(gcf, fullfile(plotsDir,'torque_sensor_output.png'));

%% =========================================================
%  VALIDATION – MULTIPLE TORQUES
% =========================================================

torque_levels = [0.2 0.5 0.8] * tau_max;

figure; hold on;
for k = 1:length(torque_levels)
    tau_k = torque_levels(k);

    joint_ode = @(t, x) [
        x(2);
        (tau_k - b*x(2) - tau_c*sign(x(2))) / J
    ];

    [t_tmp, x_tmp] = ode45(joint_ode, [0 T_sim], x0);
    plot(t_tmp, x_tmp(:,1), 'LineWidth', 1.5);
end

xlabel('Time (s)');
ylabel('Joint Angle \theta (rad)');
title('Joint Response for Different Torques');
legend('20%','50%','80%');
grid on;
saveas(gcf, fullfile(plotsDir,'torque_validation.png'));

%% =========================================================
%  SAVE DATA
% =========================================================

save(fullfile(dataDir,'joint_simulation_data.mat'), ...
     't_out','theta','omega', ...
     'encoder_s','omega_s','tau_s');

disp('✅ Digital Twin Simulation Completed Successfully');
