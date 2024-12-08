%% GRACE CORDLE EML 4312 Project DATE 11/13/2024
% Edited by J Vranicar 
clc
clear all
close all
format compact
%% Params
w_1 = 0.1; % m
h_1 = 0.1; % m
l_1 = 0.43; % m
rho_1 = 1055; % kg/m^3
m_1 = rho_1 * w_1 * h_1 * l_1; % Mass of the pendulum
I_1 = (1/3) * m_1 * l_1^2; % Moment of inertia

w_2 = 0.1; % m
h_2 = 0.1; % m
l_2 = 0.43; % m
rho_2 = 1055; % kg/m^3
m_2 = rho_2 * w_2 * h_2 * l_2; % Mass of the pendulum
I_2 = (1/3) * m_2 * l_2^2; % Moment of inertia
g = 9.81; % Gravitational acceleration (m/s^2)

w_3 = 0.1; % m
h_3 = 0.1; % m
l_3 = 0.43; % m
rho_3 = 1055; % kg/m^3
m_3 = rho_3 * w_3 * h_3 * l_3; % Mass of the pendulum
I_3 = (1/3) * m_3 * l_3^2; % Moment of inertia


% Controller
theta_des_1 = pi;
omega_des_1 = 0;
Kp_1 = 10;
Kd_1 = 1;
% Kp_1 = 0;
% Kd_1 = 0;

theta_des_2 = pi/2;
omega_des_2 = 0;
Kp_2 = 200;
Kd_2 = 15;
% Kp_2 = 0;
% Kd_2 = 0;

theta_des_3 = pi;
omega_des_3 = 0;
Kp_3 = 20;
Kd_3 = 5;
% Kp_3 = 0;
% Kd_3 = 0;

% Control torque function
tau_ctrl_1 = @(theta, omega) Kp_1 * (theta_des_1 - theta) + Kd_1 * (omega_des_1 - omega);
tau_ctrl_2 = @(theta, omega) Kp_2 * (theta_des_2 - theta) + Kd_2 * (omega_des_2 - omega);
tau_ctrl_3 = @(theta, omega) Kp_3 * (theta_des_3 - theta) + Kd_3 * (omega_des_3 - omega);

%% ODE 45 Simulation
% Define the system dynamics
dynamics_1 = @(t, x) [x(2); (tau_ctrl_1(x(1), x(2)) - (m_1 * g * (l_1 / 2) * sin(x(1)))) / I_1];
dynamics_2 = @(t, x) [x(2); (tau_ctrl_2(x(1), x(2)) - (m_2 * g * (l_2 / 2) * sin(x(1)))) / I_2];
dynamics_3 = @(t, x) [x(2); (tau_ctrl_3(x(1), x(2)) - (m_3 * g * (l_3 / 2) * sin(x(1)))) / I_3];

% Initial conditions: theta = pi/4, angular velocity = 0
[tout_1, xout_1] = ode45(dynamics_1, [0 30], [4.5*pi/4, 0]); % Black
[tout_2, xout_2] = ode45(dynamics_2, [0 30], [3*pi/4, 0]); % Blue
[tout_3, xout_3] = ode45(dynamics_3, [0 30], [1.1*pi, 0]); % Green

%% Animate Rectangle Pendulum
% Rectangle representing the pendulum
xvals_1 = w_1 * [-0.5, 0.5, 0.5, -0.5];  % Rectangle corners (x)
yvals_1 = l_1 * [0, 0, -1, -1];          % Rectangle corners (y) from pivot downward

coords_1 = [xvals_1; yvals_1];  % [x; y] initial unrotated rectangle coordinates

xvals_2 = w_2 * [-0.5, 0.5, 0.5, -0.5];  % Rectangle corners (x)
yvals_2 = l_2 * [0, 0, -1, -1];          % Rectangle corners (y) from pivot downward

coords_2 = [xvals_2; yvals_2];  % [x; y] initial unrotated rectangle coordinates

xvals_3 = w_3 * [-0.5, 0.5, 0.5, -0.5];  % Rectangle corners (x)
yvals_3 = l_3 * [0, 0, -1, -1];          % Rectangle corners (y) from pivot downward

coords_3 = [xvals_3; yvals_3];  % [x; y] initial unrotated rectangle coordinates

figure;
hold on  % Keep the figure open for multiple elements

% Ground line
plot([-1, 1], [0, 0], 'k', 'LineWidth', 2);  % Ground line along x-axis

% Brown box
chair_width = 0.35;
chair_height = 0.35;
chair_x_position = 0.1;
chair_y_position = 0;

chair_coords_x = [chair_x_position, chair_x_position + chair_width, chair_x_position + chair_width, chair_x_position];
chair_coords_y = [chair_y_position, chair_y_position, chair_y_position + chair_height, chair_y_position + chair_height];

vid = VideoWriter("ControllerAnimation.avi");
vid.FrameRate = 20;
vid.Quality = 85;
open(vid);

for i = 1:length(xout_1)/2
    theta_1 = xout_1(i, 1); % Extract angle at the current time
    rotMatrix_1 = [cos(theta_1), -sin(theta_1); sin(theta_1), cos(theta_1)];

    theta_2 = xout_2(i, 1); % Extract angle at the current time
    rotMatrix_2 = [cos(theta_2), -sin(theta_2); sin(theta_2), cos(theta_2)];

    theta_3 = xout_3(i, 1); % Extract angle at the current time
    rotMatrix_3 = [cos(theta_3), -sin(theta_3); sin(theta_3), cos(theta_3)];
    
    % Apply rotation about the pivot (base at origin)
    rotCoords_1 = rotMatrix_1 * coords_1;

    % Apply rotation about the pivot (base at origin)
    rotCoords_2 = rotMatrix_2 * coords_2;
    rotCoords_2(1, :) = rotCoords_2(1, :) + mean(rotCoords_1(1, 3:4));
    rotCoords_2(2, :) = rotCoords_2(2, :) + mean(rotCoords_1(2, 3:4));

    % Apply rotation about the pivot (base at origin)
    rotCoords_3 = rotMatrix_3 * coords_3;
    rotCoords_3(1, :) = rotCoords_3(1, :) + mean(rotCoords_2(1, 3:4));
    rotCoords_3(2, :) = rotCoords_3(2, :) + mean(rotCoords_2(2, 3:4));
    
    % Update the pendulum position
    hold on
    h1 = fill(rotCoords_1(1, :), rotCoords_1(2, :), 'k');  % Filled rectangle
    h2 = fill(rotCoords_2(1, :), rotCoords_2(2, :), 'b');  % Filled rectangle
    h3 = fill(rotCoords_3(1, :), rotCoords_3(2, :), 'g');  % Filled rectangle
    h_chair = fill(chair_coords_x, chair_coords_y, [0.5, 0.3, 0.2]);
    axis equal;
    axis([-1 1 -0.5 1.5]); % Adjust for pendulum's range of motion
    xlabel('X (m)');
    ylabel('Y (m)');
    title('Pendulum Animation');

    writeVideo(vid, getframe(gcf));

    pause(0.05);  % Slow down animation for visualization
    if i < length(xout_1)/2 - 1
        delete(h1);  % Remove previous frame
        delete(h2);  % Remove previous frame
        delete(h3);  % Remove previous frame
        delete(h_chair);
    end
end

close(vid)
%% Plot Time Response of Angles
figure;

% Desired angles for comparison
desired_angles_1 = theta_des_1 * ones(size(tout_1));
desired_angles_2 = theta_des_2 * ones(size(tout_2));
desired_angles_3 = theta_des_3 * ones(size(tout_3));

% Plot Link 1
subplot(3, 1, 1);
plot(tout_1, xout_1(:, 1), 'r', 'LineWidth', 1.5); hold on;
plot(tout_1, desired_angles_1, 'k--', 'LineWidth', 1.2);
title('Time Response of Link 1');
axis([0 10 0 4])
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('Link 1 Angle', 'Desired Angle');
grid on;

% Plot Link 2
subplot(3, 1, 2);
plot(tout_2, xout_2(:, 1), 'b', 'LineWidth', 1.5); hold on;
plot(tout_2, desired_angles_2, 'k--', 'LineWidth', 1.2);
title('Time Response of Link 2');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('Link 2 Angle', 'Desired Angle');
grid on;

% Plot Link 3
subplot(3, 1, 3);
plot(tout_3, xout_3(:, 1), 'g', 'LineWidth', 1.5); hold on;
plot(tout_3, desired_angles_3, 'k--', 'LineWidth', 1.2);
title('Time Response of Link 3');
axis([0 10 0 4])
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('Link 3 Angle', 'Desired Angle');
grid on;
