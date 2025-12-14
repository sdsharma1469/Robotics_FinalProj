% simulation.m
% Nonlinear Position Control for 2-DOF RR Arm in the X-Y plane
% Motion along wall segment [1.5,0.5] <-> [0.5,1.5] with sinusoidal motion

clear; clc; close all;

%% ------------------------------------------------------------------------
% 1. Robot Parameters
% -------------------------------------------------------------------------
L1 = 1; L2 = 1;
m1 = 3; m2 = 3;
g  = 9.81;
D1 = 0.1; D2 = 0.1;

robot_obj = robot(L1, L2, m1, m2, g, D1, D2);

%% ------------------------------------------------------------------------
% 2. Controller Gains
% -------------------------------------------------------------------------
gains.Kp = diag([400, 400]);
gains.Kd = diag([80, 80]);
gains.Ki = diag([100, 100]);

S = eye(2); % switching matrix I

%% ------------------------------------------------------------------------
% 3. Trajectory Along the Wall
% -------------------------------------------------------------------------
dt = 0.001;
t_end = 10;
t = 0:dt:t_end;

p1 = [1.5; 0.5];   % endpoint 1
p2 = [0.5; 1.5];   % endpoint 2
p_mid = [1; 1];    % midpoint
t_hat = (p2 - p1) / norm(p2 - p1);  % unit vector along the wall
d = norm(p1 - p_mid);               % half-length of segment

omega = 2*pi/5;  % 5-second period

s = d * sin(omega * t);  % scalar motion along the wall

x_desired = p_mid(1) + t_hat(1) * s;
y_desired = p_mid(2) + t_hat(2) * s;

x_dot_desired  = d * omega * cos(omega * t) * t_hat(1);
y_dot_desired  = d * omega * cos(omega * t) * t_hat(2);

x_ddot_desired = -d * omega^2 * sin(omega * t) * t_hat(1);
y_ddot_desired = -d * omega^2 * sin(omega * t) * t_hat(2);

%% ------------------------------------------------------------------------
% 4. Inverse Kinematics for Desired Joint Motion
% -------------------------------------------------------------------------
theta_desired = zeros(2, length(t));

for i = 1:length(t)
    theta_desired(:,i) = robot_obj.inverseKinematics(x_desired(i), y_desired(i));
end

%% ------------------------------------------------------------------------
% 5. Desired Joint Velocities & Accelerations (Finite Difference)
% -------------------------------------------------------------------------
theta_dot_desired  = zeros(2, length(t));
theta_ddot_desired = zeros(2, length(t));

for i = 2:length(t)
    theta_dot_desired(:,i) = (theta_desired(:,i) - theta_desired(:,i-1)) / dt;
end

for i = 3:length(t)
    theta_ddot_desired(:,i) = (theta_dot_desired(:,i) - theta_dot_desired(:,i-1)) / dt;
end

theta_dot_desired(:,1:2)  = repmat(theta_dot_desired(:,3), 1, 2);
theta_ddot_desired(:,1:2) = repmat(theta_ddot_desired(:,3), 1, 2);

%% ------------------------------------------------------------------------
% 6. Adjust Controller for Critical Damping (Joint 2)
% -------------------------------------------------------------------------
M_initial = robot_obj.massMatrix(theta_desired(:,1));
I2 = M_initial(2,2);
omega_n = 1/0.25;
zeta = 1;

gains.Kp(2,2) = I2 * omega_n^2;
gains.Kd(2,2) = 2*zeta*omega_n*I2;

%% ------------------------------------------------------------------------
% 7. Allocate Storage
% -------------------------------------------------------------------------
integral_error = [0;0];
max_integral = 100;

x_actual = zeros(1, length(t));
y_actual = zeros(1, length(t));

pos_err_history = zeros(2, length(t));
vel_err_history = zeros(2, length(t));
tau_history     = zeros(2, length(t));

s_actual  = zeros(1, length(t));
s_desired = zeros(1, length(t));

actual.theta     = theta_desired(:,1);
actual.theta_dot = [0;0];

%% ------------------------------------------------------------------------
% 8. Control Loop
% -------------------------------------------------------------------------
p_start = p1;

for i = 1:length(t)

    desired.theta      = theta_desired(:,i);
    desired.theta_dot  = theta_dot_desired(:,i);
    desired.theta_ddot = theta_ddot_desired(:,i);

    [x_actual(i), y_actual(i)] = robot_obj.forwardKinematics(actual.theta);

    pos_err = [x_desired(i) - x_actual(i);
               y_desired(i) - y_actual(i)];
    pos_err_history(:,i) = pos_err;

    vel_err = [x_dot_desired(i); y_dot_desired(i)] - actual.theta_dot;
    vel_err_history(:,i) = vel_err;

    integral_error = integral_error + pos_err * dt;
    integral_error = max(min(integral_error, max_integral), -max_integral);

    tau = tau_controller(desired, actual, gains, integral_error, robot_obj, S);
    tau_history(:,i) = tau;

    M = robot_obj.massMatrix(actual.theta);
    C = robot_obj.coriolisMatrix(actual.theta, actual.theta_dot);
    G = robot_obj.gravityVector(actual.theta);
    F = robot_obj.frictionTorque(actual.theta_dot);

    theta_ddot = M \ (tau - C*actual.theta_dot - G - F);

    actual.theta_dot = actual.theta_dot + theta_ddot * dt;
    actual.theta = actual.theta + actual.theta_dot * dt;

    % compute along-wall coordinate
    p_act = [x_actual(i); y_actual(i)];
    p_des = [x_desired(i); y_desired(i)];
    s_actual(i)  = dot(p_act - p_start, t_hat);
    s_desired(i) = dot(p_des - p_start, t_hat);

    % --------------------------- Robot Visualization -----------------------
    if mod(i, 100) == 0   % update every 100 iterations
        [x1, y1, x2, y2] = getArmXY(actual.theta, L1, L2);

        figure(20); clf;
        plot([0 x1 x2], [0 y1 y2], 'b-o', 'LineWidth', 2); hold on;
        plot(x_desired(i), y_desired(i), 'rx', 'MarkerSize', 12, 'LineWidth', 2);

        fplot(@(x) 2 - x, [0 2], 'k--'); % wall line
        axis equal; grid on;
        xlim([-0.2 2]); ylim([-0.2 2]);
        title(sprintf('Robot Arm at t = %.2f sec', t(i)));
        xlabel('X'); ylabel('Y');
        drawnow;
    end
end

%% ------------------------------------------------------------------------
% 9. Snapshot Plot for Report
% -------------------------------------------------------------------------
%% ------------------------------------------------------------------------
% 9. REQUIRED PLOTS (In Correct Assignment Order)
% ------------------------------------------------------------------------

%% 9.1 Joint Torques Over Time
figure('Name','Joint Torques vs Time');
plot(t, tau_history(1,:), 'm-', 'LineWidth', 1.3); hold on;
plot(t, tau_history(2,:), 'c-', 'LineWidth', 1.3);
xlabel('Time (s)'); ylabel('Torque (Nm)');
title('Control Torques \tau_1 and \tau_2 Over Time');
legend('\tau_1','\tau_2');
grid on;


%% 9.2 X Position vs Time
figure('Name','X Position vs Time');
plot(t, x_desired, 'r--','LineWidth',1.5); hold on;
plot(t, x_actual,  'b-','LineWidth',1.5);
xlabel('Time (s)'); ylabel('X Position');
title('End-Effector X Position vs Time');
legend('Desired X','Actual X');
grid on;


%% 9.3 Y Position vs Time
figure('Name','Y Position vs Time');
plot(t, y_desired, 'r--','LineWidth',1.5); hold on;
plot(t, y_actual,  'b-','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Y Position');
title('End-Effector Y Position vs Time');
legend('Desired Y','Actual Y');
grid on;


%% 9.4 Overhead X-Y Trajectory (Manipulator Endpoint Path)
figure('Name','End-Effector X-Y Trajectory');
plot(x_desired, y_desired, 'r--','LineWidth',1.5); hold on;
plot(x_actual,  y_actual, 'b-','LineWidth',1.5);
xlabel('X Position'); ylabel('Y Position');
title('End-Effector Trajectory in X-Y Plane');
legend('Desired Path','Actual Path');
grid on; axis equal;


%% 9.5 Endpoint Position Along the Wall
figure('Name','Position Along Wall');
plot(t, s_desired,'r--','LineWidth',1.5); hold on;
plot(t, s_actual,'b-','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Along-Wall Position (units)');
title('Endpoint Position Along Wall Line (s-coordinate)');
legend('Desired s(t)','Actual s(t)');
grid on;


%% 9.6 Manipulator Snapshots
snapshot_times = [0, 1.25, 2.5, 3.75, 5];

figure('Name','Manipulator Snapshots'); hold on;
colors = lines(length(snapshot_times));

for k = 1:length(snapshot_times)
    [~, idx] = min(abs(t - snapshot_times(k)));
    [x1, y1, x2, y2] = getArmXY(theta_desired(:,idx), L1, L2);

    plot([0 x1 x2], [0 y1 y2], 'Color', colors(k,:), 'LineWidth', 2);
end

fplot(@(x) 2-x, [0 2], 'k--','LineWidth',1.2);
axis equal; grid on;
xlabel('X'); ylabel('Y');
title('Manipulator Configuration at Select Time Instants');
legend('t=0','t=1.25','t=2.5','t=3.75','t=5','Wall');

%% ------------------------------------------------------------------------
% Helper Function for Robot Link Poses
% -------------------------------------------------------------------------
function [x1, y1, x2, y2] = getArmXY(theta, L1, L2)
    th1 = theta(1);
    th2 = theta(2);

    x1 = L1*cos(th1);
    y1 = L1*sin(th1);

    x2 = x1 + L2*cos(th1 + th2);
    y2 = y1 + L2*sin(th1 + th2);
end

