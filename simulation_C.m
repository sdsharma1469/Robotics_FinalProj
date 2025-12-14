% hybrid_control_sim.m
% Part C: Nonlinear Hybrid Control for 2-DOF RR Arm
% - Maintain desired normal force into wall x + y = 2
% - Move along the wall from p1 to p2 using a trajectory in s
%
% Requires:
%   - robot.m (your RR robot class)
%   - getArmXY helper (at bottom of this file or separate)

clear; clc; close all;

%% ------------------------------------------------------------------------
% 1. Robot Parameters
% -------------------------------------------------------------------------
L1 = 1; L2 = 1;
m1 = 3; m2 = 3;
g  = 0;
D1 = 0.1; D2 = 0.1;

robot_obj = robot(L1, L2, m1, m2, g, D1, D2);

%% ------------------------------------------------------------------------
% 2. Wall and Force Specs
% -------------------------------------------------------------------------
K_env = 1000;              % wall stiffness
F_des = 10;                % desired normal force INTO wall

% Wall: x + y = 2
n_hat   = [1; 1] / sqrt(2);  % unit normal
p_wall  = [2; 0];            % a point on the wall
n_wall  = dot(n_hat, p_wall);

% Desired penetration depth (= F_des / K_env)
d_pen_des = F_des / K_env;
n_des     = n_wall + d_pen_des;

%% ------------------------------------------------------------------------
% 3. Tangential Direction and Trajectory Along Wall
% -------------------------------------------------------------------------
p1   = [1.5; 0.5];      % start point on wall
p2   = [0.5; 1.5];      % goal point on wall (sqrt(2) away)
t_hat = (p2 - p1) / norm(p2 - p1);   % unit tangent along wall segment

% s coordinate: s = t_hat^T (p - p1)
s0 = 0;
sf = norm(p2 - p1);      % length of segment along t_hat

% Trajectory time
T_traj = 5;              % total time for move (s)

%% ------------------------------------------------------------------------
% 4. Simulation Settings
% -------------------------------------------------------------------------
dt    = 0.001;
t_end = 6;               % slightly longer than trajectory time
t     = 0:dt:t_end;
N     = length(t);

%% ------------------------------------------------------------------------
% 5. Gains
% -------------------------------------------------------------------------
% Normal (force-based via displacement in n)
Kp_n = 800;
Kd_n = 40;

% Tangential (position control in s)
Kp_s = 300;
Kd_s = 50;

%% ------------------------------------------------------------------------
% 6. Storage
% -------------------------------------------------------------------------
theta_history     = zeros(2, N);
theta_dot_history = zeros(2, N);

x_history = zeros(1, N);
y_history = zeros(1, N);

n_history      = zeros(1, N);
s_history      = zeros(1, N);
s_des_history  = zeros(1, N);

F_into_history = zeros(1, N);

%% ------------------------------------------------------------------------
% 7. Initial Condition
% -------------------------------------------------------------------------
% Start at p1 on the wall (no penetration, so initial force is 0)
theta     = robot_obj.inverseKinematics(p1(1), p1(2));
theta_dot = [0; 0];

% Pre-compute initial forward kinematics
[x0, y0] = robot_obj.forwardKinematics(theta);
p0  = [x0; y0];
n0  = dot(n_hat, p0);
s0_actual = dot(t_hat, (p0 - p1));      % should be ~0

n_prev = n0;
s_prev = s0_actual;

theta_history(:,1)     = theta;
theta_dot_history(:,1) = theta_dot;

x_history(1) = x0;
y_history(1) = y0;
n_history(1) = n0;
s_history(1) = s0_actual;
s_des_history(1) = 0;

%% ------------------------------------------------------------------------
% 8. Simulation Loop
% -------------------------------------------------------------------------
for k = 1:N

    tk = t(k);

    % ------------------- 8.1 Desired s(t) along the wall ------------------
    % Min-jerk from s=0 to s=sf over T_traj seconds.
    if tk <= T_traj
        tau = tk / T_traj;  % normalized time in [0,1]
        % position shape
        sigma = 10*tau^3 - 15*tau^4 + 6*tau^5;
        % velocity shape
        sigma_dot = (30*tau^2 - 60*tau^3 + 30*tau^4) / T_traj;

        s_des   = s0 + (sf - s0)*sigma;
        s_dot_des = (sf - s0)*sigma_dot;
    else
        % hold final position
        s_des     = sf;
        s_dot_des = 0;
    end

    s_des_history(k) = s_des;

    % Desired Cartesian point (on the wall line, no explicit penetration)
    p_des_on_wall = p1 + t_hat * s_des;

    % ------------------ 8.2 Actual Cartesian quantities -------------------
    [x, y] = robot_obj.forwardKinematics(theta);
    p      = [x; y];

    x_history(k) = x;
    y_history(k) = y;

    % Normal coordinate and penetration
    n_curr = dot(n_hat, p);
    d_pen  = max(0, n_curr - n_wall);

    % Wall force on robot (along -n_hat)
    F_env_n   = -K_env * d_pen;
    F_env_vec = F_env_n * n_hat;

    % Force into the wall (for plotting)
    F_into = -F_env_n;

    % Along-wall coordinate
    s_curr = dot(t_hat, (p - p1));

    n_history(k) = n_curr;
    s_history(k) = s_curr;
    F_into_history(k) = F_into;

    theta_history(:,k)     = theta;
    theta_dot_history(:,k) = theta_dot;

    % Approximate normal and tangential velocities
    if k == 1
        n_dot = 0;
        s_dot = 0;
    else
        n_dot = (n_curr - n_prev) / dt;
        s_dot = (s_curr - s_prev) / dt;
    end

    n_prev = n_curr;
    s_prev = s_curr;

    % ------------------ 8.3 Hybrid Control (n & s) -----------------------
    % ----- Normal direction: regulate penetration ~ desired force -----
    e_n     = n_des - n_curr;   % n_des = n_wall + d_pen_des
    e_n_dot = -n_dot;           % desired n_dot = 0

    F_cmd_n = Kp_n * e_n + Kd_n * e_n_dot;

    % ----- Tangential direction: trajectory tracking in s -----------
    e_s     = s_des     - s_curr;
    e_s_dot = s_dot_des - s_dot;

    F_cmd_s = Kp_s * e_s + Kd_s * e_s_dot;

    % Combined Cartesian force command
    F_cmd_vec = F_cmd_n * n_hat + F_cmd_s * t_hat;

    % ------------------ 8.4 Map to joint torques ------------------------
    J        = robot_obj.jacobian(theta);
    tau_cmd  = J' * F_cmd_vec;     % from controller
    tau_env  = J' * F_env_vec;     % from wall on robot

    % ------------------ 8.5 Robot dynamics integration ------------------
    M      = robot_obj.massMatrix(theta);
    C      = robot_obj.coriolisMatrix(theta, theta_dot);
    G      = robot_obj.gravityVector(theta);
    F_fric = robot_obj.frictionTorque(theta_dot);

    % NOTE: external torque from environment (tau_env) ADDS on RHS
    theta_ddot = M \ (tau_cmd - C*theta_dot - G - F_fric);


    theta_dot = theta_dot + theta_ddot * dt;
    theta     = theta     + theta_dot  * dt;

    % ------------------ Optional live visualization ----------------------
    if mod(k, 500) == 0
        [x1, y1, x2, y2] = getArmXY(theta, L1, L2);

        figure(50); clf;
        plot([0 x1 x2], [0 y1 y2], 'b-o', 'LineWidth', 2); hold on;
        % desired point on wall
        plot(p_des_on_wall(1), p_des_on_wall(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
        % wall line
        fplot(@(xline) 2 - xline, [0 2], 'k--','LineWidth',1.2);
        axis equal; grid on;
        xlim([-0.2 2]); ylim([-0.2 2]);
        title(sprintf('Hybrid Control: t = %.2f s', tk));
        xlabel('X'); ylabel('Y');
        drawnow;
    end
end

%% ------------------------------------------------------------------------
% 9. Required Plots for Part C
% -------------------------------------------------------------------------

% 9.1 X and Y vs Time
figure('Name','X Position vs Time');
plot(t, x_history, 'b-','LineWidth',1.5);
xlabel('Time (s)'); ylabel('X (m)');
title('End-Effector X vs Time');
grid on;

figure('Name','Y Position vs Time');
plot(t, y_history, 'b-','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Y (m)');
title('End-Effector Y vs Time');
grid on;

% 9.2 Endpoint position along the wall (s)
figure('Name','Along-Wall Position s(t)');
plot(t, s_des_history, 'r--','LineWidth',1.5); hold on;
plot(t, s_history,      'b-','LineWidth',1.5);
xlabel('Time (s)'); ylabel('s (along-wall coordinate)');
title('Endpoint Position Along Wall');
legend('s_{des}(t)','s_{actual}(t)');
grid on;

% 9.3 Normal force into the wall
figure('Name','Force Into Wall vs Time');
plot(t, F_into_history, 'b-','LineWidth',1.5); hold on;
yline(F_des, 'r--','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Force Into Wall');
title('Normal Force Into Wall vs Time');
legend('Actual','Desired');
grid on;

% 9.4 Overhead trajectory
figure('Name','Overhead Trajectory (Hybrid Control)');
plot(x_history, y_history, 'b-','LineWidth',1.5); hold on;
fplot(@(xline) 2 - xline, [0 2], 'k--','LineWidth',1.2);
xlabel('X'); ylabel('Y');
title('End-Effector Trajectory Under Hybrid Control');
axis equal; grid on;

% 9.5 Snapshots of motion
snapshot_times = [0, 1.5, 3, 4.5, 6];
figure('Name','Manipulator Snapshots (Hybrid Control)'); hold on;
colors = lines(length(snapshot_times));

for kk = 1:length(snapshot_times)
    [~, idx] = min(abs(t - snapshot_times(kk)));
    th_snap  = theta_history(:,idx);
    [x1, y1, x2, y2] = getArmXY(th_snap, L1, L2);
    plot([0 x1 x2], [0 y1 y2], 'Color', colors(kk,:), 'LineWidth', 2);
end

fplot(@(xline) 2 - xline, [0 2], 'k--','LineWidth',1.2);
axis equal; grid on;
xlabel('X'); ylabel('Y');
title('Manipulator Configurations at Selected Times (Hybrid)');
legend('t=0','t=1.5','t=3','t=4.5','t=6','Wall');

%% ------------------------------------------------------------------------
% Helper: getArmXY
% -------------------------------------------------------------------------
function [x1, y1, x2, y2] = getArmXY(theta, L1, L2)
    th1 = theta(1);
    th2 = theta(2);

    x1 = L1*cos(th1);
    y1 = L1*sin(th1);

    x2 = x1 + L2*cos(th1 + th2);
    y2 = y1 + L2*sin(th1 + th2);
end
