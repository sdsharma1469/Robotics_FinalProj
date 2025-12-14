% force_control_sim.m
% Part B: Pure Force Control for 2-DOF RR Arm pressing against the wall x + y = 2
% Control goal: maintain desired normal force F_des into the wall, no position control.

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
% 2. Environment (Wall) and Force Control Specs
% -------------------------------------------------------------------------
K_env = 1000;         % wall stiffness
F_des = 10;           % desired scalar force INTO the wall (units)

% Wall: line x + y = 2
n_hat   = [1; 1] / sqrt(2);   % unit normal (pointing "out" along [1,1])
p_wall  = [2; 0];             % point on wall
n_wall  = dot(n_hat, p_wall); % normal coordinate of the wall

% desired penetration depth
d_pen_des = F_des / K_env;
n_des     = n_wall + d_pen_des;  % desired normal coordinate

%% ------------------------------------------------------------------------
% 3. Simulation Settings
% -------------------------------------------------------------------------
dt    = 0.001;
t_end = 5;
t     = 0:dt:t_end;
N     = length(t);

% Storage
theta_history     = zeros(2, N);
theta_dot_history = zeros(2, N);
F_into_history    = zeros(1, N);
n_history         = zeros(1, N);
x_history         = zeros(1, N);
y_history         = zeros(1, N);

%% ------------------------------------------------------------------------
% 4. Initial Condition (EE in contact but not at steady force)
% -------------------------------------------------------------------------
% Start exactly on the wall at a non-equilibrium point
p0    = [1.5; 0.5];
theta = robot_obj.inverseKinematics(p0(1), p0(2));
theta_dot = [0; 0];

theta_history(:,1)     = theta;
theta_dot_history(:,1) = theta_dot;

% For normal velocity estimate
[x0, y0] = robot_obj.forwardKinematics(theta);
n_prev   = dot(n_hat, [x0; y0]);

%% ------------------------------------------------------------------------
% 5. Force Controller Gains (normal-direction PD)
% -------------------------------------------------------------------------
Kp_n = 800;    % tune as needed
Kd_n = 40;     % tune as needed

%% ------------------------------------------------------------------------
% 6. Simulation Loop
% -------------------------------------------------------------------------
for k = 1:N

    % Forward kinematics
    [x, y] = robot_obj.forwardKinematics(theta);
    p      = [x; y];

    % Normal coordinate and penetration depth
    n_curr = dot(n_hat, p);
    d_pen  = max(0, n_curr - n_wall); 

    % Environmental force from wall on robot (along -n_hat)
    F_env_n = -K_env * d_pen;
    
    % Force INTO wall (positive when pressing into wall)
    F_into  = -F_env_n;

    % Save histories
    x_history(k)      = x;
    y_history(k)      = y;
    n_history(k)      = n_curr;
    F_into_history(k) = F_into;
    theta_history(:,k)     = theta;
    theta_dot_history(:,k) = theta_dot;

    % Approximate normal velocity
    if k == 1
        n_dot = 0;
    else
        n_dot = (n_curr - n_prev) / dt;
    end
    n_prev = n_curr;

    % Normal position error (desired normal coordinate)
    e_n     = n_des - n_curr;
    e_n_dot = -n_dot;   % desired normal velocity is zero

    % PD in normal displacement (position-based force control)
    F_cmd_n = Kp_n * e_n + Kd_n * e_n_dot;

    % commanded Cartesian force vector at the endpoint
    F_cmd_vec = F_cmd_n * n_hat;   % 2x1

    % Map to joint torques using Jacobian transpose
    J   = robot_obj.jacobian(theta);
    tau = J' * F_cmd_vec;

    % Dynamics integration
    M = robot_obj.massMatrix(theta);
    C = robot_obj.coriolisMatrix(theta, theta_dot);
    G = robot_obj.gravityVector(theta);
    F_fric = robot_obj.frictionTorque(theta_dot);

    theta_ddot = M \ (tau - C*theta_dot - G - F_fric);

    theta_dot = theta_dot + theta_ddot * dt;
    theta     = theta     + theta_dot  * dt;

    % Optional: visualization during force control
    if mod(k, 200) == 0
        [x1, y1, x2, y2] = getArmXY(theta, L1, L2);
        figure(30); clf;
        plot([0 x1 x2], [0 y1 y2], 'b-o', 'LineWidth', 2); hold on;
        % wall
        fplot(@(xline) 2 - xline, [0 2], 'k--','LineWidth',1.2);
        axis equal; grid on;
        xlim([-0.2 2]); ylim([-0.2 2]);
        title(sprintf('Force Control: t = %.2f s', t(k)));
        xlabel('X'); ylabel('Y');
        drawnow;
    end

end

%% ------------------------------------------------------------------------
% 7. Required Plots for Force Control
% -------------------------------------------------------------------------

% 7.1 Time response of force into the wall vs desired
figure('Name','Normal Force vs Time');
plot(t, F_into_history, 'b-', 'LineWidth', 1.5); hold on;
yline(F_des, 'r--', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Force into Wall (units)');
title('Normal Force Into Wall vs Time');
legend('Actual Force','Desired Force');
grid on;

% 7.2 Overhead view (whole robot trajectory)
figure('Name','Overhead EE Trajectory (Force Control)');
plot(x_history, y_history, 'b-', 'LineWidth', 1.5); hold on;
fplot(@(xline) 2 - xline, [0 2], 'k--','LineWidth',1.2);
xlabel('X'); ylabel('Y');
title('End-Effector Trajectory Under Force Control');
grid on; axis equal;

% 7.3 Zoomed-in view near wall contact
figure('Name','Zoomed View Near Wall Contact');
plot(x_history, y_history, 'b-', 'LineWidth', 1.5); hold on;
fplot(@(xline) 2 - xline, [0 2], 'k--','LineWidth',1.2);
xlabel('X'); ylabel('Y');
title('Zoomed Endpoint Motion Near Wall');
grid on; axis equal;
xlim([0.5 1.7]);
ylim([0.2 1.8]);

% 7.4 Manipulator snapshots at several times
snapshot_times = [0, 1, 2, 3, 4];  % choose any interesting times
figure('Name','Manipulator Snapshots (Force Control)'); hold on;
colors = lines(length(snapshot_times));

for k = 1:length(snapshot_times)
    [~, idx] = min(abs(t - snapshot_times(k)));
    th = theta_history(:,idx);
    [x1, y1, x2, y2] = getArmXY(th, L1, L2);
    plot([0 x1 x2], [0 y1 y2], 'Color', colors(k,:), 'LineWidth', 2);
end
fplot(@(xline) 2 - xline, [0 2], 'k--','LineWidth',1.2);
axis equal; grid on;
xlabel('X'); ylabel('Y');
title('Manipulator Snapshots at Several Times (Force Control)');
legend('t=0','t=1','t=2','t=3','t=4','Wall');

%% ------------------------------------------------------------------------
% Helper: getArmXY
% -------------------------------------------------------------------------
function [x1, y1, x2, y2] = getArmXY(theta, L1, L2)
    th1 = theta(1);
    th2 = theta(2);
    x1  = L1*cos(th1);
    y1  = L1*sin(th1);
    x2  = x1 + L2*cos(th1 + th2);
    y2  = y1 + L2*sin(th1 + th2);
end
