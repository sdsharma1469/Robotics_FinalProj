%simulation_D.m

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
% 2. Environment: Two Walls + Corner
% -------------------------------------------------------------------------
K_env = 1000;     
F_des = 10;       

% --- Wall 1: x + y = 2 ---
n1_hat   = [1; 1] / sqrt(2);        % unit normal for wall 1
p1_wall  = [2; 0];                  % starting (wall 1)
n1_wall  = dot(n1_hat, p1_wall);    

% Use start and corner to define tangential direction along wall 1
p1_start = [1.5; 0.5];              
p_corner = [0.25; 1.75];            
t1_hat   = (p_corner - p1_start);
t1_hat   = t1_hat / norm(t1_hat);   % unit tangent along wall 1

% --- Wall 2: line through [0.5, 2] and [-2, -0.5]
p2_wall = [0.5; 2];


% made sure to check x = -0.5 -> y = 1.0 (distance ~1.12 < 2).
p2_end  = [-0.5; 1.0];              

% ran into many issues here!! 
% the fix ended up being to make sure the 
% tangent was in -X direction
t2_hat  = (p2_end - p_corner);
t2_hat  = t2_hat / norm(t2_hat);

% normal to wall 2 
n2_hat  = [1; -1] / sqrt(2);        
n2_wall = dot(n2_hat, p2_wall);

%% ------------------------------------------------------------------------
% 3. Trajectories along each wall (s-coordinates)
% -------------------------------------------------------------------------
% For Wall 1: from p1_start to corner
s1_0 = 0;
s1_f = norm(p_corner - p1_start);   % distance along wall 1
T1   = 5;                           % time for phase 1

% For Wall 2: from corner to p2_end
s2_0 = 0;
s2_f = norm(p2_end - p_corner);     % distance along wall 2
T2   = 5;                           % time for phase 2

% Total simulation time
t_end = T1 + T2 + 1;    % a little extra after 2nd segment
dt    = 0.001;
t     = 0:dt:t_end;
N     = length(t);

%% ------------------------------------------------------------------------
% 4. Gains (normal & tangential)
% -------------------------------------------------------------------------
% Normal 
Kp_n = 1000;
Kd_n = 80;

% Tangential 
Kp_s = 300;
Kd_s = 60;

% Desired penetration
d_pen_des = F_des / K_env;

%% ------------------------------------------------------------------------
% 5. Storage
% -------------------------------------------------------------------------
theta_history     = zeros(2, N);
theta_dot_history = zeros(2, N);

x_history = zeros(1, N);
y_history = zeros(1, N);

F_into_history = zeros(1, N);
n_history      = zeros(1, N);    

s_rel_history     = zeros(1, N); 
s_rel_des_history = zeros(1, N);

mode_history = zeros(1, N);     

%% ------------------------------------------------------------------------
% 6. Initial Condition (start on Wall 1 at p1_start)
% -------------------------------------------------------------------------
theta     = robot_obj.inverseKinematics(p1_start(1), p1_start(2));
theta_dot = [0; 0];

[x0, y0] = robot_obj.forwardKinematics(theta);
p0       = [x0; y0];


n_curr = dot(n1_hat, p0);
s_curr = dot(t1_hat, (p0 - p1_start));

n_prev    = n_curr;
s_prev    = s_curr;
mode_prev = 1;

theta_history(:,1)     = theta;
theta_dot_history(:,1) = theta_dot;
x_history(1)           = x0;
y_history(1)           = y0;
n_history(1)           = n_curr;
s_rel_history(1)       = s_curr;
s_rel_des_history(1)   = 0;
mode_history(1)        = 1;

%% ------------------------------------------------------------------------
% 7. Simulation Loop
% -------------------------------------------------------------------------
for k = 1:N

    tk = t(k);

    % ----------------- 7.1 Choose active wall (mode) --------------------
    % Simple time-based switching: mode 1 for t <= T1, mode 2 afterwards
    if tk <= T1
        mode = 1;
    else
        mode = 2;
    end

    % ================================  
    % SWITCHING HANDLER (Wall1 → Wall2)
    % ================================
    if mode == 2 && mode_prev == 1
    
        % -------- Project end-effector onto Wall 2 --------
        % get p from current theta
        [x_sw, y_sw] = robot_obj.forwardKinematics(theta);
        p_sw         = [x_sw; y_sw];

        % Compute new penetration
        n2_curr = dot(n2_hat, p_sw);
        dist2   = n2_curr - n2_wall;
    
        % project onto new tanegnt
        p_proj = p_sw - dist2 * n2_hat;
    
        % Solve IK 
        theta   = robot_obj.inverseKinematics(p_proj(1), p_proj(2));
        [x,y]   = robot_obj.forwardKinematics(theta);
        p       = [x;y];
    
        % -------- Reset derivative histories --------
        n_curr = dot(n2_hat, p);   % now exactly on surface
        s_curr = dot(t2_hat, (p - p_corner));
    
        n_prev = n_curr;
        s_prev = s_curr;
        n_dot  = 0;
        s_dot  = 0;
    
    end

    % Forward kinematics
    [x, y] = robot_obj.forwardKinematics(theta);
    p      = [x; y];

    x_history(k)   = x;
    y_history(k)   = y;
    mode_history(k)= mode;

    % ----------------- 7.2 Geometry for active wall ---------------------
    if mode == 1
        % Wall 1 geometry
        n_hat   = n1_hat;
        t_hat   = t1_hat;
        n_wall  = n1_wall;
        p_ref   = p1_start;     % reference for s
        
        
        tau_traj = min(max(tk / T1, 0), 1);   % 0..1
        s0       = s1_0;
        sf       = s1_f;
    else
        % Wall 2 geometry
        n_hat   = n2_hat;
        t_hat   = t2_hat;
        n_wall  = n2_wall;
        p_ref   = p_corner;     % reference for s on wall 2


        t_local  = tk - T1;
        tau_traj = min(max(t_local / T2, 0), 1);
        s0       = s2_0;
        sf       = s2_f;
    end

    % Normal coordinate & penetration for active wall
    n_curr = dot(n_hat, p);
    d_pen  = max(0, n_curr - n_wall);

    % Along-wall coordinate for active wall
    s_curr = dot(t_hat, (p - p_ref));

    % ------------- 7.3 Approximate n_dot and s_dot (with switch care) ----
    if k == 1 || mode ~= mode_prev
        % Reinitialize derivatives on switch or first sample
        n_dot  = 0;
        s_dot  = 0;
        n_prev = n_curr;
        s_prev = s_curr;
    else
        n_dot = (n_curr - n_prev) / dt;
        s_dot = (s_curr - s_prev) / dt;
    end

    n_prev    = n_curr;
    s_prev    = s_curr;
    mode_prev = mode;

    % ------------- 7.4 Desired s(t) along active wall ---------
    sigma     = 10*tau_traj^3 - 15*tau_traj^4 + 6*tau_traj^5;
    sigma_dot = (30*tau_traj^2 - 60*tau_traj^3 + 30*tau_traj^4);

    s_des = s0 + (sf - s0)*sigma;
    if mode == 1
        s_dot_des = (sf - s0)*sigma_dot / T1;
    else
        s_dot_des = (sf - s0)*sigma_dot / T2;
    end

    % Save along-wall data
    s_rel_history(k)     = s_curr;
    s_rel_des_history(k) = s_des;

    % ------------- 7.5 Normal control ----
    n_des   = n_wall + d_pen_des;
    e_n     = n_des - n_curr;
    e_n_dot = -n_dot;     

    F_cmd_n = Kp_n * e_n + Kd_n * e_n_dot;

    % ------------- 7.6 Tangential control ----------
    e_s     = s_des     - s_curr;
    e_s_dot = s_dot_des - s_dot;

    F_cmd_s = Kp_s * e_s + Kd_s * e_s_dot;

    % ------------- 7.7 Combined Cartesian force command -----------------
    F_cmd_vec = F_cmd_n * n_hat + F_cmd_s * t_hat;

    % Environment force (for plotting only)
    F_env_n    = -K_env * d_pen;        
    F_into     = -F_env_n;              

    F_into_history(k) = F_into;
    n_history(k)      = n_curr;

    % ------------- 7.8 Map to joint torques & integrate dynamics --------
    J        = robot_obj.jacobian(theta);
    tau_cmd  = J' * F_cmd_vec;

    M        = robot_obj.massMatrix(theta);
    C        = robot_obj.coriolisMatrix(theta, theta_dot);
    G        = robot_obj.gravityVector(theta);
    F_fric   = robot_obj.frictionTorque(theta_dot);

    % Position-based force control
    theta_ddot = M \ (tau_cmd - C*theta_dot - G - F_fric);

    theta_dot = theta_dot + theta_ddot * dt;
    theta     = theta     + theta_dot  * dt;

    theta_history(:,k)     = theta;
    theta_dot_history(:,k) = theta_dot;
end

%% ------------------------------------------------------------------------
% 8. PLOTTING (Done using GPT)
% -------------------------------------------------------------------------

% 8.1 X and Y positions vs time
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

% 8.2 Along-wall position (relative) vs time
figure('Name','Along-Wall Position vs Time');
plot(t, s_rel_des_history, 'r--','LineWidth',1.5); hold on;
plot(t, s_rel_history,      'b-','LineWidth',1.5);
xlabel('Time (s)'); ylabel('s (along-wall, relative)');
title('Endpoint Position Along Active Wall');
legend('s_{des}(t)','s_{actual}(t)');
grid on;

% 8.3 Force into the wall vs time
figure('Name','Force Into Wall vs Time');
plot(t, F_into_history, 'b-','LineWidth',1.5); hold on;
yline(F_des, 'r--','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Force Into Wall');
title('Normal Force Into Active Wall vs Time');
legend('Actual','Desired');
grid on;

% 8.4 Overhead trajectory with both walls and corner
figure('Name','Overhead Trajectory with Wall Switching');
plot(x_history, y_history, 'b-','LineWidth',1.5); hold on;

% Wall 1: x + y = 2
fplot(@(x) 2 - x, [ -1 2.5 ], 'k--','LineWidth',1.2);
% Wall 2: y = x + 1.5
fplot(@(x) x + 1.5, [ -2.5 2 ], 'k-.','LineWidth',1.2);

plot(p1_start(1), p1_start(2), 'go','MarkerFaceColor','g');
plot(p_corner(1), p_corner(2), 'ro','MarkerFaceColor','r');
plot(p2_end(1),   p2_end(2),   'mo','MarkerFaceColor','m');

xlabel('X'); ylabel('Y');
title('End-Effector Trajectory with Surface Switching');
legend('EE Path','Wall 1','Wall 2','Start (Wall 1)','Corner','End (Wall 2)', ...
       'Location','Best');
axis equal; grid on;

% 8.5 Snapshots of manipulator at several times
snapshot_times = [0, 3, 5, 7, 9];   % choose some representative times
figure('Name','Manipulator Snapshots (Surface Switching)'); hold on;
colors = lines(length(snapshot_times));

for kk = 1:length(snapshot_times)
    [~, idx] = min(abs(t - snapshot_times(kk)));
    th_snap  = theta_history(:,idx);
    [x1, y1, x2, y2] = getArmXY(th_snap, L1, L2);
    plot([0 x1 x2], [0 y1 y2], 'Color', colors(kk,:), 'LineWidth', 2);
end

% Plot both walls and corner
fplot(@(x) 2 - x, [ -1 2.5 ], 'k--','LineWidth',1.2);
fplot(@(x) x + 1.5, [ -2.5 2 ], 'k-.','LineWidth',1.2);
plot(p_corner(1), p_corner(2), 'ro','MarkerFaceColor','r');

axis equal; grid on;
xlabel('X'); ylabel('Y');
title('Manipulator Configurations at Selected Times (Wall Switching)');
legend('t=0','t=3','t=5','t=7','t=9','Wall 1','Wall 2','Corner','Location','Best');

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
