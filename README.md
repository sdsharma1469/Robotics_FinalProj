Shaunak Sharma Final Project
Done completely in Matlab (No simulink files)
Acknowledgements : Used Chatgpt for Commenting
 
Question A - Position Control
Code
Robot Definition
% robot.m
classdef robot
  properties
      L1; L2
      m1; m2
      g
      D
  end
  methods
      %=============================================================
      % Constructor
      %=============================================================
      function obj = robot(L1, L2, m1, m2, g, D1, D2)
          if nargin < 5, g = 9.81; end
          if nargin < 7, D1 = 0.1; D2 = 0.1; end
          obj.L1 = L1; obj.L2 = L2;
          obj.m1 = m1; obj.m2 = m2;
          obj.g  = g;
          obj.D  = diag([D1, D2]);
      end
      %=============================================================
      % Forward Kinematics
      %=============================================================
      function [x, z] = forwardKinematics(obj, theta)
          th1 = theta(1); th2 = theta(2);
          x = obj.L1*cos(th1) + obj.L2*cos(th1 + th2);
          z = obj.L1*sin(th1) + obj.L2*sin(th1 + th2);
      end
      %=============================================================
      % Inverse Kinematics (elbow-up)
      %=============================================================
      function theta = inverseKinematics(obj, x, z)
          c2 = (x^2 + z^2 - obj.L1^2 - obj.L2^2) / (2*obj.L1*obj.L2);
          c2 = min(max(c2, -1), 1);
          s2 = sqrt(1 - c2^2);
          th2 = atan2(s2, c2);
          k1 = obj.L1 + obj.L2*c2;
          k2 = obj.L2*s2;
          th1 = atan2(z, x) - atan2(k2, k1);
          theta = [th1; th2];
      end
      %=============================================================
      % Mass Matrix
      %=============================================================
      function M = massMatrix(obj, theta)
          th2 = theta(2);
          M11 = obj.m1*obj.L1^2 + obj.m2*(obj.L1^2 + obj.L2^2 + 2*obj.L1*obj.L2*cos(th2));
          M12 = obj.m2*(obj.L2^2 + obj.L1*obj.L2*cos(th2));
          M = [M11, M12;
               M12, obj.m2*obj.L2^2];
      end
      %=============================================================
      % Coriolis Matrix
      %=============================================================
      function C = coriolisMatrix(obj, theta, theta_dot)
          th2 = theta(2);
          d1 = theta_dot(1); d2 = theta_dot(2);
          h = obj.m2 * obj.L1 * obj.L2 * sin(th2);
          C = [ -h*d2,        -h*(d1+d2);
                 h*d1,         0         ];
      end
      %=============================================================
      % Gravity Vector
      %=============================================================
      function G = gravityVector(obj, theta)
          th1 = theta(1); th2 = theta(2);
          G1 = -obj.m1*obj.g*obj.L1*cos(th1) ...
               -obj.m2*obj.g*(obj.L1*cos(th1) + obj.L2*cos(th1+th2));
          G2 = -obj.m2*obj.g*obj.L2*cos(th1 + th2);
          G = [G1; G2];
      end
      %=============================================================
      % Friction Torque
      %=============================================================
      function F = frictionTorque(obj, theta_dot)
          F = obj.D * theta_dot;
      end
  end
end


Tau controller
% tau_controller.m
function tau = tau_controller(desired, actual, gains, integral_error, robot_obj, S)
%=============================================================
% Errors
%=============================================================
pos_error = desired.theta     - actual.theta;
vel_error = desired.theta_dot - actual.theta_dot;
%=============================================================
% PID terms
%=============================================================
F_p = gains.Kp * pos_error;
F_d = gains.Kd * vel_error;
F_i = gains.Ki * integral_error;
% Apply switching matrix (S = I here)
F_p = S * F_p;
F_d = S * F_d;
F_i = S * F_i;
%=============================================================
% Commanded acceleration
%=============================================================
alpha = desired.theta_ddot + F_d + F_p + F_i;
%=============================================================
% Robot dynamics
%=============================================================
M = robot_obj.massMatrix(actual.theta);
C = robot_obj.coriolisMatrix(actual.theta, actual.theta_dot);
G = robot_obj.gravityVector(actual.theta);
F = robot_obj.frictionTorque(actual.theta_dot);
%=============================================================
% Control torque
%=============================================================
tau = M*alpha + C*actual.theta_dot + G + F;
end


Simulation file 
% simulation.m
% Nonlinear Position Control for 2-DOF RR Arm moving along a wall
clear; clc; close all;
%=============================================================
% 1. Robot Parameters
%=============================================================
L1 = 1; L2 = 1;
m1 = 3; m2 = 3;
g  = 9.81;
D1 = 0.1; D2 = 0.1;
robot_obj = robot(L1, L2, m1, m2, g, D1, D2);
%=============================================================
% 2. Controller Gains
%=============================================================
gains.Kp = diag([400, 400]);
gains.Kd = diag([80, 80]);
gains.Ki = diag([100, 100]);
S = eye(2);
%=============================================================
% 3. Trajectory Along Wall
%=============================================================
dt = 0.001; t_end = 10;
t = 0:dt:t_end;
p1 = [1.5; 0.5];
p2 = [0.5; 1.5];
p_mid = [1; 1];
t_hat = (p2 - p1) / norm(p2 - p1);
d = norm(p1 - p_mid);
omega = 2*pi/5;
s = d * sin(omega * t);
x_desired = p_mid(1) + t_hat(1)*s;
y_desired = p_mid(2) + t_hat(2)*s;
x_dot_desired  = d*omega * cos(omega*t) * t_hat(1);
y_dot_desired  = d*omega * cos(omega*t) * t_hat(2);
x_ddot_desired = -d*omega^2 * sin(omega*t) * t_hat(1);
y_ddot_desired = -d*omega^2 * sin(omega*t) * t_hat(2);
%=============================================================
% 4. Inverse Kinematics
%=============================================================
theta_desired = zeros(2,length(t));
for i = 1:length(t)
   theta_desired(:,i) = robot_obj.inverseKinematics(x_desired(i), y_desired(i));
end
%=============================================================
% 5. Joint Velocities & Accelerations
%=============================================================
theta_dot_desired  = zeros(2,length(t));
theta_ddot_desired = zeros(2,length(t));
for i = 2:length(t)
   theta_dot_desired(:,i) = (theta_desired(:,i) - theta_desired(:,i-1))/dt;
end
for i = 3:length(t)
   theta_ddot_desired(:,i) = (theta_dot_desired(:,i) - theta_dot_desired(:,i-1))/dt;
end
theta_dot_desired(:,1:2)  = repmat(theta_dot_desired(:,3), 1, 2);
theta_ddot_desired(:,1:2) = repmat(theta_ddot_desired(:,3), 1, 2);
%=============================================================
% 6. Adjust Controller Gains (Joint 2 Critical Damping)
%=============================================================
M0 = robot_obj.massMatrix(theta_desired(:,1));
I2 = M0(2,2);
omega_n = 1/0.25; zeta = 1;
gains.Kp(2,2) = I2 * omega_n^2;
gains.Kd(2,2) = 2*zeta*omega_n*I2;
%=============================================================
% 7. Storage Allocation
%=============================================================
integral_error = [0;0];
max_integral = 100;
x_actual = zeros(1,length(t));
y_actual = zeros(1,length(t));
pos_err_history = zeros(2,length(t));
vel_err_history = zeros(2,length(t));
tau_history     = zeros(2,length(t));
s_actual  = zeros(1,length(t));
s_desired = zeros(1,length(t));
actual.theta     = theta_desired(:,1);
actual.theta_dot = [0;0];
%=============================================================
% 8. Control Loop
%=============================================================
p_start = p1;
for i = 1:length(t)
   desired.theta      = theta_desired(:,i);
   desired.theta_dot  = theta_dot_desired(:,i);
   desired.theta_ddot = theta_ddot_desired(:,i);
   [x_actual(i), y_actual(i)] = robot_obj.forwardKinematics(actual.theta);
   pos_err = [x_desired(i)-x_actual(i); y_desired(i)-y_actual(i)];
   pos_err_history(:,i) = pos_err;
   vel_err = [x_dot_desired(i); y_dot_desired(i)] - actual.theta_dot;
   vel_err_history(:,i) = vel_err;
   integral_error = integral_error + pos_err*dt;
   integral_error = max(min(integral_error, max_integral), -max_integral);
   tau = tau_controller(desired, actual, gains, integral_error, robot_obj, S);
   tau_history(:,i) = tau;
   M = robot_obj.massMatrix(actual.theta);
   C = robot_obj.coriolisMatrix(actual.theta, actual.theta_dot);
   G = robot_obj.gravityVector(actual.theta);
   F = robot_obj.frictionTorque(actual.theta_dot);
   theta_ddot = M \ (tau - C*actual.theta_dot - G - F);
   actual.theta_dot = actual.theta_dot + theta_ddot*dt;
   actual.theta     = actual.theta + actual.theta_dot*dt;
   % projection onto wall
   p_act = [x_actual(i); y_actual(i)];
   p_des = [x_desired(i); y_desired(i)];
   s_actual(i)  = dot(p_act - p_start, t_hat);
   s_desired(i) = dot(p_des - p_start, t_hat);
   % Visualization
   if mod(i,100)==0
       [x1,y1,x2,y2] = getArmXY(actual.theta, L1, L2);
       figure(20); clf;
       plot([0 x1 x2],[0 y1 y2],'b-o','LineWidth',2); hold on;
       plot(x_desired(i),y_desired(i),'rx','MarkerSize',12,'LineWidth',2);
       fplot(@(x) 2-x,[0 2],'k--');
       axis equal; grid on;
       xlim([-0.2 2]); ylim([-0.2 2]);
       title(sprintf('Robot Arm at t = %.2f sec', t(i)));
       drawnow;
   end
end
%=============================================================
% 9. REQUIRED PLOTS
%=============================================================
% Torques
figure; plot(t,tau_history(1,:), t,tau_history(2,:),'LineWidth',1.3);
xlabel('Time (s)'); ylabel('Torque (Nm)');
title('Joint Torques'); grid on; legend('\tau_1','\tau_2');
% X position
figure; plot(t,x_desired,'r--', t,x_actual,'b-','LineWidth',1.5);
xlabel('Time (s)'); ylabel('X'); title('X Position'); grid on;
% Y position
figure; plot(t,y_desired,'r--', t,y_actual,'b-','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Y'); title('Y Position'); grid on;
% X-Y trajectory
figure; plot(x_desired,y_desired,'r--', x_actual,y_actual,'b-','LineWidth',1.5);
xlabel('X'); ylabel('Y'); title('End-Effector Path'); grid on; axis equal;
% Along-wall coordinate
figure; plot(t,s_desired,'r--', t,s_actual,'b-','LineWidth',1.5);
xlabel('Time (s)'); ylabel('s'); title('Position Along Wall'); grid on;
% Snapshots
snapshot_times = [0,1.25,2.5,3.75,5];
figure; hold on;
colors = lines(numel(snapshot_times));
for k = 1:numel(snapshot_times)
   [~,idx]=min(abs(t-snapshot_times(k)));
   [x1,y1,x2,y2]=getArmXY(theta_desired(:,idx),L1,L2);
   plot([0 x1 x2],[0 y1 y2],'Color',colors(k,:),'LineWidth',2);
end
fplot(@(x)2-x,[0 2],'k--');
axis equal; grid on;
xlabel('X'); ylabel('Y');
title('Manipulator Snapshots');
legend('0','1.25','2.5','3.75','5','Wall');
%=============================================================
% Helper function
%=============================================================
function [x1,y1,x2,y2] = getArmXY(theta,L1,L2)
th1 = theta(1); th2 = theta(2);
x1 = L1*cos(th1); y1 = L1*sin(th1);
x2 = x1 + L2*cos(th1+th2);
y2 = y1 + L2*sin(th1+th2);
end




Results






Question B - Force Control
Code
Robot Definition (added jacobian)
% robot.m
classdef robot
   properties
       L1   % Link 1 
       L2   % Link 2 
       m1   % Link 1 mass
       m2   % Link 2 mass
       g    % gravity (LATER SET TO 0 !!)
       D    % friction matrix (2x2 diagonal)
   end
   methods
       %---------------------------------------------------------
       function obj = robot(L1, L2, m1, m2, g, D1, D2)
           % Constructor
           if nargin < 5
               g =0;
           end
           if nargin < 7
               D1 = 0.1;
               D2 = 0.1;
           end
           obj.L1 = L1;
           obj.L2 = L2;
           obj.m1 = m1;
           obj.m2 = m2;
           obj.g  = g;
           obj.D  = diag([D1, D2]);
       end
       %---------------------------------------------------------
       function [x, y] = forwardKinematics(obj, theta)
           % theta = [theta1; theta2]
           th1 = theta(1);
           th2 = theta(2);
           x = obj.L1*cos(th1) + obj.L2*cos(th1 + th2);
           y = obj.L1*sin(th1) + obj.L2*sin(th1 + th2);
       end
       %---------------------------------------------------------
       function theta = inverseKinematics(obj, x, y)
           L1 = obj.L1; L2 = obj.L2;
           c2 = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);
           c2 = min(max(c2, -1), 1);           
           s2 =  sqrt(1 - c2^2);               
           theta2 = atan2(s2, c2);
           k1 = L1 + L2*c2;
           k2 = L2*s2;
           theta1 = atan2(y, x) - atan2(k2, k1);
           theta = [theta1; theta2];
       end
       %---------------------------------------------------------
       function M = massMatrix(obj, theta)
           th2 = theta(2);
           L1 = obj.L1; L2 = obj.L2;
           m1 = obj.m1; m2 = obj.m2;
           M11 = m1*L1^2 + m2*(L1^2 + L2^2 + 2*L1*L2*cos(th2));
           M12 = m2*(L2^2 + L1*L2*cos(th2));
           M21 = M12;
           M22 = m2*L2^2;
           M = [M11 M12; M21 M22];
       end
       %---------------------------------------------------------
       function C = coriolisMatrix(obj, theta, theta_dot)
           th2 = theta(2);
           L1 = obj.L1; L2 = obj.L2; m2 = obj.m2;
           h = m2 * L1 * L2 * sin(th2);
           C11 = -h * theta_dot(2);
           C12 = -h * (theta_dot(1) + theta_dot(2));
           C21 =  h * theta_dot(1);
           C22 =  0;
           C = [C11 C12; C21 C22];
       end
       %---------------------------------------------------------
       function G = gravityVector(obj, theta)
           % Gravity neglected
           G = [0; 0];
       end
       %---------------------------------------------------------
       function F = frictionTorque(obj, theta_dot)
     
           F = obj.D * theta_dot;
       end
       %---------------------------------------------------------
       function J = jacobian(obj, theta)
           th1 = theta(1);
           th2 = theta(2);
           L1  = obj.L1;
           L2  = obj.L2;
           J11 = -L1*sin(th1) - L2*sin(th1 + th2);
           J12 = -L2*sin(th1 + th2);
           J21 =  L1*cos(th1) + L2*cos(th1 + th2);
           J22 =  L2*cos(th1 + th2);
           J = [J11 J12; J21 J22];
       end
   end
end

Simulation File
% force_control_sim.m
% Pure Force Control for 2-DOF RR Arm on wall x + y = 2
clear; clc; close all;
%=============================================================
% 1. Robot Parameters
%=============================================================
L1 = 1; L2 = 1;
m1 = 3; m2 = 3;
g  = 0;
D1 = 0.1; D2 = 0.1;
robot_obj = robot(L1, L2, m1, m2, g, D1, D2);
%=============================================================
% 2. Environment & Force Specs
%=============================================================
K_env = 1000;
F_des = 10;
n_hat  = [1;1]/sqrt(2);
p_wall = [2;0];
n_wall = dot(n_hat, p_wall);
d_pen_des = F_des / K_env;
n_des = n_wall + d_pen_des;
%=============================================================
% 3. Simulation Settings
%=============================================================
dt = 0.001; t_end = 5;
t = 0:dt:t_end; N = length(t);
theta_history     = zeros(2,N);
theta_dot_history = zeros(2,N);
F_into_history    = zeros(1,N);
n_history         = zeros(1,N);
x_history         = zeros(1,N);
y_history         = zeros(1,N);
%=============================================================
% 4. Initial Condition
%=============================================================
p0 = [1.5;0.5];
theta = robot_obj.inverseKinematics(p0(1), p0(2));
theta_dot = [0;0];
theta_history(:,1)     = theta;
theta_dot_history(:,1) = theta_dot;
[x0,y0] = robot_obj.forwardKinematics(theta);
n_prev = dot(n_hat, [x0;y0]);
%=============================================================
% 5. Normal-Direction PD Gains
%=============================================================
Kp_n = 800;
Kd_n = 40;
%=============================================================
% 6. Simulation Loop
%=============================================================
for k = 1:N
   [x,y] = robot_obj.forwardKinematics(theta);
   p = [x;y];
   % Normal projection
   n_curr = dot(n_hat, p);
   d_pen = max(0, n_curr - n_wall);
   % Wall force (robot pushing into wall = positive)
   F_env_n = -K_env * d_pen;
   F_into  = -F_env_n;
   % Save histories
   x_history(k) = x;
   y_history(k) = y;
   n_history(k) = n_curr;
   F_into_history(k) = F_into;
   theta_history(:,k) = theta;
   theta_dot_history(:,k) = theta_dot;
   % Normal velocity
   if k == 1, n_dot = 0;
   else, n_dot = (n_curr - n_prev)/dt;
   end
   n_prev = n_curr;
   % PD control in normal direction
   e_n     = n_des - n_curr;
   e_n_dot = -n_dot;
   F_cmd_n = Kp_n*e_n + Kd_n*e_n_dot;
   % Cartesian control force
   F_cmd_vec = F_cmd_n * n_hat;
   % Joint torques
   J = robot_obj.jacobian(theta);
   tau = J' * F_cmd_vec;
   % Dynamics
   M = robot_obj.massMatrix(theta);
   C = robot_obj.coriolisMatrix(theta, theta_dot);
   G = robot_obj.gravityVector(theta);
   F_fric = robot_obj.frictionTorque(theta_dot);
   theta_ddot = M \ (tau - C*theta_dot - G - F_fric);
   theta_dot = theta_dot + theta_ddot*dt;
   theta     = theta     + theta_dot*dt;
   % Visualization
   if mod(k,200)==0
       [x1,y1,x2,y2] = getArmXY(theta, L1, L2);
       figure(30); clf;
       plot([0 x1 x2],[0 y1 y2],'b-o','LineWidth',2); hold on;
       fplot(@(xline) 2-xline, [0 2], 'k--');
       axis equal; grid on;
       xlim([-0.2 2]); ylim([-0.2 2]);
       title(sprintf('Force Control: t = %.2f s', t(k)));
       drawnow;
   end
end
%=============================================================
% 7. Required Plots
%=============================================================
% Force vs time
figure('Name','Normal Force vs Time');
plot(t, F_into_history,'b-','LineWidth',1.5); hold on;
yline(F_des,'r--','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Force into Wall');
title('Normal Force vs Time');
grid on;
% Trajectory
figure('Name','EE Trajectory');
plot(x_history,y_history,'b-','LineWidth',1.5); hold on;
fplot(@(xline) 2-xline, [0 2], 'k--');
xlabel('X'); ylabel('Y');
title('End-Effector Trajectory');
grid on; axis equal;
% Zoomed trajectory
figure('Name','Zoomed Trajectory');
plot(x_history,y_history,'b-','LineWidth',1.5); hold on;
fplot(@(xline) 2-xline,[0 2],'k--');
xlabel('X'); ylabel('Y');
title('Zoomed View Near Wall');
grid on; axis equal;
xlim([0.5 1.7]); ylim([0.2 1.8]);
% Snapshots
snapshot_times = [0,1,2,3,4];
figure('Name','Manipulator Snapshots'); hold on;
colors = lines(length(snapshot_times));
for k = 1:length(snapshot_times)
   [~, idx] = min(abs(t - snapshot_times(k)));
   th = theta_history(:,idx);
   [x1,y1,x2,y2] = getArmXY(th, L1, L2);
   plot([0 x1 x2],[0 y1 y2],'Color',colors(k,:),'LineWidth',2);
end
fplot(@(xline) 2-xline,[0 2],'k--');
axis equal; grid on;
xlabel('X'); ylabel('Y');
title('Manipulator Snapshots');
legend('t=0','t=1','t=2','t=3','t=4','Wall');
%=============================================================
% Helper
%=============================================================
function [x1, y1, x2, y2] = getArmXY(theta, L1, L2)
th1 = theta(1); th2 = theta(2);
x1 = L1*cos(th1); y1 = L1*sin(th1);
x2 = x1 + L2*cos(th1 + th2);
y2 = y1 + L2*sin(th1 + th2);
end

Results



EE goes a little inside the wall due to it being modeled as a spring 
To be specific, to produce 10N of force with k = 1000, the EE has to go 
F = kx
X = F/k = 10/1000 = 0.01 units 
inside the wall - which is reflected in the plot above

 


Question C - Hybrid Control
Code
Robot Definition (Unchanged)
% robot.m
classdef robot
   properties
       L1   % Link 1 length
       L2   % Link 2 length
       m1   % Link 1 mass
       m2   % Link 2 mass
       g    % gravity (THIS IS SET TO 0)
       D    % friction matrix (2x2 diagonal)
   end
   methods
       %---------------------------------------------------------
       function obj = robot(L1, L2, m1, m2, g, D1, D2)
           % Constructor
           if nargin < 5
               g =0;
           end
           if nargin < 7
               D1 = 0.1;
               D2 = 0.1;
           end
           obj.L1 = L1;
           obj.L2 = L2;
           obj.m1 = m1;
           obj.m2 = m2;
           obj.g  = g;
           obj.D  = diag([D1, D2]);
       end
       %---------------------------------------------------------
       function [x, y] = forwardKinematics(obj, theta)
           % theta = [theta1; theta2]
           th1 = theta(1);
           th2 = theta(2);
           x = obj.L1*cos(th1) + obj.L2*cos(th1 + th2);
           y = obj.L1*sin(th1) + obj.L2*sin(th1 + th2);
       end
       %---------------------------------------------------------
       function theta = inverseKinematics(obj, x, y)
           % Elbow-up IK for 2-link planar arm
           L1 = obj.L1; L2 = obj.L2;
           c2 = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);
           c2 = min(max(c2, -1), 1);           % clamp
           s2 =  sqrt(1 - c2^2);               % elbow-up
           theta2 = atan2(s2, c2);
           k1 = L1 + L2*c2;
           k2 = L2*s2;
           theta1 = atan2(y, x) - atan2(k2, k1);
           theta = [theta1; theta2];
       end
       %---------------------------------------------------------
       function M = massMatrix(obj, theta)
           th2 = theta(2);
           L1 = obj.L1; L2 = obj.L2;
           m1 = obj.m1; m2 = obj.m2;
           M11 = m1*L1^2 + m2*(L1^2 + L2^2 + 2*L1*L2*cos(th2));
           M12 = m2*(L2^2 + L1*L2*cos(th2));
           M21 = M12;
           M22 = m2*L2^2;
           M = [M11 M12; M21 M22];
       end
       %---------------------------------------------------------
       function C = coriolisMatrix(obj, theta, theta_dot)
           th2 = theta(2);
           L1 = obj.L1; L2 = obj.L2; m2 = obj.m2;
           h = m2 * L1 * L2 * sin(th2);
           C11 = -h * theta_dot(2);
           C12 = -h * (theta_dot(1) + theta_dot(2));
           C21 =  h * theta_dot(1);
           C22 =  0;
           C = [C11 C12; C21 C22];
       end
       %---------------------------------------------------------
       function G = gravityVector(obj, theta)
           % Gravity neglected for this project
           G = [0; 0];
       end
       %---------------------------------------------------------
       function F = frictionTorque(obj, theta_dot)
           % Viscous friction
           F = obj.D * theta_dot;
       end
       %---------------------------------------------------------
       function J = jacobian(obj, theta)
           th1 = theta(1);
           th2 = theta(2);
           L1  = obj.L1;
           L2  = obj.L2;
           J11 = -L1*sin(th1) - L2*sin(th1 + th2);
           J12 = -L2*sin(th1 + th2);
           J21 =  L1*cos(th1) + L2*cos(th1 + th2);
           J22 =  L2*cos(th1 + th2);
           J = [J11 J12; J21 J22];
       end
   end
end

Simulation File
% hybrid_control_sim.m

clear; clc; close all;
%=============================================================
%  Robot Parameters
%=============================================================
L1 = 1; L2 = 1;
m1 = 3; m2 = 3;
g  = 0;
D1 = 0.1; D2 = 0.1;
robot_obj = robot(L1, L2, m1, m2, g, D1, D2);
%=============================================================
% Wall and Force Specs
%=============================================================
K_env = 1000;
F_des = 10;
n_hat  = [1;1]/sqrt(2);
p_wall = [2;0];
n_wall = dot(n_hat, p_wall);
d_pen_des = F_des / K_env;
n_des = n_wall + d_pen_des;
%=============================================================
% Tangential Direction & Trajectory
%=============================================================
p1 = [1.5;0.5];
p2 = [0.5;1.5];
t_hat = (p2 - p1) / norm(p2 - p1);
s0 = 0;
sf = norm(p2 - p1);
T_traj = 5;
%=============================================================
%  Simulation Settings
%=============================================================
dt = 0.001;
t_end = 6;
t = 0:dt:t_end;
N = length(t);
%=============================================================
% Gains
%=============================================================
Kp_n = 800; Kd_n = 40;
Kp_s = 300; Kd_s = 50;
%=============================================================
%  Storage
%=============================================================
theta_history     = zeros(2,N);
theta_dot_history = zeros(2,N);
x_history         = zeros(1,N);
y_history         = zeros(1,N);
n_history         = zeros(1,N);
s_history         = zeros(1,N);
s_des_history     = zeros(1,N);
F_into_history    = zeros(1,N);
%=============================================================
%  Initial Condition
%=============================================================
theta = robot_obj.inverseKinematics(p1(1), p1(2));
theta_dot = [0;0];
[x0,y0] = robot_obj.forwardKinematics(theta);
p0 = [x0;y0];
n0 = dot(n_hat, p0);
s0_actual = dot(t_hat, (p0 - p1));
n_prev = n0;
s_prev = s0_actual;
theta_history(:,1) = theta;
theta_dot_history(:,1) = theta_dot;
x_history(1) = x0;
y_history(1) = y0;
n_history(1) = n0;
s_history(1) = s0_actual;
%=============================================================
%  Simulation Loop
%=============================================================
for k = 1:N
   tk = t(k);
   %----------------------- Desired s(t) -----------------------
   if tk <= T_traj
       tau = tk/T_traj;
       sigma = 10*tau^3 - 15*tau^4 + 6*tau^5;
       sigma_dot = (30*tau^2 - 60*tau^3 + 30*tau^4)/T_traj;
       s_des = s0 + (sf-s0)*sigma;
       s_dot_des = (sf-s0)*sigma_dot;
   else
       s_des = sf;
       s_dot_des = 0;
   end
   s_des_history(k) = s_des;
   % desired point on wall
   p_des_on_wall = p1 + t_hat*s_des;
   %----------------------- Actual p, n, s -----------------------
   [x,y] = robot_obj.forwardKinematics(theta);
   p = [x;y];
   x_history(k) = x;
   y_history(k) = y;
   n_curr = dot(n_hat, p);
   d_pen  = max(0, n_curr - n_wall);
   F_env_n = -K_env*d_pen;
   F_into  = -F_env_n;
   s_curr = dot(t_hat, (p - p1));
   n_history(k) = n_curr;
   s_history(k) = s_curr;
   F_into_history(k) = F_into;
   theta_history(:,k) = theta;
   theta_dot_history(:,k) = theta_dot;
   % velocities
   if k == 1
       n_dot = 0; s_dot = 0;
   else
       n_dot = (n_curr - n_prev)/dt;
       s_dot = (s_curr - s_prev)/dt;
   end
   n_prev = n_curr;
   s_prev = s_curr;
   %----------------------- Hybrid Control -----------------------
   % normal direction (force regulation)
   e_n = n_des - n_curr;
   e_n_dot = -n_dot;
   F_cmd_n = Kp_n*e_n + Kd_n*e_n_dot;
   % tangential direction (trajectory tracking)
   e_s = s_des - s_curr;
   e_s_dot = s_dot_des - s_dot;
   F_cmd_s = Kp_s*e_s + Kd_s*e_s_dot;
   % commanded Cartesian force
   F_cmd_vec = F_cmd_n*n_hat + F_cmd_s*t_hat;
   F_env_vec = F_env_n*n_hat;
   %----------------------- Joint Torques -----------------------
   J = robot_obj.jacobian(theta);
   tau_cmd = J' * F_cmd_vec;
   %----------------------- Dynamics -----------------------
   M = robot_obj.massMatrix(theta);
   C = robot_obj.coriolisMatrix(theta, theta_dot);
   G = robot_obj.gravityVector(theta);
   F_fric = robot_obj.frictionTorque(theta_dot);
   theta_ddot = M \ (tau_cmd - C*theta_dot - G - F_fric);
   theta_dot = theta_dot + theta_ddot*dt;
   theta     = theta     + theta_dot*dt;
   %----------------------- Visualization -----------------------
   if mod(k,500)==0
       [x1,y1,x2,y2] = getArmXY(theta, L1, L2);
       figure(50); clf;
       plot([0 x1 x2],[0 y1 y2],'b-o','LineWidth',2); hold on;
       plot(p_des_on_wall(1), p_des_on_wall(2),'rx','MarkerSize',10,'LineWidth',2);
       fplot(@(xx) 2-xx,[0 2],'k--');
       axis equal; grid on;
       xlim([-0.2 2]); ylim([-0.2 2]);
       title(sprintf('Hybrid Control: t = %.2f s', tk));
       drawnow;
   end
end
%=============================================================
% Plots
%=============================================================
figure; plot(t,x_history,'b','LineWidth',1.5);
xlabel('Time'); ylabel('X'); title('X vs Time'); grid on;
figure; plot(t,y_history,'b','LineWidth',1.5);
xlabel('Time'); ylabel('Y'); title('Y vs Time'); grid on;
figure;
plot(t,s_des_history,'r--','LineWidth',1.5); hold on;
plot(t,s_history,'b','LineWidth',1.5);
xlabel('Time'); ylabel('s');
title('Along-Wall Position'); legend('Desired','Actual'); grid on;
figure;
plot(t,F_into_history,'b','LineWidth',1.5); hold on;
yline(F_des,'r--');
xlabel('Time'); ylabel('Force Into Wall');
title('Normal Force vs Time'); grid on;
figure;
plot(x_history,y_history,'b','LineWidth',1.5); hold on;
fplot(@(xx)2-xx,[0 2],'k--');
xlabel('X'); ylabel('Y');
title('Overhead Trajectory'); axis equal; grid on;
snapshot_times = [0,1.5,3,4.5,6];
figure; hold on;
colors = lines(length(snapshot_times));
for kk = 1:length(snapshot_times)
   [~,idx] = min(abs(t - snapshot_times(kk)));
   [x1,y1,x2,y2] = getArmXY(theta_history(:,idx), L1, L2);
   plot([0 x1 x2],[0 y1 y2],'Color',colors(kk,:),'LineWidth',2);
end
fplot(@(xx)2-xx,[0 2],'k--');
xlabel('X'); ylabel('Y');
title('Manipulator Snapshots'); axis equal; grid on;
%=============================================================
% Helper
%=============================================================
function [x1,y1,x2,y2] = getArmXY(theta,L1,L2)
th1 = theta(1); th2 = theta(2);
x1 = L1*cos(th1); y1 = L1*sin(th1);
x2 = x1 + L2*cos(th1+th2);
y2 = y1 + L2*sin(th1+th2);
end


Results








Question D
Code
Robot Definition (Unchanged)
% robot.m
classdef robot
   properties
       L1   % Link 1 length
       L2   % Link 2 length
       m1   % Link 1 mass
       m2   % Link 2 mass
       g    % gravity 
       D    % friction matrix (2x2 diagonal)
   end
   methods
       %---------------------------------------------------------
       function obj = robot(L1, L2, m1, m2, g, D1, D2)
           % Constructor
           if nargin < 5
               g = 0;
           end
           if nargin < 7
               D1 = 0.1;
               D2 = 0.1;
           end
           obj.L1 = L1;
           obj.L2 = L2;
           obj.m1 = m1;
           obj.m2 = m2;
           obj.g  = g;
           obj.D  = diag([D1, D2]);
       end
       %---------------------------------------------------------
       function [x, y] = forwardKinematics(obj, theta)
           % theta = [theta1; theta2]
           th1 = theta(1);
           th2 = theta(2);
           x = obj.L1*cos(th1) + obj.L2*cos(th1 + th2);
           y = obj.L1*sin(th1) + obj.L2*sin(th1 + th2);
       end
       %---------------------------------------------------------
       function theta = inverseKinematics(obj, x, y)
           % Elbow-up IK for 2-link planar arm
           L1 = obj.L1; L2 = obj.L2;
           c2 = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);
           c2 = min(max(c2, -1), 1);           % clamp
           s2 =  sqrt(1 - c2^2);               % elbow-up
           theta2 = atan2(s2, c2);
           k1 = L1 + L2*c2;
           k2 = L2*s2;
           theta1 = atan2(y, x) - atan2(k2, k1);
           theta = [theta1; theta2];
       end
       %---------------------------------------------------------
       function M = massMatrix(obj, theta)
           th2 = theta(2);
           L1 = obj.L1; L2 = obj.L2;
           m1 = obj.m1; m2 = obj.m2;
           M11 = m1*L1^2 + m2*(L1^2 + L2^2 + 2*L1*L2*cos(th2));
           M12 = m2*(L2^2 + L1*L2*cos(th2));
           M21 = M12;
           M22 = m2*L2^2;
           M = [M11 M12; M21 M22];
       end
       %---------------------------------------------------------
       function C = coriolisMatrix(obj, theta, theta_dot)
           th2 = theta(2);
           L1 = obj.L1; L2 = obj.L2; m2 = obj.m2;
           h = m2 * L1 * L2 * sin(th2);
           C11 = -h * theta_dot(2);
           C12 = -h * (theta_dot(1) + theta_dot(2));
           C21 =  h * theta_dot(1);
           C22 =  0;
           C = [C11 C12; C21 C22];
       end
       %---------------------------------------------------------
       function G = gravityVector(obj, theta)
           % Gravity neglected for this project
           G = [0; 0];
       end
       %---------------------------------------------------------
       function F = frictionTorque(obj, theta_dot)
           % Viscous friction
           F = obj.D * theta_dot;
       end
       %---------------------------------------------------------
       function J = jacobian(obj, theta)
           th1 = theta(1);
           th2 = theta(2);
           L1  = obj.L1;
           L2  = obj.L2;
           J11 = -L1*sin(th1) - L2*sin(th1 + th2);
           J12 = -L2*sin(th1 + th2);
           J21 =  L1*cos(th1) + L2*cos(th1 + th2);
           J22 =  L2*cos(th1 + th2);
           J = [J11 J12; J21 J22];
       end
   end
end

Simulation
% simulation_D.m
clear; clc; close all;
%=============================================================
% Robot Parameters
%=============================================================
L1 = 1; L2 = 1;
m1 = 3; m2 = 3;
g  = 0;
D1 = 0.1; D2 = 0.1;
robot_obj = robot(L1, L2, m1, m2, g, D1, D2);
%=============================================================
% Environment
%=============================================================
K_env = 1000;
F_des = 10;
% Wall 1: x + y = 2
n1_hat  = [1;1]/sqrt(2);
p1_wall = [2;0];
n1_wall = dot(n1_hat, p1_wall);
p1_start = [1.5;0.5];
p_corner = [0.25;1.75];
t1_hat   = (p_corner - p1_start);
t1_hat   = t1_hat / norm(t1_hat);
% Wall 2: through [0.5,2] and [-0.5,1]
p2_wall = [0.5;2];
p2_end  = [-0.5;1];
t2_hat  = (p2_end - p_corner);
t2_hat  = t2_hat / norm(t2_hat);
n2_hat  = [1;-1]/sqrt(2);
n2_wall = dot(n2_hat, p2_wall);
%=============================================================
% Wall Trajectories (s-coordinates)
%=============================================================
s1_0 = 0;  s1_f = norm(p_corner - p1_start);  T1 = 5;
s2_0 = 0;  s2_f = norm(p2_end - p_corner);    T2 = 5;
t_end = T1 + T2 + 1;
dt = 0.001;
t  = 0:dt:t_end; 
N  = length(t);
%=============================================================
% Gains
%=============================================================
Kp_n = 1000;   Kd_n = 80;
Kp_s = 300;    Kd_s = 60;
d_pen_des = F_des / K_env;
%=============================================================
% Storage
%=============================================================
theta_history     = zeros(2,N);
theta_dot_history = zeros(2,N);
x_history         = zeros(1,N);
y_history         = zeros(1,N);
F_into_history    = zeros(1,N);
n_history         = zeros(1,N);
s_rel_history     = zeros(1,N);
s_rel_des_history = zeros(1,N);
mode_history      = zeros(1,N);
%=============================================================
% Initial Condition (start on Wall 1)
%=============================================================
theta = robot_obj.inverseKinematics(p1_start(1), p1_start(2));
theta_dot = [0;0];
[x0,y0] = robot_obj.forwardKinematics(theta);
p0 = [x0;y0];
n_curr = dot(n1_hat,p0);
s_curr = dot(t1_hat,(p0 - p1_start));
n_prev = n_curr;
s_prev = s_curr;
mode_prev = 1;
theta_history(:,1) = theta;
theta_dot_history(:,1) = theta_dot;
x_history(1) = x0;
y_history(1) = y0;
n_history(1) = n_curr;
s_rel_history(1) = s_curr;
s_rel_des_history(1) = 0;
mode_history(1) = 1;
%=============================================================
% Simulation Loop
%=============================================================
for k = 1:N
   tk = t(k);
   %------------------ Mode selection ------------------
   if tk <= T1
       mode = 1;
   else
       mode = 2;
   end
   %------------------ Surface switching ------------------
   if mode == 2 && mode_prev == 1
       [x_sw,y_sw] = robot_obj.forwardKinematics(theta);
       p_sw = [x_sw;y_sw];
       n2_curr = dot(n2_hat,p_sw);
       dist2   = n2_curr - n2_wall;
       p_proj = p_sw - dist2*n2_hat;
       theta  = robot_obj.inverseKinematics(p_proj(1), p_proj(2));
       [x,y] = robot_obj.forwardKinematics(theta);
       p = [x;y];
       n_curr = dot(n2_hat,p);
       s_curr = dot(t2_hat,(p - p_corner));
       n_prev = n_curr;
       s_prev = s_curr;
       n_dot = 0;
       s_dot = 0;
   end
   [x,y] = robot_obj.forwardKinematics(theta);
   p = [x;y];
   x_history(k) = x;
   y_history(k) = y;
   mode_history(k) = mode;
   %------------------ Geometry for active wall ------------------
   if mode == 1
       n_hat = n1_hat; t_hat = t1_hat; n_wall = n1_wall; p_ref = p1_start;
       tau_traj = min(max(tk/T1,0),1);
       s0 = s1_0; sf = s1_f;
   else
       n_hat = n2_hat; t_hat = t2_hat; n_wall = n2_wall; p_ref = p_corner;
       tau_traj = min(max((tk-T1)/T2,0),1);
       s0 = s2_0; sf = s2_f;
   end
   n_curr = dot(n_hat,p);
   d_pen  = max(0,n_curr - n_wall);
   s_curr = dot(t_hat,(p - p_ref));
   %------------------ Velocities ------------------
   if k == 1 || mode ~= mode_prev
       n_dot = 0; s_dot = 0;
   else
       n_dot = (n_curr - n_prev)/dt;
       s_dot = (s_curr - s_prev)/dt;
   end
   n_prev = n_curr;
   s_prev = s_curr;
   mode_prev = mode;
   %------------------ Desired s(t) ------------------
   sigma     = 10*tau_traj^3 - 15*tau_traj^4 + 6*tau_traj^5;
   sigma_dot = 30*tau_traj^2 - 60*tau_traj^3 + 30*tau_traj^4;
   s_des = s0 + (sf - s0)*sigma;
   s_dot_des = (sf - s0)*sigma_dot / (mode==1)*T1 + (mode==2)*T2;
   s_rel_history(k) = s_curr;
   s_rel_des_history(k) = s_des;
   %------------------ Normal control ------------------
   n_des = n_wall + d_pen_des;
   e_n = n_des - n_curr;
   e_n_dot = -n_dot;
   F_cmd_n = Kp_n*e_n + Kd_n*e_n_dot;
   %------------------ Tangential control ------------------
   e_s = s_des - s_curr;
   e_s_dot = s_dot_des - s_dot;
   F_cmd_s = Kp_s*e_s + Kd_s*e_s_dot;
   F_cmd_vec = F_cmd_n*n_hat + F_cmd_s*t_hat;
   F_env_n = -K_env*d_pen;
   F_into_history(k) = -F_env_n;
   n_history(k) = n_curr;
   %------------------ Dynamics ------------------
   J = robot_obj.jacobian(theta);
   tau_cmd = J' * F_cmd_vec;
   M = robot_obj.massMatrix(theta);
   C = robot_obj.coriolisMatrix(theta, theta_dot);
   G = robot_obj.gravityVector(theta);
   F_fric = robot_obj.frictionTorque(theta_dot);
   theta_ddot = M \ (tau_cmd - C*theta_dot - G - F_fric);
   theta_dot = theta_dot + theta_ddot*dt;
   theta     = theta     + theta_dot*dt;
   theta_history(:,k) = theta;
   theta_dot_history(:,k) = theta_dot;
end
%=============================================================
% 8. Plots
%=============================================================
figure; plot(t,x_history,'b','LineWidth',1.5);
xlabel('Time'); ylabel('X'); title('X vs Time'); grid on;
figure; plot(t,y_history,'b','LineWidth',1.5);
xlabel('Time'); ylabel('Y'); title('Y vs Time'); grid on;
figure; plot(t,s_rel_des_history,'r--','LineWidth',1.5); hold on;
plot(t,s_rel_history,'b','LineWidth',1.5);
xlabel('Time'); ylabel('s'); title('Along-Wall Position'); grid on;
figure; plot(t,F_into_history,'b','LineWidth',1.5); hold on;
yline(F_des,'r--'); xlabel('Time'); ylabel('Force');
title('Normal Force Into Wall'); grid on;
figure; hold on;
plot(x_history,y_history,'b','LineWidth',1.5);
fplot(@(x)2-x,[-1 2.5],'k--');
fplot(@(x)x+1.5,[-2.5 2],'k-.');
plot(p1_start(1),p1_start(2),'go','MarkerFaceColor','g');
plot(p_corner(1),p_corner(2),'ro','MarkerFaceColor','r');
plot(p2_end(1),p2_end(2),'mo','MarkerFaceColor','m');
axis equal; grid on;
title('Overhead Trajectory with Switching');
% Snapshots
snapshot_times = [0,3,5,7,9];
figure; hold on;
colors = lines(length(snapshot_times));
for kk = 1:length(snapshot_times)
   [~,idx] = min(abs(t - snapshot_times(kk)));
   [x1,y1,x2,y2] = getArmXY(theta_history(:,idx),L1,L2);
   plot([0 x1 x2],[0 y1 y2],'Color',colors(kk,:),'LineWidth',2);
end
fplot(@(x)2-x,[-1 2.5],'k--');
fplot(@(x)x+1.5,[-2.5 2],'k-.');
plot(p_corner(1),p_corner(2),'ro','MarkerFaceColor','r');
axis equal; grid on;
title('Manipulator Snapshots');
%=============================================================
% Helper
%=============================================================
function [x1,y1,x2,y2] = getArmXY(theta,L1,L2)
th1 = theta(1); th2 = theta(2);
x1 = L1*cos(th1); y1 = L1*sin(th1);
x2 = x1 + L2*cos(th1+th2);
y2 = y1 + L2*sin(th1+th2);
end

Results









