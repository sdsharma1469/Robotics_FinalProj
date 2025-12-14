% tau_controller.m
% Computed Torque Controller for 2-DOF Planar RR Manipulator
% Implements PID control using the computed-torque (inverse dynamics) method

function tau = tau_controller(desired, actual, gains, integral_error, robot_obj, S)
% -------------------------------------------------------------------------
% Inputs:
%   desired :
%       .theta        Desired joint angles (2x1)
%       .theta_dot    Desired joint velocities (2x1)
%       .theta_ddot   Desired joint accelerations (2x1)
%
%   actual :
%       .theta        Actual joint angles (2x1)
%       .theta_dot    Actual joint velocities (2x1)
%
%   gains :
%       .Kp   Proportional gain matrix (2x2)
%       .Kd   Derivative gain matrix (2x2)
%       .Ki   Integral gain matrix (2x2)
%
%   integral_error : Integral of joint errors (2x1)
%
%   robot_obj : Instance of robot class (dynamics)
%   S         : Switching matrix (2x2), for this assignment S = I
%
% Output:
%   tau : Control torques [tau1; tau2]
% -------------------------------------------------------------------------

%% 1. Compute position and velocity errors
pos_error = desired.theta     - actual.theta;      % e_theta
vel_error = desired.theta_dot - actual.theta_dot;  % e_theta_dot

%% 2. PID control terms (with switching matrix)
F_p = gains.Kp * pos_error;        % Proportional term
F_d = gains.Kd * vel_error;        % Derivative term
F_i = gains.Ki * integral_error;   % Integral term

% Apply switching matrix S (identity for this assignment)
F_p = S * F_p;
F_d = S * F_d;
F_i = S * F_i;

%% 3. Compute commanded acceleration (alpha)
alpha = desired.theta_ddot + F_d + F_p + F_i;
% alpha = θ̈_des + Kd(θ̇_des - θ̇) + Kp(θ_des - θ) + Ki∫e dt

%% 4. Get robot dynamic terms at actual state
M = robot_obj.massMatrix(actual.theta);                 % Mass matrix
C = robot_obj.coriolisMatrix(actual.theta, actual.theta_dot);  % Coriolis matrix
G = robot_obj.gravityVector(actual.theta);              % Gravity vector
F = robot_obj.frictionTorque(actual.theta_dot);         % Friction torque

%% 5. Compute control torque using inverse dynamics
tau = M * alpha + C * actual.theta_dot + G + F;

% -------------------------------------------------------------------------
% Notes:
%   - Computed torque uses: τ = M(q)·α + C(q, q̇)q̇ + G(q) + F(q̇)
%   - where α includes PID feedback shaping.
%   - S is the switching matrix; here S = I (identity).
% -------------------------------------------------------------------------
end
