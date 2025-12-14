% robot.m
% 2-DOF Planar RR Manipulator (X-Y plane) with friction

classdef robot
    properties
        L1   % Link 1 length
        L2   % Link 2 length
        m1   % Link 1 mass
        m2   % Link 2 mass
        g    % gravity (SET TO 0 LATER!!) 
        D    % friction matrix  
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
            % IK for 2-link planar arm
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
