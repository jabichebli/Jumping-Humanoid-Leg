%------------------------------------------------------------------------
%------------------------------------------------------------------------
% MECENG239 Final Project - Hopping Leg  (+ Front Flip as extension)
% Course: ME239: Robotic Locomotion
% Semester: Fall 2025
%------------------------------------------------------------------------
% By: Jason Abi Chebli
% Student ID: 3042017306
% Last Modified: 2025-09-17
% The following is the MATLAB code used to determine the dynamics of a
% robotic leg jumping up and down. 
%------------------------------------------------------------------------
%------------------------------------------------------------------------

clear all; close all; clc;

% -------------------------------------------------------------------------
% ----------------- Calculating Dynamics (Symbollically) ------------------
% -------------------------------------------------------------------------

% ---------------------------- Symbolic Setup -----------------------------
% Define Symbolic Variables for 2-link leg system
syms x y q1 q2 q3 real
syms xdot ydot q1dot q2dot q3dot real
syms xddot yddot q1ddot q2ddot q3ddot real
syms m1 I1 l1 d1 m2 I2 l2 d2 m3 I3 l3 d3 g real
syms x_hip y_hip real

Pi = sym(pi);

% Generalized coords and rates
q  = [x; y; q1; q2; q3];
dq = [xdot; ydot; q1dot; q2dot; q3dot];
ddq = [xddot; yddot; q1ddot; q2ddot; q3ddot];

% -------------------------- Forward kinematics ---------------------------
% COM of link1 (hip->knee)
p1 = [ x + d1*(-sin(q1));
       y + d1*(-cos(q1)) ];

% Knee position
pknee = [ x + l1*(-sin(q1));
          y + l1*(-cos(q1)) ];

% COM of link2 (knee->foot)
% p2 = [ x + l1*sin(q1) + d2*sin(q1 + q2);
%        y + l1*cos(q1) + d2*cos(q1 + q2) ];
p2 = [ x + l1*(-sin(q1)) + d2*sin(q2 - q1);
          y + l1*(-sin(q1)) + d2*(-cos(q2 - q1)) ] ;

% Foot (stance) position
pfoot = [ x + l1*(-sin(q1)) + l2*sin(q2 - q1);
          y + l1*(-cos(q1)) + l2*(-cos(q2 - q1)) ] ;

pfoot_y = pfoot(2) ;

matlabFunction(pfoot_y, 'File', 'auto_pfoot_y');

p3 = [x + d3 * (-sin(q3));
      y + d3 * cos(q3)] ;

phip = [x + l3 * (-sin(q3));
        y + l3 * cos(q3)] ;


% Total System COM
pCOM = [ (m1 * p1(1) + m2 * p2(1) + m3 * p3(1))/(m1 + m2 + m3);
         (m1 * p1(2) + m2 * p2(2) + m3 * p3(2))/(m1 + m2 + m3)];

matlabFunction(pCOM, 'File', 'auto_pCOM');

% ----------------------------- Velocities --------------------------------
% linear velocities of COMs calcualted using Jacobian: v = J_q(p) * dq
Jp1 = jacobian(p1, q);    % 2x4
Jp2 = jacobian(p2, q);
Jp3 = jacobian(p3, q);

v1  = simplify(Jp1 * dq); % 2x1
v2  = simplify(Jp2 * dq);
v3  = simplify(Jp3 * dq);

% angular velocities definition
omega3 = q3dot;
omega1 = -q1dot;
omega2 = -(q2dot - q1dot);


% --------------------- Kinetic and potential energy ----------------------
% Note for potential and kinetic energy we need to use position and
% velocity of COM of each link.

% kinetic energy
T1 = simplify( (1/2)*m1*(v1.'*v1) + (1/2)*I1*omega1^2 );
T2 = simplify( (1/2)*m2*(v2.'*v2) + (1/2)*I2*omega2^2 );
T3 = simplify( (1/2)*m3*(v3.'*v3) + (1/2)*I3*omega3^2 );
T  = simplify( T1 + T2 + T3);

% potential energy
U1 = m1 * g * p1(2); % p1(2) is the y-component (height)
U2 = m2 * g * p2(2);
U3 = m3 * g * p3(2);
U  = simplify( U1 + U2 + U3 );

% ----------- Stance Jacobian and Stance Jacobian Derivative --------------
% This is needed for dynamic equation with constraints 

% Stance Jacobian
Jst = jacobian(pfoot, q);
Jstdot = jacobian(jacobian(pfoot, q) * dq, q) * dq ;


% ------------------------- Dynamics Equations ----------------------------
q_act = [q1; q2]; % The hip angle and knee angle are being actuated

% Flight dynamics (no contact)
[D, C, G, B] = LagrangianDynamics(T, U, q, dq, q_act);   

matlabFunction(D, 'File', 'auto_D');
matlabFunction(C, 'File', 'auto_C');
matlabFunction(G, 'File', 'auto_G');
matlabFunction(B, 'File', 'auto_B');
matlabFunction(Jst, 'File', 'auto_Jst');
matlabFunction(Jstdot, 'File', 'auto_Jstdot');


% Define Virtual Constraints
% 2) Virtual constraint: CoM above foot (horizontal alignment)

COM_y_desired = 0.9;

h = [pCOM(1) - pfoot(1);
    pCOM(2) - COM_y_desired] ;% output to regulate
dh_dq = jacobian(h, q);
d2h__ = [jacobian(dh_dq*dq, q),   dh_dq] ;

% First derivative
Lfh = simplify(dh_dq * dq);

% Store symbolic forms
matlabFunction(h, 'File', 'auto_h');
matlabFunction(dh_dq, 'File', 'auto_dh_dq');
matlabFunction(d2h__, 'File', 'auto_d2h__');
matlabFunction(Lfh, 'File', 'auto_Lfh');


% % y = h(q) = h_0(q) - h_d(theta(q))
% h = pCOM(1) - pfoot(1) ;
% dh_dq = jacobian(h, q) ;
% d2h__ = [jacobian(dh_dq*dq, q),   dh_dq] ;
% matlabFunction(h, 'File', 'auto_h')
% matlabFunction(dh_dq, 'File', 'auto_dh_dq')
% matlabFunction(d2h__, 'File', 'auto_d2h__')




% % Stance dynamics (foot fixed)
% [ddq_st, Fst] = stance_dynamics(q, dq, U, D, C, G, B, Jst, Jstdot);

% % Impact dynamics (touchdown)
% [dq_plus, Lambda] = impact_dynamics(q, dq_minus, D, Jst);




% -------------------------------------------------------------------------
% ---------------------------- Path Trajectory ----------------------------
% -------------------------------------------------------------------------


% % Time
% t_max = 4; 
% dt = 0.01;
% time = 0:dt:t_max;
% 
% % Phase durations
% t0 = 1.0;   % Standing idle
% t1 = 1.0;   % Crouching
% t2 = 0.8;   % push-off
% t3 = 0.7;   % landing fall
% t4 = 0.25;  % impact compression
% t5 = t_max - (t0+t1+t2+t3+t4); % absorption back
% 
% % ----------------- Determine the y position over time -------------------- 
% 
% % Heights
% y0 = 0.2;           % initial y position of hip
% y_st = 0.2;         % initial stand height
% y_c = 0.15;         % crouch minimum height
% y_max = 0.4;        % jump peak
% y_impact_min = 0.17; % small compression below standing
% 
% % y-height over time
% yhip_traj = zeros(size(time));
% 
% for k = 1:length(time)
%     t = time(k);
%     if t < t0
%         % initial standing
%         yhip_traj(k) = y0;
%     elseif t < t0+t1
%         % crouch phase
%         tau = t - t0;
%         yhip_traj(k) = y0 - (y0 - y_c)*(1 - cos(pi*tau/t1))/2;
%     elseif t < t0+t1+t2
%         % push-off
%         tau = t - (t0+t1);
%         yhip_traj(k) = y_c + (y_max - y_c)*(1 - cos(pi*tau/t2))/2;
%     elseif t < t0+t1+t2+t3
%         % falling
%         tau = t - (t0+t1+t2);
%         yhip_traj(k) = y_max - (y_max - y0)*(1 - cos(pi*tau/t3))/2;
%     elseif t < t0+t1+t2+t3+t4
%         % impact compression
%         tau = t - (t0+t1+t2+t3);
%         yhip_traj(k) = y0 - (y0 - y_impact_min)*(1 - cos(pi*tau/t4))/2;
%     else
%         % absorption / settle
%         tau = t - (t0+t1+t2+t3+t4);
%         yhip_traj(k) = y_impact_min + (y0 - y_impact_min)*(1 - cos(pi*tau/t5))/2;
%     end
% end
% 
% % ------- Determine the x, q1, q2 over time to ensure stability -----------
% 
% % Pre-allocate arrays for the results
% q1_traj = zeros(size(time));
% q2_traj = zeros(size(time));
% xhip_traj = zeros(size(time));
% 
% % Call the generated function once to get all trajectory values
% [xhip_traj, q1_traj, q2_traj] = inv_kin_func(yhip_traj, l1, l2, d1, d2, m1, m2);
% 
% % Plotting the results
% figure;
% plot(time, rad2deg(q1_traj), 'LineWidth', 2);
% hold on;
% plot(time, rad2deg(q2_traj), 'LineWidth', 2);
% xlabel('Time [s]');
% ylabel('Joint Angle [deg]');
% title('Joint Angles from Symbolic Inverse Kinematics');
% legend('q1 (Thigh Angle)', 'q2 (Knee Angle)');
% grid on;
% 
% figure;
% plot(time, xhip_traj, 'LineWidth', 2);
% xlabel('Time [s]');
% ylabel('Hip X Position [m]');
% title('Hip Horizontal Trajectory for COM Stability');
% grid on;





% x_traj = zeros(size(time)); % no horizontal motion
% 
% % Plot path trajectory
% figure;
% plot(time, yhip_traj, 'LineWidth',2);
% xlabel('Time [s]'); ylabel('Hip height y(t) [m]');
% title('Hip trajectory with standing, crouch, jump, landing, impact, and absorption');
% grid on;
% 
% % Determine xdot, ydot
% xdot_traj = [0, diff(x_traj)/dt]; 
% ydot_traj = [0, diff(yhip_traj)/dt];
% 
% % Plot velocity path trajectory
% figure;
% plot(time, ydot_traj, 'LineWidth',2);
% xlabel('Time [s]'); ylabel('ydot(t) [m/s]');
% title('Path Trajectory Derivative');
% grid on;
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% % solve simultaneously for q1, q2
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% ....
% 
% 
% 
% q1 = zeros(size(time));
% q2 = zeros(size(time));
% 
% for k = 1:length(time)
%     x_e = x_traj(k);
%     y_e = yhip_traj(k);
% 
%     r2 = x_e^2 + y_e^2;
%     cos_q2 = (r2 - l1^2 - l2^2) / (2 * l1 * l2);
% 
%     % Clamp to avoid numerical errors (arccos domain)
%     cos_q2 = max(min(cos_q2, 1), -1);
%     q2(k) = acos(cos_q2);  % Elbow-down solution
% 
%     phi = atan2(y_e, x_e);
%     psi = atan2(l2 * sin(q2(k)), l1 + l2 * cos(q2(k)));
% 
%     q1(k) = phi - psi;
% end
% 
% % Plot joint angles
% figure;
% subplot(2,1,1)
% plot(time, rad2deg(q1), 'b', 'LineWidth', 2); hold on;
% plot(time, rad2deg(q2), 'r', 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Angle (deg)');
% legend('q1 (hip to knee)', 'q2 (knee to foot)');
% title('Joint Angles');
% grid on;
% 
% % Determine q1dot, q2dot 
% q1dot = [0, diff(q1)/dt];
% q2dot = [0, diff(q2)/dt];
% 
% % Plot angular velocities
% subplot(2,1,2)
% plot(time, rad2deg(q1dot), 'b--', 'LineWidth', 2); hold on;
% plot(time, rad2deg(q2dot), 'r--', 'LineWidth', 2);
% xlabel('Time (s)');
% ylabel('Angular Velocity (deg/s)');
% legend('q1dot', 'q2dot');
% title('Joint Angular Velocities');
% grid on;
% 
% 
% % -------------------------------------------------------------------------
% % ----------------------- Calculating Dynamics (Real) ---------------------
% % -------------------------------------------------------------------------
% 
% 
% 
% 
% 
% 
% % Set numeric values
% % subsList = {x, y, q1, q2, ...
% %             xdot, ydot, q1dot, q2dot, ...
% %             m1, I1, l1, d1, ...
% %             m2, I2, l2, d2, ...
% %             g};
% % 
% % valsList = {
% %     };
% 
% % Determine Position
% 
% 
% 
% % Determine Velocity
% 
% 
% % Determine Kinetic and Potential Energy 
% 
% 
% % Determine Dynamics Equation
% 
% 
