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
% ------------------------- Dynamics Functions ----------------------------
% -------------------------------------------------------------------------

% ----------------------- During Flight (Unpinned) ------------------------

function [D, C, G, B] = flight_dynamics(T, U, q, dq, q_act)
    % Inputs:
    %   T: Kinetic Energy scalar
    %   U: Potential Energy scalar
    %   q: Generalized coordinates
    %   dq: Time-derivative of the generalized coordinates
    %   q_act: Actuated angles of the system

    % Outputs:
    %   D: D(q) Inertia matrix
    %   C: C(q,dq) Coriollis matrix
    %   G: G(q) Gravity matrix
    %   B: B(q) Input Matrix

    % Compute Inertia Matrix
    D = simplify( jacobian(jacobian(T,dq), dq) ) ;

    % Compute the Coriollis matrix
    for k=1:length(q)
        for j=1:length(q)
            C(k,j) = sym(0) ;
            for i=1:length(q)
                C(k,j) = C(k,j) + 1/2 * ( diff(D(k,j),q(i)) + diff(D(k,i),q(j)) - diff(D(i,j),q(k)) ) * dq(i) ;
            end
        end
    end
    C = simplify(C) ;

    % Compute the gravity vector
    G = simplify( jacobian(U,q) )' ;

    % Compute the input matrix
    B = jacobian(q_act, q)' ;
end

% --------------------- During Stance (Constrained) -----------------------
function [ddq, Fst] = stance_dynamics(q, dq, u, D, C, G, B, Jst, Jstdot)
    % Inputs:
    %   q: Generalized coordinates
    %   dq: Generalized velocities
    %   u: Input torques/forces for actuated joints
    %   D, C, G, B: Flight dynamics matrices
    %   Jst: Stance Jacobian (partial_pfoot/partial_q)
    %   Jstdot: Time derivative of Jst
    %
    % Outputs:
    %   ddq: Generalized accelerations during stance
    %   Fst: Ground reaction forces at stance foot

    % Assemble the block system
    A = [D, -Jst.'; 
         Jst, zeros(size(Jst,1))];

    % Right-hand side
    b = [B*u - C*dq - G; 
         -Jstdot*dq];

    % Solve for [ddq; Fst]
    sol = A \ b;
    ddq = simplify(sol(1:length(q)));
    Fst = simplify(sol(length(q)+1:end));
end


% ---------------------------- During Impact ------------------------------
function [dq_plus, Lambda] = impact_dynamics(q, dq_minus, D, Jst)
    % Inputs:
    %   q: Generalized coordinates
    %   dq_minus: Pre-impact generalized velocities
    %   D: Inertia matrix
    %   Jst: Stance Jacobian (partial_pfoot/partial_q) at impact
    %
    % Outputs:
    %   dq_plus: Post-impact generalized velocities
    %   Lambda: Contact impulse at impact

    % Assemble the block system
    A = [D, -Jst.'; 
         Jst, zeros(size(Jst,1))];

    % Right-hand side
    b = [D*dq_minus; 
         zeros(size(Jst,1),1)];

    % Solve for [dq_plus; Lambda]
    sol = A \ b;
    dq_plus = simplify(sol(1:length(q)));
    Lambda  = simplify(sol(length(q)+1:end));
end

% -------------------------------------------------------------------------
% ----------------- Calculating Dynamics (Symbollically) ------------------
% -------------------------------------------------------------------------

% ---------------------------- Symbolic Setup -----------------------------
% Define Symbolic Variables for 2-link leg system
syms x y q1 q2 real
syms xdot ydot q1dot q2dot real
syms xddot yddot q1ddot q2ddot real
syms m1 I1 l1 d1 m2 I2 l2 d2 g real
syms x_hip y_hip q1 q2 real

% Generalized coords and rates
q  = [x; y; q1; q2];
dq = [xdot; ydot; q1dot; q2dot];
ddq = [xddot; yddot; q1ddot; q2ddot];

% -------------------------- Forward kinematics ---------------------------
% COM of link1 (hip->knee)
p1 = [ x + d1*sin(q1);
       y + d1*cos(q1) ];

% Knee position
pknee = [ x + l1*sin(q1);
          y + l1*cos(q1) ];

% COM of link2 (knee->foot)
p2 = [ x + l1*sin(q1) + d2*sin(q1 + q2);
       y + l1*cos(q1) + d2*cos(q1 + q2) ];

% Foot (stance) position
pfoot = [ x + l1*sin(q1) + l2*sin(q1 + q2);
          y + l1*cos(q1) + l2*cos(q1 + q2) ];

% Total System COM
pCOM = [ (m1 * p1(1) + m2 * p2(1))/(m1 + m2);
         (m1 * p1(2) + m2 * p2(2))/(m1 + m2)];

% ---------- Inverse Kinematics Problem (with COM Constraint) ------------- 
% Assume the foot is at (0,0) in a world frame
eq1 = (x_hip + l1*sin(q1) + l2*sin(q1 + q2)) == 0; %xf = 0 
eq2 = (y_hip + l1*cos(q1) + l2*cos(q1 + q2)) == 0; %yf = 0 
eq3 = (m1*(x_hip + d1*sin(q1)) + m2*(x_hip + l1*sin(q1) + d2*sin(q1 + q2))) / (m1 + m2) == 0; %pCOM_x = 0

% Solve the system for the unknowns x_hip, q1, and q2
solutions = solve([eq1, eq2, eq3], [x_hip, q1, q2], 'Real', true);
x_hip_sym = solutions.x_hip;
q1_sym = solutions.q1;
q2_sym = solutions.q2;

% Convert the symbolic solution to a fast MATLAB function
inv_kin_func = matlabFunction([x_hip_sym(1), q1_sym(1), q2_sym(1)], 'Vars', {y_hip, l1, l2, d1, d2, m1, m2});

% ----------------------------- Velocities --------------------------------
% linear velocities of COMs calcualted using Jacobian: v = J_q(p) * dq
Jp1 = jacobian(p1, q);    % 2x4
Jp2 = jacobian(p2, q);

v1  = simplify(Jp1 * dq); % 2x1
v2  = simplify(Jp2 * dq);

% angular velocities definition
omega1 = q1dot;
omega2 = q1dot + q2dot;

% --------------------- Kinetic and potential energy ----------------------
% Note for potential and kinetic energy we need to use position and
% velocity of COM of each link.

% kinetic energy
T1 = simplify( (1/2)*m1*(v1.'*v1) + (1/2)*I1*omega1^2 );
T2 = simplify( (1/2)*m2*(v2.'*v2) + (1/2)*I2*omega2^2 );
T  = simplify( T1 + T2 );

% potential energy
U1 = m1 * g * p1(2); % p1(2) is the y-component (height)
U2 = m2 * g * p2(2);
U  = simplify( U1 + U2 );

% ----------- Stance Jacobian and Stance Jacobian Derivative --------------
% This is needed for dynamic equation with constraints 

% Stance Jacobian
Jst = jacobian(pfoot, q);

% Stance Jacoboian Derivative
Jstdot = sym(zeros(size(Jst)));
for k = 1:length(q)
    Jstdot = Jstdot + diff(Jst, q(k)) * dq(k);
end
Jstdot = simplify(Jstdot);


% ------------------------- Dynamics Equations ----------------------------
q_act = [q1; q2]; % The hip angle and knee angle are being actuated

% Flight dynamics (no contact)
[D, C, G, B] = flight_dynamics(T, U, q, dq, q_act);   

% Stance dynamics (foot fixed)
[ddq_st, Fst] = stance_dynamics(q, dq, U, D, C, G, B, Jst, Jstdot);

% Impact dynamics (touchdown)
[dq_plus, Lambda] = impact_dynamics(q, dq_minus, D, Jst);


% -------------------------------------------------------------------------
% ---------------------------- Path Trajectory ----------------------------
% -------------------------------------------------------------------------

% Leg properties
m1 = 0.05; % mass of link 1, kg
I1 = 0.0001; % moment of inertia of link 1, m^4
l1 = 0.12; % length of link 1, m
d1 = 0.06; % absolute distance to COM of link 1, m

m2 = 0.5; % mass of link 2, kg
I2 = 0.000053; % moment of inertia of link 2, m^4
l2 = 0.08; % length of link 2, m
d2 = 0.04; % absolute distance to COM of link 2, m

g = 9.81; % gravity, m/s^2


% Time
t_max = 4; 
dt = 0.01;
time = 0:dt:t_max;

% Phase durations
t0 = 1.0;   % Standing idle
t1 = 1.0;   % Crouching
t2 = 0.8;   % push-off
t3 = 0.7;   % landing fall
t4 = 0.25;  % impact compression
t5 = t_max - (t0+t1+t2+t3+t4); % absorption back

% ----------------- Determine the y position over time -------------------- 

% Heights
y0 = 0.2;           % initial y position of hip
y_st = 0.2;         % initial stand height
y_c = 0.15;         % crouch minimum height
y_max = 0.4;        % jump peak
y_impact_min = 0.17; % small compression below standing

% y-height over time
yhip_traj = zeros(size(time));

for k = 1:length(time)
    t = time(k);
    if t < t0
        % initial standing
        yhip_traj(k) = y0;
    elseif t < t0+t1
        % crouch phase
        tau = t - t0;
        yhip_traj(k) = y0 - (y0 - y_c)*(1 - cos(pi*tau/t1))/2;
    elseif t < t0+t1+t2
        % push-off
        tau = t - (t0+t1);
        yhip_traj(k) = y_c + (y_max - y_c)*(1 - cos(pi*tau/t2))/2;
    elseif t < t0+t1+t2+t3
        % falling
        tau = t - (t0+t1+t2);
        yhip_traj(k) = y_max - (y_max - y0)*(1 - cos(pi*tau/t3))/2;
    elseif t < t0+t1+t2+t3+t4
        % impact compression
        tau = t - (t0+t1+t2+t3);
        yhip_traj(k) = y0 - (y0 - y_impact_min)*(1 - cos(pi*tau/t4))/2;
    else
        % absorption / settle
        tau = t - (t0+t1+t2+t3+t4);
        yhip_traj(k) = y_impact_min + (y0 - y_impact_min)*(1 - cos(pi*tau/t5))/2;
    end
end

% ------- Determine the x, q1, q2 over time to ensure stability -----------

% Pre-allocate arrays for the results
q1_traj = zeros(size(time));
q2_traj = zeros(size(time));
xhip_traj = zeros(size(time));

% Call the generated function once to get all trajectory values
[xhip_traj, q1_traj, q2_traj] = inv_kin_func(yhip_traj, l1, l2, d1, d2, m1, m2);

% Plotting the results
figure;
plot(time, rad2deg(q1_traj), 'LineWidth', 2);
hold on;
plot(time, rad2deg(q2_traj), 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Joint Angle [deg]');
title('Joint Angles from Symbolic Inverse Kinematics');
legend('q1 (Thigh Angle)', 'q2 (Knee Angle)');
grid on;

figure;
plot(time, xhip_traj, 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Hip X Position [m]');
title('Hip Horizontal Trajectory for COM Stability');
grid on;























































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
