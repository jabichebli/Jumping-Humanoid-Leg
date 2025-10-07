%------------------------------------------------------------------------
%------------------------------------------------------------------------
% MECENG239 Final Project - Hopping Leg  (+ Front Flip as extension)
% Course: ME239: Robotic Locomotion
% Semester: Fall 2025
%------------------------------------------------------------------------
% By: Jason Abi Chebli and Andrew Tai
% Student ID: 3042017306 and ___________
% Last Modified: 2025-10-01
% The following is the MATLAB code used to determine the dynamics of a
% robotic leg jumping up and down. 
%------------------------------------------------------------------------
%------------------------------------------------------------------------

clear all; close all; clc;

% -------------------------------------------------------------------------
% ----------------- Calculating Dynamics (Symbollically) ------------------
% -------------------------------------------------------------------------

% ---------------------------- Symbolic Setup -----------------------------
% Define Symbolic Variables for 3-link leg system
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
p1 = [ x - d1*sin(q1);
       y - d1*cos(q1)];

% Knee position
pknee = [ x - l1*sin(q1);
          y - l1*cos(q1)];

% COM of link2 (knee->foot)
p2 = [ x + l1*(-sin(q1)) + d2*sin(q2 - q1);
          y - l1*cos(q1) - d2*cos(q2 - q1)] ;

% Foot position
pfoot = [ x - l1* sin(q1) + l2*sin(q2 - q1);
          y - l1* cos(q1) - l2*cos(q2 - q1)] ;

% COM of link3 (hip->torso)
p3 = [x - d3 * sin(q3);
      y + d3 * cos(q3)] ;

% Torso position
ptorso = [x - l3 * sin(q3);
        y + l3 * cos(q3)] ;


% Total System COM
pCOM = [ (m1 * p1(1) + m2 * p2(1) + m3 * p3(1))/(m1 + m2 + m3);
         (m1 * p1(2) + m2 * p2(2) + m3 * p3(2))/(m1 + m2 + m3)];


% ----------------------------- Velocities --------------------------------
% linear velocities of COMs calcualted using Jacobian: v = J_q(p) * dq
Jp1 = jacobian(p1, q);    % 2x5
Jp2 = jacobian(p2, q);
Jp3 = jacobian(p3, q);

v1  = simplify(Jp1 * dq); % 2x1
v2  = simplify(Jp2 * dq);
v3  = simplify(Jp3 * dq);

% angular velocities definition
omega1 = -q1dot;
omega2 = -(q2dot - q1dot);
omega3 = q3dot;

% --------------------- Kinetic and Potential Energy ----------------------
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

Jstdot = sym('Jdot', size(Jst));   

for i = 1:size(Jst,1)
    for j = 1:size(Jst,2)
        temp = 0;
        for k = 1:length(q)
            temp = temp + diff(Jst(i,j), q(k)) * dq(k);
        end
        Jstdot(i,j) = temp;
    end
end

Jstdot = simplify(Jstdot);
% ------------------------- Dynamics Equations ----------------------------
q_act = [q1; q2]; % Hip, knee angles actuated

% Flight dynamics (no contact)
[D, C, G, B] = LagrangianDynamics(T, U, q, dq, q_act);   

% ------------------------ Virtual Constraints ----------------------------
% Define Symbolic Variables for virual constraint
syms Kp Kd real
syms u1 u2 real % for three holonomic constraints
syms pCOMy_d real % to set COM position and torso angle
syms qd1 qd2 real
syms lambda1 lambda2 real

% Generalized input and contact force
u_sym = [u1; u2]; % for two holonomic constraints
lambda_sym = [lambda1; lambda2];

% Holonomic Virtual Constraint
h = [pCOM(1) - pfoot(1); % x-constraint, CoM above foot (horizontal alignment)
    pCOM(2) - pCOMy_d];      % y-constraint

h_flight = [q1 - qd1; % x-constraint, CoM above foot (horizontal alignment)
            q2 - qd2];      % y-constraint

% Holonomic Jacobian
Jh = jacobian(h, q); %  Jh = dh/dq 3 x 5
Jh = simplify(Jh);

Jhdotdq = jacobian(Jh * dq, q) * dq; % Jhdot * dq
Jhdotdq = simplify(Jhdotdq);

Jhdot = sym(zeros(size(Jh)));  % initialize  (3×5 if Jh is 3×5)
for i = 1:size(Jh,1)
    for j = 1:length(q)
        Jhdot(i,:) = Jhdot(i,:) + diff(Jh(i,:), q(j)) * dq(j);
    end
end

% First Derivative
Lfh = Jh * dq; % dh/dt
Lfh = simplify(Lfh);

% Useful for Second Derivative (not computed symbolically as its too
% intense) 
d2h__ = [jacobian(Jh*dq, q), Jh]; % later d2h__ * {f(x) + g1(x)u + g2(x)lambda}


% ------------------------ Flight Constraints ----------------------------
% Define Symbolic Variables for virual constraint
syms Kp Kd real
syms u1 u2 real % for three holonomic constraints
syms pCOMy_d real % to set COM position and torso angle
syms qd1 qd2 real
syms lambda1 lambda2 real

% Holonomic Virtual Constraint
h_flight = [q1 - qd1; % x-constraint, CoM above foot (horizontal alignment)
            q2 - qd2];      % y-constraint

% Holonomic Jacobian
Jh_flight = jacobian(h_flight, q); %  Jh = dh/dq 3 x 5
Jh_flight = simplify(Jh_flight);

Jhdotdq_flight = jacobian(Jh_flight * dq, q) * dq; % Jhdot * dq
Jhdotdq_flight = simplify(Jhdotdq_flight);

Jhdot_flight = sym(zeros(size(Jh_flight)));  % initialize  (3×5 if Jh is 3×5)
for i = 1:size(Jh_flight,1)
    for j = 1:length(q)
        Jhdot_flight(i,:) = Jhdot_flight(i,:) + diff(Jh_flight(i,:), q(j)) * dq(j);
    end
end

% First Derivative
Lfh_flight = Jh_flight * dq; % dh/dt
Lfh_flight = simplify(Lfh_flight);

% Useful for Second Derivative (not computed symbolically as it's too intense)
d2h__flight = [jacobian(Jh_flight*dq, q), Jh_flight]; % later d2h__ * {f(x) + g1(x)u + g2(x)lambda}

% ------------------------ Export Functions ----------------------------

if ~exist('./auto')
    mkdir('./auto')
end
addpath('./auto')

% Generate position files
matlabFunction(pCOM, 'File', 'auto/auto_pCOM');
matlabFunction(pfoot, 'File', 'auto/auto_pfoot');

% Generate Lagrangian Dynamics
matlabFunction(D, 'File', 'auto/auto_D');
matlabFunction(C, 'File', 'auto/auto_C');
matlabFunction(G, 'File', 'auto/auto_G');
matlabFunction(B, 'File', 'auto/auto_B');

% Generate stationary Jacobians
matlabFunction(Jst, 'File', 'auto/auto_Jst');
matlabFunction(Jstdot, 'File', 'auto/auto_Jstdot');

% Generate corresponding virtual constraints for stance
matlabFunction(h, 'File', 'auto/auto_h');
matlabFunction(Jh, 'File', 'auto/auto_Jh');
matlabFunction(Jhdot, 'File', 'auto/auto_Jhdot');
matlabFunction(Jhdotdq, 'File', 'auto/auto_Jhdotdq');
matlabFunction(Lfh, 'File', 'auto/auto_Lfh');
matlabFunction(d2h__, 'File', 'auto/auto_d2h__');

% Generate corresponding virtual constraints for flight
matlabFunction(h_flight, 'File', 'auto/auto_h_flight');
matlabFunction(Jh_flight, 'File', 'auto/auto_Jh_flight');
matlabFunction(Jhdot_flight, 'File', 'auto/auto_Jhdot_flight');
matlabFunction(Jhdotdq_flight, 'File', 'auto/auto_Jhdotdq_flight');
matlabFunction(Lfh_flight, 'File', 'auto/auto_Lfh_flight');
matlabFunction(d2h__flight, 'File', 'auto/auto_d2h__flight');
