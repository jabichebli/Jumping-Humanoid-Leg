% -------------------------------------------------------------------------
% dynamics_stabilize.m
% -------------------------------------------------------------------------
% Created by: Jason Abi Chebli
% Last Modified: 30-October-2025
% -------------------------------------------------------------------------

function [dx, u, lambda] = dynamics_stabilize(~, x, params)
% -------------------------------------------------------------------------
% Computes the stabilized stance-phase dynamics for a planar 3-link leg
% using a holonomic stance constraint and a virtual constraint controller.
%
% State x = [q; dq] where
%   q  = [x; y; q1; q2; q3]
%   dq = [xdot; ydot; q1dot; q2dot; q3dot]
%
% Dynamics:
%   D(q)*ddq + C(q,dq)*dq + G(q) = B*u + Jst.'*lambda
%   Jst(q)*ddq + Jstdot(q,dq)*dq = 0     (stance constraint)
%
% Control Law:
%   u = (LgLfh)^(-1) [ -Lf2h - Kp*h - Kd*Lfh ]
%
% OUTPUTS:
%   dx      - time derivative of state
%   u       - joint torques
%   lambda  - ground reaction forces
% -------------------------------------------------------------------------

    % ------------------------ Extract States -----------------------------
    q  = x(1:5);
    dq = x(6:10);

    % Unpack individual states
    x = q(1);  y = q(2);
    q1 = q(3); q2 = q(4); q3 = q(5);
    q1dot = dq(3); q2dot = dq(4); q3dot = dq(5);

    % ----------------------- Extract Parameters --------------------------
    m1=params.m1; m2=params.m2; m3=params.m3;
    I1=params.I1; I2=params.I2; I3=params.I3;
    l1=params.l1; l2=params.l2; l3=params.l3;
    d1=params.d1; d2=params.d2; d3=params.d3;
    g = params.g; 
    pCOMy_d = params.y_stand;
    Kp=params.Kp; Kd=params.Kd;

    % --------------------- Compute System Matrices -----------------------
    D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
    C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
    G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
    B = auto_B();
    Jst = auto_Jst(l1,l2,q1,q2);
    Jstdot = auto_Jstdot(l1,l2,q1,q2,q1dot,q2dot);

    % ------------------- Virtual Constraint Definition -------------------
    % Defines holonomic constraint
    h   = auto_h(d1,d2,d3,l1,l2,m1,m2,m3,pCOMy_d,q1,q2,q3,x,y); 
    Jh  = auto_Jh(d1,d2,d3,l1,l2,m1,m2,m3,q1,q2,q3);
    d2h = auto_d2h__(d1,d2,d3,l1,l2,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);

    % --------------------- Compute System Vector Fields ------------------
    f = [dq;
         D \ (-C*dq - G)];
    g1 = [zeros(5,2);
          D \ B];
    g2 = [zeros(5,2);
          D \ Jst.'];

    % ------------------------ Lie Derivatives ----------------------------
    Lfh     = Jh * dq;         % First-order Lie derivative
    Lf2h    = d2h * f;         % Second-order Lie derivative
    Lg_1Lfh = d2h * g1;        % Control influence on virtual constraint
    Lg_2Lfh = d2h * g2;

    % ---------------------- Virtual Constraint Control -------------------
    % Control term (PD controller)
    v = -Kp * h - Kd * Lfh;

    % --------------------- Constrained Dynamics System -------------------
    % Assemble full system for accelerations, torques, and constraint forces
    LHS = [D, -B, -Jst.';
           zeros(2,5), Lg_1Lfh, Lg_2Lfh;
           Jst, zeros(2,2), zeros(2,2)];

    RHS = [-C*dq - G;
            v - Lf2h;
           -Jstdot*dq];

    % Solve linear system
    sol = LHS \ RHS;

    % -------------------------- Extract Results --------------------------
    ddq     = sol(1:5);   % Accelerations
    u       = sol(6:7);   % Control torques
    lambda  = sol(8:9);   % Ground reaction forces

    % ------------------------ State Derivative ---------------------------
    dx = [dq; ddq];
end
