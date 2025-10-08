function [dx] = dynamics_flight(~, x, params)
% -------------------------------------------------------------------------
% dynamics_flight.m
% -------------------------------------------------------------------------
% Computes the state derivative for the hopping leg when in flight.
%
% INPUTS:
%   x    = [q; dq] where
%          q  = [x; y; q1; q2; q3]
%          dq = [xdot; ydot; q1dot; q2dot; q3dot]
%   params = robot parameters
%
% OUTPUTS:
%   dx    = [dq; ddq] derivative of state
%   u_out = same as input u_in, for continuity
% -------------------------------------------------------------------------

    % Define states
    q  = x(1:5);
    dq = x(6:10);

    % Unpack states
    q1 = q(3); q2 = q(4); q3 = q(5);
    q1dot = dq(3); q2dot = dq(4); q3dot = dq(5);

    % Unpack parameters
    m1=params.m1; m2=params.m2; m3=params.m3;
    I1=params.I1; I2=params.I2; I3=params.I3;
    l1=params.l1; l2=params.l2; l3=params.l3;
    d1=params.d1; d2=params.d2; d3=params.d3;
    g = params.g;

    % ---------------- Dynamics Matrices ----------------
    D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
    C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
    G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
    B      = auto_B();

    % -------------------- Control Torques --------------------
    % For now, no control torques (pure ballistic motion)
    u = [0; 0];

    % -------------------- Equations of Motion --------------------
    % D(q)*ddq + C(q,dq)*dq + G(q) = B*u
    ddq = D \ (-C*dq - G + B*u);

    % -------------------- Output State Derivative --------------------
    dx = [dq; ddq];

end


% 
% 
% function [dx] = dynamics_flight(~, x, params)
% % -------------------------------------------------------------------------
% % dynamics_flight.m
% % -------------------------------------------------------------------------
% % Computes the state derivative for the hopping leg when in flight.
% %
% % INPUTS:
% %   x    = [q; dq] where
% %          q  = [x; y; q1; q2; q3]
% %          dq = [xdot; ydot; q1dot; q2dot; q3dot]
% %   u_in = [u1; u2] control input carried from previous stance (constant)
% %   params = robot parameters
% %
% % OUTPUTS:
% %   dx    = [dq; ddq] derivative of state
% %   u_out = same as input u_in, for continuity
% % -------------------------------------------------------------------------
% 
%     % Define states
%     q  = x(1:5);
%     dq = x(6:10);
% 
%     % Unpack states
%     q1 = q(3); q2 = q(4); q3 = q(5);
%     q1dot = dq(3); q2dot = dq(4); q3dot = dq(5);
% 
%     % Unpack parameters
%     m1=params.m1; m2=params.m2; m3=params.m3;
%     I1=params.I1; I2=params.I2; I3=params.I3;
%     l1=params.l1; l2=params.l2; l3=params.l3;
%     d1=params.d1; d2=params.d2; d3=params.d3;
%     qd1 = params.q1d; qd2 = params.q2d;
%     g = params.g;
%     Kp = params.Kp; Kd = params.Kd;
% 
%     % ---------------- Dynamics Matrices ----------------
%     D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
%     C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
%     G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
%     B      = auto_B();
%     Jst    = auto_Jst(l1,l2,q1,q2);
%     Jstdot = auto_Jstdot(l1,l2,q1,q2,q1dot,q2dot);
% 
%     % ---------------- Virtual Constraint ----------------
%     % pCOMy_d = desired_COM_height(t, params);
%     h = auto_h_flight(q1,q2,qd1,qd2) ;
%     Jh = auto_Jh_flight();
%     d2h = auto_d2h__flight();
% 
% 
%     % Drift and input vector fields
%     f  = [dq; D \ (-C*dq - G)];
%     g1 = [zeros(5,2); D \ B];
%     g2 = [zeros(5,2); D \ Jst.'];
% 
%     % Lie derivatives
%     Lfh    = Jh * dq;
%     Lf2h   = d2h * f;
%     Lg_1Lfh = d2h * g1;
%     Lg_2Lfh = d2h * g2;
% 
%     % Control law
%     v = -Kp*h - Kd*Lfh;           % desired virtual control
%     LHS = [D, -B, -Jst.';
%            zeros(2,5), Lg_1Lfh, Lg_2Lfh;
%            Jst, zeros(2,2), zeros(2,2)];
%     RHS = [-C*dq - G;
%            v - Lf2h;
%            -Jstdot*dq];
% 
%     sol = LHS \ RHS;
% 
%     ddq     = sol(1:5);
%     u = sol(6:7);       % raw control input
%     lambda     = sol(8:9);       % constraint forces
% 
%     % ---------------- Smooth control update ----------------
%     % u_out = u_prev + alpha*(u_computed - u_prev);
% 
%     % ---------------- Output ----------------
%     dx = [dq; ddq];  % state derivative
% end
