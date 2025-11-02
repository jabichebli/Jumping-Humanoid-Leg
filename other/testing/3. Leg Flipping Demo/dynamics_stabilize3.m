function [dx, u, lambda] = dynamics_stabilize3(~, x, params)
% -------------------------------------------------------------------------
% leg_ode.m
% -------------------------------------------------------------------------
% Computes the state derivative for the hopping leg with holonomic stance
% constraint and a virtual constraint controller.
%
% State x = [q; dq] where
%   q  = [x; y; q1; q2; q3]
%   dq = [xdot; ydot; q1dot; q2dot; q3dot]
%
% Dynamics:
%   D(q) * ddq + C(q,dq)*dq + G(q) = B*u + Jst.'*lambda
%   Jst(q) * ddq + Jstdot(q,dq)*dq = 0   (stance constraint)
%
% Control law:
%   u = (LgLfh)^(-1) [ -Lf2h - Kp*h - Kd*Lfh ]
%   where h(q) = pCOM_x - pfoot_x
%
% OUTPUT:
%   dx = [dq; ddq]
%
% -------------------------------------------------------------------------

    % Define states
    q  = x(1:5);
    dq = x(6:10);

    % Unpack states
    x = q(1);
    y = q(2);
    q1 = q(3);
    q2 = q(4);
    q3 = q(5);
    q1dot = dq(3);
    q2dot = dq(4);
    q3dot = dq(5);

    % Unpack parameters
    m1=params.m1; m2=params.m2; m3=params.m3;
    I1=params.I1; I2=params.I2; I3=params.I3;
    l1=params.l1; l2=params.l2; l3=params.l3;
    d1=params.d1; d2=params.d2; d3=params.d3;
    g = params.g; pCOMy_d = params.y_stand;
    Kp=params.Kp; Kd=params.Kd;

    % ---------------- Dynamics Matrices ----------------
    D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
    C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
    G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
    B = auto_B();
    Jst = auto_Jst(l1,l2,q1,q2);
    Jstdot = auto_Jstdot(l1,l2,q1,q2,q1dot,q2dot);

    % ---------------- Virtual Constraint ----------------
    h   = auto_h(d1,d2,d3,l1,l2,m1,m2,m3,pCOMy_d,q1,q2,q3,x,y); 
    Jh  = auto_Jh(d1,d2,d3,l1,l2,m1,m2,m3,q1,q2,q3);
    d2h = auto_d2h__(d1,d2,d3,l1,l2,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);

    % Drift and input vector fields
    f = [dq;
         D \ (-C*dq - G)];
    g1 = [zeros(5,2);
            D \ B];
    g2 = [zeros(5,2);
            D \ Jst.'];


    % Lie derivatives
    Lfh  = Jh * dq;
    Lf2h = d2h * f;
    Lg_1Lfh = d2h * g1 ;
    Lg_2Lfh = d2h * g2 ;

    % Control input
    % u = LgLfh \ (-Lf2h - Kp*h - Kd*Lfh); % this is for two holonomic virtual constraints
    % u = (-Lf2h - Kp*h - Kd*Lfh) / LgLfh; % for one holonomic virtual constraint
    
    v = - Kp*h - Kd*Lfh; % Controller

    % Debug statements
    % disp(size(B))
    % disp(size(h))
    % disp(size(Lfh))
    % disp(size(Lf2h))
    % disp(size(LgLfh))
    % disp(size(u))
    %disp(size(Jstdot))
    % disp(size(dq))
    % disp(size(Lg_1Lfh))
    % disp(size(zeros(2,5)))
    % disp(size(Lg_2Lfh))
    % disp(size(Jst))

    % ---------------- Constrained Dynamics ----------------
    % Assemble system 
    LHS = [D, -B, -Jst.';
           zeros(2,5), Lg_1Lfh, Lg_2Lfh;
           Jst, zeros(2,2), zeros(2,2)];
    RHS = [-C*dq - G;
            v - Lf2h;
            - Jstdot*dq];
    sol = LHS \ RHS;

    ddq = sol(1:5);   % accelerations
    u = sol(6:7);

    % disp(sol(6:7))
    % disp(u)
    lambda = sol(8:9); % ground reaction (not used)

    % State derivative
    dx = [dq; ddq];
end
