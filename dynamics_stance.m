function [dx, lambda] = dynamics_stance(t, x, params)
% -------------------------------------------------------------------------
% dynamics_stance.m
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
%   where h(q) = constraints x-COM, y-COM and torso angle
%
% OUTPUT:
%   dx = [dq; ddq]
%   lambda = [lambda1; lambda2]
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
    g = params.g; q3_d = params.q3d;
    Kp=params.Kp; Kd=params.Kd;

    % ---------------- Dynamics Matrices ----------------
    D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
    C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
    G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
    B = auto_B();
    Jst = auto_Jst(l1,l2,q1,q2);
    Jstdot = auto_Jstdot(l1,l2,q1,q2,q1dot,q2dot);

    % ---------------- Virtual Constraint ----------------
    % Set Desired COM trajectory
    [p_COMy_d, dp_COMy_d, ddp_COMy_d] = desired_COM_y(t, params);

    h   = auto_h(d1,d2,d3,l1,l2,m1,m2,m3,p_COMy_d,q1,q2,q3,q3_d,x,y); 
    Jh  = auto_Jh(d1,d2,d3,l1,l2,m1,m2,m3,q1,q2,q3);
    d2h = auto_d2h__(d1,d2,d3,l1,l2,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);

    % Drift and input vector fields    
    f = [dq;
         D \ (-C*dq - G)];
    g1 = [zeros(5,3); 
            D \ B];
    g2 = [zeros(5,2);
            D \ Jst.'];


    % Lie derivatives
    Lfh  = Jh * dq - [0; dp_COMy_d; 0];      % subtract desired velocity
    Lf2h = d2h * f  - [0; ddp_COMy_d; 0];    % subtract desired acceleration

    % Lfh  = Jh * dq;
    % Lf2h = d2h * f;

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

    % fprintf('Lg_1Lfh = [%s]\n', mat2str(Lg_1Lfh,6));
    % fprintf('Lg_2Lfh = [%s]\n', mat2str(Lg_2Lfh,6));
    % fprintf('Lf2h = [%s]\n', mat2str(Lf2h,6));
    % fprintf('h = [%s]\n', mat2str(h,6));

    % --- Debug prints---
    % compute norms/conds
    norm_Lg1 = norm(Lg_1Lfh);
    norm_Lg2 = norm(Lg_2Lfh);
    condD = cond(D);
    
    % fprintf('[DEBUG] t=%.3f: norm(Lg1)=%.3e, norm(Lg2)=%.3e, cond(D)=%.3e\n', t, norm_Lg1, norm_Lg2, condD);


    % ---------------- Constrained Dynamics ----------------
    % Assemble system 
    LHS = [D, -B, -Jst.';
           zeros(3,5), Lg_1Lfh, Lg_2Lfh;
           Jst, zeros(2,3), zeros(2,2)];
    RHS = [-C*dq - G;
            v - Lf2h;
            - Jstdot*dq];
    sol = LHS \ RHS;

    ddq = sol(1:5);   % accelerations
    u = sol(6:8);
    lambda = sol(9:end); % ground reaction

    % State derivative
    dx = [dq; ddq; u];

end

