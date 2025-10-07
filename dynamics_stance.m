function [dx, u, lambda, pCOMy_d] = dynamics_stance(t, x, params)
% dynamics_stance.m
% t: current time
% x: state [q; dq]
% u_prev: previous control input [2x1]
% params: struct with robot parameters
%
% Returns:
% dx: state derivative [dq; ddq]
% lambda: constraint forces [2x1]
% u_out: updated control input [2x1]

    % ---------------- Extract states ----------------
    q  = x(1:5);
    dq = x(6:10);

    x_pos = q(1); y_pos = q(2);
    q1 = q(3); q2 = q(4); q3 = q(5);
    q1dot = dq(3); q2dot = dq(4); q3dot = dq(5);

    % ---------------- Extract parameters ----------------
    m1 = params.m1; m2 = params.m2; m3 = params.m3;
    I1 = params.I1; I2 = params.I2; I3 = params.I3;
    l1 = params.l1; l2 = params.l2; l3 = params.l3;
    d1 = params.d1; d2 = params.d2; d3 = params.d3;
    g  = params.g;
    Kp = params.Kp; Kd = params.Kd;

    % ---------------- Dynamics Matrices ----------------
    D      = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
    C      = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
    G      = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
    B      = auto_B();
    Jst    = auto_Jst(l1,l2,q1,q2);
    Jstdot = auto_Jstdot(l1,l2,q1,q2,q1dot,q2dot);

    % ---------------- Virtual Constraint ----------------
    pCOMy_d = desired_COM_height(t, params);
    h       = auto_h(d1,d2,d3,l1,l2,m1,m2,m3,pCOMy_d,q1,q2,q3,x_pos,y_pos); 
    Jh      = auto_Jh(d1,d2,d3,l1,l2,m1,m2,m3,q1,q2,q3);
    d2h     = auto_d2h__(d1,d2,d3,l1,l2,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);


    % Drift and input vector fields
    f  = [dq; D \ (-C*dq - G)];
    g1 = [zeros(5,2); D \ B];
    g2 = [zeros(5,2); D \ Jst.'];

    % Lie derivatives
    Lfh    = Jh * dq;
    Lf2h   = d2h * f;
    Lg_1Lfh = d2h * g1;
    Lg_2Lfh = d2h * g2;

    % Control law
    v = -Kp*h - Kd*Lfh;           % desired virtual control
    LHS = [D, -B, -Jst.';
           zeros(2,5), Lg_1Lfh, Lg_2Lfh;
           Jst, zeros(2,2), zeros(2,2)];
    RHS = [-C*dq - G;
           v - Lf2h;
           -Jstdot*dq];

    sol = LHS \ RHS;

    ddq     = sol(1:5);
    u = sol(6:7);       % raw control input
    lambda     = sol(8:9);       % constraint forces

    % ---------------- Smooth control update ----------------
    % u_out = u_prev + alpha*(u_computed - u_prev);

    % ---------------- Output ----------------
    dx = [dq; ddq];  % state derivative

end
