% dynamics_stabilize1.m
function [dx, u, lambda] = dynamics_stabilize1(t, x, params)
    % -------------------------------------------------------------------------
    % A simple stance controller whose only goal is to hold the leg at the
    % standing height (params.y_stand). It is used for stabilizing after landing.
    % -------------------------------------------------------------------------

    q = x(1:5);
    dq = x(6:10);
    q1 = q(3); q2 = q(4); q3 = q(5);
    q1dot = dq(3); q2dot = dq(4); q3dot = dq(5);
    
    Kp = params.Kp; Kd = params.Kd;
    m1=params.m1; m2=params.m2; m3=params.m3;
    I1=params.I1; I2=params.I2; I3=params.I3;
    l1=params.l1; l2=params.l2; l3=params.l3;
    d1=params.d1; d2=params.d2; d3=params.d3;
    g = params.g;

    % --- Desired Trajectory: Hold a constant position ---
    p_COMy_d = params.y_stand;
    dp_COMy_d = 0;
    ddp_COMy_d = 0;

    % (The rest of the dynamics are identical to dynamics_stance1.m)
    D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
    C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
    G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
    B = auto_B();
    Jst = auto_Jst(l1,l2,q1,q2);
    Jstdot = auto_Jstdot(l1,l2,q1,q2,q1dot,q2dot);

    pCOM_actual = auto_pCOM(d1,d2,d3,l1,m1,m2,m3,q(3),q(4),q(5),q(1),q(2));
    Jh_com = auto_JpCOM(d1,d2,d3,l1,m1,m2,m3,q(3),q(4),q(5));
    dpCOM_actual = Jh_com * dq;
    J_com_dot = auto_dJpCOM(d1,d2,d3,l1,m1,m2,m3,dq(3),dq(4),dq(5),q(3),q(4),q(5));

    p_tilde = [pCOM_actual(1) - 0;
               pCOM_actual(2) - p_COMy_d];
    dp_tilde = [dpCOM_actual(1) - 0;
                dpCOM_actual(2) - dp_COMy_d];
    
    v = [0; ddp_COMy_d] - Kp*p_tilde - Kd*dp_tilde;

    n_q = 5; n_u = 2; n_c = 2;
    LHS = [ D,      -B,             -Jst';
            Jh_com, zeros(n_c, n_u), zeros(n_c, n_c);
            Jst,    zeros(n_c, n_u), zeros(n_c, n_c) ];
    RHS = [ -C*dq - G;
            v - J_com_dot*dq;
            -Jstdot*dq ];
    
    sol = LHS \ RHS;
    ddq = sol(1:5);
    u = sol(6:7);
    lambda = sol(8:9);
    
    dx = [dq; ddq];
end