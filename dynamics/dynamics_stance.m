% -------------------------------------------------------------------------
% dynamics_stance.m
% -------------------------------------------------------------------------
% Created by: Jason Abi Chebli
% Last Modified: 30-October-2025
% -------------------------------------------------------------------------

function [dx, u, lambda, p_COM_d] = dynamics_stance(t, x, params)
    % ---------------------------------------------------------------------
    % Implements stance-phase dynamics and control for a planar 3-link leg
    % with 2 actuated joints. The controller tracks a full [COMx, COMy]
    % trajectory to generate a stable, leaning jump.
    %
    % INPUTS:
    %   t       - current simulation time
    %   x       - state vector [q; dq]
    %   params  - structure containing model parameters
    %
    % OUTPUTS:
    %   dx      - time derivative of the state
    %   u       - joint torques
    %   lambda  - ground reaction forces
    %   p_COM_d - desired COM position (for logging)
    % ---------------------------------------------------------------------

    % ------------------ Extract states and parameters --------------------
    q  = x(1:5); dq = x(6:10);
    q1 = q(3); q2 = q(4); q3 = q(5);
    q1dot = dq(3); q2dot = dq(4); q3dot = dq(5);

    Kp = params.Kp;  Kd = params.Kd;
    m1 = params.m1;  m2 = params.m2;  m3 = params.m3;
    I1 = params.I1;  I2 = params.I2;  I3 = params.I3;
    l1 = params.l1;  l2 = params.l2;  l3 = params.l3;
    d1 = params.d1;  d2 = params.d2;  d3 = params.d3;
    g  = params.g;

    % --------- Desired COM trajectory (from trajectory planner) ----------
    [p_COM_d, v_COM_d, a_COM_d] = desired_jump_trajectory(t, params);
    ddp_COMy_d = a_COM_d(2); % For optional logging

    % ----------------- Compute dynamic model components ------------------
    D  = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
    C  = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
    G  = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
    B  = auto_B();
    Jst    = auto_Jst(l1,l2,q1,q2);
    Jstdot = auto_Jstdot(l1,l2,q1,q2,q1dot,q2dot);

    % ------------- Compute actual COM position and velocity -------------- 
    pCOM_actual = auto_pCOM(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q(1),q(2));
    Jh_com      = auto_JpCOM(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
    dpCOM_actual = Jh_com * dq;
    J_com_dot   = auto_dJpCOM(d1,d2,d3,l1,m1,m2,m3,...
                    dq(3),dq(4),dq(5),q(3),q(4),q(5));

    % --------------------- COM tracking control law ---------------------- 
    % Tracking error in position and velocity
    p_tilde  = pCOM_actual - p_COM_d;
    dp_tilde = dpCOM_actual - v_COM_d;

    % Desired COM acceleration (feedforward + PD feedback)
    v = a_COM_d - Kp * p_tilde - Kd * dp_tilde;

    % ------------- Joint limit avoidance (penalty torques) ---------------
    q1_min = 0.5; 
    q2_max = 2.9;
    K_limit = 500; 
    D_limit = 50;
    tau_penalty = [0; 0];

    if q1 < q1_min
        tau_penalty(1) = K_limit * (q1_min - q1) - D_limit * q1dot;
    end
    if q2 > q2_max
        tau_penalty(2) = K_limit * (q2_max - q2) - D_limit * q2dot;
    end

    % ---- Solve coupled system for accelerations, torques, and forces ----
    n_q = 5; n_u = 2; n_c = 2;

    % Construct augmented linear system
    LHS = [ D,      -B,             -Jst';
            Jh_com, zeros(n_c, n_u), zeros(n_c, n_c);
            Jst,    zeros(n_c, n_u), zeros(n_c, n_c) ];

    RHS = [ -C*dq - G + B*tau_penalty;
            v - J_com_dot*dq;
            -Jstdot*dq ];

    % Solve system
    sol     = LHS \ RHS;
    ddq     = sol(1:5);
    u       = sol(6:7);
    lambda  = sol(8:9);

    % Return state derivative
    dx = [dq; ddq];
end