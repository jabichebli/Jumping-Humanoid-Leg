% dynamics_stance1.m (with Joint Limit Avoidance)
function [dx, u, lambda, p_COMy_d] = dynamics_stance1(t, x, params)
    % -------------------------------------------------------------------------
    % This is the definitive, robust stance controller with singularity avoidance.
    % -------------------------------------------------------------------------

    % --- Extract states and parameters ---
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

    % --- Get Desired Trajectory from Planner ---
    [p_COMy_d, dp_COMy_d, ddp_COMy_d] = desired_COMy(t, params);

    % --- Calculate System Matrices ---
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

    % --- Primary Control Law (Task Space) ---
    p_tilde = [pCOM_actual(1) - 0; pCOM_actual(2) - p_COMy_d];
    dp_tilde = [dpCOM_actual(1) - 0; dpCOM_actual(2) - dp_COMy_d];
    v = [0; ddp_COMy_d] - Kp*p_tilde - Kd*dp_tilde;

    % =========================================================================
    % --- FINAL FIX: JOINT LIMIT AVOIDANCE ---
    % =========================================================================
    % Define safe joint limits (in radians) to avoid singularity
    q1_min = 0.5;  % Prevent hip from going fully straight
    q2_max = 2.9;  % Prevent knee from going fully straight (pi is ~3.14)
    
    % High gains for the "virtual spring" that activates near limits
    K_limit = 500;
    D_limit = 50;
    
    tau_penalty = [0; 0]; % Initialize penalty torques
    
    if q1 < q1_min
        tau_penalty(1) = K_limit * (q1_min - q1) - D_limit * q1dot;
    end
    if q2 > q2_max
        tau_penalty(2) = K_limit * (q2_max - q2) - D_limit * q2dot;
    end
    % =========================================================================

    % --- Solve for all unknowns simultaneously ---
    n_q = 5; n_u = 2; n_c = 2;
    LHS = [ D,      -B,             -Jst';
            Jh_com, zeros(n_c, n_u), zeros(n_c, n_c);
            Jst,    zeros(n_c, n_u), zeros(n_c, n_c) ];
    
    % The penalty torque is added to the equations of motion
    RHS = [ -C*dq - G + B*tau_penalty;
            v - J_com_dot*dq;
            -Jstdot*dq ];
    
    % Add regularization as a final layer of safety
    LHS = LHS + eye(size(LHS)) * 1e-6;
    
    sol = LHS \ RHS;
    ddq = sol(1:5);
    u = sol(6:7);
    lambda = sol(8:9);
    
    dx = [dq; ddq];
end
% % dynamics_stance1.m
% function [dx, u, lambda, p_COMy_d, dp_COMy_d, ddp_COMy_d] = dynamics_stance1(t, x, params)
%     % -------------------------------------------------------------------------
%     % This is the definitive, robust stance controller. It solves for all
%     % dynamics and control variables simultaneously to ensure numerical stability.
%     % -------------------------------------------------------------------------
% 
%     % --- Extract states and parameters ---
%     q = x(1:5);
%     dq = x(6:10);
%     q1 = q(3); q2 = q(4); q3 = q(5);
%     q1dot = dq(3); q2dot = dq(4); q3dot = dq(5);
% 
%     Kp = params.Kp; Kd = params.Kd;
%     m1=params.m1; m2=params.m2; m3=params.m3;
%     I1=params.I1; I2=params.I2; I3=params.I3;
%     l1=params.l1; l2=params.l2; l3=params.l3;
%     d1=params.d1; d2=params.d2; d3=params.d3;
%     g = params.g;
% 
%     % --- Get Desired Trajectory from Planner ---
%     [p_COMy_d, dp_COMy_d, ddp_COMy_d] = desired_COMy(t, params);
% 
%     % --- Calculate System Matrices ---
%     D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
%     C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
%     G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
%     B = auto_B();
%     Jst = auto_Jst(l1,l2,q1,q2);
%     Jstdot = auto_Jstdot(l1,l2,q1,q2,q1dot,q2dot);
% 
%     % --- Calculate Controller Terms ---
%     pCOM_actual = auto_pCOM(d1,d2,d3,l1,m1,m2,m3,q(3),q(4),q(5),q(1),q(2));
%     Jh_com = auto_JpCOM(d1,d2,d3,l1,m1,m2,m3,q(3),q(4),q(5));
%     dpCOM_actual = Jh_com * dq;
%     J_com_dot = auto_dJpCOM(d1,d2,d3,l1,m1,m2,m3,dq(3),dq(4),dq(5),q(3),q(4),q(5));
% 
%     p_tilde = [pCOM_actual(1) - 0;
%                pCOM_actual(2) - p_COMy_d];
%     dp_tilde = [dpCOM_actual(1) - 0;
%                 dpCOM_actual(2) - dp_COMy_d];
% 
%     % Define the desired COM acceleration (feedforward + feedback)
%     v = [0; ddp_COMy_d] - Kp*p_tilde - Kd*dp_tilde;
% 
%     % =========================================================================
%     % --- DEFINITIVE FIX: Solve for everything simultaneously ---
%     % =========================================================================
%     % We are solving the combined system of equations:
%     % 1) D*ddq - B*u - Jst'*lambda = -C*dq - G       (Equations of Motion)
%     % 2) Jh_com*ddq                 = v - J_com_dot*dq (Control Law)
%     % 3) Jst*ddq                    = -Jstdot*dq       (Foot Constraint)
%     %
%     % In matrix form: [LHS] * [ddq; u; lambda] = [RHS]
% 
%     n_q = 5; % num generalized coordinates
%     n_u = 2; % num actuated joints
%     n_c = 2; % num constraints
% 
%     % The big matrix on the Left-Hand Side
%     LHS = [ D,      -B,             -Jst';
%             Jh_com, zeros(n_c, n_u), zeros(n_c, n_c);
%             Jst,    zeros(n_c, n_u), zeros(n_c, n_c) ];
% 
%     % The vector on the Right-Hand Side
%     RHS = [ -C*dq - G;
%             v - J_com_dot*dq;
%             -Jstdot*dq ];
% 
%     damping = 1e-6;
%     LHS = LHS + eye(size(LHS)) * damping;
% 
%     % Solve the full system for all unknowns
%     sol = LHS \ RHS;
% 
%     ddq = sol(1:5);
%     u = sol(6:7);
%     lambda = sol(8:9);
% 
%     % --- Output State Derivative ---
%     dx = [dq; ddq];
% end