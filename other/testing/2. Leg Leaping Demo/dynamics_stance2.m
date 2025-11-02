% dynamics_stance2.m (Leaning Flip Controller)
function [dx, u, lambda, p_COM_d] = dynamics_stance2(t, x, params)
    % -------------------------------------------------------------------------
    % This stance controller tracks a full [COMx, COMy] trajectory
    % to generate a stable, leaning front flip.
    % -------------------------------------------------------------------------

    % --- Extract states and parameters ---
    q = x(1:5); dq = x(6:10);
    q1 = q(3); q2 = q(4); q3 = q(5);
    q1dot = dq(3); q2dot = dq(4); q3dot = dq(5);
    
    Kp = params.Kp; Kd = params.Kd;
    m1=params.m1; m2=params.m2; m3=params.m3;
    I1=params.I1; I2=params.I2; I3=params.I3;
    l1=params.l1; l2=params.l2; l3=params.l3;
    d1=params.d1; d2=params.d2; d3=params.d3;
    g = params.g;

    % --- Get Desired Trajectory from NEW Planner ---
    [p_COM_d, v_COM_d, a_COM_d] = desired_leap_trajectory(t, params);
    ddp_COMy_d = a_COM_d(2); % For logging
    
    % --- Calculate System Matrices ---
    D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
    C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
    G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
    B = auto_B();
    Jst = auto_Jst(l1,l2,q1,q2);
    Jstdot = auto_Jstdot(l1,l2,q1,q2,q1dot,q2dot);
    
    % --- Calculate Controller Terms ---
    pCOM_actual = auto_pCOM(d1,d2,d3,l1,m1,m2,m3,q(3),q(4),q(5),q(1),q(2));
    Jh_com = auto_JpCOM(d1,d2,d3,l1,m1,m2,m3,q(3),q(4),q(5));
    dpCOM_actual = Jh_com * dq;
    J_com_dot = auto_dJpCOM(d1,d2,d3,l1,m1,m2,m3,dq(3),dq(4),dq(5),q(3),q(4),q(5));

    % --- NEW CONTROL LAW (Tracking [COMx, COMy]) ---
    p_tilde = pCOM_actual - p_COM_d;
    dp_tilde = dpCOM_actual - v_COM_d;
    
    % Desired acceleration (feedforward + feedback)
    v = a_COM_d - Kp*p_tilde - Kd*dp_tilde;
    % =========================================================================

    % --- Joint Limit Avoidance (Keep this!) ---
    q1_min = 0.5; q2_max = 2.9;
    K_limit = 500; D_limit = 50;
    tau_penalty = [0; 0];
    if q1 < q1_min, tau_penalty(1) = K_limit * (q1_min - q1) - D_limit * q1dot; end
    if q2 > q2_max, tau_penalty(2) = K_limit * (q2_max - q2) - D_limit * q2dot; end
    % =========================================================================

    % --- Solve for all unknowns simultaneously ---
    n_q = 5; n_u = 2; n_c = 2;
    LHS = [ D,      -B,             -Jst';
            Jh_com, zeros(n_c, n_u), zeros(n_c, n_c);
            Jst,    zeros(n_c, n_u), zeros(n_c, n_c) ];
    
    RHS = [ -C*dq - G + B*tau_penalty;
            v - J_com_dot*dq;
            -Jstdot*dq ];
    
    LHS = LHS + eye(size(LHS)) * 1e-6; % Regularization
    
    sol = LHS \ RHS;
    ddq = sol(1:5);
    u = sol(6:7);
    lambda = sol(8:9);
    
    dx = [dq; ddq];
end

% % dynamics_stance2.m (Flipping Controller)
% function [dx, u, lambda, p_COMy_d, p_q3_d] = dynamics_stance2(t, x, params)
%     % -------------------------------------------------------------------------
%     % This stance controller tracks a trajectory for BOTH:
%     % 1. Vertical COM height (p_COMy_d)
%     % 2. Torso angle (p_q3_d)
%     % This is used to generate a flip.
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
%     % --- Get Desired Trajectory from NEW Planner ---
%     [p_COMy_d, dp_COMy_d, ddp_COMy_d, ...
%      p_q3_d,   dp_q3_d,   ddp_q3_d] = desired_flip_trajectory(t, params);
% 
%     % --- Calculate System Matrices ---
%     D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
%     C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
%     G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
%     B = auto_B();
%     Jst = auto_Jst(l1,l2,q1,q2);
%     Jstdot = auto_Jstdot(l1,l2,q1,q2,q1dot,q2dot);
% 
%     % --- Calculate Controller Jacobians ---
%     pCOM_actual = auto_pCOM(d1,d2,d3,l1,m1,m2,m3,q(3),q(4),q(5),q(1),q(2));
%     Jh_com_full = auto_JpCOM(d1,d2,d3,l1,m1,m2,m3,q(3),q(4),q(5));
%     dpCOM_actual = Jh_com_full * dq;
%     J_com_dot_full = auto_dJpCOM(d1,d2,d3,l1,m1,m2,m3,dq(3),dq(4),dq(5),q(3),q(4),q(5));
% 
%     % --- NEW TASK: [COMy; q3] ---
%     % Stack the Y-component of the COM Jacobian and the q3 Jacobian
%     Jh_task = [ Jh_com_full(2, :);  % 2nd row (Y-component)
%                 0 0 0 0 1 ];       % Jacobian for q3 is just [0 0 0 0 1]
% 
%     % Stack the Y-component of the COM Jacobian derivative
%     J_task_dot = [ J_com_dot_full(2, :); % 2nd row (Y-component)
%                    0 0 0 0 0 ];
% 
%     % --- NEW Control Law (Task Space) ---
%     p_tilde = [ pCOM_actual(2) - p_COMy_d;  % y-error
%                 q(5)           - p_q3_d ];  % q3-error
% 
%     dp_tilde = [ dpCOM_actual(2) - dp_COMy_d; % y-vel-error
%                  dq(5)           - dp_q3_d ]; % q3-vel-error
% 
%     % Desired acceleration for the new task
%     v = [ ddp_COMy_d;
%           ddp_q3_d ] - Kp*p_tilde - Kd*dp_tilde;
% 
%     % --- (Optional) Joint Limit Avoidance (copied from stance1) ---
%     q1_min = 0.5; q2_max = 2.9;
%     K_limit = 500; D_limit = 50;
%     tau_penalty = [0; 0];
%     if q1 < q1_min, tau_penalty(1) = K_limit * (q1_min - q1) - D_limit * q1dot; end
%     if q2 > q2_max, tau_penalty(2) = K_limit * (q2_max - q2) - D_limit * q2dot; end
%     % =========================================================================
% 
%     % --- Solve for all unknowns simultaneously ---
%     n_q = 5; n_u = 2; n_c_task = 2; n_c_constr = 2;
% 
%     LHS = [ D,      -B,               -Jst';
%             Jh_task, zeros(n_c_task, n_u), zeros(n_c_task, n_c_constr);
%             Jst,     zeros(n_c_constr, n_u), zeros(n_c_constr, n_c_constr) ];
% 
%     RHS = [ -C*dq - G + B*tau_penalty;
%             v - J_task_dot*dq;
%             -Jstdot*dq ];
% 
%     LHS = LHS + eye(size(LHS)) * 1e-6; % Regularization
% 
%     sol = LHS \ RHS;
%     ddq = sol(1:5);
%     u = sol(6:7);
%     lambda = sol(8:9);
% 
%     dx = [dq; ddq];
% end