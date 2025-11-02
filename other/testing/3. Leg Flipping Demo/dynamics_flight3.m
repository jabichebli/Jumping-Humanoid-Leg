function [dx, u] = dynamics_flight3(~, x, params)
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
    %u = [0; 0];

    % --- flight PD controller on q1 and q2 ---
    % desired angles
    % if isfield(params,'q_des_flight')
    %     qd = params.q_des_flight(:);   % [q1_des; q2_des]
    % elseif isfield(params,'q_takeoff')
    %     qd = params.q_takeoff(:);
    % else
    %     qd = [q1; q2]; % fallback: hold current
    % end

    if isfield(params,'q_tuck_des')
        qd = params.q_tuck_des(:);   % [q1_tuck; q2_tuck]
    else
        qd = [q1; q2]; % fallback: hold current
    end

    Kp = params.flight_Kp;
    Kd = params.flight_Kd;
    umax = params.flight_u_max;

    % PD (on q1,q2 only)
    err = qd - [q1; q2];
    derr = - [q1dot; q2dot];   % desire zero rate relative to target
    u_raw = Kp .* err + Kd .* derr;   % 2x1

    % saturate torques elementwise
    u = max(min(u_raw, umax(:)), -umax(:));
    

    % -------------------- Equations of Motion --------------------
    % D(q)*ddq + C(q,dq)*dq + G(q) = B*u
    ddq = D \ (-C*dq - G + B*u);

    % -------------------- Output State Derivative --------------------
    dx = [dq; ddq];

end

% % dynamics_flight1.m (Pure Ballistic Flight)
% function [dx, u] = dynamics_flight1(~, x, params)
% % -------------------------------------------------------------------------
% % Computes the state derivative for the leg in a pure ballistic flight.
% % No control torques are applied (u=0). The system's motion is governed
% % only by its internal dynamics (momentum) and gravity.
% % -------------------------------------------------------------------------
% 
%     % --- Extract states and parameters ---
%     q = x(1:5);
%     dq = x(6:10);
%     q1 = q(3); q2 = q(4); q3 = q(5);
%     q1dot = dq(3); q2dot = dq(4); q3dot = dq(5);
% 
%     m1=params.m1; m2=params.m2; m3=params.m3;
%     I1=params.I1; I2=params.I2; I3=params.I3;
%     l1=params.l1; l2=params.l2; l3=params.l3;
%     d1=params.d1; d2=params.d2; d3=params.d3;
%     g = params.g;
% 
%     % --- System Dynamics Matrices ---
%     D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
%     C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
%     G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
%     B = auto_B();
% 
%     % --- Control Torques ---
%     % No control torques are applied during pure ballistic flight.
%     u = [0; 0];
% 
%     % --- Equations of Motion ---
%     % Solve for the accelerations of the system: D*ddq + C*dq + G = B*u
%     ddq = D \ (B*u - C*dq - G);
% 
%     % --- Output State Derivative ---
%     dx = [dq; ddq];
% end

% function [dx, u] = dynamics_flight1(~, x, params)
% % -------------------------------------------------------------------------
% % dynamics_flight.m
% % -------------------------------------------------------------------------
% % Computes the state derivative for the hopping leg when in flight.
% %
% % INPUTS:
% %   x    = [q; dq] where
% %          q  = [x; y; q1; q2; q3]
% %          dq = [xdot; ydot; q1dot; q2dot; q3dot]
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
%     g = params.g;
% 
%     % ---------------- Dynamics Matrices ----------------
%     D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
%     C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
%     G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
%     B      = auto_B();
% 
%     % -------------------- Control Torques --------------------
%     % For now, no control torques (pure ballistic motion)
%     u = [0; 0];
% 
% 
%     % -------------------- Equations of Motion --------------------
%     % D(q)*ddq + C(q,dq)*dq + G(q) = B*u
%     ddq = D \ (-C*dq - G + B*u);
% 
%     % -------------------- Output State Derivative --------------------
%     dx = [dq; ddq];
% 
% end

% % dynamics_flight1.m (Ballistic Flight with Locked Joints)
% function [dx, u] = dynamics_flight1(t, x, params)
% % -------------------------------------------------------------------------
% % Models the leg in flight with locked joints. A high-gain PD controller
% % actively holds the joint angles constant at their takeoff configuration,
% % while the overall system follows a ballistic trajectory.
% % -------------------------------------------------------------------------
% 
%     % A 'persistent' variable acts like static memory. It's initialized once
%     % and holds its value across multiple calls to this function.
%     persistent q_target_lock;
% 
%     % This checks if we are at the very first timestep of the flight phase.
%     % If so, we 'latch' the current joint angles as the target to hold.
%     time_in_flight = t - params.t_start_flight;
%     if time_in_flight < 1e-9 
%         q_target_lock = x(3:4); % Latch angles q1 and q2 at takeoff
%     end
% 
%     % --- Extract states and parameters ---
%     q = x(1:5);
%     dq = x(6:10);
%     q1 = q(3); q2 = q(4); q3 = q(5);
%     q1dot = dq(3); q2dot = dq(4); q3dot = dq(5);
% 
%     m1=params.m1; m2=params.m2; m3=params.m3;
%     I1=params.I1; I2=params.I2; I3=params.I3;
%     l1=params.l1; l2=params.l2; l3=params.l3;
%     d1=params.d1; d2=params.d2; d3=params.d3;
%     g = params.g;
% 
%     % --- High-Gain Controller to Lock Joints ---
%     % These gains are high to act like a very stiff brake.
%     Kp_lock = [500; 500];
%     Kd_lock = [50; 50];
% 
%     % The control goal is to eliminate any error from the locked angles.
%     error_q = q_target_lock - [q1; q2];
%     error_dq = -[q1dot; q2dot]; % Target velocity is zero
% 
%     u = Kp_lock .* error_q + Kd_lock .* error_dq;
% 
%     % --- Equations of Motion (Unconstrained) ---
%     % We MUST use the full dynamics, as the torques 'u' are needed
%     % to counteract the gravitational forces 'G'.
%     D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
%     C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
%     G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
%     B = auto_B();
% 
%     ddq = D \ (B*u - C*dq - G);
% 
%     % --- Output State Derivative ---
%     dx = [dq; ddq];
% end

% % dynamics_flight.m
% function [dx, u] = dynamics_flight1(t, x, params)
% % -------------------------------------------------------------------------
% % Computes the state derivative for the leg in flight.
% % The COM follows a ballistic trajectory under gravity, while a PD controller
% % repositions the leg joints for landing.
% % -------------------------------------------------------------------------
% 
% % -------------------- Flight Dynamics: ballistic base + frozen joints --------------------
% 
%     % Unpack states
%     q = x(1:5);
%     dq = x(6:10);
% 
%     % Parameters
%     g = params.g;
% 
%     % --- Ballistic motion of the floating base (x, y) ---
%     ddx = 0;          % no horizontal acceleration
%     ddy = -g;         % gravity acts downward
% 
%     % --- Freeze joint dynamics (q1, q2, q3) ---
%     dq(3:5) = 0;       % stop any joint rotation
%     ddq_joints = zeros(3,1);
% 
%     % --- Combine into full derivative ---
%     ddq = [ddx; ddy; ddq_joints];
%     dx = [dq; ddq];
% 
%     % --- No torque output during flight ---
%     u = [0; 0];
% 
% 
% 
%     % % --- Extract states and parameters ---
%     % q = x(1:5);
%     % dq = x(6:10);
%     % q1 = q(3); q2 = q(4); q3 = q(5);
%     % q1dot = dq(3); q2dot = dq(4); q3dot = dq(5);
%     % 
%     % m1=params.m1; m2=params.m2; m3=params.m3;
%     % I1=params.I1; I2=params.I2; I3=params.I3;
%     % l1=params.l1; l2=params.l2; l3=params.l3;
%     % d1=params.d1; d2=params.d2; d3=params.d3;
%     % g = params.g;
%     % 
%     % % --- Equations of Motion (Unconstrained) ---
%     % % Get the standard dynamics matrices
%     % D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
%     % C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
%     % G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
%     % B = auto_B();
%     % 
%     % 
%     % persistent step_counter
%     % if isempty(step_counter), step_counter = 0; end
%     % step_counter = step_counter + 1;
%     % 
%     % if mod(step_counter, 100) == 0
%     %     % Debug info every ~100 calls
%     %     q = x(1:5); dq = x(6:10);
%     %     fprintf('[Flight] t=%.4f | y_COM=%.3f | q1=%.3f q2=%.3f q3=%.3f | dq norm=%.3f\n', ...
%     %         t, q(2), q(3), q(4), q(5), norm(dq));
%     % end
%     % 
%     % % --- flight PD controller on q1 and q2 ---
%     % % desired angles
%     % if isfield(params,'q_des_flight')
%     %     qd = params.q_des_flight(:);   % [q1_des; q2_des]
%     % elseif isfield(params,'q_takeoff')
%     %     qd = params.q_takeoff(:);
%     % else
%     %     qd = [q1; q2]; % fallback: hold current
%     % end
%     % 
%     % Kp = params.flight_Kp;
%     % Kd = params.flight_Kd;
%     % umax = params.flight_u_max;
%     % 
%     % % PD (on q1,q2 only)
%     % err = qd - [q1; q2];
%     % derr = - [q1dot; q2dot];   % desire zero rate relative to target
%     % u_raw = Kp .* err + Kd .* derr;   % 2x1
%     % 
%     % % saturate torques elementwise
%     % u = max(min(u_raw, umax(:)), -umax(:));
%     % 
%     % % Solve for accelerations: D*ddq = B*u - C*dq - G
%     % ddq = D \ (B*u - C*dq - G);
%     % 
%     % % --- Output State Derivative ---
%     % dx = [dq; ddq];
% end