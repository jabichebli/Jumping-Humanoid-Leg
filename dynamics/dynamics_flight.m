% -------------------------------------------------------------------------
% dynamics_flight.m
% -------------------------------------------------------------------------
% Created by: Jason Abi Chebli
% Last Modified: 30-October-2025
% -------------------------------------------------------------------------

function [dx, u] = dynamics_flight(~, x, params)
% -------------------------------------------------------------------------
% Computes the continuous-time dynamics of the hopping leg during flight.
% The foot is airborne, so the system evolves freely under gravity with
% optional PD control on the hip and knee joints.
%
% INPUTS:
%   x       - state vector [q; dq]
%             q  = [x; y; q1; q2; q3]
%             dq = [xdot; ydot; q1dot; q2dot; q3dot]
%   params  - structure containing robot parameters and flight gains
%
% OUTPUTS:
%   dx      - state derivative [dq; ddq]
%   u       - control torque vector [tau1; tau2]
%
% Equations of motion:
%   D(q)*ddq + C(q,dq)*dq + G(q) = B*u
% -------------------------------------------------------------------------

    % ----------------------- Extract States ------------------------------
    q  = x(1:5);
    dq = x(6:10);

    q1 = q(3); 
    q2 = q(4); 
    q3 = q(5);

    q1dot = dq(3); 
    q2dot = dq(4); 
    q3dot = dq(5);

    % --------------------- Extract Parameters ----------------------------
    m1 = params.m1;  m2 = params.m2;  m3 = params.m3;
    I1 = params.I1;  I2 = params.I2;  I3 = params.I3;
    l1 = params.l1;  l2 = params.l2;  l3 = params.l3;
    d1 = params.d1;  d2 = params.d2;  d3 = params.d3;
    g  = params.g;

    % --------------------- Compute Dynamics Matrices ---------------------
    D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
    C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
    G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
    B = auto_B();

    % ------------------------ Flight PD Control --------------------------
    % Determine desired joint angles (hip, knee)
    qd = params.q_des_flight(:);
    Kp    = params.flight_Kp;
    Kd    = params.flight_Kd;
    umax  = params.flight_u_max;

    err   = qd - [q1; q2];
    derr  = -[q1dot; q2dot];
    u_raw = Kp .* err + Kd .* derr;

    % Torque saturation
    u = max(min(u_raw, umax(:)), -umax(:));

    % -------------------- Equations of Motion ----------------------------
    ddq = D \ (-C*dq - G + B*u);

    % -------------------- Output State Derivative ------------------------
    dx = [dq; ddq];
end

