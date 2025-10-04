function dx = dynamics_flight(~, x, params)
% -------------------------------------------------------------------------
% dynamics_flight.m
% -------------------------------------------------------------------------
% Computes the state derivative for the hopping leg when the leg is in flight.
%
% State x = [q; dq] where
%   q  = [x; y; q1; q2; q3]
%   dq = [xdot; ydot; q1dot; q2dot; q3dot]
%
% Dynamics:
%   D(q) * ddq + C(q,dq)*dq + G(q) = B*u
%   u = 0   (flight)
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
    g = params.g;

    % ---------------- Dynamics Matrices ----------------
    D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
    C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot);
    G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3);
    B = auto_B();

    % Leg is in flight
    u = zeros(3,1); 
    ddq = D \ (B*u - C*dq - G);

    % State derivative
    dx = [dq; ddq];
end
