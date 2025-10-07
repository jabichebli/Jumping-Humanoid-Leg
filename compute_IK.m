function [q1_d, q2_d] = compute_IK(x_COM_desired, y_COM_desired, q3_d, params)
% -------------------------------------------------------------------------
% Compute joint angles q1_d, q2_d to achieve desired COM position and torso angle
%
% Inputs:
%   x_COM_desired, y_COM_desired : desired COM positions
%   q3_d                        : desired torso angle
%   params                       : struct with m1,m2,m3,d1,d2,d3,l1,l2,l3
%
% Outputs:
%   q1_d, q2_d : joint angles for leg links
% -------------------------------------------------------------------------

    m1 = params.m1; m2 = params.m2; m3 = params.m3;
    d1 = params.d1; d2 = params.d2; d3 = params.d3;
    l1 = params.l1; l2 = params.l2; l3 = params.l3;

    % Anonymous function for COM x/y given q1,q2
    fun = @(q) [ ...
        (m1*( -d1*sin(q(1))) + m2*( -l1*sin(q(1)) + d2*sin(q(2)-q(1))) + m3*(-d3*sin(q3_d)))/(m1+m2+m3) - x_COM_desired;
        (m1*(-d1*cos(q(1))) + m2*(-l1*cos(q(1)) - d2*cos(q(2)-q(1))) + m3*(d3*cos(q3_d)))/(m1+m2+m3) - y_COM_desired ...
    ];

    % Initial guess (standing pose)
    q0 = [0; pi/4];

    % Solve
    options = optimoptions(@fsolve, 'Display', 'off', 'FunctionTolerance', 1e-12);
    q_sol = fsolve(fun, q0, options);

    q1_d = q_sol(1);
    q2_d = q_sol(2);
end
