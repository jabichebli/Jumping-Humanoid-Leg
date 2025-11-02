% -------------------------------------------------------------------------
% dynamics_impact.m
% -------------------------------------------------------------------------
% Created by: Jason Abi Chebli
% Last Modified: 30-October-2025
% -------------------------------------------------------------------------

function x_plus = dynamics_impact(x_minus, params)
% -------------------------------------------------------------------------
% Models the instantaneous, perfectly inelastic collision of the foot with 
% the ground. Positions remain continuous through impact, while velocities 
% change discontinuously according to momentum and holonomic constraints.
%
% INPUTS:
%   x_minus - pre-impact state vector [q; dq_minus]
%   params  - structure containing model parameters
%
% OUTPUT:
%   x_plus  - post-impact state vector [q; dq_plus]
%
% Equations of motion:
%   1) D(q)*(dq_plus - dq_minus) = Jst.' * F_impulse   (momentum balance)
%   2) Jst * dq_plus = 0                              (no slip at impact)
% -------------------------------------------------------------------------

    % -------------------- Extract Pre-Impact State -----------------------
    q        = x_minus(1:5);      % Positions (continuous through impact)
    dq_minus = x_minus(6:10);     % Velocities (discontinuous across impact)

    % --------------------- Extract Model Parameters ----------------------
    m1 = params.m1;  m2 = params.m2;  m3 = params.m3;
    I1 = params.I1;  I2 = params.I2;  I3 = params.I3;
    l1 = params.l1;  l2 = params.l2;  l3 = params.l3;
    d1 = params.d1;  d2 = params.d2;  d3 = params.d3;

    % Extract relevant joint angles
    q1 = q(3); 
    q2 = q(4); 
    q3 = q(5);

    % -------------------- Compute System Matrices ------------------------
    D   = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);   % Inertia matrix
    Jst = auto_Jst(l1,l2,q1,q2);                            % Foot Jacobian

    % ------------------ Formulate Impact Equations -----------------------
    n_q = 5;  % number of generalized coordinates
    n_c = 2;  % number of constraints (foot x and y)

    LHS = [ D,   -Jst';
            Jst,  zeros(n_c, n_c) ];
        
    RHS = [ D * dq_minus;
            zeros(n_c, 1) ];

    % -------------------- Solve for Post-Impact Velocities ---------------
    sol     = LHS \ RHS;
    dq_plus = sol(1:n_q);    % Extract joint velocities after impact

    % ------------------ Construct Post-Impact State ----------------------
    x_plus = [q; dq_plus];
end
