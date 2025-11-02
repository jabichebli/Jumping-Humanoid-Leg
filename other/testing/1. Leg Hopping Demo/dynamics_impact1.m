% dynamics_impact.m
function x_plus = dynamics_impact(x_minus, params)
% -------------------------------------------------------------------------
% Models the inelastic collision of the foot with the ground.
% Calculates the state vector x_plus (post-impact) given the state
% x_minus (pre-impact). Positions remain the same, velocities change
% instantaneously.
% -------------------------------------------------------------------------

    % --- Extract Pre-Impact State ---
    q = x_minus(1:5); % Positions are continuous through impact
    dq_minus = x_minus(6:10); % Velocities are discontinuous

    % Unpack parameters needed for dynamics matrices
    m1=params.m1; m2=params.m2; m3=params.m3;
    I1=params.I1; I2=params.I2; I3=params.I3;
    l1=params.l1; l2=params.l2; l3=params.l3;
    d1=params.d1; d2=params.d2; d3=params.d3;
    q1=q(3); q2=q(4); q3=q(5);

    % --- Get Dynamics Matrices at the moment of impact ---
    D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
    Jst = auto_Jst(l1,l2,q1,q2); % Jacobian of the foot constraint

    % --- Set up the Impact Equations ---
    % The core equations are:
    % 1) D(q) * (dq_plus - dq_minus) = Jst' * F_impulse  (Change in momentum = Impulse)
    % 2) Jst * dq_plus = 0                               (Post-impact velocity of foot is zero)
    %
    % We can write this as a single linear system to solve for dq_plus and F_impulse:
    % [ D   -Jst' ] [ dq_plus ] = [ D*dq_minus ]
    % [ Jst   0   ] [ F_impulse ] = [     0      ]
    
    n_q = 5; % Number of generalized coordinates
    n_c = 2; % Number of constraints (foot x and y)
    
    LHS = [ D,   -Jst';
            Jst,  zeros(n_c, n_c) ];
            
    RHS = [ D * dq_minus;
            zeros(n_c, 1) ];

    % --- Solve for Post-Impact Velocities ---
    solution = LHS \ RHS;
    
    dq_plus = solution(1:n_q);
    % The other part of the solution, solution(n_q+1:end), is the impulse F_impulse,
    % but we only need the post-impact velocities.

    % --- Construct the Post-Impact State Vector ---
    x_plus = [q; dq_plus];
end