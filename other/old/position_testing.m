%% ------------------------------------------------------------------------
% Compute initial hip height y0 so foot is on the ground (y = 0)
% -------------------------------------------------------------------------

% ---------------- Parameters ----------------
params.m1 = 0.15;   params.m2 = 0.15;   params.m3 = 0.8;
params.l1 = 0.15;   params.l2 = 0.15;   params.l3 = 0.1; params.w3 = 0.15;
params.d1 = 0.075;  params.d2 = 0.075;  params.d3 = 0.0; % torso COM aligned with hip
params.g  = 9.81;

% Moments of inertia
params.I1 = (1/12) * params.m1 * params.l1^2; 
params.I2 = (1/12) * params.m2 * params.l2^2; 
params.I3 = (1/12) * params.m3 * (params.w3^2 + params.l3^2); % rectangular body

% ---------------- Nominal joint angles ----------------
q1_0 = 0.7504;
q2_0 = 1.5007;
q3_0 = 0.0;
x_0  = 0;           % hip horizontal position guess
y_guess = 0.2200;      % initial guess for hip height

% ---------------- Solve for y0 so foot_y = 0 ----------------
y0 = fzero(@(yy) get_foot_y(yy, params.l1, params.l2, q1_0, q2_0, x_0), y_guess);

% ---------------- Sanity check ----------------
pfoot0 = auto_pfoot(params.l1, params.l2, q1_0, q2_0, x_0, y0);
fprintf('Initial hip height y0 = %.5f m\n', y0);
fprintf('Initial foot position (x, y) = [%.5f, %.5f]\n', pfoot0(1), pfoot0(2));

% ---------------- Build initial state vector ----------------
x0 = [ x_0; y0; q1_0; q2_0; q3_0; ...
       0; 0; 0; 0; 0; ...
       0; 0 ];   % velocities and torques initially zero

%% ------------------------------------------------------------------------
% Helper function: returns vertical coordinate of foot
% ------------------------------------------------------------------------
function val = get_foot_y(yy, l1, l2, q1_0, q2_0, x_0)
    % Evaluate foot position (2D vector)
    p = auto_pfoot(l1, l2, q1_0, q2_0, x_0, yy);
    % Return only the vertical coordinate
    val = p(2);
end
