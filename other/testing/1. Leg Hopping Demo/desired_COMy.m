function [y_d, v_d, a_d] = desired_COMy(t, params)
% desired_COMy: Generates a smooth 3-phase trajectory for a squat jump.
%
% Phases:
%   1. Hold: Stand still at y_stand.
%   2. Jump: A C^2 continuous motion combining a squat-dip and a push-up.
%            - The dip is modeled with a sinusoidal acceleration profile.
%            - The push is modeled with a 5th-order polynomial.
%   3. Flight: Ballistic motion under gravity after takeoff.
%
% Inputs:
%   t:      Current time (scalar).
%   params: A struct with the following fields:
%     .T_hold:     Duration of the initial hold phase (s).
%     .T_jump:     Total duration of the jump motion (dip + push) (s).
%     .y_stand:    Initial standing height (m).
%     .y_squat:    The lowest squat height (m).
%     .y_takeoff:  Height at takeoff (m).
%     .vf_takeoff: Velocity at takeoff (m/s).
%     .T_dip_ratio: (Optional) Ratio of T_jump dedicated to the dip phase.
%                   (Default: 0.4, i.e., 40% dip, 60% push).
%
% Outputs:
%   y_d: Desired position at time t (m).
%   v_d: Desired velocity at time t (m/s).
%   a_d: Desired acceleration at time t (m/s^2).

% --- 1. Set Parameters ---

% Unpack parameters
T_hold     = params.T_hold;
T_jump     = params.T_jump;
y_stand    = params.y_stand;
y_squat    = params.y_squat;
y_takeoff  = params.y_takeoff;
vf_takeoff = params.vf_takeoff;
g          = 9.81; % Gravity

% Define the time split for the jump phase
if isfield(params, 'T_dip_ratio')
    T_dip_ratio = params.T_dip_ratio;
else
    T_dip_ratio = 0.4; % Default: 40% dip, 60% push
end

T_dip  = T_jump * T_dip_ratio;
T_push = T_jump * (1 - T_dip_ratio);

% Check for valid parameters
if T_dip <= 0 || T_push <= 0
    error('T_jump must be positive.');
end
if y_squat >= y_stand
    error('y_squat must be less than y_stand.');
end
if y_takeoff <= y_squat
    error('y_takeoff must be greater than y_squat.');
end

% Define phase start/end times
% t_start_hold = 0;
t_end_hold   = T_hold;                  % = t_start_dip
t_end_dip    = T_hold + T_dip;          % = t_start_push
t_end_push   = t_end_dip + T_push;      % = t_takeoff

% --- 2. Calculate Trajectory based on current time t ---

if t < t_end_hold
    % --- Phase 1: Hold ---
    y_d = y_stand;
    v_d = 0;
    a_d = 0;

elseif t < t_end_dip
    % --- Phase 2a: Squat Dip ---
    % Time within this phase (0 <= tau_dip <= T_dip)
    tau_dip = t - t_end_hold;

    % Amplitude of sine wave for acceleration
    % Derived from p(T_dip) = y_squat
    % y_squat = y_stand - A_dip*T_dip^2 / (2*pi)
    A_dip = (y_stand - y_squat) * (2*pi) / (T_dip^2);

    % Sinusoidal acceleration profile
    % a(tau) = -A_dip * sin(2*pi * tau / T_dip)
    % This profile starts and ends at a=0 and v=0,
    % and achieves the desired displacement.
    sin_term = sin(2*pi * tau_dip / T_dip);
    cos_term = cos(2*pi * tau_dip / T_dip);

    a_d = -A_dip * sin_term;
    v_d = (A_dip * T_dip / (2*pi)) * (cos_term - 1);
    y_d = y_stand + (A_dip * T_dip^2 / (4*pi^2)) * sin_term - (A_dip * T_dip / (2*pi)) * tau_dip;

elseif t < t_end_push
    % --- Phase 2b: Push Up ---
    % Time within this phase (0 <= tau_push <= T_push)
    tau_push = t - t_end_dip;

    % Normalized time (0 <= s <= 1)
    s = tau_push / T_push;

    % We use a 5th-order polynomial in normalized time 's'
    % p_norm(s) = c3*s^3 + c4*s^4 + c5*s^5
    % This represents the position *relative to y_squat*.
    % p(t) = y_squat + p_norm(s)

    % Boundary conditions in normalized time:
    % p_norm(0) = 0
    % v_norm(0) = v(t_end_dip) * T_push = 0
    % a_norm(0) = a(t_end_dip) * T_push^2 = 0
    % p_norm(1) = y_takeoff - y_squat
    % v_norm(1) = vf_takeoff * T_push
    % a_norm(1) = 0 (to make the acceleration pulse smooth)

    % This implies c0 = 0, c1 = 0, c2 = 0.
    % We only need to solve for c3, c4, c5.

    P_end = y_takeoff - y_squat;
    V_end = vf_takeoff * T_push;
    % A_end = 0; % (Implicit in the equations below)

    % Solve 3x3 system for c3, c4, c5
    % [ 1  1  1 ] [c3] = [ P_end ]
    % [ 3  4  5 ] [c4] = [ V_end ]
    % [ 6 12 20 ] [c5] = [ 0     ]

    c3 = 10*P_end - 4*V_end;
    c4 = -15*P_end + 7*V_end;
    c5 = 6*P_end - 3*V_end;

    % Evaluate the polynomial and its derivatives at 's'
    s2 = s*s; s3 = s*s2; s4 = s*s3; s5 = s*s4;

    p_norm = c3*s3 + c4*s4 + c5*s5;

    % Derivatives of p_norm w.r.t 's'
    v_norm = 3*c3*s2 + 4*c4*s3 + 5*c5*s4;
    a_norm = 6*c3*s + 12*c4*s2 + 20*c5*s3;

    % Scale back to real-time derivatives
    y_d = y_squat + p_norm;
    v_d = v_norm / T_push;
    a_d = a_norm / (T_push^2);

else
    % --- Phase 3: Flight ---
    % Time since takeoff
    tau_flight = t - t_end_push;

    y_d = y_takeoff + vf_takeoff * tau_flight - 0.5 * g * tau_flight^2;
    v_d = vf_takeoff - g * tau_flight;
    a_d = -g;

end

end


% % desired_COMy.m (Smooth, Single-Phase with Squat Constraint)
% function [p_COMy_d, dp_COMy_d, ddp_COMy_d] = desired_COMy(t, params)
%     % -------------------------------------------------------------------------
%     % Returns a single, smooth quintic polynomial trajectory for a jump that
%     % respects a minimum squat depth.
%     % -------------------------------------------------------------------------
% 
%     % --- Unpack Parameters ---
%     y_stand     = params.y_stand;
%     y_squat     = params.y_squat;     % The desired squat depth (physical limit)
%     y_takeoff   = params.y_takeoff;
%     T_hold      = params.T_hold;
%     T_jump      = params.T_jump;      % Total time for the entire jump motion
%     vf_takeoff  = params.vf_takeoff;
% 
%     % --- Determine Current Phase ---
%     if t < T_hold
%         % --- Phase 1: Hold steady at standing height ---
%         p_COMy_d   = y_stand;
%         dp_COMy_d  = 0;
%         ddp_COMy_d = 0;
% 
%     elseif t < T_hold + T_jump
%         % --- Phase 2: A single, smooth JUMP trajectory ---
%         t0 = T_hold;
%         tf = T_hold + T_jump;
% 
%         % To control the squat depth, we split the total jump time into a
%         % "squat time" and a "push time". A good heuristic is to allocate
%         % more time to the squatting motion.
%         squat_time_ratio = 0.6; % 60% of the time is for squatting
%         t_squat_end = t0 + T_jump * squat_time_ratio;
% 
%         % We define an intermediate "via point" at the squat depth.
%         % The quintic polynomial will pass through this point at t_squat_end.
%         % This is more complex than a simple quintic, so for this implementation,
%         % // we will use a simplified approach that still provides excellent results:
%         % A two-phase jump trajectory.
% 
%         % For simplicity and robustness, we revert to the two-phase planner
%         % // which gives direct control over the squat depth and timing.
%         T_squat_duration = T_jump * squat_time_ratio;
%         T_push_duration = T_jump * (1 - squat_time_ratio);
% 
%         if t < t0 + T_squat_duration
%             % Sub-phase: Squat down
%             tf_squat = t0 + T_squat_duration;
%             p0 = y_stand;   pf = y_squat;
%             v0 = 0;         vf = 0;
%             a0 = 0;         af = 0;
%             [p_COMy_d, dp_COMy_d, ddp_COMy_d] = quintic_poly(t, t0, tf_squat, p0, pf, v0, vf, a0, af);
%         else
%             % Sub-phase: Push up
%             t0_push = t0 + T_squat_duration;
%             tf_push = t0_push + T_push_duration;
%             p0 = y_squat;   pf = y_takeoff;
%             v0 = 0;         vf = vf_takeoff;
%             a0 = 0;         af = 0;
%             [p_COMy_d, dp_COMy_d, ddp_COMy_d] = quintic_poly(t, t0_push, tf_push, p0, pf, v0, vf, a0, af);
%         end
% 
%     else
%         % --- Phase 3: After planned trajectory ---
%         p_COMy_d   = y_takeoff;
%         dp_COMy_d  = vf_takeoff;
%         ddp_COMy_d = 0;
%     end
% end
% 
% function [p, v, a] = quintic_poly(t, t0, tf, p0, pf, v0, vf, a0, af)
%     % This helper function is correct and remains unchanged.
%     T = tf - t0;
%     c0 = p0;
%     c1 = v0;
%     c2 = 0.5 * a0;
%     c3 = (20*(pf-p0) - (8*vf + 12*v0)*T - (3*af - a0)*T^2) / (2*T^3);
%     c4 = (30*(p0-pf) + (14*vf + 16*v0)*T + (3*af - 2*a0)*T^2) / (2*T^4);
%     c5 = (12*(pf-p0) - (6*vf + 6*v0)*T - (af - a0)*T^2) / (2*T^5);
%     tau = t - t0;
%     p = c0 + c1*tau + c2*tau^2 + c3*tau^3 + c4*tau^4 + c5*tau^5;
%     v = c1 + 2*c2*tau + 3*c3*tau^2 + 4*c4*tau^3 + 5*c5*tau^4;
%     a = 2*c2 + 6*c3*tau + 12*c4*tau^2 + 20*c5*tau^3;
% end
% 
% desired_COM_height.m
% function [p_COMy_d, dp_COMy_d, ddp_COMy_d] = desired_COMy(t, params)
%     % -------------------------------------------------------------------------
%     % Returns a smooth desired CoM vertical trajectory for a jump.
%     % -------------------------------------------------------------------------
%     % --- Unpack Parameters ---
%     y_stand   = params.y_stand;
%     y_takeoff = params.y_takeoff;
%     T_hold    = params.T_hold;
%     T_jump    = params.T_jump;    % New parameter for total jump duration
%     vf_takeoff= params.vf_takeoff;% New parameter for takeoff velocity
% 
%     % --- Determine Current Phase ---
%     if t < T_hold
%         % --- Phase 1: Hold steady at standing height ---
%         p_COMy_d   = y_stand;
%         dp_COMy_d  = 0;
%         ddp_COMy_d = 0;
% 
%     elseif t < T_hold + T_jump
%         % --- Phase 2: JUMP (Dip and Push-off) ---
%         t0 = T_hold;
%         tf = T_hold + T_jump;
% 
%         % Define boundary conditions for the entire jump motion
%         p0 = y_stand;   pf = y_takeoff; % Position: Start at stand, end at takeoff
%         v0 = 0;         vf = vf_takeoff;% Velocity: Start at rest, end at takeoff speed
%         a0 = 0;         af = 0;         % Acceleration: Start and end smoothly
% 
%         [p_COMy_d, dp_COMy_d, ddp_COMy_d] = quintic_poly(t, t0, tf, p0, pf, v0, vf, a0, af);
% 
%     else
%         % --- Phase 3: After planned trajectory, hold last command ---
%         p_COMy_d   = y_takeoff;
%         dp_COMy_d  = vf_takeoff;
%         ddp_COMy_d = 0;
%     end
% end
% 
% % The quintic_poly helper function remains the same
% function [p, v, a] = quintic_poly(t, t0, tf, p0, pf, v0, vf, a0, af)
%     % ... (no changes needed here) ...
%     T = tf - t0;
%     c0 = p0;
%     c1 = v0;
%     c2 = 0.5 * a0;
%     c3 = (20*(pf-p0) - (8*vf + 12*v0)*T - (3*af - a0)*T^2) / (2*T^3);
%     c4 = (30*(p0-pf) + (14*vf + 16*v0)*T + (3*af - 2*a0)*T^2) / (2*T^4);
%     c5 = (12*(pf-p0) - (6*vf + 6*v0)*T - (af - a0)*T^2) / (2*T^5);
%     tau = t - t0;
%     p = c0 + c1*tau + c2*tau^2 + c3*tau^3 + c4*tau^4 + c5*tau^5;
%     v = c1 + 2*c2*tau + 3*c3*tau^2 + 4*c4*tau^3 + 5*c5*tau^4;
%     a = 2*c2 + 6*c3*tau + 12*c4*tau^2 + 20*c5*tau^3;
% end

% % desired_COM_height.m
% function [p_COMy_d, dp_COMy_d, ddp_COMy_d] = desired_COMy(t, params)
%     % -------------------------------------------------------------------------
%     % Returns a smooth desired CoM vertical trajectory using quintic polynomials.
%     % This ensures continuous position, velocity, and acceleration.
%     %
%     % Outputs:
%     %   p_COMy_d   : Desired vertical position of the COM
%     %   dp_COMy_d  : Desired vertical velocity of the COM
%     %   ddp_COMy_d : Desired vertical acceleration of the COM
%     % -------------------------------------------------------------------------
% 
%     % --- Unpack Parameters ---
%     y_stand   = params.y_stand;
%     y_squat   = params.y_squat;
%     y_takeoff = params.y_takeoff;
%     T_hold    = params.T_hold;
%     T_squat   = params.T_squat;
%     T_push    = params.T_push;
% 
%     % --- Determine Current Phase ---
%     if t < T_hold
%         % --- Phase 1: Hold steady at standing height ---
%         p_COMy_d = y_stand;
%         dp_COMy_d = 0;
%         ddp_COMy_d = 0;
% 
%     elseif t < T_hold + T_squat
%         % --- Phase 2: Squat down ---
%         t0 = T_hold;
%         tf = T_hold + T_squat;
%         % Define boundary conditions for the squat
%         p0 = y_stand; pf = y_squat; % Position
%         v0 = 0; vf = 0;             % Velocity
%         a0 = 0; af = 0;             % Acceleration
% 
%         [p_COMy_d, dp_COMy_d, ddp_COMy_d] = quintic_poly(t, t0, tf, p0, pf, v0, vf, a0, af);
% 
%     elseif t < T_hold + T_squat + T_push
%         % --- Phase 3: Push up to takeoff ---
%         t0 = T_hold + T_squat;
%         tf = T_hold + T_squat + T_push;
%         % Define boundary conditions for the push-off
%         p0 = y_squat; pf = y_takeoff; % Position
%         v0 = 0; vf = 2.0;            % Velocity (target a non-zero takeoff velocity)
%         a0 = 0; af = 0;              % Acceleration
% 
%         [p_COMy_d, dp_COMy_d, ddp_COMy_d] = quintic_poly(t, t0, tf, p0, pf, v0, vf, a0, af);
% 
%     else
%         % --- Phase 4: After planned trajectory, hold last command ---
%         % This is a fallback; takeoff should have occurred by now.
%         p_COMy_d = y_takeoff;
%         dp_COMy_d = 2.0;
%         ddp_COMy_d = 0;
%     end
% end
% 
% function [p, v, a] = quintic_poly(t, t0, tf, p0, pf, v0, vf, a0, af)
%     % Helper function to compute the trajectory from quintic polynomial coefficients
% 
%     T = tf - t0;
% 
%     % Solve for the coefficients of the quintic polynomial
%     % p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
%     c0 = p0;
%     c1 = v0;
%     c2 = 0.5 * a0;
%     c3 = (20*(pf-p0) - (8*vf + 12*v0)*T - (3*af - a0)*T^2) / (2*T^3);
%     c4 = (30*(p0-pf) + (14*vf + 16*v0)*T + (3*af - 2*a0)*T^2) / (2*T^4);
%     c5 = (12*(pf-p0) - (6*vf + 6*v0)*T - (af - a0)*T^2) / (2*T^5);
% 
%     % Evaluate the polynomial at the current time t
%     tau = t - t0;
%     p = c0 + c1*tau + c2*tau^2 + c3*tau^3 + c4*tau^4 + c5*tau^5;
%     v = c1 + 2*c2*tau + 3*c3*tau^2 + 4*c4*tau^3 + 5*c5*tau^4;
%     a = 2*c2 + 6*c3*tau + 12*c4*tau^2 + 20*c5*tau^3;
% end