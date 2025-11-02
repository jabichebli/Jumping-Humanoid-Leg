function [p_COM_d, v_COM_d, a_COM_d] = desired_flip_trajectory(t, params)
% desired_flip_trajectory: Generates a 3-phase trajectory for a squat JUMP
% and a front FLIP by planning a path for the full [COMx; COMy].
%
% This version uses TWO x-positions:
%   1. x_squat_lean: The lean position at the bottom of the squat.
%   2. x_takeoff:      The final lean position at takeoff.

% --- 1. Unpack Parameters ---
T_hold     = params.T_hold;
T_jump     = params.T_jump;
y_stand    = params.y_stand;
y_squat    = params.y_squat;
y_takeoff  = params.y_takeoff;
vf_takeoff = params.vf_takeoff;
g          = 9.81;

% --- NEW: Unpack both lean parameters ---
x_squat_lean = params.x_squat_lean;
x_takeoff      = params.x_takeoff;

if isfield(params, 'T_dip_ratio')
    T_dip_ratio = params.T_dip_ratio;
else
    T_dip_ratio = 0.4;
end

T_dip  = T_jump * T_dip_ratio;
T_push = T_jump * (1 - T_dip_ratio);

t_end_hold   = T_hold;
t_end_dip    = T_hold + T_dip;
t_end_push   = t_end_dip + T_push;

% --- 2. Calculate Trajectory for COMy (Unchanged) ---
if t < t_end_hold
    y_d = y_stand; v_d = 0; a_d = 0;
elseif t < t_end_dip
    tau_dip = t - t_end_hold;
    A_dip = (y_stand - y_squat) * (2*pi) / (T_dip^2);
    sin_term = sin(2*pi * tau_dip / T_dip); cos_term = cos(2*pi * tau_dip / T_dip);
    a_d = -A_dip * sin_term;
    v_d = (A_dip * T_dip / (2*pi)) * (cos_term - 1);
    y_d = y_stand + (A_dip * T_dip^2 / (4*pi^2)) * sin_term - (A_dip * T_dip / (2*pi)) * tau_dip;
elseif t < t_end_push
    tau_push = t - t_end_dip; s = tau_push / T_push;
    P_end = y_takeoff - y_squat; V_end = vf_takeoff * T_push;
    c3 = 10*P_end - 4*V_end; c4 = -15*P_end + 7*V_end; c5 = 6*P_end - 3*V_end;
    s2 = s*s; s3 = s*s2; s4 = s*s3; s5 = s*s4;
    p_norm = c3*s3 + c4*s4 + c5*s5; v_norm = 3*c3*s2 + 4*c4*s3 + 5*c5*s4; a_norm = 6*c3*s + 12*c4*s2 + 20*c5*s3;
    y_d = y_squat + p_norm; v_d = v_norm / T_push; a_d = a_norm / (T_push^2);
else
    tau_flight = t - t_end_push;
    y_d = y_takeoff + vf_takeoff * tau_flight - 0.5 * g * tau_flight^2;
    v_d = vf_takeoff - g * tau_flight; a_d = -g;
end

% --- 3. Calculate Trajectory for COMx (The Lean) ---
if t < t_end_hold
    % --- Phase 1: Hold ---
    x_d = 0; vx_d = 0; ax_d = 0;
    
elseif t < t_end_dip
    % --- Phase 2a: Dip and Lean Forward to Squat Position ---
    % Plans a smooth motion from x=0 to x=x_squat_lean
    % Starts and ends with v=0, a=0.
    [x_d, vx_d, ax_d] = quintic_poly(t, t_end_hold, t_end_dip, 0, x_squat_lean, 0, 0, 0, 0);
    
elseif t < t_end_push
    % --- Phase 2b: Push and Continue Leaning to Takeoff Position ---
    % THIS IS THE NEW LOGIC
    % Plans a *new* smooth motion from x=x_squat_lean to x=x_takeoff
    % Starts and ends with v=0, a=0.
    [x_d, vx_d, ax_d] = quintic_poly(t, t_end_dip, t_end_push, x_squat_lean, x_takeoff, 0, 0, 0, 0);

else
    % --- Phase 3: Flight (Ballistic) ---
    % Takeoff x-velocity is 0 (from the poly), so it will just
    % continue at whatever x-velocity it had.
    vx_takeoff = 0; % From quintic poly end condition
    
    tau_flight = t - t_end_push;
    x_d = x_takeoff + vx_takeoff * tau_flight;
    vx_d = vx_takeoff;
    ax_d = 0;
end

% --- 4. Package Outputs ---
p_COM_d = [x_d; y_d];
v_COM_d = [vx_d; v_d];
a_COM_d = [ax_d; a_d];

end

% Helper function for smooth motion
function [p, v, a] = quintic_poly(t, t0, tf, p0, pf, v0, vf, a0, af)
    T = tf - t0;
    if T <= 0, p=pf; v=vf; a=af; return; end
    
    c0 = p0; c1 = v0; c2 = 0.5 * a0;
    c3 = (20*(pf-p0) - (8*vf + 12*v0)*T - (3*af - a0)*T^2) / (2*T^3);
    c4 = (30*(p0-pf) + (14*vf + 16*v0)*T + (3*af - 2*a0)*T^2) / (2*T^4);
    c5 = (12*(pf-p0) - (6*vf + 6*v0)*T - (af - a0)*T^2) / (2*T^5);
    
    tau = t - t0;
    tau2 = tau*tau; tau3 = tau*tau2; tau4 = tau*tau3; tau5 = tau*tau4;
    
    p = c0 + c1*tau + c2*tau2 + c3*tau3 + c4*tau4 + c5*tau5;
    v = c1 + 2*c2*tau + 3*c3*tau2 + 4*c4*tau3 + 5*c5*tau4;
    a = 2*c2 + 6*c3*tau + 12*c4*tau2 + 20*c5*tau3;
end
% function [p_COM_d, v_COM_d, a_COM_d] = desired_flip_trajectory(t, params)
% % desired_flip_trajectory: Generates a 3-phase trajectory for a squat JUMP
% % and a front FLIP by planning a path for the full [COMx; COMy].
% %
% % Outputs:
% %   p_COM_d: [x_d; y_d] Desired COM position
% %   v_COM_d: [vx_d; vy_d] Desired COM velocity
% %   a_COM_d: [ax_d; ay_d] Desired COM acceleration
% 
% % --- 1. Unpack Parameters ---
% T_hold     = params.T_hold;
% T_jump     = params.T_jump;
% y_stand    = params.y_stand;
% y_squat    = params.y_squat;
% y_takeoff  = params.y_takeoff;
% vf_takeoff = params.vf_takeoff;
% g          = 9.81;
% 
% % --- NEW: Lean Parameter ---
% % This is the key tuning parameter. How far forward to lean.
% x_lean_max = params.x_lean_max; % e.g., 0.1 meters
% 
% if isfield(params, 'T_dip_ratio')
%     T_dip_ratio = params.T_dip_ratio;
% else
%     T_dip_ratio = 0.4;
% end
% 
% T_dip  = T_jump * T_dip_ratio;
% T_push = T_jump * (1 - T_dip_ratio);
% 
% t_end_hold   = T_hold;
% t_end_dip    = T_hold + T_dip;
% t_end_push   = t_end_dip + T_push;
% 
% % --- 2. Calculate Trajectory for COMy (Unchanged from before) ---
% if t < t_end_hold
%     y_d = y_stand; v_d = 0; a_d = 0;
% elseif t < t_end_dip
%     tau_dip = t - t_end_hold;
%     A_dip = (y_stand - y_squat) * (2*pi) / (T_dip^2);
%     sin_term = sin(2*pi * tau_dip / T_dip);
%     cos_term = cos(2*pi * tau_dip / T_dip);
%     a_d = -A_dip * sin_term;
%     v_d = (A_dip * T_dip / (2*pi)) * (cos_term - 1);
%     y_d = y_stand + (A_dip * T_dip^2 / (4*pi^2)) * sin_term - (A_dip * T_dip / (2*pi)) * tau_dip;
% elseif t < t_end_push
%     tau_push = t - t_end_dip; s = tau_push / T_push;
%     P_end = y_takeoff - y_squat; V_end = vf_takeoff * T_push;
%     c3 = 10*P_end - 4*V_end; c4 = -15*P_end + 7*V_end; c5 = 6*P_end - 3*V_end;
%     s2 = s*s; s3 = s*s2; s4 = s*s3; s5 = s*s4;
%     p_norm = c3*s3 + c4*s4 + c5*s5; v_norm = 3*c3*s2 + 4*c4*s3 + 5*c5*s4; a_norm = 6*c3*s + 12*c4*s2 + 20*c5*s3;
%     y_d = y_squat + p_norm; v_d = v_norm / T_push; a_d = a_norm / (T_push^2);
% else
%     tau_flight = t - t_end_push;
%     y_d = y_takeoff + vf_takeoff * tau_flight - 0.5 * g * tau_flight^2;
%     v_d = vf_takeoff - g * tau_flight; a_d = -g;
% end
% 
% % --- 3. Calculate Trajectory for COMx (The Lean) ---
% % We will lean forward during the *dip* and hold that lean during the *push*.
% if t < t_end_hold
%     % --- Phase 1: Hold ---
%     x_d = 0; vx_d = 0; ax_d = 0;
% elseif t < t_end_dip
%     % --- Phase 2a: Dip and Lean Forward ---
%     % Use a 5th-order poly to go from x=0 to x=x_lean_max
%     [x_d, vx_d, ax_d] = quintic_poly(t, t_end_hold, t_end_dip, 0, x_lean_max, 0, 0, 0, 0);
% elseif t < t_end_push
%     % --- Phase 2b: Push (Hold the lean) ---
%     x_d = x_lean_max;
%     vx_d = 0;
%     ax_d = 0;
% else
%     % --- Phase 3: Flight (Ballistic) ---
%     % Get velocity at takeoff (which was 0)
%     [~, vx_takeoff, ~] = quintic_poly(t_end_dip, t_end_hold, t_end_dip, 0, x_lean_max, 0, 0, 0, 0);
%     vx_takeoff = 0; % Force it to 0 from phase 2b
% 
%     tau_flight = t - t_end_push;
%     x_d = x_lean_max + vx_takeoff * tau_flight;
%     vx_d = vx_takeoff;
%     ax_d = 0;
% end
% 
% % --- 4. Package Outputs ---
% p_COM_d = [x_d; y_d];
% v_COM_d = [vx_d; v_d];
% a_COM_d = [ax_d; a_d];
% 
% end
% 
% % Helper function for smooth motion
% function [p, v, a] = quintic_poly(t, t0, tf, p0, pf, v0, vf, a0, af)
%     T = tf - t0;
%     if T <= 0, p=pf; v=vf; a=af; return; end
% 
%     c0 = p0; c1 = v0; c2 = 0.5 * a0;
%     c3 = (20*(pf-p0) - (8*vf + 12*v0)*T - (3*af - a0)*T^2) / (2*T^3);
%     c4 = (30*(p0-pf) + (14*vf + 16*v0)*T + (3*af - 2*a0)*T^2) / (2*T^4);
%     c5 = (12*(pf-p0) - (6*vf + 6*v0)*T - (af - a0)*T^2) / (2*T^5);
% 
%     tau = t - t0;
%     tau2 = tau*tau; tau3 = tau*tau2; tau4 = tau*tau3; tau5 = tau*tau4;
% 
%     p = c0 + c1*tau + c2*tau2 + c3*tau3 + c4*tau4 + c5*tau5;
%     v = c1 + 2*c2*tau + 3*c3*tau2 + 4*c4*tau3 + 5*c5*tau4;
%     a = 2*c2 + 6*c3*tau + 12*c4*tau2 + 20*c5*tau3;
% end
