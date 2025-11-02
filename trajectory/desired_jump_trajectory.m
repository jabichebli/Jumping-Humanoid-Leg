% -------------------------------------------------------------------------
% desired_jump_trajectory.m
% -------------------------------------------------------------------------
% Created by: Jason Abi Chebli
% Last Modified: 30-October-2025
% -------------------------------------------------------------------------

function [p_COM_d, v_COM_d, a_COM_d] = desired_jump_trajectory(t, params)
    % ---------------------------------------------------------------------
    % Generates a 3-phase desired trajectory for the full COM motion [x; y]
    % during a squat jump and front flip. The function defines smooth, 
    % time-based trajectories for both vertical (y) and horizontal (x) motion 
    % using quintic polynomials and ballistic flight dynamics.
    %
    % Inputs:
    %   t       - current simulation time
    %   params  - structure containing trajectory parameters:
    %               .T_hold, .T_jump, .T_dip_ratio,
    %               .y_stand, .y_squat, .y_takeoff,
    %               .vf_takeoff, .x_squat_lean, .x_takeoff
    %
    % Outputs:
    %   p_COM_d - desired COM position        [x_d; y_d]
    %   v_COM_d - desired COM velocity        [vx_d; vy_d]
    %   a_COM_d - desired COM acceleration    [ax_d; ay_d]
    % ---------------------------------------------------------------------
    
    % =====================================================================
    % --------------------------- Setup the Problem -----------------------
    % =====================================================================
    
    % Unpack parameters
    T_hold       = params.T_hold;
    T_jump       = params.T_jump;
    T_dip_ratio  = params.T_dip_ratio;
    y_stand      = params.y_stand;
    y_squat      = params.y_squat;
    y_takeoff    = params.y_takeoff;
    vf_takeoff   = params.vf_takeoff;
    g            = 9.81;
    x_squat_lean = params.x_squat_lean;
    x_takeoff    = params.x_takeoff;
    
    % Compute phase durations
    T_dip  = T_jump * T_dip_ratio;
    T_push = T_jump * (1 - T_dip_ratio);
    
    % Assign absolute times for phase transitions
    t_end_hold = T_hold;
    t_end_dip  = T_hold + T_dip;
    t_end_push = t_end_dip + T_push;
    
    % =====================================================================
    % --------------------- Calculate Trajectory of COMy ------------------
    % =====================================================================
    
    if t < t_end_hold
        % ---------------------- Phase 1: Initial Hold --------------------
        % The body remains upright in standing configuration. 
        % COM stays stationary at y_stand with zero velocity and acceleration.
        y_d = y_stand; 
        v_d = 0; 
        a_d = 0;
    
    elseif t < t_end_dip
        % ---------------------- Phase 2a: Downward Dip -------------------
        % The COM lowers smoothly to y_squat following a sinusoidal pattern.
        % Velocity and acceleration start and end at zero for smooth transition.
        tau_dip = t - t_end_hold;
        A_dip = (y_stand - y_squat) * (2*pi) / (T_dip^2);
        sin_term = sin(2*pi * tau_dip / T_dip);
        cos_term = cos(2*pi * tau_dip / T_dip);
        a_d = -A_dip * sin_term;
        v_d = (A_dip * T_dip / (2*pi)) * (cos_term - 1);
        y_d = y_stand + (A_dip * T_dip^2 / (4*pi^2)) * sin_term ...
                      - (A_dip * T_dip / (2*pi)) * tau_dip;
    
    elseif t < t_end_push
        % ---------------------- Phase 2b: Push-Off -----------------------
        % The COM accelerates upward from y_squat to y_takeoff using a 
        % 5th-order polynomial ensuring continuous position, velocity, and 
        % acceleration profiles up to takeoff.
        tau_push = t - t_end_dip;
        s = tau_push / T_push;
        P_end = y_takeoff - y_squat;    % Total displacement
        V_end = vf_takeoff * T_push;    % Scaled target velocity
        
        % Quintic polynomial coefficients for normalized s ∈ [0,1]
        c3 = 10*P_end - 4*V_end;
        c4 = -15*P_end + 7*V_end;
        c5 = 6*P_end - 3*V_end;

        % Normalized powers of s
        s2 = s*s; s3 = s*s2; s4 = s*s3; s5 = s*s4;

        % Compute normalized position, velocity, acceleration
        p_norm = c3*s3 + c4*s4 + c5*s5;
        v_norm = 3*c3*s2 + 4*c4*s3 + 5*c5*s4;
        a_norm = 6*c3*s + 12*c4*s2 + 20*c5*s3;

        % Scale back to physical units
        y_d = y_squat + p_norm;
        v_d = v_norm / T_push;
        a_d = a_norm / (T_push^2);
    
    else
        % ---------------------- Phase 3: Flight --------------------------
        % After takeoff, COM follows ballistic flight under gravity.
        % Vertical velocity decreases linearly; acceleration = -g.
        tau_flight = t - t_end_push;
        y_d = y_takeoff + vf_takeoff * tau_flight - 0.5 * g * tau_flight^2;
        v_d = vf_takeoff - g * tau_flight;
        a_d = -g;
    end
    
    % =====================================================================
    % --------------------- Calculate Trajectory of COMx ------------------
    % =====================================================================
    
    if t < t_end_hold
        % ---------------------- Phase 1: Hold ----------------------------
        % No horizontal motion during the initial stance.
        x_d = 0; vx_d = 0; ax_d = 0;
        
    elseif t < t_end_dip
        % ---------------------- Phase 2a: Forward Lean -------------------
        % Smooth forward lean from x=0 to x_squat_lean using quintic profile.
        [x_d, vx_d, ax_d] = quintic_poly(t, t_end_hold, t_end_dip, ...
                                         0, x_squat_lean, 0, 0, 0, 0);
        
    elseif t < t_end_push
        % ---------------------- Phase 2b: Lean to Takeoff ----------------
        % Continue leaning from x_squat_lean to x_takeoff using another quintic.
        [x_d, vx_d, ax_d] = quintic_poly(t, t_end_dip, t_end_push, ...
                                         x_squat_lean, x_takeoff, 0, 0, 0, 0);
    
    else
        % ---------------------- Phase 3: Flight --------------------------
        % After takeoff, COM moves horizontally at constant velocity.
        vx_takeoff = 0;
        tau_flight = t - t_end_push;
        x_d = x_takeoff + vx_takeoff * tau_flight;
        vx_d = vx_takeoff;
        ax_d = 0;
    end
    
    % =====================================================================
    % ---------------------------- Output Results -------------------------
    % =====================================================================
    
    p_COM_d = [x_d; y_d];
    v_COM_d = [vx_d; v_d];
    a_COM_d = [ax_d; a_d];

end

function [p, v, a] = quintic_poly(t, t0, tf, p0, pf, v0, vf, a0, af)
    % ---------------------------------------------------------------------
    % Generates a smooth position, velocity, and acceleration trajectory 
    % between boundary conditions using a 5th-order polynomial.
    % Ensures continuity in p, v, and a at both endpoints.
    %
    % Inputs:
    %   t   - current time
    %   t0  - start time
    %   tf  - final time
    %   p0  - initial position
    %   pf  - final position
    %   v0  - initial velocity
    %   vf  - final velocity
    %   a0  - initial acceleration
    %   af  - final acceleration
    %
    % Outputs:
    %   p   - interpolated position
    %   v   - interpolated velocity
    %   a   - interpolated acceleration
    % ---------------------------------------------------------------------

    % Compute total duration
    T = tf - t0;
    if T <= 0
        p = pf;
        v = vf;
        a = af;
        return;
    end

    % Coefficients ensuring continuity in p, v, and a at both endpoints
    c0 = p0;
    c1 = v0;
    c2 = 0.5 * a0;
    c3 = (20*(pf-p0) - (8*vf + 12*v0)*T - (3*af - a0)*T^2) / (2*T^3);
    c4 = (30*(p0-pf) + (14*vf + 16*v0)*T + (3*af - 2*a0)*T^2) / (2*T^4);
    c5 = (12*(pf-p0) - (6*vf + 6*v0)*T - (af - a0)*T^2) / (2*T^5);

    % Time since start of motion
    tau = t - t0;
    tau2 = tau^2; tau3 = tau^3; tau4 = tau^4; tau5 = tau^5;

    % Evaluate position, velocity, and acceleration
    p = c0 + c1*tau + c2*tau2 + c3*tau3 + c4*tau4 + c5*tau5;
    v = c1 + 2*c2*tau + 3*c3*tau2 + 4*c4*tau3 + 5*c5*tau4;
    a = 2*c2 + 6*c3*tau + 12*c4*tau2 + 20*c5*tau3;
end