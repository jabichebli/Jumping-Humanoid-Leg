function [p_COMy_d, dp_COMy_d, ddp_COMy_d] = desired_COM_y(t, params)
% -------------------------------------------------------------------------
% Returns desired CoM vertical position, velocity, and acceleration
% for hopping, with physically feasible acceleration limits.
% -------------------------------------------------------------------------

    % --- Parameters ---
    y_stand   = params.y_stand;
    y_squat   = params.y_squat;
    y_takeoff = params.y_takeoff;
    T_hold    = params.T_hold;
    T_squat   = params.T_squat;
    T_push    = params.T_push;
    
    % Max vertical acceleration (m/s^2) to keep torques realistic
    ddp_max = 15;   % ~1.5 g, adjust if necessary

    % Default outputs
    p_COMy_d  = y_stand;
    dp_COMy_d = 0;
    ddp_COMy_d = 0;

    % --- Phases ---
    if t < T_hold
        % Hold phase
        p_COMy_d  = y_stand;
        dp_COMy_d = 0;
        ddp_COMy_d = 0;

    elseif t < T_hold + T_squat
        % Squat down: cosine interpolation
        tau = (t - T_hold) / T_squat;
        dy = y_stand - y_squat;
        p_COMy_d  = y_stand - 0.5*dy*(1 - cos(pi*tau));
        dp_COMy_d = -0.5*dy*(pi/T_squat)*sin(pi*tau);
        ddp_COMy_d = -0.5*dy*(pi/T_squat)^2*cos(pi*tau);

        % Limit acceleration
        ddp_COMy_d = max(min(ddp_COMy_d, ddp_max), -ddp_max);

    elseif t < T_hold + T_squat + T_push
        % Push up: cosine interpolation
        tau = (t - (T_hold + T_squat)) / T_push;
        dy = y_takeoff - y_squat;
        p_COMy_d  = y_squat + 0.5*dy*(1 - cos(pi*tau));
        dp_COMy_d = 0.5*dy*(pi/T_push)*sin(pi*tau);
        ddp_COMy_d = 0.5*dy*(pi/T_push)^2*cos(pi*tau);

        % Limit acceleration
        ddp_COMy_d = max(min(ddp_COMy_d, ddp_max), -ddp_max);

    else
        % After push
        p_COMy_d  = y_takeoff;
        dp_COMy_d = 0;
        ddp_COMy_d = 0;
    end
end
