function y_des = desired_COM_height(t, params)
% -------------------------------------------------------------------------
% Returns desired CoM vertical position for hopping with smooth transitions.
% -------------------------------------------------------------------------

    % --- Parameters ---
    y_stand   = params.y_stand;   % 0.22
    y_squat   = params.y_squat;   % 0.08
    y_takeoff = params.y_takeoff; % 0.267
    T_hold    = params.T_hold;    % 2.0
    T_squat   = params.T_squat;   % 0.5
    T_push    = params.T_push;    % 0.3

    if t < T_hold
        % --- Phase 1: hold upright ---
        y_des = y_stand;

    elseif t < T_hold + T_squat
        % --- Phase 2: squat down (smooth cubic) ---
        tau = (t - T_hold) / T_squat;
        %y_des = y_stand + (y_squat - y_stand)*(3*tau^2 - 2*tau^3);  % cubic
         y_des = (1 - tau)*y_stand + tau*y_squat;

    elseif t < T_hold + T_squat + T_push
        % --- Phase 3: push up (smooth cubic) ---
        tau = (t - T_hold - T_squat) / T_push;
        y_des = (1 - tau)*y_squat + tau*y_takeoff;
        % tau = (t - T_hold - T_squat) / T_push;
        % y_des = y_squat + (y_takeoff - y_squat)*(3*tau^2 - 2*tau^3);

    else
        % --- Phase 4: hold extended (ready for takeoff) ---
        y_des = y_takeoff;
    end
end

% 
% 
%     % Max vertical acceleration (m/s^2) to keep torques realistic
%     ddp_max = 15;   % ~1.5 g, adjust if necessary
% 
%     % Default outputs
%     p_COMy_d  = y_stand;
%     dp_COMy_d = 0;
%     ddp_COMy_d = 0;
% 
%     % --- Phases ---
%     if t < T_hold
%         % Hold phase
%         p_COMy_d  = y_stand;
%         dp_COMy_d = 0;
%         ddp_COMy_d = 0;
% 
%     elseif t < T_hold + T_squat
%         % Squat down: cosine interpolation
%         tau = (t - T_hold) / T_squat;
%         dy = y_stand - y_squat;
%         p_COMy_d  = y_stand - 0.5*dy*(1 - cos(pi*tau));
%         dp_COMy_d = -0.5*dy*(pi/T_squat)*sin(pi*tau);
%         ddp_COMy_d = -0.5*dy*(pi/T_squat)^2*cos(pi*tau);
% 
%         % Limit acceleration
%         ddp_COMy_d = max(min(ddp_COMy_d, ddp_max), -ddp_max);
% 
%     elseif t < T_hold + T_squat + T_push
%         % Push up: cosine interpolation
%         tau = (t - (T_hold + T_squat)) / T_push;
%         dy = y_takeoff - y_squat;
%         p_COMy_d  = y_squat + 0.5*dy*(1 - cos(pi*tau));
%         dp_COMy_d = 0.5*dy*(pi/T_push)*sin(pi*tau);
%         ddp_COMy_d = 0.5*dy*(pi/T_push)^2*cos(pi*tau);
% 
%         % Limit acceleration
%         ddp_COMy_d = max(min(ddp_COMy_d, ddp_max), -ddp_max);
% 
%     else
%         % After push
%         p_COMy_d  = y_takeoff;
%         dp_COMy_d = 0;
%         ddp_COMy_d = 0;
%     end
% end
