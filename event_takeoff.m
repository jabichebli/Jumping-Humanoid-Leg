function [value, isterminal, direction] = event_takeoff(t, x, params)
    % EVENT_TAKEOFF detects loss of contact when Fy < threshold
    % but disables detection before min_time

    min_time = 3.6;          % seconds
    Fy_threshold = -5e-3;    % allow small negative noise

    % Default
    Fy = 0;

    fprintf('[event_takeoff] called at t=%.3f, Fy=%.3f\n', t, Fy);

    % Compute ground reaction
    try
        [~, lambda] = dynamics_stance(t, x, params);
        Fy = lambda(2);
    catch
        % fallback if lambda not valid
    end

    if t < min_time
        value = 1;  % always positive
    elseif t < min_time + 1e-3
        % Blend smoothly to avoid discontinuity
        alpha = (t - min_time) / 1e-3;
        value = (1 - alpha)*1 + alpha*(Fy - Fy_threshold);
    else
        value = Fy - Fy_threshold;
    end

    isterminal = 1;
    direction = -1;
end



% function [value,isterminal,direction] = event_takeoff(t,x,params)
% 
%     [~,lambda] = dynamics_stance(t,x,params);
%     Fy = lambda(2);   % vertical Ground Force
%     value = Fy;       % event when vertical Ground Force = 0
%     isterminal = 1;   % stop integration
%     direction = -1;   % only detect downward crossing
% end