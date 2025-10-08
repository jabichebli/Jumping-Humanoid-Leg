function [value, isterminal, direction] = event_takeoff(t,x,params)
    [~,~,lambda] = dynamics_stance(t,x,params);
    Fy = lambda(2);
    value = Fy;          % detect when vertical ground force = 0
    isterminal = 1;      % stop integration
    direction = -1;      % downward crossing only

    % Debug print
    fprintf('[event_takeoff] t=%.3f, Fy=%.4f, value=%.4f\n', t, Fy, value);
end
% 
% function [value, isterminal, direction] = event_takeoff(t, x, params)
%     % EVENT_TAKEOFF  Detects when vertical ground reaction force goes to zero
%     % (i.e., when the foot loses contact with the ground).
% 
%     % Parameters
%     min_time = 0.05;       % prevent early triggering
%     Fy_threshold = -5e-3;  % allow small numerical noise
% 
%     % Ensure column vector
%     x = x(:);
% 
%     % Default fallback
%     Fy = 1;  % start positive
% 
%     % Compute stance dynamics to get constraint forces
%     [~, ~, lambda] = dynamics_stance(t, x, params);
%     Fy = -lambda(2);
% 
%     % Disable detection before min_time
%     if t < min_time
%         value = 1;
%     else
%         value = Fy - Fy_threshold;
%     end
% 
%     isterminal = 1;    % stop integration
%     direction  = -1;   % detect when Fy decreases through threshold
% 
%     % Debug print
%     fprintf('[event_takeoff] t=%.3f, Fy=%.4f, value=%.4f\n', t, Fy, value);
% end



% function [value, isterminal, direction] = event_takeoff(t, x, params)
%     % EVENT_TAKEOFF detects loss of contact when Fy < threshold
%     % but disables detection before min_time
% 
%     min_time = 3.6;          % seconds
%     Fy_threshold = -5e-3;    % allow small negative noise
% 
%     % Default
%     Fy = 0;
% 
%     fprintf('[event_takeoff] called at t=%.3f, Fy=%.3f\n', t, Fy);
% 
%     % Compute ground reaction
%     try
%         [~, lambda] = dynamics_stance(t, x, params);
%         Fy = lambda(2);
%     catch
%         % fallback if lambda not valid
%     end
% 
%     if t < min_time
%         value = 1;  % always positive
%     elseif t < min_time + 1e-3
%         % Blend smoothly to avoid discontinuity
%         alpha = (t - min_time) / 1e-3;
%         value = (1 - alpha)*1 + alpha*(Fy - Fy_threshold);
%     else
%         value = Fy - Fy_threshold;
%     end
% 
%     isterminal = 1;
%     direction = -1;
% end
% 
