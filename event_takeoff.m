function [value,isterminal,direction] = event_takeoff(t,x,params)
    [~,lambda] = dynamics_stance(t,x,params);
    Fy = lambda(2);   % vertical Ground Force
    value = Fy;       % event when vertical Ground Force = 0
    isterminal = 1;   % stop integration
    direction = -1;   % only detect downward crossing
end