function [value,isterminal,direction] = event_touchdown(t,x,params)
    q = x(1:5);
    x = q(1);
    y = q(2);
    q1 = q(3);
    q2 = q(4);
    l1 = params.l1;
    l2 = params.l2;
    

    % Compute foot position
    pFoot = auto_pfoot(l1,l2,q1,q2,x,y);
    yFoot = pFoot(2);

    if yFoot == 0
disp("touchdown activated")
    end
    value = yFoot;      % event when foot hits ground (y=0)
    isterminal = 1;     % stop integration
    direction = -1;     % detect downward crossing
end