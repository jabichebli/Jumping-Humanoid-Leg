function [value, isterminal, direction] = event_touchdown(t, x, params)
    q = x(1:5);
    x_hip = q(1);
    y_hip = q(2);
    q1 = q(3);
    q2 = q(4);

    l1 = params.l1;
    l2 = params.l2;

    % Compute foot position (world coordinates)
    pFoot = auto_pfoot(l1, l2, q1, q2, x_hip, y_hip);
    yFoot = pFoot(2);

    % Event condition
    value = yFoot;        % When foot touches the ground (yFoot = 0)
    isterminal = 1;       % Stop integration when event happens
    direction = -1;       % Only detect downward crossings (from + to -)
    fprintf('Touchdown at t=%.3f, yFoot=%.4f\n', t, yFoot);
end
