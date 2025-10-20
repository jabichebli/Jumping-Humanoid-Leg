function animate_leg(t, X, params)
% -------------------------------------------------------------------------
% animate_leg.m
% -------------------------------------------------------------------------
% Simple 2D animation of the 3-link hopping leg trajectory.
%
% INPUTS:
%   t     : time vector
%   X     : state trajectory (Nx10)
%   params: struct with link lengths
%
% Visualizes hip, knee, foot, and torso in Cartesian coordinates.
% -------------------------------------------------------------------------

    % Unpack parameters
    m1=params.m1; m2=params.m2; m3=params.m3;
    I1=params.I1; I2=params.I2; I3=params.I3;
    l1=params.l1; l2=params.l2; l3=params.l3;
    d1=params.d1; d2=params.d2; d3=params.d3;
    g = params.g;

    figure(1); clf; hold on;
    axis([-1 1 -0.5 1.5]);
    axis equal;
    grid on;
    xlabel('X'); ylabel('Y');
    plot(0,0,'kx','MarkerSize',10,'LineWidth',2); % ground ref

    h_title = title(sprintf('Time: %.2f s', 0));
    
    % Pre-create plot handles
    h_leg   = plot(nan, nan, 'b-o', 'LineWidth', 2);   % leg links
    h_torso = plot(nan, nan, 'r-o', 'LineWidth', 2);   % torso link
    h_com   = plot(nan, nan, 'ko', 'MarkerFaceColor', 'y'); % COM

    % Animation loop
    realtime = true;   % set false if you want max speed
    dt = t(2) - t(1);  % simulation timestep
    tic;
    for k = 1:10:length(t)
        q = X(k,1:5).';
        x = q(1); y = q(2); q1=q(3); q2=q(4); q3=q(5);
    
        % Joint positions
        hip   = [x; y];
        knee  = [x - l1*sin(q1); y - l1*cos(q1)];
        foot  = auto_pfoot(l1,l2,q1,q2,x,y);
        foot  = foot(:);
        torso = [x - l3*sin(q3); y + l3*cos(q3)];
        pCOM  = auto_pCOM(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,x,y);
    
        % Update plot data
        set(h_leg,   'XData', [hip(1), knee(1), foot(1)], ...
                     'YData', [hip(2), knee(2), foot(2)]);
        set(h_torso, 'XData', [hip(1), torso(1)], ...
                     'YData', [hip(2), torso(2)]);
        set(h_com,   'XData', pCOM(1), 'YData', pCOM(2));

        set(h_title, 'String', sprintf('Time: %.2f s', t(k)));
    
        drawnow limitrate nocallbacks;
    
        % Real-time sync
        if realtime
            elapsed = toc;
            target_time = t(k);
            pause(max(0, target_time - elapsed));
        end
    end
end
