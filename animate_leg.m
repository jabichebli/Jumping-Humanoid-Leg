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

    l1 = params.l1; l2 = params.l2; l3 = params.l3;

    figure; hold on;
    axis equal;
    xlabel('X [m]'); ylabel('Y [m]');
    title('Hopping Leg Animation');
    grid on;

    for k = 1:10:length(t)
        q = X(k,1:5).';
        x = q(1); y = q(2); q1=q(3); q2=q(4); q3=q(5);

        % Joint positions
        hip   = [x; y];
        knee  = [x - l1*sin(q1); y - l1*cos(q1)];
        foot  = auto_pfoot(l1,l2,q1,q2,x,y);
        foot  = foot(:);

        torso = [x - l3*sin(q3); y + l3*cos(q3)];

        % Plot
        cla;
        plot([hip(1), knee(1), foot(1)], [hip(2), knee(2), foot(2)], 'b-o','LineWidth',2);
        plot([hip(1), torso(1)], [hip(2), torso(2)], 'r-o','LineWidth',2);
        plot(0,0,'kx','MarkerSize',10,'LineWidth',2); % ground ref
        axis([-1 1 0 2]); % fixed axes
        drawnow;
    end
end
