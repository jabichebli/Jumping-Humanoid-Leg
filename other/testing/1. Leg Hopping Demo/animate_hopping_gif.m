function animate_jump_gif(t, X, params, filename)
% animate_jump_gif Animates the leg jump simulation and saves it as a GIF.
%
% This function creates a smooth animation at a fixed frame rate by 
% interpolating the simulation data. It then saves the animation as a GIF
% file in the current directory.
%
% Inputs:
%   t        - Time vector from the simulation (t_all).
%   X        - State matrix from the simulation (X_all).
%   params   - Struct containing the physical model parameters.
%   filename - (Optional) A string for the output GIF file name. 
%              Defaults to 'leg_jump_animation.gif'.

% =========================================================================
% --- 1. Setup and Input Handling ---
% =========================================================================
if nargin < 4
    filename = 'leg_jump_animation.gif';
end
fprintf('Generating animation GIF: %s\n', filename);

% =========================================================================
% --- 2. Resample Data for a Constant Frame Rate ---
% =========================================================================
frame_rate = 30; % frames per second
t_anim = (t(1) : 1/frame_rate : t(end))';
X_anim = interp1(t, X, t_anim);

% =========================================================================
% --- 3. Create, Configure Figure, and Define Styles ---
% =========================================================================
fig = figure('Name', 'Jumping Animation', 'Color', 'w');
hold on; 
grid on; 
axis equal;

max_y = max(X(:,2)) + params.l3 + 0.1;
max_x = max(abs(X(:,1))) + params.l1 + 0.1;
axis([-max_x, max_x, -0.1, max_y]);
xlabel('X Position (m)');
ylabel('Y Position (m)');

% --- STYLE CHANGE: Define custom marker properties for COM (outside the loop) ---
com_marker_radius = 0.02;
com_marker_color = 'k';
com_line_width = 1.5;

% =========================================================================
% --- 4. Animation Loop and GIF Generation ---
% =========================================================================
for k = 1:length(t_anim)
    cla; 
    line([-1, 1], [0, 0], 'Color', [0.3 0.3 0.3], 'LineWidth', 2);
    title(sprintf('Leg Jump:  t = %.2f s', t_anim(k)));
    
    % --- Extract and calculate state for the current frame ---
    q_k = X_anim(k, 1:5)';
    x=q_k(1); y=q_k(2); q1=q_k(3); q2=q_k(4); q3=q_k(5);
    
    hip   = [x; y];
    knee  = [x - params.l1*sin(q1); y - params.l1*cos(q1)];
    foot  = auto_pfoot(params.l1, params.l2, q1, q2, x, y);
    torso = [x - params.l3*sin(q3); y + params.l3*cos(q3)]; 
    pCOM  = auto_pCOM(params.d1,params.d2,params.d3,params.l1,params.m1,params.m2,params.m3,q1,q2,q3,x,y);
    
    % --- STYLE CHANGE: Plot leg/torso with hollow markers ---
    plot([hip(1), knee(1), foot(1)], [hip(2), knee(2), foot(2)], 'b-o','LineWidth',2.5);
    plot([hip(1), torso(1)], [hip(2), torso(2)], 'r-o','LineWidth',2.5);
    
    % --- STYLE CHANGE: Plot the CUSTOM Center of Mass marker (circle + cross) ---
    com_x = pCOM(1);
    com_y = pCOM(2);
    theta = 0:0.1:2*pi;
    plot(com_x + com_marker_radius*cos(theta), com_y + com_marker_radius*sin(theta), 'Color', com_marker_color, 'LineWidth', com_line_width);
    plot([com_x - com_marker_radius, com_x + com_marker_radius], [com_y, com_y], 'Color', com_marker_color, 'LineWidth', com_line_width);
    plot([com_x, com_x], [com_y - com_marker_radius, com_y + com_marker_radius], 'Color', com_marker_color, 'LineWidth', com_line_width);
    
    % --- Capture the frame and append it to the GIF file ---
    drawnow;
    frame = getframe(fig);
    [im, map] = rgb2ind(frame.cdata, 256); 
    
    if k == 1
        imwrite(im, map, filename, 'gif', 'LoopCount', inf, 'DelayTime', 1/frame_rate);
    else
        imwrite(im, map, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 1/frame_rate);
    end
end
fprintf('Animation saved successfully! \n');
end