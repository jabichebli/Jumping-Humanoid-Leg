function animate_leg_group(t, X, params, group_scenarios, phase_duration, group_name, filename)
% animate_leg_group Animates grouped leg scenarios and saves as a GIF.
%
% This function creates a smooth animation for a sequence of leg movement
% scenarios (e.g., 'Crouch', 'Push', 'Flight'). It interpolates the data for
% a fixed frame rate, visualizes the forces applied, and saves the final
% animation as a GIF file.
%
% Inputs:
%   t               - Time vector from the simulation.
%   X               - State matrix from the simulation.
%   params          - Struct containing the physical model parameters.
%   group_scenarios - Cell array defining the sequence of phases.
%   phase_duration  - Duration of each phase in the sequence.
%   group_name      - A string used for the figure window title.
%   filename        - (Optional) A string for the output GIF file name. 
%                     Defaults to 'leg_group_animation.gif'.
% =========================================================================
% --- 1. Setup and Input Handling ---
% =========================================================================
if nargin < 7
    filename = 'leg_group_animation.gif';
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
fig = figure('Name', ['Animation: ' char(group_name)], 'Color', 'w');
hold on; 
grid on; 
axis equal;
axis([-0.25, 0.25, -0.1, 0.5]); % Set fixed axis limits
xlabel('X Position (m)');
ylabel('Y Position (m)');

% --- STYLE CHANGE: Define custom marker and line properties ---
com_marker_radius = 0.02;
com_marker_color = 'k';
com_line_width = 1.5;
leg_line_width = 2.5;
force_arrow_color = 'g';
force_arrow_width = 3;

% =========================================================================
% --- 4. Animation Loop and GIF Generation ---
% =========================================================================
for k = 1:length(t_anim)
    cla;
    line([-1, 1], [0, 0], 'Color', [0.3 0.3 0.3], 'LineWidth', 2);
    
    % --- Extract and calculate state for the current frame ---
    tk = t_anim(k);
    q_k = X_anim(k, 1:5)';
    x=q_k(1); y=q_k(2); q1=q_k(3); q2=q_k(4); q3=q_k(5);
    
    hip   = [x; y];
    knee  = [x - params.l1*sin(q1); y - params.l1*cos(q1)];
    foot  = auto_pfoot(params.l1, params.l2, q1, q2, x, y);
    torso = [x - params.l3*sin(q3); y + params.l3*cos(q3)]; 
    pCOM  = auto_pCOM(params.d1,params.d2,params.d3,params.l1,params.m1,params.m2,params.m3,q1,q2,q3,x,y);
    
    % --- Determine current phase and force for title and plotting ---
    phase_index = min(floor(tk / phase_duration) + 1, length(group_scenarios)); 
    status_text = group_scenarios{phase_index}.name;
    display_force = group_scenarios{phase_index}.force;
    [instantaneous_force, ~] = get_grouped_force(tk, group_scenarios, phase_duration);
    
    % Set the title for the current frame
    title_str = sprintf('%s: F = [%.1f, %.1f] N | t = %.2f s', status_text, display_force(1), display_force(2), tk);
    title(title_str, 'FontSize', 12);
    
    % --- STYLE CHANGE: Plot leg/torso with markers ---
    plot([hip(1), knee(1), foot(1)], [hip(2), knee(2), foot(2)], 'b-o','LineWidth', leg_line_width);
    plot([hip(1), torso(1)], [hip(2), torso(2)], 'r-o','LineWidth', leg_line_width);
    
    % --- STYLE CHANGE: Plot the CUSTOM Center of Mass marker (circle + cross) ---
    com_x = pCOM(1);
    com_y = pCOM(2);
    theta = 0:0.1:2*pi;
    plot(com_x + com_marker_radius*cos(theta), com_y + com_marker_radius*sin(theta), 'Color', com_marker_color, 'LineWidth', com_line_width);
    plot([com_x - com_marker_radius, com_x + com_marker_radius], [com_y, com_y], 'Color', com_marker_color, 'LineWidth', com_line_width);
    plot([com_x, com_x], [com_y - com_marker_radius, com_y + com_marker_radius], 'Color', com_marker_color, 'LineWidth', com_line_width);
    
    % --- Plot the force vector arrow when applicable ---
    if norm(instantaneous_force) > 0.1
        quiver(hip(1), hip(2), instantaneous_force(1)*0.02, instantaneous_force(2)*0.02, ...
               force_arrow_color, 'LineWidth', force_arrow_width, 'MaxHeadSize', 0.5);
    end
    
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

% % animate_leg_group.m
% function animate_leg_group(t, X, params, group_scenarios, phase_duration, group_name)
% 
%     figure('Name', ['Animation: ' char(group_name)], 'Position', [200, 200, 700, 600]);
% 
%     playback_speed = 2.0;
%     t_anim = (t(1) : 0.03*playback_speed : t(end))';
% 
%     % Define custom marker properties for COM
%     com_marker_radius = 0.02;
%     com_marker_color = 'k';
%     com_line_width = 1.5;
% 
%     for k = 1:length(t_anim)
%         tk = t_anim(k);
%         cla; hold on; grid on; axis equal;
% 
%         q = interp1(t, X(:,1:5), tk);
%         x=q(1); y=q(2); q1=q(3); q2=q(4); q3=q(5);
% 
%         % Calculate all body positions
%         hip   = [x; y];
%         knee  = [x - params.l1*sin(q1); y - params.l1*cos(q1)];
%         foot  = auto_pfoot(params.l1, params.l2, q1, q2, x, y);
%         foot  = foot(:);
%         torso = [x - params.l3*sin(q3); y + params.l3*cos(q3)];
%         pCOM = auto_pCOM(params.d1, params.d2, params.d3, params.l1, params.m1, params.m2, params.m3, q1, q2, q3, x, y);
% 
%         % Plot the leg and custom COM marker
%         plot([hip(1), knee(1), foot(1)], [hip(2), knee(2), foot(2)], 'b-o','LineWidth',2.5);
%         plot([hip(1), torso(1)], [hip(2), torso(2)], 'r-o','LineWidth',2.5);
% 
%         % --- Plot the CUSTOM Center of Mass marker ---
%         com_x = pCOM(1);
%         com_y = pCOM(2);
%         theta = 0:0.1:2*pi;
%         plot(com_x + com_marker_radius*cos(theta), com_y + com_marker_radius*sin(theta), 'Color', com_marker_color, 'LineWidth', com_line_width);
% 
%         plot([com_x - com_marker_radius, com_x + com_marker_radius], [com_y, com_y], 'Color', com_marker_color, 'LineWidth', com_line_width);
%         plot([com_x, com_x], [com_y - com_marker_radius, com_y + com_marker_radius], 'Color', com_marker_color, 'LineWidth', com_line_width);
% 
%         % --- Title and Force Logic ---
%         phase_index = floor(tk / phase_duration) + 1;
%         phase_index = min(phase_index, length(group_scenarios)); 
%         status_text = group_scenarios{phase_index}.name;
%         display_force = group_scenarios{phase_index}.force;
%         [instantaneous_force, ~] = get_grouped_force(tk, group_scenarios, phase_duration);
% 
%         title_str = sprintf('%s | F = [%.1f, %.1f] N', status_text, display_force(1), display_force(2));
%         title(title_str, 'FontSize', 14);
% 
%         % Draw the force arrow only when the force is actually being applied
%         if norm(instantaneous_force) > 0.1
%             quiver(hip(1), hip(2), instantaneous_force(1)*0.02, instantaneous_force(2)*0.02, ...
%                    'g', 'LineWidth', 3, 'MaxHeadSize', 0.5);
%         end
% 
%         xlabel('X [m]'); ylabel('Y [m]');
%         axis([-0.8 0.8 -0.1 0.7]);
%         drawnow;
%     end
% end