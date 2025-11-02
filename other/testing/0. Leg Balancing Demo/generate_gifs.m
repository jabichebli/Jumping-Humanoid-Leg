% animate_leg_group.m
function generate_gifs(t, X, params, group_scenarios, phase_duration, group_name)
    % Exports the animation to a high-quality GIF file.
    
    fig = figure('Name', ['Animation: ' char(group_name)], 'Position', [200, 200, 700, 600]);
    
    % --- GIF Export Setup ---
    % Create a clean filename (e.g., "Horizontal Pushes" -> "Horizontal_Pushes.gif")
    filename = [strrep(char(group_name), ' ', '_') '.gif'];
    
    % Set playback speed to 1.0 for a real-time GIF
    playback_speed = 1.0; 
    delay_time = 0.03 * playback_speed; % Time between frames in the GIF
    t_anim = (t(1) : delay_time : t(end))';
    
    % --- COM Marker Setup ---
    com_marker_radius = 0.02;
    com_marker_color = 'k';
    com_line_width = 1.5;
    
    for k = 1:length(t_anim)
        tk = t_anim(k);
        cla; hold on; grid on; axis equal;
        
        % (All your plotting code is the same...)
        q = interp1(t, X(:,1:5), tk);
        x=q(1); y=q(2); q1=q(3); q2=q(4); q3=q(5);
        
        hip   = [x; y];
        knee  = [x - params.l1*sin(q1); y - params.l1*cos(q1)];
        foot  = auto_pfoot(params.l1, params.l2, q1, q2, x, y);
        foot  = foot(:);
        torso = [x - params.l3*sin(q3); y + params.l3*cos(q3)];
        pCOM = auto_pCOM(params.d1, params.d2, params.d3, params.l1, params.m1, params.m2, params.m3, q1, q2, q3, x, y);

        plot([hip(1), knee(1), foot(1)], [hip(2), knee(2), foot(2)], 'b-o','LineWidth',2.5);
        plot([hip(1), torso(1)], [hip(2), torso(2)], 'r-o','LineWidth',2.5);
        
        com_x = pCOM(1); com_y = pCOM(2);
        theta = 0:0.1:2*pi;
        plot(com_x + com_marker_radius*cos(theta), com_y + com_marker_radius*sin(theta), 'Color', com_marker_color, 'LineWidth', com_line_width);
        plot([com_x - com_marker_radius, com_x + com_marker_radius], [com_y, com_y], 'Color', com_marker_color, 'LineWidth', com_line_width);
        plot([com_x, com_x], [com_y - com_marker_radius, com_y + com_marker_radius], 'Color', com_marker_color, 'LineWidth', com_line_width);
        
        phase_index = floor(tk / phase_duration) + 1;
        phase_index = min(phase_index, length(group_scenarios)); 
        status_text = group_scenarios{phase_index}.name;
        display_force = group_scenarios{phase_index}.force;
        [instantaneous_force, ~] = get_grouped_force(tk, group_scenarios, phase_duration);
        
        title_str = sprintf('%s | F = [%.1f, %.1f] N', status_text, display_force(1), display_force(2));
        title(title_str, 'FontSize', 14);
        
        if norm(instantaneous_force) > 0.1
            quiver(hip(1), hip(2), instantaneous_force(1)*0.02, instantaneous_force(2)*0.02, ...
                   'g', 'LineWidth', 3, 'MaxHeadSize', 0.5);
        end
        
        xlabel('X [m]'); ylabel('Y [m]');
        axis([-0.4 0.4 -0.1 0.7]);
        drawnow;
        
        % --- Frame Capture and GIF Writing ---
        frame = getframe(fig); % Capture the entire figure as a frame
        im = frame2im(frame);
        [imind, cm] = rgb2ind(im, 256); % Convert RGB to indexed image for GIF
        
        % Write to the GIF File 
        if k == 1 
            % For the first frame, create the file
            imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', delay_time); 
        else 
            % For subsequent frames, append to the existing file
            imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delay_time); 
        end 
    end
end