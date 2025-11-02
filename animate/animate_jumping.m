% -------------------------------------------------------------------------
% animate_jumping.m
% -------------------------------------------------------------------------
% Created by: Jason Abi Chebli
% Last Modified: 30-October-2025
% -------------------------------------------------------------------------
function animate_jumping(t, X, params, mode, filename)
    % ---------------------------------------------------------------------
    % This function interpolates the simulation data to create a smooth 
    % animation. It can either display the motion in real-time or save it 
    % as a GIF in the 'media/' folder.
    %
    % INPUTS:
    %   t        - Time vector from simulation (Nx1)
    %   X        - State trajectory matrix (NxM), where M >= 5
    %   params   - Struct of physical and geometric model parameters
    %   mode     - "gif" or "realtime" (string) to choose animation type
    %   filename - (Optional) Output GIF filename 
    %              (default: 'media/leg_leaping_animation.gif')
    %
    % Example:
    %   animate_jumping(t_all, X_all, params, "gif");
    %   animate_jumping(t_all, X_all, params, "realtime");
    % ---------------------------------------------------------------------

    % =====================================================================
    % ----------------- Initialization & Input Handling -------------------
    % =====================================================================
    
    % --- Set default filename if not provided ---
    if nargin < 5
        filename = "media/leg_leaping_animation.gif";
    end
    
    % --- Set default animation mode if not provided ---
    if nargin < 4
        mode = "realtime";
    end
    mode = string(mode);  % ensure mode is a string
    
    % --- Ensure media folder exists for saving GIF ---
    [media_dir, ~, ~] = fileparts(filename);
    if ~isempty(media_dir) && ~isfolder(media_dir)
        mkdir(media_dir);
    end
    
    % =====================================================================
    % -------------- Resample Data for Smooth Animation -------------------
    % =====================================================================
    
    frame_rate = 30; % Desired animation frames per second (fps)
    t_anim = (t(1):1/frame_rate:t(end))'; % Create new time vector at target fps
    X_anim = interp1(t, X, t_anim); % Interpolate state data to match new time
    
    % =====================================================================
    % ---------------------------- Figure Setup ---------------------------
    % =====================================================================
    
    fig = figure('Name','Leg Leaping Animation','Color','w');
    hold on; grid on; axis equal;
    
    % --- Calculate dynamic axis limits ---
    max_y = max(X(:,2)) + params.l3 + 0.1;
    max_x = max(abs(X(:,1))) + params.l1 + 0.1;
    axis([-max_x, max_x, -0.1, max_y]);
    xlabel('X Position (m)'); 
    ylabel('Y Position (m)');
    
    % --- Graphics handles & style ---
    com_marker_radius = 0.02;
    com_marker_color  = 'k';
    com_line_width    = 1.5;

    % =====================================================================
    % ----------------------- Main Animation Loop -------------------------
    % =====================================================================
    
    for k = 1:length(t_anim)
        % --- Clear and set up frame ---
        cla; % Clear previous frame
        
        % Plot ground line
        line([-1,1],[0,0],'Color',[0.3 0.3 0.3],'LineWidth',2);
        title(sprintf('Leg Leap: t = %.2f s', t_anim(k)));
        
        % --- Extract state for current frame ---
        % Assumes state vector X is [x, y, q1, q2, q3, ...]
        q_k = X_anim(k,1:5)';
        x   = q_k(1); % hip x-position
        y   = q_k(2); % hip y-position
        q1  = q_k(3); % thigh angle
        q2  = q_k(4); % calf angle
        q3  = q_k(5); % torso angle
        
        % --- Forward Kinematics (Compute joint positions) ---
        hip   = [x; y];
        knee  = [x - params.l1*sin(q1); y - params.l1*cos(q1)];
        foot  = auto_pfoot(params.l1, params.l2, q1, q2, x, y);
        torso = [x - params.l3*sin(q3); y + params.l3*cos(q3)];
        
        % Calculate full-body Center of Mass
        pCOM  = auto_pCOM(params.d1, params.d2, params.d3, ...
                          params.l1, params.m1, params.m2, params.m3, ...
                          q1, q2, q3, x, y);
                      
        % --- Plot robot links ---
        plot([hip(1),knee(1),foot(1)],[hip(2),knee(2),foot(2)],'b-o','LineWidth',2.5);
        plot([hip(1),torso(1)],[hip(2),torso(2)],'r-o','LineWidth',2.5);
        
        % --- Plot Center of Mass (COM) ---
        com_x = pCOM(1); 
        com_y = pCOM(2);
        theta = 0:0.1:2*pi;
        
        % Plot circle
        plot(com_x + com_marker_radius*cos(theta), ...
             com_y + com_marker_radius*sin(theta), ...
             'Color', com_marker_color, 'LineWidth', com_line_width);
         
        % Plot crosshairs
        plot([com_x - com_marker_radius, com_x + com_marker_radius], ...
             [com_y, com_y], 'Color', com_marker_color, 'LineWidth', com_line_width);
        plot([com_x, com_x], ...
             [com_y - com_marker_radius, com_y + com_marker_radius], ...
             'Color', com_marker_color, 'LineWidth', com_line_width);
        
        % --- Update figure and save frame ---
        drawnow;
        
        if mode == "gif"
            % Capture the frame
            frame = getframe(fig);
            [im, map] = rgb2ind(frame.cdata, 256);
            
            % Write to GIF file
            if k == 1
                % Create new GIF
                imwrite(im, map, filename, 'gif', 'LoopCount', inf, 'DelayTime', 1/frame_rate);
            else
                % Append to existing GIF
                imwrite(im, map, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 1/frame_rate);
            end
        else
            % Pause for real-time playback
            pause(1/frame_rate); 
        end
    end
    
    % =====================================================================
    % ------------------------- Post-Processing ---------------------------
    % =====================================================================
    
    if mode == "gif"
        fprintf('Animation GIF saved to: %s\n', filename);
    end
    
end