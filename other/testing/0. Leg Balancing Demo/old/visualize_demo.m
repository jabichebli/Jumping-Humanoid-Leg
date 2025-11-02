% visualize_demo.m
function visualize_demo(results, params, scenarios)
    % This function creates comparative state plots and then generates
    % a SEPARATE animation figure for each scenario.

    num_scenarios = length(results);
    colors = lines(num_scenarios); % Generate distinct colors for plots

    % --- 1. Comparative State Plots (One Figure) ---
    figure('Name', 'Comparative State Plots', 'Position', [50, 400, 1200, 400]);
    
    % Plot Hip X Position
    subplot(1, 2, 1);
    hold on; grid on;
    for i = 1:num_scenarios
        plot(results{i}.t, results{i}.X(:, 1), 'LineWidth', 2, 'Color', colors(i,:), 'DisplayName', results{i}.name);
    end
    title('Hip X Position vs. Time');
    xlabel('Time (s)'); ylabel('X (m)');
    legend('Location', 'best');

    % Plot Hip Y Position
    subplot(1, 2, 2);
    hold on; grid on;
    for i = 1:num_scenarios
        plot(results{i}.t, results{i}.X(:, 2), 'LineWidth', 2, 'Color', colors(i,:), 'DisplayName', results{i}.name);
    end
    title('Hip Y Position vs. Time');
    xlabel('Time (s)'); ylabel('Y (m)');
    legend('Location', 'best');

    % --- 2. Individual Animations (Separate Figures) ---
    
    playback_speed = 2.0; % Set to 1.0 for real-time

    % Loop through each scenario to create a separate figure and animation
    for i = 1:num_scenarios
        
        % Create a new figure for this specific scenario
        figure('Name', ['Animation: ' results{i}.name], 'Position', [100 + 50*i, 100 + 50*i, 600, 500]);
        
        t_max = results{i}.t(end);
        t_anim = (0 : 0.03*playback_speed : t_max)';
        
        % Animation loop for the current figure
        for k = 1:length(t_anim)
            tk = t_anim(k);
            cla; hold on; grid on; axis equal;
            
            % Use interpolation for a smoother animation
            q = interp1(results{i}.t, results{i}.X(:, 1:5), tk);
            x=q(1); y=q(2); q1=q(3); q2=q(4); q3=q(5);
            
            % Calculate joint positions
            hip   = [x; y];
            knee  = [x - params.l1*sin(q1); y - params.l1*cos(q1)];
            foot  = auto_pfoot(params.l1, params.l2, q1, q2, x, y);
            foot  = foot(:);
            torso = [x - params.l3*sin(q3); y + params.l3*cos(q3)];

            % Plot the leg
            plot([hip(1), knee(1), foot(1)], [hip(2), knee(2), foot(2)], 'b-o','LineWidth',2.5);
            plot([hip(1), torso(1)], [hip(2), torso(2)], 'r-o','LineWidth',2.5);
            plot(0,0,'kx','MarkerSize',12,'LineWidth',2); % Ground
            
            % Add a force vector arrow
            force_vec = scenarios{i}.force_func(tk);
            if norm(force_vec) > 0.1 
                quiver(hip(1), hip(2), force_vec(1)*0.02, force_vec(2)*0.02, 'g', 'LineWidth', 3, 'MaxHeadSize', 0.5);
            end

            title(sprintf('%s | Time: %.2f s', results{i}.name, tk), 'FontSize', 14);
            xlabel('X (m)'); ylabel('Y (m)');
            axis([-0.5 0.5 -0.1 0.6]); % Keep axes consistent
            
            drawnow;
        end
    end
end