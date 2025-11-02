% simulate_leg_groups.m
function simulate_leg_balancing_demo()

    % addpath('../auto/');

    clear all; close all; clc;
    % ---------------- Simulation Parameters ----------------
    params.m1 = 0.15;   params.m2 = 0.15;   params.m3 = 0.8;
    params.l1 = 0.15;   params.l2 = 0.15;   params.l3 = 0.1; params.w3 = 0.15;
    params.d1 = 0.075;   params.d2 = 0.075;  params.d3 = 0.0;
    params.g  = 9.81;
    params.I1 = (1/12) * params.m1 * params.l1^2;
    params.I2 = (1/12) * params.m2 * params.l2^2;
    params.I3 = (1/12) * params.m3 * (params.w3^2 + params.l3^2);
    params.pCOMy_d = 0.2;
    params.Kp = 50;
    params.Kd = 10;
    
    % --- Coefficient of static friction ---
    params.mu = 0.8;

    % Initial state and solver options
    x0 = [0; 0.2970; 0.2; 0.2; 0; 0; 0; 0; 0; 0];
    opts = odeset('RelTol',1e-6,'AbsTol',1e-6);

    % --- DEFINE THE GROUPS OF SCENARIOS ---
    group1 = { struct('name', 'Downward Push', 'force', [0; -8]), ...
               struct('name', 'Upward Push',   'force', [0; 3.5]) };
    group2 = { struct('name', 'Rightward Push', 'force', [5; 0]), ...
               struct('name', 'Leftward Push',  'force', [-5; 0]) };
    group3 = { struct('name', 'Down-Right Push', 'force', [5; -6]), ...
               struct('name', 'Down-Left Push',  'force', [-5; -6]), ...
               struct('name', 'Up-Right Push',   'force', [3; 1.5]), ...
               struct('name', 'Up-Left Push',    'force', [-4; 1.5]) };
    all_groups = {group1, group2, group3};
    group_names = ["Vertical Pushes", "Horizontal Pushes", "Combination Pushes"];
    phase_duration = 3;

    for g = 1:length(all_groups)
        current_group = all_groups{g};
        current_group_name = group_names(g);
        fprintf('--- Running Simulation for Group: %s ---\n', current_group_name);
        
        num_pushes = length(current_group);
        tspan = [0, num_pushes * phase_duration];
        force_func = @(t) get_grouped_force(t, current_group, phase_duration);
        ode_func = @(t, x) leg_push_ode(t, x, params, force_func(t));
        
        [t, X] = ode45(ode_func, tspan, x0, opts);
        
        fprintf('      -> Calculating and plotting reaction forces...\n');
        lambda_hist = zeros(length(t), 2);
        for i = 1:length(t)
            [~, ~, lambda] = leg_push_ode(t(i), X(i, :)', params, force_func(t(i)));
            lambda_hist(i, :) = lambda';
        end
        Fx = lambda_hist(:, 1);
        Fy = lambda_hist(:, 2);
        
        % =====================================================================
        % --- PLOT UPDATE: Combined Subplot for Ground Reaction Forces ---
        % =====================================================================
        figure('Name', ['Ground Reaction Forces: ' char(current_group_name)]);
        
        % --- Subplot 1: Vertical Force (No Lift-off) ---
        subplot(2,1,1);
        plot(t, Fy, 'b-', 'LineWidth', 2, 'DisplayName', 'F_y (must be > 0)');
        hold on;
        yline(0, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Limit');
        grid on; 
        title(['Vertical Ground Reaction Force: ' char(current_group_name)]);
        ylabel('F_y (N)'); 
        legend('Location', 'best');
        hold off;
        
        % --- Subplot 2: Friction Ratio (No Slip) ---
        subplot(2,1,2);
        ratio = abs(Fx) ./ Fy;
        plot(t, ratio, 'b-', 'LineWidth', 2, 'DisplayName', 'Ratio (must be < \mu)');
        hold on;
        yline(params.mu, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Friction Limit \mu');
        grid on; 
        title('Friction Constraint Check');
        xlabel('Time (s)'); 
        ylabel('$|F_x| / F_y$', 'Interpreter', 'latex');
        legend('Location', 'best');
        ylim([0, params.mu * 2]); % Keep the y-limit reasonable
        hold off;
        
        % =====================================================================
        % --- ANIMATION UPDATE: Save with a unique filename ---
        % =====================================================================
        
        % 1. Create a safe filename (e.g., "Vertical Pushes" -> "Vertical_Pushes")
        safe_group_name = strrep(current_group_name, ' ', '_');
        
        % 2. Define the output filename
        output_filename = sprintf('animation_%s.gif', safe_group_name);
    
        % 3. Call the animation function with all 7 arguments
        animate_leg_group(t, X, params, current_group, phase_duration, ...
                          current_group_name, output_filename);
        
        % The 'generate_gifs' call is commented out as it seems redundant
        % % generate_gifs(t, X, params, current_group, phase_duration, group_names(g));
    end
    fprintf('All simulations and animations are complete!\n');

end