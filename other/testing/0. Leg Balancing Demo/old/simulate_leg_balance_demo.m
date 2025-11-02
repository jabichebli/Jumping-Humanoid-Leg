% simulate_leg_balance_demo.m
function simulate_leg_balance_demo()
    % Add the path to your auto-generated functions
    % addpath('./auto'); 
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
    
    % Initial state
    x0 = [0; 0.2970; 0.2; 0.2; 0; 0; 0; 0; 0; 0];         
    tspan = [0 10];
    opts = odeset('RelTol',1e-8,'AbsTol',1e-8);

% --- DEFINE FORCE SCENARIOS ---
    push_start_time = 2.0; % s
    push_duration = 0.2;   % s
    
    % Scenario 1: No external force (the baseline)
    force1 = @(t) [0; 0];
    
    % Scenario 2: A short push to the right
    force2 = @(t) (t > push_start_time && t < push_start_time + push_duration) * [8; 0];
    
    % Scenario 3: A short push to the left
    force3 = @(t) (t > push_start_time && t < push_start_time + push_duration) * [-8; 0];
    
    % Scenario 4: A short push upward
    force4 = @(t) (t > push_start_time && t < push_start_time + push_duration) * [0; 10];
    
    % Scenario 5: A short push downward
    force5 = @(t) (t > push_start_time && t < push_start_time + push_duration) * [0; -8];

    % Scenario 6: A combination push (down and right)
    force6 = @(t) (t > push_start_time && t < push_start_time + push_duration) * [6; -6];
    
    scenarios = { ...
        struct('name', 'No Force', 'force_func', force1), ...
        struct('name', 'Rightward Push', 'force_func', force2), ...
        struct('name', 'Leftward Push', 'force_func', force3), ...
        struct('name', 'Upward Push', 'force_func', force4), ...
        struct('name', 'Downward Push', 'force_func', force5), ...
        struct('name', 'Combo Push (Down-Right)', 'force_func', force6) ...
    };

    num_scenarios = length(scenarios);
    results = cell(1, num_scenarios);

    % --- RUN SIMULATIONS ---
    fprintf('Running simulations...\n');
    for i = 1:num_scenarios
        fprintf('  - Running Scenario %d: %s\n', i, scenarios{i}.name);
        
        % Define the ODE function with the specific force for this scenario
        ode_func = @(t, x) leg_push_ode(t, x, params, scenarios{i}.force_func(t));
        
        % Solve the ODE
        [t, X] = ode45(ode_func, tspan, x0, opts);
        
        % Store results
        results{i}.t = t;
        results{i}.X = X;
        results{i}.name = scenarios{i}.name;
    end
    fprintf('All simulations complete!\n');

    % --- VISUALIZE RESULTS ---
    % This new function will handle plotting and animation for the demo
    visualize_demo(results, params, scenarios);
end