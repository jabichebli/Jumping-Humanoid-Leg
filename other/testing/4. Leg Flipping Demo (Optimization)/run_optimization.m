clear all; close all; clc;

% 1. Initial Guess for the parameters [q3_takeoff, q3dot_takeoff, vf_takeoff, timing_ratio]
p0 = [-0.1, 6.0, 0.5, 0.45]; % Your script's starting values

% 2. Lower and Upper Bounds
% Helps keep the solver in a reasonable search space
lb = [-pi/2,  2.0, 0.2, 0.3];  % [rad, rad/s, m/s, ratio]
ub = [pi/2,   20.0, 2.0, 0.9]; % [rad, rad/s, m/s, ratio]

% 3. Set fmincon options
options = optimoptions('fmincon', ...
    'Display', 'iter', ...          % Show progress
    'Algorithm', 'sqp', ...         % A good default algorithm
    'MaxIterations', 100);

% 4. Call the optimizer!
cost_fun = @cost_function_flip;
[p_optimal, cost_final] = fmincon(cost_fun, p0, [], [], [], [], lb, ub, [], options);

% 5. Display the results
fprintf('\n--- Optimization Finished ---\n');
fprintf('Final Cost: %.4f\n', cost_final);
fprintf('Optimal Parameters:\n');
fprintf('  q3_takeoff:        %.4f rad\n', p_optimal(1));
fprintf('  q3dot_takeoff:     %.4f rad/s\n', p_optimal(2));
fprintf('  vf_takeoff:        %.4f m/s\n', p_optimal(3));
fprintf('  flip_timing_ratio: %.4f\n', p_optimal(4));

% You can now run the simulation one last time with the optimal parameters
% and visualize the result.