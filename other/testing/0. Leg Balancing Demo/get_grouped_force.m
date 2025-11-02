% get_grouped_force.m
function [F, description] = get_grouped_force(t, group_scenarios, phase_duration)
    % This function returns the correct force and description for a given time
    % within a sequence of pushes defined by 'group_scenarios'.

    % Default values
    F = [0; 0];
    description = 'Stabilizing';
    push_duration = 0.3; % Duration of the force application

    % Determine which phase we are in (1st push, 2nd push, etc.)
    phase_index = floor(t / phase_duration) + 1;
    
    % Determine the time within the current phase
    time_in_phase = mod(t, phase_duration);

    % Check if we are in a valid push phase and within the push duration
    if phase_index <= length(group_scenarios) && time_in_phase < push_duration
        F = group_scenarios{phase_index}.force;
        description = group_scenarios{phase_index}.name;
    end
end