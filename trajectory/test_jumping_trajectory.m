% -------------------------------------------------------------------------
% test_jumping_trajectory.m
% -------------------------------------------------------------------------
% Created by: Jason Abi Chebli
% Last Modified: 30-October-2025
% -------------------------------------------------------------------------
% Tests and visualizes the desired jump trajectory function for both 
% vertical (y) and horizontal (x) COM motion.
% Generates position, velocity, and acceleration profiles over time, 
% including ballistic flight and rotational phases.
% -------------------------------------------------------------------------

function test_jumping_trajectory()
    % =====================================================================
    % -------------------------- Initialization ---------------------------
    % =====================================================================
    clear all; close all; clc;
    
    % Define motion parameters
    params.y_stand_target = 0.22;
    params.y_stand        = params.y_stand_target;
    params.T_jump         = 0.7;  % Total time for the jump motion
    params.y_squat        = 0.06; % The deepest the CoM is allowed to go.
    params.y_takeoff      = 0.26;
    params.T_hold         = 0.6; 
    params.vf_takeoff     = 0.6; % vertical takeoff speed
    params.T_dip_ratio    = 0.75; % 75% of jump time spent on dip/lean

    % Trajectory parameters - forward/backwards/upwards only 
    % x_squat_lean: upwards: 0.0;   forward: 0.04;   backwards: -0.01
    % x_takeoff:    upwards: 0.0;   forward: 0.18;   backwards: -0.06
    params.x_squat_lean = 0.0;%  % The lean at the bottom of the squat  
    params.x_takeoff = 0.0; % 0.18; % The lean at takeoff
    
    % Simulate slightly beyond jump duration to include flight dynamics.
    t_end  = params.T_hold + params.T_jump + 0.5;
    t_step = 0.001;
    t      = 0:t_step:t_end;
    
    % Preallocate storage arrays
    p_y_d = zeros(size(t)); v_y_d = zeros(size(t)); a_y_d = zeros(size(t));
    p_x_d = zeros(size(t)); v_x_d = zeros(size(t)); a_x_d = zeros(size(t));
    
    % =====================================================================
    % -------- Compute desired trajectory values at each time step --------
    % =====================================================================

    for i = 1:length(t)
        
        [p_vec, v_vec, a_vec] = desired_jump_trajectory(t(i), params);
        
        % Extract the 1st element (the x-component) for the 'lean' plot
        p_x_d(i) = p_vec(1);
        v_x_d(i) = v_vec(1);
        a_x_d(i) = a_vec(1);

        % Extract the 2nd element (the y-component) for the 'jump' plot
        p_y_d(i) = p_vec(2);
        v_y_d(i) = v_vec(2);
        a_y_d(i) = a_vec(2);
    end
    
    % Define phase transition times for plotting reference lines
    t_hold_end = params.T_hold;
    t_dip_end  = params.T_hold + params.T_jump * params.T_dip_ratio;
    t_jump_end = params.T_hold + params.T_jump;
    
    % =====================================================================
    % ---------------------------- Plot Results ---------------------------
    % =====================================================================
    
    % FIGURE 1: COM Vertical Motion (Jump)
    figure;
    sgtitle("COM-Y Trajectory (Jump)", 'FontSize', 16, 'FontWeight', 'bold');
    
    % Position
    subplot(3,1,1);
    plot(t, p_y_d, 'b-', 'LineWidth', 2); hold on;
    xline(t_hold_end, 'k--', 'Hold End');
    xline(t_dip_end, 'k--', 'Squat Bottom');
    xline(t_jump_end, 'k--', 'Takeoff');
    grid on;
    title('Desired COM Position ($y_d$)', 'Interpreter', 'latex');
    ylabel('Position (m)');
    
    % Velocity
    subplot(3,1,2);
    plot(t, v_y_d, 'r-', 'LineWidth', 2); hold on;
    xline(t_hold_end, 'k--');
    xline(t_dip_end, 'k--');
    xline(t_jump_end, 'k--');
    grid on;
    title('Desired COM Velocity ($\dot{y}_d$)', 'Interpreter', 'latex');
    ylabel('Velocity (m/s)');
    
    % Acceleration
    subplot(3,1,3);
    plot(t, a_y_d, 'g-', 'LineWidth', 2); hold on;
    xline(t_hold_end, 'k--');
    xline(t_dip_end, 'k--');
    xline(t_jump_end, 'k--');
    grid on;
    title('Desired COM Acceleration ($\ddot{y}_d$)', 'Interpreter', 'latex');
    ylabel('Acceleration (m/s$^2$)', 'Interpreter', 'latex');
    xlabel('Time (s)');
        
    % FIGURE 2: COM Horizontal Motion (Lean)
    figure;
    sgtitle("COM-X Trajectory (Lean)", 'FontSize', 16, 'FontWeight', 'bold');

    
    % Position
    subplot(3,1,1);
    plot(t, p_x_d, 'b-', 'LineWidth', 2); hold on;
    xline(t_hold_end, 'k--', 'Hold End');
    xline(t_dip_end, 'k--', 'Squat Bottom');
    xline(t_jump_end, 'k--', 'Takeoff');
    grid on;
    title('Desired COM Position ($x_d$)', 'Interpreter', 'latex');
    ylabel('Position (m)');
    
    % Velocity
    subplot(3,1,2);
    plot(t, v_x_d, 'r-', 'LineWidth', 2); hold on;
    xline(t_hold_end, 'k--');
    xline(t_dip_end, 'k--');
    xline(t_jump_end, 'k--');
    grid on;
    title('Desired COM Velocity ($\dot{x}_d$)', 'Interpreter', 'latex');
    ylabel('Velocity (m/s)');
    
    % Angular Acceleration
    subplot(3,1,3);
    plot(t, a_x_d, 'g-', 'LineWidth', 2); hold on;
    xline(t_hold_end, 'k--');
    xline(t_dip_end, 'k--');
    xline(t_jump_end, 'k--');
    grid on;
    title('Desired COM Acceleration ($\ddot{x}_d$)', 'Interpreter', 'latex');
    ylabel('Acceleration (m/s$^2$)', 'Interpreter', 'latex');
    xlabel('Time (s)');
    
end