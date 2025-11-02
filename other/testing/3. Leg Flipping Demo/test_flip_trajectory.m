% test_flip_trajectory.m
function test_flip_trajectory()
    clear all; close all; clc;
    
    % --- Define parameters for the test ---
    % Jump parameters (from your test_trajectory.m)
    params.y_stand_target = 0.22;
    params.y_stand        = params.y_stand_target;
    params.T_jump         = 0.7;
    params.y_squat        = 0.06;
    params.y_takeoff      = 0.26;
    params.T_hold         = 1.0;
    params.vf_takeoff     = 0.5;
    params.T_dip_ratio    = 0.75;
    
    % --- NEW: FLIP PARAMETERS ---
    % You will need to TUNE these to get a good flip!
    params.q3_takeoff     = pi / 2; % Target angle at takeoff (90 deg)
    params.q3dot_takeoff  = 4.0;    % Target angular velocity (rad/s)
    
    % --- Generate the trajectory data ---
    % We'll simulate a little into the "flight" phase to see the ballistic part
    t_end = params.T_hold + params.T_jump + 0.5; 
    t_step = 0.001;
    t = 0:t_step:t_end;
    
    % Pre-allocate arrays for all 6 outputs
    p_y_d = zeros(size(t));
    v_y_d = zeros(size(t));
    a_y_d = zeros(size(t));
    
    p_q3_d = zeros(size(t));
    v_q3_d = zeros(size(t));
    a_q3_d = zeros(size(t));
    
    for i = 1:length(t)
        [p_y_d(i), v_y_d(i), a_y_d(i), ...
         p_q3_d(i), v_q3_d(i), a_q3_d(i)] = desired_flip_trajectory(t(i), params);
    end
    
    % --- Define phase times for plotting ---
    t_hold_end = params.T_hold;
    if isfield(params, 'T_dip_ratio')
        t_dip_ratio = params.T_dip_ratio;
    else
        t_dip_ratio = 0.4;
    end
    t_dip_end = params.T_hold + params.T_jump * t_dip_ratio;
    t_jump_end = params.T_hold + params.T_jump;
    
    % =========================================================================
    % --- FIGURE 1: Plot COM-Y Results ---
    % =========================================================================
    figure('Name', 'COM-Y Trajectory (Jump)');
    
    % Position Plot
    subplot(3,1,1);
    plot(t, p_y_d, 'b-', 'LineWidth', 2);
    hold on;
    xline(t_hold_end, 'k--', 'Hold End');
    xline(t_dip_end, 'k--', 'Squat Bottom');
    xline(t_jump_end, 'k--', 'Takeoff');
    grid on;
    title('Desired COM Position ($y_d$)', 'Interpreter', 'latex');
    ylabel('Position (m)');
    legend('y_d', 'Location', 'southeast');
    
    % Velocity Plot
    subplot(3,1,2);
    plot(t, v_y_d, 'r-', 'LineWidth', 2);
    hold on;
    xline(t_hold_end, 'k--');
    xline(t_dip_end, 'k--');
    xline(t_jump_end, 'k--');
    grid on;
    title('Desired COM Velocity ($\dot{y}_d$)', 'Interpreter', 'latex');
    ylabel('Velocity (m/s)');
    
    % Acceleration Plot
    subplot(3,1,3);
    plot(t, a_y_d, 'g-', 'LineWidth', 2);
    hold on;
    xline(t_hold_end, 'k--');
    xline(t_dip_end, 'k--');
    xline(t_jump_end, 'k--');
    grid on;
    title('Desired COM Acceleration ($\ddot{y}_d$)', 'Interpreter', 'latex');
    ylabel('Accel (m/s^2)');
    xlabel('Time (s)');
    
    % =========================================================================
    % --- FIGURE 2: Plot Torso-q3 Results ---
    % =========================================================================
    figure('Name', 'Torso-q3 Trajectory (Flip)');
    
    % Angle Plot
    subplot(3,1,1);
    plot(t, p_q3_d, 'b-', 'LineWidth', 2);
    hold on;
    xline(t_hold_end, 'k--', 'Hold End');
    xline(t_dip_end, 'k--', 'Flip Start'); % Flip starts at push-off
    xline(t_jump_end, 'k--', 'Takeoff');
    grid on;
    title('Desired Torso Angle ($q_3$)', 'Interpreter', 'latex');
    ylabel('Angle (rad)');
    legend('q3_d', 'Location', 'southeast');
    
    % Angular Velocity Plot
    subplot(3,1,2);
    plot(t, v_q3_d, 'r-', 'LineWidth', 2);
    hold on;
    xline(t_hold_end, 'k--');
    xline(t_dip_end, 'k--');
    xline(t_jump_end, 'k--');
    grid on;
    title('Desired Torso Velocity ($\dot{q}_3$)', 'Interpreter', 'latex');
    ylabel('Ang. Vel. (rad/s)');
    
    % Angular Acceleration Plot
    subplot(3,1,3);
    plot(t, a_q3_d, 'g-', 'LineWidth', 2);
    hold on;
    xline(t_hold_end, 'k--');
    xline(t_dip_end, 'k--');
    xline(t_jump_end, 'k--');
    grid on;
    title('Desired Torso Acceleration ($\ddot{q}_3$)', 'Interpreter', 'latex');
    ylabel('Ang. Accel. (rad/s^2)');
    xlabel('Time (s)');
end