%------------------------------------------------------------------------
%------------------------------------------------------------------------
% MECENG239 Final Project - Hopping Leg  (+ Front Flip as extension)
% Course: ME239: Robotic Locomotion
% Semester: Fall 2025
%------------------------------------------------------------------------
% By: Jason Abi Chebli
% Student ID: 3042017306
% Last Modified: 2025-09-17
% The following is the MATLAB code used to determine the dynamics of a
% robotic leg jumping up and down. 
%------------------------------------------------------------------------
%------------------------------------------------------------------------

% clear all; close all; clc;

% ---------------------------- Symbolic Setup -----------------------------
% Define Symbolic Variables for 2-link leg system
% syms x y q1 q2 real
% syms xdot ydot q1dot q2dot real
% syms xddot yddot q1ddot q2ddot real
% syms m1 I1 l1 d1 m2 I2 l2 d2 g real
% syms q1 q2 real

% Generalized coords and rates

% Pi = sym(pi) ;

% % Leg properties
% m1 = 0.05; % mass of link 1, kg
% I1 = 0.0001; % moment of inertia of link 1, m^4
% l1 = 0.12; % length of link 1, m
% d1 = 0.06; % absolute distance to COM of link 1, m
% 
% m2 = 0.5; % mass of link 2, kg
% I2 = 0.000053; % moment of inertia of link 2, m^4
% l2 = 0.08; % length of link 2, m
% d2 = 0.04; % absolute distance to COM of link 2, m
% 
% x = 0;
% y = sin(Pi/3)*l2+ cos(Pi/3)*l1;

% q0 = [deg2rad(15);
%       deg2rad(0)] ;
% 
% dq0 = zeros(2,1) ;
% x0 = [q0;
%       dq0] ;

% x = [x ; y ; dx ; dy] ;


% q  = [x; y; q1; q2];
% dq = [xdot; ydot; q1dot; q2dot];
% ddq = [xddot; yddot; q1ddot; q2ddot];

% x = [q ; dq] ;


% -------------------------------------------------------------------------
% ------------------------- Dynamics Functions ----------------------------
% -------------------------------------------------------------------------

function simulate_leg()
    % q1 = 240*Pi/180;
    % q2 = Pi/180 ;
    % xdot = 0 ;
    % ydot = 0;
    % q1dot = 0 ;
    % q2dot = 0 ;

    l1 = 0.12; % length of link 1, m
    l2 = 0.08; % length of link 2, m
    l3 = 0.05; % length of link 3, m
    Pi = 3.14;

    q0 = [0;
            cos(Pi/6)*l2 + cos(Pi/3)*l1;
            deg2rad(60);
            deg2rad(90);
            0] ;

    dq0 = zeros(5,1) ;

    x0 = [q0;
          dq0] ;

    [t_sol, x_sol] = ode45(@leg_ode, [0, 10], x0) ;

    figure(1) ; plot(t_sol, x_sol(:,3:4)) ; grid on ;
        legend('th1', 'th2') ; 
        xlabel('time') ; 
        ylabel('rad') ;

    figure(2) ; plot(t_sol, x_sol(:,1:2)) ; grid on ;
        legend('thigh height', 'knee height') ; 
        xlabel('time') ; 
        ylabel('height') ;

    % disp(x_sol)

    animate_leg(t_sol, x_sol) ;

    % gamma_sol = zeros(size(t_sol)) ;
    % for j=1:length(t_sol)
    %     gamma_sol(j) = auto_gamma(x_sol(j,2)) ;
    % end
    % figure(2) ; plot(t_sol, gamma_sol) ; grid on ;
    %     xlabel('Time') ; title('gamma') ;

    % animate_pendulum(t_sol, x_sol) ;
end

function dx = leg_ode(t, x)
    q = x(1:5) ; dq = x(6:10) ;
    x = q(1) ; y = q(2) ; q1 = q(3) ; q2 = q(4) ; q3 = q(5) ;
    dx = dq(1) ; dy = dq(2) ; q1dot = dq(3) ; q2dot = dq(4) ; q3dot = dq(5) ;

    % Leg properties
    m1 = 0.05; % mass of link 1, kg
    I1 = 0.0001; % moment of inertia of link 1, m^4
    l1 = 0.12; % length of link 1, m
    d1 = 0.06; % absolute distance to COM of link 1, m
    
    m2 = 0.5; % mass of link 2, kg
    I2 = 0.000053; % moment of inertia of link 2, m^4
    l2 = 0.08; % length of link 2, m
    d2 = 0.04; % absolute distance to COM of link 2, m

    m3 = 0.5; % mass of link 2, kg
    I3 = 0.000053; % moment of inertia of link 2, m^4
    l3 = 0.05; % length of link 2, m
    d3 = 0.05; % absolute distance to COM of link 2, m

    % Pi = 3.14;
    g = 9.81; % gravity, m/s^2

    D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3) ;
    C = auto_C(d1,d2,d3,l1,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot) ;
    G = auto_G(d1,d2,d3,g,l1,m1,m2,m3,q1,q2,q3) ;
    B = auto_B ;
    
    Jst = auto_Jst(l1,l2,q1,q2) ;
    Jstdot = auto_Jstdot(l1,l2,q1,q2,q1dot,q2dot) ;

    LHS = [D, -Jst.';
        Jst, zeros(2,2)] ;

    RHS = [-C * dq - G + B * [0,0].';
        -Jstdot] ;
    
    d2q_tauc = LHS \ RHS;
    d2q = d2q_tauc(1:5) ;

    dx = [dq;
          d2q] ;

        % Vector fields
    f = [dq;
        inv(D)*(-C*dq - G)] ;

    disp('debug')
    disp(size(f))
    disp(size(inv(D)*B))
    g = [zeros(5, 2);
         inv(D)*B] ;

    % Control
    % u = 0
    u = FeedbackControl(q, dq, f, g) ;

    dx = dx + f + g*u ;
end


function u = FeedbackControl(q, dq, f, g)
    % q = x(1:5) ; dq = x(6:10) ;
    x = q(1) ; y = q(2) ; q1 = q(3) ; q2 = q(4) ; q3 = q(5) ;
    dx = dq(1) ; dy = dq(2) ; q1dot = dq(3) ; q2dot = dq(4) ; q3dot = dq(5) ;

    % Leg properties
    m1 = 0.05; % mass of link 1, kg
    I1 = 0.0001; % moment of inertia of link 1, m^4
    l1 = 0.12; % length of link 1, m
    d1 = 0.06; % absolute distance to COM of link 1, m
    
    m2 = 0.5; % mass of link 2, kg
    I2 = 0.000053; % moment of inertia of link 2, m^4
    l2 = 0.08; % length of link 2, m
    d2 = 0.04; % absolute distance to COM of link 2, m

    m3 = 0.5; % mass of link 2, kg
    I3 = 0.000053; % moment of inertia of link 2, m^4
    l3 = 0.05; % length of link 2, m
    d3 = 0.05; % absolute distance to COM of link 2, m

    % Pi = 3.14;
    % g = 9.81; % gravity, m/s^2

    h = auto_h(d1,d2,d3,l1,l2,m1,m2,m3,q1,q2,q3,x) ;
    dh_dq = auto_dh_dq(d1,d2,d3,l1,l2,m1,m2,m3,q1,q2,q3) ;
    d2h__ = auto_d2h__(d1,d2,d3,l1,l2,m1,m2,m3,q1,q2,q3,q1dot,q2dot,q3dot) ;

    % Compute Lie Derivatives
    Lfh = dh_dq * dq ;
    Lf2h = d2h__ * f ;
    LgLfh = d2h__ * g ;

    disp(d2h__)
    disp(LgLfh)

    % PD gains
    Kp = 5 ; Kd = 1 ;
    u = inv(LgLfh)*(-Lf2h - Kp*h - Kd*Lfh) ;
end



function animate_leg(t_sol, x_sol)

    Pi = 3.14;
    % --- Define constants for the leg's geometry ---
    l1 = 0.12; % length of thigh, m
    l2 = 0.08; % length of shin, m
    l3 = 0.05;

    % --- Set up the figure and plot objects once ---
    figure(4); % Use a new figure window
    clf; % Clear the figure
    ax = gca;
    axis(ax, 'equal', [-0.4 0.4 -0.4 0.4]); % Use 'equal' for correct aspect ratio
    hold(ax, 'on');
    grid on;
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    title('2D Bipedal Leg Animation');

    % --- Initialize plot objects to get handles ---
    ground = plot(ax, [-1 1], [0 0], 'k-', 'LineWidth', 2);
    leg_plot = plot(ax, NaN, NaN, '-o', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    foot_trace = plot(ax, NaN, NaN, 'r--', 'LineWidth', 1.5); % For the foot's path

    % --- Animation Loop ---
    for j = 1:length(t_sol)
        % Extract the state variables for the current time step
        hip_x = x_sol(j, 1);
        hip_y = x_sol(j, 2);
        q1    = x_sol(j, 3); % Thigh angle
        q2    = x_sol(j, 4); % Shin angle (relative to thigh)
        q3    = x_sol(j, 5);

        % Calculate the Cartesian coordinates of the knee and foot
        % Assumes q1=0 is pointing straight down
        % knee_x = hip_x + l1 * sin(q1);
        % knee_y = hip_y - l1 * cos(q1);
        % 
        % foot_x = knee_x + l2 * sin(q1 + q2);
        % foot_y = knee_y - l2 * cos(q1 + q2);

        % Knee position
        knee_x = hip_x + l1*(-sin(q1));
        knee_y = hip_y + l1*(-cos(q1));
       
        
        % % Foot (stance) position
        % foot_x = hip_x + l1*sin(q1) + l2*sin(q1 + q2);
        % foot_y = hip_y + l1*cos(q1) + l2*cos(q1 + q2);
        foot_x = hip_x + l1*(-sin(q1)) + l2*sin(q2 -q1);
        foot_y = hip_y + l1*(-cos(q1)) + l2*(-cos(q2 -q1)) ;

        torso_x = hip_x + l3 * (-sin(q3));
        torso_y = hip_y + l3 * cos(q3);

        % --- Update plot data for efficiency ---
        % Update the leg's position
        set(leg_plot, 'XData', [torso_x, hip_x, knee_x, foot_x], 'YData', [torso_y, hip_y, knee_y, foot_y]);

        % Append the new foot position to the trace
        trace_x = [get(foot_trace, 'XData'), foot_x];
        trace_y = [get(foot_trace, 'YData'), foot_y];
        set(foot_trace, 'XData', trace_x, 'YData', trace_y);

        % Update the title with the current simulation
        title(ax, sprintf('Leg Animation | Time: %.2f s', t_sol(j)));
        
        % Render the frame
        drawnow;
        pause(0.05) ;
    end
end


% 
% % ---------------------------- During Impact ------------------------------
% function [dq_plus, Lambda] = impact_dynamics(q, dq_minus, D, Jst)
%     % Inputs:
%     %   q: Generalized coordinates
%     %   dq_minus: Pre-impact generalized velocities
%     %   D: Inertia matrix
%     %   Jst: Stance Jacobian (partial_pfoot/partial_q) at impact
%     %
%     % Outputs:
%     %   dq_plus: Post-impact generalized velocities
%     %   Lambda: Contact impulse at impact
% 
%     % Assemble the block system
%     A = [D, -Jst.'; 
%          Jst, zeros(size(Jst,1))];
% 
%     % Right-hand side
%     b = [D*dq_minus; 
%          zeros(size(Jst,1),1)];
% 
%     % Solve for [dq_plus; Lambda]
%     sol = A \ b;
%     dq_plus = simplify(sol(1:length(q)));
%     Lambda  = simplify(sol(length(q)+1:end));
% end


% % ---------- Inverse Kinematics Problem (with COM Constraint) ------------- 
% % Assume the foot is at (0,0) in a world frame
% eq1 = (x_hip + l1*sin(q1) + l2*sin(q1 + q2)) == 0; %xf = 0 
% eq2 = (y_hip + l1*cos(q1) + l2*cos(q1 + q2)) == 0; %yf = 0 
% eq3 = (m1*(x_hip + d1*sin(q1)) + m2*(x_hip + l1*sin(q1) + d2*sin(q1 + q2))) / (m1 + m2) == 0; %pCOM_x = 0
% 
% % Solve the system for the unknowns x_hip, q1, and q2
% solutions = solve([eq1, eq2, eq3], [x_hip, q1, q2], 'Real', true);
% x_hip_sym = solutions.x_hip;
% q1_sym = solutions.q1;
% q2_sym = solutions.q2;
% 
% % Convert the symbolic solution to a fast MATLAB function
% inv_kin_func = matlabFunction([x_hip_sym(1), q1_sym(1), q2_sym(1)], 'Vars', {y_hip, l1, l2, d1, d2, m1, m2});