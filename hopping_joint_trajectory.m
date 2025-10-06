function [q1_d, q2_d] = hopping_joint_trajectory(t, params)
% Generates smooth squat->push->takeoff joint angles
% Inputs: time t, params struct
% Outputs: q1_d, q2_d (joint angles)

% --- Time scaling ---
T_hold  = params.T_hold;
T_squat = params.T_squat;
T_push  = params.T_push;

% --- Desired COM height (for reference) ---
y_stand   = params.y_stand;
y_squat   = params.y_squat;
y_takeoff = params.y_takeoff;
q3_d      = params.q3d;

% --- Solve IK once for each key pose ---
[q1_stand, q2_stand] = compute_IK(0, y_stand, q3_d, params);
[q1_squat, q2_squat] = compute_IK(0, y_squat, q3_d, params);
[q1_push,  q2_push]  = compute_IK(0, y_takeoff, q3_d, params);

% --- Piecewise trajectory ---
if t < T_hold
    q1_d = q1_stand;
    q2_d = q2_stand;
elseif t < T_hold + T_squat
    tau = (t - T_hold)/T_squat;
    q1_d = q1_stand + (q1_squat - q1_stand)*(1 - cos(pi*tau))/2;
    q2_d = q2_stand + (q2_squat - q2_stand)*(1 - cos(pi*tau))/2;
elseif t < T_hold + T_squat + T_push
    tau = (t - T_hold - T_squat)/T_push;
    q1_d = q1_squat + (q1_push - q1_squat)*(1 - cos(pi*tau))/2;
    q2_d = q2_squat + (q2_push - q2_squat)*(1 - cos(pi*tau))/2;
else
    q1_d = q1_push;
    q2_d = q2_push;
end
end
