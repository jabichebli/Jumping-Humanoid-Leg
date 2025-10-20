function params = getParams()

        % ---------------- Simulation Parameters ----------------
    params.m1 = 0.15;   params.m2 = 0.15;   params.m3 = 0.8;
    params.l1 = 0.15;   params.l2 = 0.15;   params.l3 = 0.1; params.w3 = 0.15;
    params.d1 = 0.075;   params.d2 = 0.075;  params.d3 = 0.0; % The COM of torso would need to be aligned with hip 
    params.g  = 9.81; 

    % Moments of inertia
    params.I1 = (1/12) * params.m1 * params.l1^2; % thin rod approximation about center
    params.I2 = (1/12) * params.m2 * params.l2^2; % thin rod approximation about center
    params.I3 = (1/12) * params.m3 * (params.w3^2 + params.l3^2); % rectangular approximation about center

    params.pCOMy_d = 0.2; % standing: 0.20 --> squatting: 0.08  --> takeoff: 0.258

    % Virtual constraint gains
    params.Kp = 30; 
    params.Kd = 8;
    params.flight_Kp = 50;
    params.flight_Kd = 10;
    params.flight_u_max = 3;

    % Trajectory Parameters (Be careful tuning these)
    params.y_stand   = 0.2;
    params.y_squat   = 0.08;
    params.y_takeoff = 0.65;    % Takeoff height to produce enough y-velocity (actual takeoff height: ~0.258m)
    params.T_hold    = 2.0;
    params.T_squat   = 0.6;
    params.T_push    = 0.3;

end