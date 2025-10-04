function x_plus = impact_map(x_minus, params)

    % Extract states
    q  = x_minus(1:5);
    q1 = q(3);
    q2 = q(4);
    q3 = q(5);
    
    dq_minus = x_minus(6:10);

    % Unpack parameters
    m1=params.m1; m2=params.m2; m3=params.m3;
    I1=params.I1; I2=params.I2; I3=params.I3;
    l1=params.l1; l2=params.l2; l3=params.l3;
    d1=params.d1; d2=params.d2; d3=params.d3;

    % Dynamics matrices
    D = auto_D(I1,I2,I3,d1,d2,d3,l1,m1,m2,m3,q1,q2,q3);
    Jst = auto_Jst(l1,l2,q1,q2);

    % Build KKT system
    LHS = [Jst, zeros(size(Jst,1)); 
           D, -Jst'];
    RHS = [zeros(size(Jst,1),1);
           D * dq_minus];

    sol = LHS \ RHS;
    dq_plus = sol(1:5);   % post-impact velocity

    % Keep positions the same, only velocities reset
    x_plus = [q; dq_plus];
end