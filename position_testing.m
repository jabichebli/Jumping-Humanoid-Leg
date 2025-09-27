l1 = 0.12; % length of thigh, m
l2 = 0.08; % length of shin, m
Pi = pi;

x = 0;
y = sin(Pi/3)*l2+ cos(Pi/3)*l1

q1 = deg2rad(240);
q2 = deg2rad(90);



pknee = [ x + l1*sin(q1);
          y + l1*cos(q1) ]

% Foot (stance) position
pfoot = [ x + l1*sin(q1) + l2*sin(q1 + q2 - Pi);
          y + l1*cos(q1) + l2*cos(q1 + q2 - Pi) ]