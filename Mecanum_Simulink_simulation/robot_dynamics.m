function [q_dot, nu_dot] = robot_dynamics(q,nu,tau,p)
% =========================================================
% DONG LUC HOC ROBOT MECANUM
%
% M*nu_dot + C(nu)*nu + D*nu = tau
%
% q  = [x; y; theta]
% nu = [vx; vy; omega]
% =========================================================

theta = q(3);

vx = nu(1);
vy = nu(2);
w  = nu(3);

%% Ma tran quay: body -> world

R = [
    cos(theta) -sin(theta) 0;
    sin(theta)  cos(theta) 0;
    0           0          1
];

%% Dong hoc robot

q_dot = R*nu;

%% Ma tran Coriolis
%
% Mo hinh vat ran phang trong he toa do body

C = [
     0       -p.m*w    0;
     p.m*w    0        0;
     0        0        0
];

%% Dong luc hoc
%
% M*nu_dot + C*nu + D*nu = tau

nu_dot = p.M \ ...
          (tau - C*nu - p.D*nu);

end