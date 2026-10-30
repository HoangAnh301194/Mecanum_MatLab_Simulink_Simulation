function [tau,error,integral_error] = controller_PID( ...
    q,nu,qd,qd_dot,integral_error,p)

% =========================================================
% PID CONTROLLER DAY DU
%
% tau = Kp*e + Ki*integral(e) + Kd*e_dot
%
% q   = [x; y; theta]
% nu  = [vx; vy; omega]
% =========================================================

theta = q(3);

%% Ma tran quay body -> world
R = [
    cos(theta) -sin(theta) 0;
    sin(theta)  cos(theta) 0;
    0           0          1
];

%% Van toc robot trong he toa do world
q_dot = R*nu;

%% Sai so vi tri
error = qd - q;

% Chuan hoa sai so goc
error(3) = atan2( ...
    sin(error(3)), ...
    cos(error(3)));

%% Sai so dao ham
error_dot = qd_dot - q_dot;

%% Tich phan sai so
integral_error = ...
    integral_error + error*p.dt;

%% Anti-windup
integral_error = max( ...
    min(integral_error,p.integral_limit), ...
    -p.integral_limit);

%% PID trong he toa do world
tau_world = ...
      p.Kp_pid*error ...
    + p.Ki_pid*integral_error ...
    + p.Kd_pid*error_dot;

%% Chuyen luc world -> body
tau = R'*tau_world;

%% Saturation
tau = max( ...
    min(tau,p.tau_max), ...
    -p.tau_max);

end