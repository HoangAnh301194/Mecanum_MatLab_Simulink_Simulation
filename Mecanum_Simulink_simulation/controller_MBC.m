function [tau,error,nu_d] = controller_MBC( ...
    q,nu,qd,qd_dot,qd_ddot,p)

% =========================================================
% MODEL-BASED CONTROLLER
% =========================================================

theta = q(3);

%% Sai so vi tri

error = qd - q;

% Chuan hoa sai so goc
error(3) = atan2( ...
    sin(error(3)), ...
    cos(error(3)));

%% Ma tran quay

R = [
    cos(theta) -sin(theta) 0;
    sin(theta)  cos(theta) 0;
    0           0          1
];

%% Van toc tham chieu
%
% qdot_ref = qd_dot + Kp*e

qdot_ref = ...
    qd_dot + p.Kp_mbc*error;

%% Chuyen sang he toa do robot

nu_d = R'*qdot_ref;

%% Gia toc feedforward

nu_ddot_ff = R'*qd_ddot;

%% Sai so van toc

e_v = nu_d - nu;

%% Gia toc dieu khien mong muon

nu_dot_cmd = ...
    nu_ddot_ff + p.Kv_mbc*e_v;

%% Ma tran Coriolis

w = nu(3);

C = [
     0       -p.m*w    0;
     p.m*w    0        0;
     0        0        0
];

%% ========================================================
% MODEL-BASED CONTROL
%
% tau = M*nu_dot_cmd + C*nu + D*nu
% =========================================================

tau = ...
    p.M*nu_dot_cmd + ...
    C*nu + ...
    p.D*nu;

%% Saturation

tau = max( ...
      min(tau,p.tau_max), ...
      -p.tau_max);

end