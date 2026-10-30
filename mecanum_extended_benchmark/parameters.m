function p = parameters()
% =========================================================
% PARAMETERS - MECANUM ROBOT VERSION 2
% =========================================================

%% Thong so vat ly robot
p.m  = 10;                 % Khoi luong [kg]
p.Iz = 0.8;                % Moment quan tinh [kg.m^2]

p.r  = 0.05;               % Ban kinh banh [m]
p.L  = 0.30;               % Nua chieu dai robot [m]
p.W  = 0.25;               % Nua chieu rong robot [m]

%% Ma tran quan tinh
p.M = diag([p.m p.m p.Iz]);

%% He so ma sat
p.D = diag([2.0 2.0 0.5]);

%% ================= MODEL-BASED CONTROL ===================
% Vong vi tri
p.Kp_mbc = diag([2.0 2.0 2.5]);

% Vong van toc
p.Kv_mbc = diag([5.0 5.0 4.0]);

%% ======================== PID =============================
% PID vi tri trong he toa do the gioi

p.Kp_pid = diag([40 40 3.2]);
p.Ki_pid = diag([1.0 1.0 0.15]);
p.Kd_pid = diag([34 34 2.38]);

%% Gioi han tich phan PID
p.integral_limit = [3; 3; 2];

%% Gioi han luc/mo-men
p.tau_max = [100; 100; 50];

%% Thoi gian mo phong
p.t0 = 0;
p.tf = 40;
p.dt = 0.01;

%% Quy dao tron
p.Rtraj = 2;               % Ban kinh [m]
p.wtraj = 0.20;            % Toc do goc [rad/s]

%% Dieu kien ban dau
% Co tinh dat robot lech khoi quy dao de kiem tra controller
p.q0  = [0; 0; 0];

% Van toc ban dau
p.nu0 = [0; 0; 0];

end