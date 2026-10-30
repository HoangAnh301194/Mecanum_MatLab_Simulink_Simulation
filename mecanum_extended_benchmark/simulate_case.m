function result = simulate_case(p_ctrl,p_plant,case_type)
% =========================================================
% SIMULATE ONE EXPERIMENT
%
% p_ctrl  : tham so controller
% p_plant : tham so robot thuc te
%
% case_type:
% 1 = Nominal
% 2 = Payload
% 3 = Disturbance
% =========================================================

t = p_ctrl.t0:p_ctrl.dt:p_ctrl.tf;
N = length(t);

%% Bo nho

qd      = zeros(3,N);
qd_dot  = zeros(3,N);
qd_ddot = zeros(3,N);

q_mbc   = zeros(3,N);
nu_mbc  = zeros(3,N);

q_pid   = zeros(3,N);
nu_pid  = zeros(3,N);

tau_mbc = zeros(3,N);
tau_pid = zeros(3,N);

wheel_mbc = zeros(4,N);
wheel_pid = zeros(4,N);

%% Initial conditions

q_mbc(:,1) = p_ctrl.q0;
q_pid(:,1) = p_ctrl.q0;

nu_mbc(:,1) = p_ctrl.nu0;
nu_pid(:,1) = p_ctrl.nu0;

integral_pid = zeros(3,1);

%% ========================================================
% SIMULATION
% ========================================================

for k = 1:N-1

    tk = t(k);

    %% Reference trajectory

    [qd(:,k),qd_dot(:,k),qd_ddot(:,k)] = ...
        trajectory(tk,p_ctrl);

    %% ====================================================
    % MODEL-BASED CONTROL
    % =====================================================

    [tau_mbc(:,k),~,~] = ...
        controller_MBC( ...
        q_mbc(:,k), ...
        nu_mbc(:,k), ...
        qd(:,k), ...
        qd_dot(:,k), ...
        qd_ddot(:,k), ...
        p_ctrl);

    %% ====================================================
    % PID
    % =====================================================

    [tau_pid(:,k),~,integral_pid] = ...
        controller_PID( ...
        q_pid(:,k), ...
        nu_pid(:,k), ...
        qd(:,k), ...
        qd_dot(:,k), ...
        integral_pid, ...
        p_ctrl);

    %% ====================================================
    % DISTURBANCE
    % =====================================================

    disturbance = [0;0;0];

    if case_type == 3

        % Tac dong tu giay 15 den giay 20

        if tk >= 15 && tk <= 20

            disturbance = [
                15;     % Fx disturbance [N]
               -10;     % Fy disturbance [N]
                 2      % Mz disturbance [N.m]
            ];

        end

    end

    %% ====================================================
    % PLANT - MBC
    % =====================================================

    tau_total_mbc = ...
        tau_mbc(:,k) + disturbance;

    [qdot_mbc,nudot_mbc] = ...
        robot_dynamics( ...
        q_mbc(:,k), ...
        nu_mbc(:,k), ...
        tau_total_mbc, ...
        p_plant);

    %% Integration

    q_mbc(:,k+1) = ...
        q_mbc(:,k) + qdot_mbc*p_ctrl.dt;

    nu_mbc(:,k+1) = ...
        nu_mbc(:,k) + nudot_mbc*p_ctrl.dt;

    %% ====================================================
    % PLANT - PID
    % =====================================================

    tau_total_pid = ...
        tau_pid(:,k) + disturbance;

    [qdot_pid,nudot_pid] = ...
        robot_dynamics( ...
        q_pid(:,k), ...
        nu_pid(:,k), ...
        tau_total_pid, ...
        p_plant);

    q_pid(:,k+1) = ...
        q_pid(:,k) + qdot_pid*p_ctrl.dt;

    nu_pid(:,k+1) = ...
        nu_pid(:,k) + nudot_pid*p_ctrl.dt;

    %% Wheel speed

    wheel_mbc(:,k) = ...
        mecanum_kinematics( ...
        nu_mbc(:,k),p_plant);

    wheel_pid(:,k) = ...
        mecanum_kinematics( ...
        nu_pid(:,k),p_plant);

end

% Complete command logs using zero-order hold at the last time stamp.
tau_mbc(:,N) = tau_mbc(:,N-1);
tau_pid(:,N) = tau_pid(:,N-1);

%% Last reference point

[qd(:,N),qd_dot(:,N),qd_ddot(:,N)] = ...
    trajectory(t(N),p_ctrl);
wheel_mbc(:,N) = ...
    mecanum_kinematics( ...
    nu_mbc(:,N),p_plant);

wheel_pid(:,N) = ...
    mecanum_kinematics( ...
    nu_pid(:,N),p_plant);
%% ========================================================
% ERRORS
% ========================================================

error_mbc = qd - q_mbc;
error_pid = qd - q_pid;

%% Angle normalization

error_mbc(3,:) = atan2( ...
    sin(error_mbc(3,:)), ...
    cos(error_mbc(3,:)));

error_pid(3,:) = atan2( ...
    sin(error_pid(3,:)), ...
    cos(error_pid(3,:)));

%% Position errors

ep_mbc = sqrt( ...
    error_mbc(1,:).^2 + ...
    error_mbc(2,:).^2);

ep_pid = sqrt( ...
    error_pid(1,:).^2 + ...
    error_pid(2,:).^2);
%% ========================================================
% MAX POSITION ERROR
% ========================================================

max_error_mbc = max(ep_mbc);
max_error_pid = max(ep_pid);

%% RMSE sau giai doan qua do ban dau
% Chi tinh tu t >= 5 s

idx_ss = t >= 5;

RMSE_mbc_ss = sqrt( ...
    mean(ep_mbc(idx_ss).^2));

RMSE_pid_ss = sqrt( ...
    mean(ep_pid(idx_ss).^2));
%% ========================================================
% RMSE
% ========================================================

RMSE_mbc_x = ...
    sqrt(mean(error_mbc(1,:).^2));

RMSE_mbc_y = ...
    sqrt(mean(error_mbc(2,:).^2));

RMSE_mbc_theta = ...
    sqrt(mean(error_mbc(3,:).^2));

RMSE_mbc_pos = ...
    sqrt(mean(ep_mbc.^2));

RMSE_pid_x = ...
    sqrt(mean(error_pid(1,:).^2));

RMSE_pid_y = ...
    sqrt(mean(error_pid(2,:).^2));

RMSE_pid_theta = ...
    sqrt(mean(error_pid(3,:).^2));

RMSE_pid_pos = ...
    sqrt(mean(ep_pid.^2));
%% ========================================================
% RMSE TRONG KHOANG NHIEM - CHI DUNG CHO TN3
% ========================================================

if case_type == 3

    idx_dist = (t >= 15) & (t <= 20);

    RMSE_dist_mbc = sqrt( ...
        mean(ep_mbc(idx_dist).^2));

    RMSE_dist_pid = sqrt( ...
        mean(ep_pid(idx_dist).^2));

else

    RMSE_dist_mbc = NaN;
    RMSE_dist_pid = NaN;

end
%% ========================================================
% SAVE RESULTS
% ========================================================

result.t = t;

result.qd = qd;
result.qd_dot = qd_dot;
result.qd_ddot = qd_ddot;
result.nu_mbc = nu_mbc;
result.nu_pid = nu_pid;

result.q_mbc = q_mbc;
result.q_pid = q_pid;

result.error_mbc = error_mbc;
result.error_pid = error_pid;

result.ep_mbc = ep_mbc;
result.ep_pid = ep_pid;

result.tau_mbc = tau_mbc;
result.tau_pid = tau_pid;

result.wheel_mbc = wheel_mbc;
result.wheel_pid = wheel_pid;

result.rmse_mbc_x = RMSE_mbc_x;
result.rmse_mbc_y = RMSE_mbc_y;
result.rmse_mbc_theta = RMSE_mbc_theta;
result.rmse_mbc_pos = RMSE_mbc_pos;

result.rmse_pid_x = RMSE_pid_x;
result.rmse_pid_y = RMSE_pid_y;
result.rmse_pid_theta = RMSE_pid_theta;
result.rmse_pid_pos = RMSE_pid_pos;
result.rmse_dist_mbc = RMSE_dist_mbc;
result.rmse_dist_pid = RMSE_dist_pid;
result.improvement = ...
    (RMSE_pid_pos - RMSE_mbc_pos) ...
    / RMSE_pid_pos*100;
result.max_error_mbc = max_error_mbc;
result.max_error_pid = max_error_pid;

result.rmse_mbc_ss = RMSE_mbc_ss;
result.rmse_pid_ss = RMSE_pid_ss;

end