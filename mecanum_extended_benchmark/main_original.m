clc;
clear;
close all;

%% ========================================================
% MECANUM ROBOT
%
% MODEL-BASED CONTROL vs PID
%
% 3 THI NGHIEM:
%
% 1. Danh dinh
% 2. Tang tai
% 3. Nhieu ngoai
% =========================================================

%% ========================================================
% CONTROLLER PARAMETERS
% ========================================================

p_ctrl = parameters();

%% ========================================================
% THI NGHIEM 1
% DANH DINH
% ========================================================

p1 = p_ctrl;

fprintf('\n');
fprintf('============================================\n');
fprintf('THI NGHIEM 1 - DIEU KIEN DANH DINH\n');
fprintf('m = %.1f kg\n',p1.m);
fprintf('============================================\n');

R1 = simulate_case( ...
    p_ctrl,p1,1);

%% ========================================================
% THI NGHIEM 2
% TANG TAI
%
% Robot thuc te tang tu 10 kg -> 15 kg
%
% Controller VAN SU DUNG model 10 kg
% ========================================================

p2 = p_ctrl;

p2.m = 15;
p2.Iz = ...
    p_ctrl.Iz*(p2.m/p_ctrl.m);


%% Cap nhat ma tran quan tinh robot thuc

p2.M = diag([
    p2.m;
    p2.m;
    p2.Iz
]);

fprintf('\n');
fprintf('============================================\n');
fprintf('THI NGHIEM 2 - TANG TAI\n');
fprintf('Model controller = %.1f kg\n',p_ctrl.m);
fprintf('Robot thuc       = %.1f kg\n',p2.m);
fprintf('============================================\n');

R2 = simulate_case( ...
    p_ctrl,p2,2);
fprintf('\n===== KIEM TRA TN2 =====\n');
fprintf('Plant mass = %.2f kg\n',p2.m);
fprintf('Plant Iz   = %.2f kg.m^2\n',p2.Iz);

fprintf('MBC theta RMSE = %.6f rad\n', ...
    R2.rmse_mbc_theta);

fprintf('PID theta RMSE = %.6f rad\n', ...
    R2.rmse_pid_theta);
%% ========================================================
% THI NGHIEM 3
% NHIEU NGOAI
% ========================================================

p3 = p_ctrl;

fprintf('\n');
fprintf('============================================\n');
fprintf('THI NGHIEM 3 - NHIEU NGOAI\n');
fprintf('Nhieu tac dong tu 15s den 20s\n');
fprintf('============================================\n');

R3 = simulate_case( ...
    p_ctrl,p3,3);
%% ========================================================
% KIEM TRA CHI TIET TN3
% ========================================================

fprintf('\n');
fprintf('===== TN3 - TRONG KHOANG CO NHIEU 15-20s =====\n');

fprintf('MBC RMSE disturbance = %.5f m\n', ...
    R3.rmse_dist_mbc);

fprintf('PID RMSE disturbance = %.5f m\n', ...
    R3.rmse_dist_pid);
fprintf('\n');
fprintf('===== RMSE SAU QUA DO t >= 5s =====\n');

fprintf('TN1 MBC = %.5f m\n', ...
    R1.rmse_mbc_ss);

fprintf('TN1 PID = %.5f m\n', ...
    R1.rmse_pid_ss);

fprintf('TN2 MBC = %.5f m\n', ...
    R2.rmse_mbc_ss);

fprintf('TN2 PID = %.5f m\n', ...
    R2.rmse_pid_ss);

fprintf('TN3 MBC = %.5f m\n', ...
    R3.rmse_mbc_ss);

fprintf('TN3 PID = %.5f m\n', ...
    R3.rmse_pid_ss);
fprintf('\n');
fprintf('===== MAX POSITION ERROR =====\n');

fprintf('TN1 MBC = %.5f m\n', ...
    R1.max_error_mbc);

fprintf('TN1 PID = %.5f m\n', ...
    R1.max_error_pid);

fprintf('TN2 MBC = %.5f m\n', ...
    R2.max_error_mbc);

fprintf('TN2 PID = %.5f m\n', ...
    R2.max_error_pid);

fprintf('TN3 MBC = %.5f m\n', ...
    R3.max_error_mbc);

fprintf('TN3 PID = %.5f m\n', ...
    R3.max_error_pid);
fprintf('\n');
fprintf('===== MAX CONTROL TN3 =====\n');

fprintf('MBC max Fx = %.3f N\n', ...
    max(abs(R3.tau_mbc(1,:))));

fprintf('MBC max Fy = %.3f N\n', ...
    max(abs(R3.tau_mbc(2,:))));

fprintf('MBC max Mz = %.3f Nm\n', ...
    max(abs(R3.tau_mbc(3,:))));

fprintf('PID max Fx = %.3f N\n', ...
    max(abs(R3.tau_pid(1,:))));

fprintf('PID max Fy = %.3f N\n', ...
    max(abs(R3.tau_pid(2,:))));

fprintf('PID max Mz = %.3f Nm\n', ...
    max(abs(R3.tau_pid(3,:))));
fprintf('\n');
fprintf('===== MAX WHEEL SPEED TN3 =====\n');

fprintf('MBC max wheel speed = %.3f rad/s\n', ...
    max(abs(R3.wheel_mbc(:))));

fprintf('PID max wheel speed = %.3f rad/s\n', ...
    max(abs(R3.wheel_pid(:))));
%% ========================================================
% BANG KET QUA
% ========================================================

Condition = [
    "Nominal";
    "Payload 15 kg";
    "Disturbance"
];

MBC_RMSE = [
    R1.rmse_mbc_pos;
    R2.rmse_mbc_pos;
    R3.rmse_mbc_pos
];

PID_RMSE = [
    R1.rmse_pid_pos;
    R2.rmse_pid_pos;
    R3.rmse_pid_pos
];

Improvement_percent = [
    R1.improvement;
    R2.improvement;
    R3.improvement
];

Results = table( ...
    Condition, ...
    MBC_RMSE, ...
    PID_RMSE, ...
    Improvement_percent);

fprintf('\n');
fprintf('============================================\n');
fprintf('       BANG KET QUA TONG HOP\n');
fprintf('============================================\n');

disp(Results);

%% ========================================================
% CHI TIET RMSE
% ========================================================

fprintf('\n');
fprintf('=========== THI NGHIEM 1 ===================\n');
fprintf('MBC Position RMSE = %.5f m\n',R1.rmse_mbc_pos);
fprintf('PID Position RMSE = %.5f m\n',R1.rmse_pid_pos);
fprintf('Cai thien          = %.2f %%\n',R1.improvement);

fprintf('\n');
fprintf('=========== THI NGHIEM 2 ===================\n');
fprintf('MBC Position RMSE = %.5f m\n',R2.rmse_mbc_pos);
fprintf('PID Position RMSE = %.5f m\n',R2.rmse_pid_pos);
fprintf('Cai thien          = %.2f %%\n',R2.improvement);

fprintf('\n');
fprintf('=========== THI NGHIEM 3 ===================\n');
fprintf('MBC Position RMSE = %.5f m\n',R3.rmse_mbc_pos);
fprintf('PID Position RMSE = %.5f m\n',R3.rmse_pid_pos);
fprintf('Cai thien          = %.2f %%\n',R3.improvement);
fprintf('\n===== RMSE GOC =====\n');

fprintf('TN1 MBC theta = %.6f rad\n', ...
    R1.rmse_mbc_theta);

fprintf('TN1 PID theta = %.6f rad\n', ...
    R1.rmse_pid_theta);

fprintf('TN2 MBC theta = %.6f rad\n', ...
    R2.rmse_mbc_theta);

fprintf('TN2 PID theta = %.6f rad\n', ...
    R2.rmse_pid_theta);

fprintf('TN3 MBC theta = %.6f rad\n', ...
    R3.rmse_mbc_theta);

fprintf('TN3 PID theta = %.6f rad\n', ...
    R3.rmse_pid_theta);

%% ========================================================
% FIGURE 1
% QUY DAO - DANH DINH
% ========================================================

figure('Name','TN1 - Nominal');

plot( ...
    R1.qd(1,:), ...
    R1.qd(2,:), ...
    'k--','LineWidth',2);

hold on;

plot( ...
    R1.q_mbc(1,:), ...
    R1.q_mbc(2,:), ...
    'LineWidth',1.5);

plot( ...
    R1.q_pid(1,:), ...
    R1.q_pid(2,:), ...
    'LineWidth',1.5);

grid on;
axis equal;

xlabel('X [m]');
ylabel('Y [m]');

title('TN1 - Dieu kien danh dinh');

legend( ...
    'Reference', ...
    'Model-Based', ...
    'PID', ...
    'Location','best');

%% ========================================================
% FIGURE 2
% QUY DAO - PAYLOAD
% ========================================================

figure('Name','TN2 - Payload');

plot( ...
    R2.qd(1,:), ...
    R2.qd(2,:), ...
    'k--','LineWidth',2);

hold on;

plot( ...
    R2.q_mbc(1,:), ...
    R2.q_mbc(2,:), ...
    'LineWidth',1.5);

plot( ...
    R2.q_pid(1,:), ...
    R2.q_pid(2,:), ...
    'LineWidth',1.5);

grid on;
axis equal;

xlabel('X [m]');
ylabel('Y [m]');

title('TN2 - Tang khoi luong 10 kg -> 15 kg');

legend( ...
    'Reference', ...
    'Model-Based', ...
    'PID', ...
    'Location','best');

%% ========================================================
% FIGURE 3
% QUY DAO - DISTURBANCE
% ========================================================

figure('Name','TN3 - Disturbance');

plot( ...
    R3.qd(1,:), ...
    R3.qd(2,:), ...
    'k--','LineWidth',2);

hold on;

plot( ...
    R3.q_mbc(1,:), ...
    R3.q_mbc(2,:), ...
    'LineWidth',1.5);

plot( ...
    R3.q_pid(1,:), ...
    R3.q_pid(2,:), ...
    'LineWidth',1.5);

grid on;
axis equal;

xlabel('X [m]');
ylabel('Y [m]');

title('TN3 - Nhieu ngoai tu 15s den 20s');

legend( ...
    'Reference', ...
    'Model-Based', ...
    'PID', ...
    'Location','best');

%% ========================================================
% FIGURE 4
% POSITION ERROR - 3 CASES
% ========================================================

figure('Name','Position Error - Nominal');

plot( ...
    R1.t,R1.ep_mbc, ...
    'LineWidth',1.5);

hold on;

plot( ...
    R1.t,R1.ep_pid, ...
    'LineWidth',1.5);

grid on;

xlabel('Time [s]');
ylabel('Position Error [m]');

title('Sai so vi tri - Dieu kien danh dinh');

legend('MBC','PID');

%% ========================================================

figure('Name','Position Error - Payload');

plot( ...
    R2.t,R2.ep_mbc, ...
    'LineWidth',1.5);

hold on;

plot( ...
    R2.t,R2.ep_pid, ...
    'LineWidth',1.5);

grid on;

xlabel('Time [s]');
ylabel('Position Error [m]');

title('Sai so vi tri - Tang tai');

legend('MBC','PID');

%% ========================================================

figure('Name','Position Error - Disturbance');

plot( ...
    R3.t,R3.ep_mbc, ...
    'LineWidth',1.5);

hold on;

plot( ...
    R3.t,R3.ep_pid, ...
    'LineWidth',1.5);

xline(15,'--');
xline(20,'--');

grid on;

xlabel('Time [s]');
ylabel('Position Error [m]');

title('Sai so vi tri khi co nhieu');

legend( ...
    'MBC', ...
    'PID', ...
    'Bat dau nhieu', ...
    'Ket thuc nhieu');

%% ========================================================
% FIGURE 5
% RMSE COMPARISON
% ========================================================

figure('Name','RMSE Comparison');

RMSEdata = [
    R1.rmse_mbc_pos R1.rmse_pid_pos;
    R2.rmse_mbc_pos R2.rmse_pid_pos;
    R3.rmse_mbc_pos R3.rmse_pid_pos
];

bar(RMSEdata);

grid on;

set(gca, ...
    'XTickLabel', ...
    {'Nominal','Payload','Disturbance'});

ylabel('RMSE Position [m]');

title('So sanh RMSE MBC va PID');

legend( ...
    'Model-Based', ...
    'PID', ...
    'Location','best');

%% ========================================================
% FIGURE 6
% IMPROVEMENT
% ========================================================

figure('Name','Improvement');

bar([
    R1.improvement;
    R2.improvement;
    R3.improvement
]);

grid on;

set(gca, ...
    'XTickLabel', ...
    {'Nominal','Payload','Disturbance'});

ylabel('Improvement [%]');

title('Muc cai thien RMSE cua MBC so voi PID');
%% ========================================================
% CONTROL SIGNAL - TN3
% ========================================================

figure('Name','TN3 - Control MBC');

plot(R3.t,R3.tau_mbc(1,:), ...
    'LineWidth',1.3);

hold on;

plot(R3.t,R3.tau_mbc(2,:), ...
    'LineWidth',1.3);

plot(R3.t,R3.tau_mbc(3,:), ...
    'LineWidth',1.3);

xline(15,'--');
xline(20,'--');

grid on;

xlabel('Time [s]');
ylabel('Control input');

title('TN3 - Tin hieu dieu khien MBC');

legend( ...
    'F_x', ...
    'F_y', ...
    'M_z', ...
    'Bat dau nhieu', ...
    'Ket thuc nhieu');
figure('Name','TN3 - Control PID');

plot(R3.t,R3.tau_pid(1,:), ...
    'LineWidth',1.3);

hold on;

plot(R3.t,R3.tau_pid(2,:), ...
    'LineWidth',1.3);

plot(R3.t,R3.tau_pid(3,:), ...
    'LineWidth',1.3);

xline(15,'--');
xline(20,'--');

grid on;

xlabel('Time [s]');
ylabel('Control input');

title('TN3 - Tin hieu dieu khien PID');

legend( ...
    'F_x', ...
    'F_y', ...
    'M_z', ...
    'Bat dau nhieu', ...
    'Ket thuc nhieu');
%% ========================================================
% LUU BANG KET QUA
% ========================================================

writetable( ...
    Results, ...
    'Simulation_Results.csv');

fprintf('\n');
fprintf('Da luu bang vao file:\n');
fprintf('Simulation_Results.csv\n');

fprintf('\n');
fprintf('Mo phong 3 thi nghiem hoan tat!\n');
figure('Name','TN2 - Angular Error');

plot(R2.t,R2.error_mbc(3,:), ...
    'LineWidth',1.5);

hold on;

plot(R2.t,R2.error_pid(3,:), ...
    'LineWidth',1.5);

grid on;

xlabel('Time [s]');
ylabel('e_\theta [rad]');

title('TN2 - Sai so goc khi thay doi I_z');

legend( ...
    'MBC', ...
    'PID', ...
    'Location','best');