function T = run_simulink_benchmark(traj,condition)
% RUN_SIMULINK_BENCHMARK Compare real Simulink signals with MATLAB solver.
%  T = run_simulink_benchmark('circle','nominal');
%  T = run_simulink_benchmark('eight','disturbance');
%  Metric evaluation reuses compute_metrics.m supplied in extended project.
if nargin < 1 || isempty(traj), traj = 'circle'; end
if nargin < 2 || isempty(condition), condition = 'nominal'; end
base = fileparts(mfilename('fullpath'));
addpath(base);
mdl = 'mecanum_control_benchmark';
if ~exist(fullfile(base,[mdl '.slx']),'file')
    error('Run build_mecanum_simulink_R2022b first.');
end
if ~bdIsLoaded(mdl), load_system(fullfile(base,[mdl '.slx'])); end
cfg = configure_simulink_benchmark(traj,condition,mdl);
set_param(mdl,'SimulationCommand','update');
S = sim(mdl,'StopTime','40');

[t,qd] = getXYZ(S,'sim_qd',3);
[~,qM] = getXYZ(S,'sim_q_mbc',3);
[~,qP] = getXYZ(S,'sim_q_pid',3);
[~,nM] = getXYZ(S,'sim_nu_mbc',3);
[~,nP] = getXYZ(S,'sim_nu_pid',3);
[~,tauM] = getXYZ(S,'sim_tau_mbc',3);
[~,tauP] = getXYZ(S,'sim_tau_pid',3);
[~,wM] = getXYZ(S,'sim_wheel_mbc',4);
[~,wP] = getXYZ(S,'sim_wheel_pid',4);

case_id = find(strcmp(condition,{'nominal','payload_15kg','disturbance'}),1);
if strcmpi(condition,'payload'), case_id = 2; end
if isempty(case_id)
    switch lower(char(condition))
        case 'payload', case_id=2;
        otherwise, error('Unrecognized experiment condition.');
    end
end
Rref = simulate_case(cfg.p_ctrl,cfg.p_plant,case_id);
if numel(t) ~= numel(Rref.t) || max(abs(t(:)-Rref.t(:))) > 1e-8
    error('Wrong signal logging grid. Expected 4001 samples at dt=0.01 s.');
end
% Rebuild a result structure from the Simulink signals, preserving analytic
% reference derivatives from the original MATLAB test for metric evaluation.
R = Rref;
R.t = t';
R.qd = qd';
R.q_mbc = qM'; R.q_pid = qP';
R.nu_mbc = nM'; R.nu_pid = nP';
R.tau_mbc = tauM'; R.tau_pid = tauP';
R.wheel_mbc = wM'; R.wheel_pid = wP';
R.error_mbc = R.qd - R.q_mbc;
R.error_pid = R.qd - R.q_pid;
R.error_mbc(3,:) = atan2(sin(R.error_mbc(3,:)),cos(R.error_mbc(3,:)));
R.error_pid(3,:) = atan2(sin(R.error_pid(3,:)),cos(R.error_pid(3,:)));
R.ep_mbc = hypot(R.error_mbc(1,:),R.error_mbc(2,:));
R.ep_pid = hypot(R.error_pid(1,:),R.error_pid(2,:));

m1 = compute_metrics(R,'mbc',cfg.p_ctrl,case_id);
m2 = compute_metrics(R,'pid',cfg.p_ctrl,case_id);
max_mbc = max(abs(R.q_mbc(:)-Rref.q_mbc(:)));
max_pid = max(abs(R.q_pid(:)-Rref.q_pid(:)));
max_ref = max(abs(R.qd(:)-Rref.qd(:)));
fprintf('\n=== SIMULINK VS MATLAB: %s / %s ===\n',cfg.trajectory,cfg.condition);
fprintf('Max |q_ref SLX-MATLAB| : %.3e\n',max_ref);
fprintf('Max |q_MBC SLX-MATLAB| : %.3e\n',max_mbc);
fprintf('Max |q_PID SLX-MATLAB| : %.3e\n',max_pid);
fprintf('RMSE after 5s: MBC %.6f m; PID %.6f m\n', ...
    m1.rmse_pos_after5_m,m2.rmse_pos_after5_m);
if max([max_mbc,max_pid,max_ref]) > 1e-6
    warning('Mismatch >1e-6, check solver, sample time, PID integral order.');
else
    fprintf('PASS: MATLAB-Simulink state difference <= 1e-6.\n');
end

T = table(string(cfg.trajectory),string(cfg.condition), ...
    m1.rmse_pos_after5_m,m2.rmse_pos_after5_m, ...
    m1.rmse_yaw_after5_deg,m2.rmse_yaw_after5_deg, ...
    m1.control_effort_norm,m2.control_effort_norm, ...
    m1.saturation_pct,m2.saturation_pct, ...
    m1.recovery_s,m2.recovery_s,max_mbc,max_pid, ...
    'VariableNames',{'Trajectory','Condition','RMSE_MBC_m','RMSE_PID_m', ...
    'Yaw_MBC_deg','Yaw_PID_deg','Effort_MBC','Effort_PID', ...
    'Sat_MBC_pct','Sat_PID_pct','Recovery_MBC_s','Recovery_PID_s', ...
    'MaxDiffMBC','MaxDiffPID'});

out_dir = fullfile(base,'results_simulink');
if ~exist(out_dir,'dir'),mkdir(out_dir);end
stem = [cfg.trajectory '_' cfg.condition];
writetable(T,fullfile(out_dir,[stem '_metrics.csv']));
f = figure('Name',['Tracking: ' stem],'Color','w');
subplot(1,2,1);
plot(qd(:,1),qd(:,2),'k--','LineWidth',1.6);hold on;
plot(qM(:,1),qM(:,2),'b','LineWidth',1.1);
plot(qP(:,1),qP(:,2),'r','LineWidth',1.1);
axis equal;grid on;xlabel('x (m)');ylabel('y (m)');
legend('Reference','MBC','PID','Location','best');
title('Path');
subplot(1,2,2);
plot(t,R.ep_mbc(:),'b',t,R.ep_pid(:),'r');grid on;
xlabel('Time (s)');ylabel('Position error (m)');
legend('MBC','PID','Location','best');title('Tracking error');
saveas(f,fullfile(out_dir,[stem '_paths_errors.png']));
end

function [t,A] = getXYZ(S,name,channels)
ts = S.get(name);
if ~isa(ts,'timeseries'), error('%s not a timeseries; check To Workspace.',name); end
t = double(ts.Time(:));
X = double(ts.Data);
N = numel(t);
if numel(X) ~= N*channels
    error('%s has %d numbers; expected %d.',name,numel(X),N*channels);
end
if size(X,1) == N && size(X,2) == channels && ndims(X)==2
    A = X;
elseif size(X,1) == channels && size(X,2) == N && ndims(X)==2
    A = X.';
else
    A = reshape(X,[channels,N]).';
end
end
