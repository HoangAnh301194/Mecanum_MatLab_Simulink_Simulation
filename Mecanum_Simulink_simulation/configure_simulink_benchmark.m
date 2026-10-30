function cfg = configure_simulink_benchmark(traj,condition,mdl)
% CONFIGURE_SIMULINK_BENCHMARK Change reference and operating scenario.
% MATLAB R2022b; use after build_mecanum_simulink_R2022b.
%   configure_simulink_benchmark('circle','nominal');
%   configure_simulink_benchmark('eight','disturbance');
%
% Model workspace stores all numerical constants IN THE .slx, so opening
% the generated file does not require an InitFcn callback.
if nargin < 1 || isempty(traj), traj = 'circle'; end
if nargin < 2 || isempty(condition), condition = 'nominal'; end
if nargin < 3 || isempty(mdl), mdl = 'mecanum_control_benchmark'; end

traj = lower(char(traj));
condition = lower(char(condition));
switch traj
    case 'circle', traj_id = 1;
    case 'ellipse', traj_id = 2;
    case {'eight','figure8'}, traj_id = 3; traj = 'eight';
    case {'lateral_sine','sine'}, traj_id = 4; traj = 'lateral_sine';
    otherwise
        error('Unknown trajectory. Use circle, ellipse, eight, lateral_sine.');
end
switch condition
    case 'nominal', case_id = 1;
    case {'payload','payload_15kg'}, case_id = 2; condition = 'payload_15kg';
    case 'disturbance', case_id = 3;
    otherwise
        error('Unknown condition. Use nominal, payload_15kg, disturbance.');
end

p = parameters();
p.traj_type = traj;
[qd0,~,~] = trajectory(0,p);
initial_error = [0.20;-0.10;10*pi/180];
p.q0 = qd0 + initial_error;
p.nu0 = [0;0;0];
plant = p;
if case_id == 2
    plant.m = 15;
    plant.Iz = p.Iz*(plant.m/p.m);
    plant.M = diag([plant.m,plant.m,plant.Iz]);
end

% ctrl indices:
% 1=m,2=Iz,3:5=Ddiag,6:8=KpMBC,9:11=KvMBC,
% 12:14=KpPID,15:17=KiPID,18:20=KdPID,
% 21:23=integral limit,24:26=tau max,27=dt,
% 28=Rtraj,29=wtraj,30=r,31=L,32=W.
ctrl_vec = [p.m;p.Iz;diag(p.D);diag(p.Kp_mbc);diag(p.Kv_mbc); ...
    diag(p.Kp_pid);diag(p.Ki_pid);diag(p.Kd_pid); ...
    p.integral_limit(:);p.tau_max(:);p.dt;p.Rtraj;p.wtraj;p.r;p.L;p.W];
plant_vec = [plant.m;plant.Iz;diag(plant.D);plant.r;plant.L;plant.W];
assert(numel(ctrl_vec)==32 && numel(plant_vec)==8);
if ~bdIsLoaded(mdl), load_system(mdl); end
mws = get_param(mdl,'ModelWorkspace');
assignin(mws,'SLX_CTRL',ctrl_vec);
assignin(mws,'SLX_PLANT',plant_vec);
assignin(mws,'SLX_Q0',p.q0);
assignin(mws,'SLX_NU0',p.nu0);
assignin(mws,'SLX_TRAJ_ID',double(traj_id));
assignin(mws,'SLX_CASE_ID',double(case_id));

cfg = struct('trajectory',traj,'condition',condition,'p_ctrl',p, ...
    'p_plant',plant,'model',mdl,'initial_error',initial_error);
fprintf('[SIMULINK] %s | %s | mass plant = %.1f kg\n',traj,condition,plant.m);
end
