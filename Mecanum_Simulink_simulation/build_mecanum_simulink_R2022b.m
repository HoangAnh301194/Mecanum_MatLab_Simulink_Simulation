function build_mecanum_simulink_R2022b()
% BUILD_MECANUM_SIMULINK_R2022B Creates real Simulink .slx project.
% Requirements: MATLAB R2022b, Simulink and MATLAB Function block support.
% Note: no powergui; this is a mathematical dynamics model, not SPS.
%
% Creates: mecanum_control_benchmark.slx
% Diagram: reference generator -> independent MBC/PID closed-loop plants
%          -> scopes and To Workspace.
%
base = fileparts(mfilename('fullpath'));
old = pwd;
cleanup = onCleanup(@() cd(old)); %#ok<NASGU>
cd(base);
addpath(base);
mdl = 'mecanum_control_benchmark';
if bdIsLoaded(mdl), close_system(mdl,0); end
if exist([mdl '.slx'],'file')
    backup = [mdl '_backup.slx'];
    copyfile([mdl '.slx'],backup,'f');
end
new_system(mdl);
set_param(mdl,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','0.01','StartTime','0','StopTime','40', ...
    'ReturnWorkspaceOutputs','on');

% -------------------------- sources / constants -------------------------
add_block('simulink/Sources/Clock',[mdl '/Clock'], ...
    'Position',[45 75 75 105]);
addConstant(mdl,'Trajectory_Mode','SLX_TRAJ_ID',[50 152 165 186]);
addConstant(mdl,'Condition_Mode','SLX_CASE_ID',[50 290 165 324]);
addConstant(mdl,'Controller_Parameters','SLX_CTRL',[45 400 182 437]);
addConstant(mdl,'Plant_Parameters','SLX_PLANT',[45 505 182 542]);
addMF(mdl,'Trajectory_Generator','Trajectory_Block.txt',[300 85 480 194],base);
addMF(mdl,'Disturbance_Generator','Disturbance_Block.txt',[300 255 480 330],base);
addLoop(mdl,'MBC_ClosedLoop','MBC', [690 82 1015 300],base);
addLoop(mdl,'PID_ClosedLoop','PID', [690 410 1015 628],base);

% Top-level lines: trajectory reference and shared experimental inputs.
wire(mdl,'Clock/1','Trajectory_Generator/1');
wire(mdl,'Trajectory_Mode/1','Trajectory_Generator/2');
wire(mdl,'Controller_Parameters/1','Trajectory_Generator/3');
wire(mdl,'Clock/1','Disturbance_Generator/1');
wire(mdl,'Condition_Mode/1','Disturbance_Generator/2');
for ii = 1:3
    wire(mdl,sprintf('Trajectory_Generator/%d',ii), ...
        sprintf('MBC_ClosedLoop/%d',ii));
    wire(mdl,sprintf('Trajectory_Generator/%d',ii), ...
        sprintf('PID_ClosedLoop/%d',ii));
end
wire(mdl,'Disturbance_Generator/1','MBC_ClosedLoop/4');
wire(mdl,'Disturbance_Generator/1','PID_ClosedLoop/4');
wire(mdl,'Controller_Parameters/1','MBC_ClosedLoop/5');
wire(mdl,'Controller_Parameters/1','PID_ClosedLoop/5');
wire(mdl,'Plant_Parameters/1','MBC_ClosedLoop/6');
wire(mdl,'Plant_Parameters/1','PID_ClosedLoop/6');

% ---------------------------- main monitors ----------------------------
addMF(mdl,'Tracking_Errors','Error_Block.txt',[1125 280 1295 410],base);
wire(mdl,'Trajectory_Generator/1','Tracking_Errors/1');
wire(mdl,'MBC_ClosedLoop/1','Tracking_Errors/2');
wire(mdl,'PID_ClosedLoop/1','Tracking_Errors/3');

add_block('simulink/Signal Routing/Demux',[mdl '/Ref_xyz'], ...
    'Outputs','3','Position',[1150 63 1155 133]);
add_block('simulink/Signal Routing/Demux',[mdl '/MBC_xyz'], ...
    'Outputs','3','Position',[1150 150 1155 220]);
add_block('simulink/Signal Routing/Demux',[mdl '/PID_xyz'], ...
    'Outputs','3','Position',[1150 460 1155 530]);
wire(mdl,'Trajectory_Generator/1','Ref_xyz/1');
wire(mdl,'MBC_ClosedLoop/1','MBC_xyz/1');
wire(mdl,'PID_ClosedLoop/1','PID_xyz/1');

for ii=1:3
    signals = {'X','Y','Heading'};
    name = signals{ii};
    yy = 55+(ii-1)*130;
    addMux(mdl,['Mux_' name],3,[1370 yy 1390 yy+65]);
    addScope(mdl,['Scope_' name '_Tracking'],[1505 yy+4 1660 yy+60]);
    wire(mdl,sprintf('Ref_xyz/%d',ii),['Mux_' name '/1']);
    wire(mdl,sprintf('MBC_xyz/%d',ii),['Mux_' name '/2']);
    wire(mdl,sprintf('PID_xyz/%d',ii),['Mux_' name '/3']);
    wire(mdl,['Mux_' name '/1'],['Scope_' name '_Tracking/1']);
end

addMux(mdl,'Mux_PosError',2,[1375 477 1395 532]);
addScope(mdl,'Scope_Position_Error',[1510 480 1660 530]);
wire(mdl,'Tracking_Errors/1','Mux_PosError/1');
wire(mdl,'Tracking_Errors/2','Mux_PosError/2');
wire(mdl,'Mux_PosError/1','Scope_Position_Error/1');

addMux(mdl,'Mux_HeadingError',2,[1375 565 1395 620]);
addScope(mdl,'Scope_Heading_Error',[1510 570 1660 620]);
wire(mdl,'Tracking_Errors/3','Mux_HeadingError/1');
wire(mdl,'Tracking_Errors/4','Mux_HeadingError/2');
wire(mdl,'Mux_HeadingError/1','Scope_Heading_Error/1');

addScope(mdl,'Scope_MBC_Forces',[1510 685 1660 730]);
addScope(mdl,'Scope_PID_Forces',[1510 775 1660 820]);
wire(mdl,'MBC_ClosedLoop/3','Scope_MBC_Forces/1');
wire(mdl,'PID_ClosedLoop/3','Scope_PID_Forces/1');

% Wheel speed is logged/visualized only; does not affect the plant.
addMF(mdl,'WheelSpeed_MBC','WheelSpeed_Block.txt',[1120 680 1300 750],base);
addMF(mdl,'WheelSpeed_PID','WheelSpeed_Block.txt',[1120 790 1300 860],base);
wire(mdl,'MBC_ClosedLoop/2','WheelSpeed_MBC/1');
wire(mdl,'PID_ClosedLoop/2','WheelSpeed_PID/1');
wire(mdl,'Plant_Parameters/1','WheelSpeed_MBC/2');
wire(mdl,'Plant_Parameters/1','WheelSpeed_PID/2');

% Time-series logs automatically captured by the SimulationOutput object.
logSignal(mdl,'Trajectory_Generator/1','sim_qd',[1820 40 1945 75]);
logSignal(mdl,'MBC_ClosedLoop/1','sim_q_mbc',[1820 100 1945 135]);
logSignal(mdl,'PID_ClosedLoop/1','sim_q_pid',[1820 160 1945 195]);
logSignal(mdl,'MBC_ClosedLoop/2','sim_nu_mbc',[1820 220 1945 255]);
logSignal(mdl,'PID_ClosedLoop/2','sim_nu_pid',[1820 280 1945 315]);
logSignal(mdl,'MBC_ClosedLoop/3','sim_tau_mbc',[1820 340 1945 375]);
logSignal(mdl,'PID_ClosedLoop/3','sim_tau_pid',[1820 400 1945 435]);
logSignal(mdl,'Disturbance_Generator/1','sim_disturbance',[1820 460 1945 495]);
logSignal(mdl,'WheelSpeed_MBC/1','sim_wheel_mbc',[1820 520 1945 555]);
logSignal(mdl,'WheelSpeed_PID/1','sim_wheel_pid',[1820 580 1945 615]);

% Start with the reproducible reference+nominal scenario.
configure_simulink_benchmark('circle','nominal',mdl);
save_system(mdl,[mdl '.slx']);
try
    set_param(mdl,'SimulationCommand','update');
    save_system(mdl);
    fprintf('Model compiled/updated successfully.\n');
catch ME
    fprintf(2,'Model was saved, but Update Diagram failed:\n%s\n', ...
        getReport(ME,'basic','hyperlinks','off'));
    rethrow(ME);
end
open_system(mdl);
fprintf('\nCREATED: %s\n',fullfile(base,[mdl '.slx']));
fprintf('Next: run_simulink_benchmark(''circle'',''nominal'')\n');
end

function addConstant(mdl,name,val,pos)
add_block('simulink/Sources/Constant',[mdl '/' name], ...
    'Value',val,'Position',pos);
end

function addMF(sys,name,filename,pos,base)
path = [sys '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function',path, ...
    'Position',pos);
chart = find(sfroot,'-isa','Stateflow.EMChart','Path',path);
if isempty(chart), error('Cannot find MATLAB Function block %s',path); end
chart.Script = fileread(fullfile(base,'blocks',filename));
end

function addMux(mdl,name,n,pos)
add_block('simulink/Signal Routing/Mux',[mdl '/' name], ...
    'Inputs',num2str(n),'Position',pos);
end

function addScope(mdl,name,pos)
add_block('simulink/Sinks/Scope',[mdl '/' name], 'Position',pos);
end

function logSignal(mdl,source,varname,pos)
name = ['Log_' varname];
add_block('simulink/Sinks/To Workspace',[mdl '/' name], ...
    'VariableName',varname,'SaveFormat','Timeseries', ...
    'Position',pos);
wire(mdl,source,[name '/1']);
end

function wire(sys,from,to)
add_line(sys,from,to,'autorouting','on');
end

function addLoop(mdl,loopname,kind,pos,base)
ss = [mdl '/' loopname];
add_block('simulink/Ports & Subsystems/Subsystem',ss,'Position',pos);
% Remove the stock In1/Out1 pair from the new empty subsystem.
delete_line(ss,'In1/1','Out1/1');
delete_block([ss '/In1']);
delete_block([ss '/Out1']);
in = {'qd','qd_dot','qd_ddot','disturbance','ctrl','plant'};
for i=1:6
    yy=55+(i-1)*70;
    add_block('simulink/Sources/In1',[ss '/' in{i}], ...
        'Port',num2str(i),'Position',[25 yy 65 yy+20]);
end
out={'q','nu','tau'};
for i=1:3
    yy=90+(i-1)*125;
    add_block('simulink/Sinks/Out1',[ss '/' out{i}], ...
        'Port',num2str(i),'Position',[1165 yy 1195 yy+20]);
end

if strcmp(kind,'MBC')
    ctrlblock = 'MBC_Controller';
    addMF(ss,ctrlblock,'MBC_Block.txt',[305 145 505 285],base);
    wire(ss,'qd_ddot/1',[ctrlblock '/5']);
    wire(ss,'ctrl/1',[ctrlblock '/6']);
else
    ctrlblock = 'PID_Controller';
    addMF(ss,ctrlblock,'PID_Block.txt',[305 145 505 285],base);
    add_block('simulink/Discrete/Unit Delay',[ss '/PID_Integral_Memory'], ...
        'InitialCondition','[0;0;0]', 'SampleTime','0.01', ...
        'Position',[265 360 305 400]);
    wire(ss,'PID_Integral_Memory/1',[ctrlblock '/5']);
    wire(ss,[ctrlblock '/2'],'PID_Integral_Memory/1');
    wire(ss,'ctrl/1',[ctrlblock '/6']);
    add_block('simulink/Sinks/Terminator',[ss '/Unused_Accel'], ...
        'Position',[155 205 175 225]);
    wire(ss,'qd_ddot/1','Unused_Accel/1');
end
wire(ss,'qd/1',[ctrlblock '/3']);
wire(ss,'qd_dot/1',[ctrlblock '/4']);

add_block('simulink/Math Operations/Sum',[ss '/Input_Force_Plus_Disturbance'], ...
    'Inputs','++','Position',[570 185 595 240]);
addMF(ss,'Robot_Dynamics','Plant_Block.txt',[685 155 855 300],base);
add_block('simulink/Discrete/Discrete-Time Integrator', ...
    [ss '/Euler_Integrate_q'], 'IntegratorMethod','Integration: Forward Euler', ...
    'SampleTime','0.01', 'InitialCondition','SLX_Q0', ...
    'Position',[935 125 1000 165]);
add_block('simulink/Discrete/Discrete-Time Integrator', ...
    [ss '/Euler_Integrate_nu'], 'IntegratorMethod','Integration: Forward Euler', ...
    'SampleTime','0.01', 'InitialCondition','SLX_NU0', ...
    'Position',[935 310 1000 350]);

wire(ss,[ctrlblock '/1'],'Input_Force_Plus_Disturbance/1');
wire(ss,'disturbance/1','Input_Force_Plus_Disturbance/2');
wire(ss,'Input_Force_Plus_Disturbance/1','Robot_Dynamics/3');
wire(ss,'plant/1','Robot_Dynamics/4');
wire(ss,'Robot_Dynamics/1','Euler_Integrate_q/1');
wire(ss,'Robot_Dynamics/2','Euler_Integrate_nu/1');
wire(ss,'Euler_Integrate_q/1',[ctrlblock '/1']);
wire(ss,'Euler_Integrate_nu/1',[ctrlblock '/2']);
wire(ss,'Euler_Integrate_q/1','Robot_Dynamics/1');
wire(ss,'Euler_Integrate_nu/1','Robot_Dynamics/2');
wire(ss,'Euler_Integrate_q/1','q/1');
wire(ss,'Euler_Integrate_nu/1','nu/1');
wire(ss,[ctrlblock '/1'],'tau/1');
end
