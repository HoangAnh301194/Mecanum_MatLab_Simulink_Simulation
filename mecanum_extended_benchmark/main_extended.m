% MAIN_EXTENDED: four trajectories x three plant conditions x MBC/PID.
% Run from the folder containing this file (MATLAB Current Folder).
% Original controllers are kept UNCHANGED. No need for extra toolboxes.
clear; clc; close all;

p0 = parameters();
traj_list = {'circle','ellipse','eight','lateral_sine'};
condition_list = {'nominal','payload_15kg','disturbance'};
output_dir = fullfile(pwd,'results_extended');
if ~exist(output_dir,'dir'), mkdir(output_dir); end

% Fix the SAME initial tracking error for every trajectory and both
% controllers. Do not compare startup RMSE across dissimilar qd(0).
initial_error_world = [0.20; -0.10; 10*pi/180];

rows = cell(numel(traj_list)*numel(condition_list)*2,1);
all_cases = cell(numel(traj_list),numel(condition_list));
idx_row = 0;

for it = 1:numel(traj_list)
    p = p0;
    p.traj_type = traj_list{it};
    [qstart,~,~] = trajectory(p.t0,p);
    p.q0 = qstart + initial_error_world;

    for ic = 1:numel(condition_list)
        plant = p;
        if ic == 2
            plant.m = 15;
            plant.Iz = p.Iz*plant.m/p.m;
            plant.M = diag([plant.m,plant.m,plant.Iz]);
        end

        % Case 1: nominal; 2: increased mass; 3: external disturbance.
        R = simulate_case(p,plant,ic);
        all_cases{it,ic} = R;

        path_length = sum(sqrt(sum(diff(R.qd(1:2,:),1,2).^2,1)));
        for im = 1:2
            if im == 1, controller = 'MBC'; else, controller = 'PID'; end
            m = compute_metrics(R,controller,p,ic);
            idx_row = idx_row+1;
            rows{idx_row} = struct( ...
                'Trajectory',string(traj_list{it}), ...
                'Condition',string(condition_list{ic}), ...
                'Controller',string(controller), ...
                'Path_length_m',path_length, ...
                'RMSE_all_m',m.rmse_pos_all_m, ...
                'RMSE_first5_m',m.rmse_pos_first5_m, ...
                'RMSE_after10_m',m.rmse_pos_after10_m, ...
                'RMSE_along_after5_m',m.rmse_along_after5_m, ...
                'RMSE_cross_after5_m',m.rmse_cross_after5_m, ...
                'RMSE_yaw_all_deg',m.rmse_yaw_all_deg, ...
                'RMSE_after5_m',m.rmse_pos_after5_m, ...
                'RMSE_yaw_after5_deg',m.rmse_yaw_after5_deg, ...
                'MAE_after5_m',m.mae_pos_after5_m, ...
                'P95_after5_m',m.p95_pos_after5_m, ...
                'Max_after5_m',m.max_pos_after5_m, ...
                'IAE_after5_m_s',m.iae_pos_after5_m_s, ...
                'Final_pos_m',m.final_pos_m, ...
                'Final_yaw_deg',m.final_yaw_deg, ...
                'Control_effort_norm',m.control_effort_norm, ...
                'Saturation_pct',m.saturation_pct, ...
                'Control_variation_per_s',m.control_variation_per_s, ...
                'Peak_wheel_rad_s',m.peak_wheel_rad_s, ...
                'Mechanical_work_proxy_J',m.mechanical_work_proxy_J, ...
                'Recovery_s',m.recovery_s);
        end
    end
end

% Build flat table in consistent column order (row structures are identical).
T = struct2table(vertcat(rows{:}));
writetable(T,fullfile(output_dir,'all_metrics.csv'));

% Paired comparisons: improvements computed only WITHIN identical
% trajectory and operating condition (not across different geometries).
P = table();
for it = 1:numel(traj_list)
    for ic = 1:numel(condition_list)
        idx_m = T.Trajectory == string(traj_list{it}) & ...
                T.Condition == string(condition_list{ic}) & T.Controller == "MBC";
        idx_p = T.Trajectory == string(traj_list{it}) & ...
                T.Condition == string(condition_list{ic}) & T.Controller == "PID";
        a = T(idx_m,:); b = T(idx_p,:);
        cmp = table(string(traj_list{it}), string(condition_list{ic}), ...
            a.RMSE_after5_m,b.RMSE_after5_m, ...
            100*(b.RMSE_after5_m-a.RMSE_after5_m)/b.RMSE_after5_m, ...
            a.RMSE_yaw_after5_deg,b.RMSE_yaw_after5_deg, ...
            a.Control_effort_norm,b.Control_effort_norm, ...
            a.Saturation_pct,b.Saturation_pct, ...
            a.Recovery_s,b.Recovery_s, ...
            'VariableNames',{'Trajectory','Condition','MBC_RMSE_m','PID_RMSE_m', ...
            'Improvement_pct','MBC_yaw_deg','PID_yaw_deg', ...
            'MBC_effort','PID_effort','MBC_sat_pct','PID_sat_pct', ...
            'MBC_recovery_s','PID_recovery_s'});
        if isempty(P), P = cmp; else, P = [P;cmp]; end %#ok<AGROW>
    end
end
writetable(P,fullfile(output_dir,'paired_comparison.csv'));

% Isolate the disturbance-induced error from baseline tracking error.
D = table();
for it = 1:numel(traj_list)
    R0 = all_cases{it,1}; R3 = all_cases{it,3};
    tidx = R3.t >= 15 & R3.t <= 30;
    for im = 1:2
        if im == 1
            e0 = R0.ep_mbc; e3 = R3.ep_mbc; method = "MBC";
        else
            e0 = R0.ep_pid; e3 = R3.ep_pid; method = "PID";
        end
        excess_area = trapz(R3.t(tidx),max(0,e3(tidx)-e0(tidx)));
        wdist = (R3.t >= 15 & R3.t <= 20);
        delta_dist_rmse = sqrt(mean(e3(wdist).^2))-sqrt(mean(e0(wdist).^2));
        newrow = table(string(traj_list{it}),method,delta_dist_rmse,excess_area, ...
            'VariableNames',{'Trajectory','Controller', ...
            'Delta_RMSE_during_dist_m','Excess_error_area_15_30_m_s'});
        if isempty(D), D = newrow; else, D = [D;newrow]; end %#ok<AGROW>
    end
end
writetable(D,fullfile(output_dir,'disturbance_impact.csv'));

disp('=== MATCHED PAIR COMPARISONS ==='); disp(P);
disp('=== DISTURBANCE IMPACT vs MATCHED NOMINAL ==='); disp(D);

% Save 2 figures per trajectory, each with 3 operating conditions.
for it = 1:numel(traj_list)
    figure('Name',['Paths: ' traj_list{it}],'Color','w');
    for ic = 1:3
        R = all_cases{it,ic};
        subplot(1,3,ic);
        plot(R.qd(1,:),R.qd(2,:),'k--','LineWidth',1.5); hold on;
        plot(R.q_mbc(1,:),R.q_mbc(2,:),'b','LineWidth',1.0);
        plot(R.q_pid(1,:),R.q_pid(2,:),'r','LineWidth',1.0);
        grid on; axis equal;
        xlabel('x (m)'); ylabel('y (m)'); title(condition_list{ic},'Interpreter','none');
        if ic == 1, legend('Reference','MBC','PID','Location','best'); end
    end
    saveas(gcf,fullfile(output_dir,['path_' traj_list{it} '.png']));

    figure('Name',['Position tracking error: ' traj_list{it}],'Color','w');
    for ic = 1:3
        R = all_cases{it,ic};
        subplot(3,1,ic);
        plot(R.t,R.ep_mbc,'b',R.t,R.ep_pid,'r','LineWidth',1.0);
        grid on; ylabel('e_p (m)'); title(condition_list{ic},'Interpreter','none');
        if ic == 3, xline(15,'k--'); xline(20,'k--'); end
        if ic == 1, legend('MBC','PID','Location','best'); end
        if ic == 3, xlabel('Time (s)'); end
    end
    saveas(gcf,fullfile(output_dir,['error_' traj_list{it} '.png']));
end

fprintf('\n24 runs completed. CSV + graphs are in: %s\n',output_dir);
