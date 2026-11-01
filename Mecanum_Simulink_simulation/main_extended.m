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

% ------------------------------------------------------------
% ONE PNG PER GRAPH (MATLAB R2022b). All simulation/CSV code above
% remains unchanged. No subplot: 4 trajectories x 3 conditions.
% Exports: 12 paths + 12 errors + 12 path zooms (36 PNGs).
% The older grouped images in results_extended/ are not overwritten.
% ------------------------------------------------------------
figure_dir = fullfile(output_dir,'figures_individual');
if ~exist(figure_dir,'dir'), mkdir(figure_dir); end
figure_visible = 'off';  % Change to 'on' to show every figure.
make_zoom = true;        % Additional separate close-up images.
dpi = 300;

ref_color = [0.55 0.55 0.55];
mbc_color = [0.03 0.34 0.76];
pid_color = [0.85 0.16 0.13];
ref_width = 0.65;
track_width = 0.85;
count = 0;

for it = 1:numel(traj_list)
    for ic = 1:numel(condition_list)
        R = all_cases{it,ic};
        tag = [traj_list{it} '_' condition_list{ic}];

        % A. Full reference / MBC / PID trajectory.
        f = figure('Visible',figure_visible,'Color','w', ...
                   'Units','centimeters','Position',[2 2 17 13]);
        ax = axes('Parent',f); hold(ax,'on');
        h_ref = plot(ax,R.qd(1,:),R.qd(2,:),':', ...
            'Color',ref_color,'LineWidth',ref_width);
        h_mbc = plot(ax,R.q_mbc(1,:),R.q_mbc(2,:),'-', ...
            'Color',mbc_color,'LineWidth',track_width);
        h_pid = plot(ax,R.q_pid(1,:),R.q_pid(2,:),'--', ...
            'Color',pid_color,'LineWidth',track_width);
        set(ax,'FontName','Arial','FontSize',11);
        box(ax,'on'); grid(ax,'on');
        xlabel(ax,'x (m)'); ylabel(ax,'y (m)');
        title(ax,['Trajectory: ' tag],'Interpreter','none');
        legend(ax,[h_ref h_mbc h_pid],{'Reference','MBC','PID'}, ...
            'Location','southoutside','Orientation','horizontal', ...
            'Box','off');
        xx = [R.qd(1,:),R.q_mbc(1,:),R.q_pid(1,:)];
        yy = [R.qd(2,:),R.q_mbc(2,:),R.q_pid(2,:)];
        padx = 0.08*max(max(xx)-min(xx),0.1);
        pady = 0.10*max(max(yy)-min(yy),0.1);
        xlim(ax,[min(xx)-padx max(xx)+padx]);
        ylim(ax,[min(yy)-pady max(yy)+pady]);
        if strcmp(traj_list{it},'lateral_sine')
            axis(ax,'normal');  % Stretch y to reveal lateral motion.
        else
            axis(ax,'equal');
        end
        exportgraphics(f,fullfile(figure_dir,['path_' tag '.png']), ...
            'Resolution',dpi);
        close(f); count = count+1;

        % B. Position error, separate from the tracking plot.
        f = figure('Visible',figure_visible,'Color','w', ...
                   'Units','centimeters','Position',[2 2 17 11]);
        ax = axes('Parent',f); hold(ax,'on');
        h_mbc = plot(ax,R.t,R.ep_mbc,'-', ...
            'Color',mbc_color,'LineWidth',0.9);
        h_pid = plot(ax,R.t,R.ep_pid,'--', ...
            'Color',pid_color,'LineWidth',0.9);
        if ic == 3
            xline(ax,15,':','Color',[0.5 0.5 0.5]);
            xline(ax,20,':','Color',[0.5 0.5 0.5]);
        end
        set(ax,'FontName','Arial','FontSize',11);
        box(ax,'on'); grid(ax,'on');
        xlabel(ax,'Time (s)'); ylabel(ax,'Position error e_p (m)');
        title(ax,['Position error: ' tag],'Interpreter','none');
        legend(ax,[h_mbc h_pid],{'MBC','PID'},'Location','northeast');
        xlim(ax,[R.t(1) R.t(end)]);
        ylim(ax,[0 max(0.05,1.08*max([R.ep_mbc(:);R.ep_pid(:)]))]);
        exportgraphics(f,fullfile(figure_dir,['error_' tag '.png']), ...
            'Resolution',dpi);
        close(f); count = count+1;

        % C. Optional standalone zoom at maximum PID/MBC separation.
        % This magnifies differences without altering the full path.
        if make_zoom
            sep = hypot(R.q_mbc(1,:)-R.q_pid(1,:), ...
                        R.q_mbc(2,:)-R.q_pid(2,:));
            [max_sep,k_peak] = max(sep);
            sel = abs(R.t-R.t(k_peak)) <= 1.25;
            if nnz(sel)<2
                sel = max(1,k_peak-1):min(numel(R.t),k_peak+1);
            end
            f = figure('Visible',figure_visible,'Color','w', ...
                       'Units','centimeters','Position',[2 2 17 13]);
            ax = axes('Parent',f); hold(ax,'on');
            h_ref = plot(ax,R.qd(1,sel),R.qd(2,sel),':', ...
                'Color',ref_color,'LineWidth',ref_width);
            h_mbc = plot(ax,R.q_mbc(1,sel),R.q_mbc(2,sel),'-', ...
                'Color',mbc_color,'LineWidth',track_width);
            h_pid = plot(ax,R.q_pid(1,sel),R.q_pid(2,sel),'--', ...
                'Color',pid_color,'LineWidth',track_width);
            plot(ax,R.q_mbc(1,k_peak),R.q_mbc(2,k_peak),'o', ...
                 'Color',mbc_color,'MarkerSize',5, ...
                 'HandleVisibility','off');
            plot(ax,R.q_pid(1,k_peak),R.q_pid(2,k_peak),'s', ...
                 'Color',pid_color,'MarkerSize',5, ...
                 'HandleVisibility','off');
            set(ax,'FontName','Arial','FontSize',11);
            box(ax,'on'); grid(ax,'on');
            xlabel(ax,'x (m)'); ylabel(ax,'y (m)');
            title(ax,sprintf('Zoom: %s | t=%.2f s | gap=%.4f m', ...
                tag,R.t(k_peak),max_sep),'Interpreter','none');
            legend(ax,[h_ref h_mbc h_pid],{'Reference','MBC','PID'}, ...
                'Location','southoutside','Orientation','horizontal', ...
                'Box','off');
            xx = [R.qd(1,sel),R.q_mbc(1,sel),R.q_pid(1,sel)];
            yy = [R.qd(2,sel),R.q_mbc(2,sel),R.q_pid(2,sel)];
            padx = 0.13*max(max(xx)-min(xx),0.025);
            pady = 0.13*max(max(yy)-min(yy),0.025);
            xlim(ax,[min(xx)-padx max(xx)+padx]);
            ylim(ax,[min(yy)-pady max(yy)+pady]);
            if strcmp(traj_list{it},'lateral_sine')
                axis(ax,'normal');
            else
                axis(ax,'equal');
            end
            exportgraphics(f,fullfile(figure_dir, ...
                ['path_zoom_' tag '.png']),'Resolution',dpi);
            close(f); count = count+1;
        end
    end
end

disp(['24 simulation runs completed; CSV in ',output_dir]);
disp(['Exported ',num2str(count),' separate PNGs to ',figure_dir]);
