function export_reference_trajectories()
% Export four reference trajectories before running the MBC/PID benchmark.
% Uses exactly the same trajectory.m and parameters.m as the simulation.
% MATLAB R2022b. Figures: 300 DPI PNG + vector PDF; data: CSV.
base = fileparts(mfilename('fullpath'));
addpath(base);
p = parameters();
t = (p.t0:p.dt:p.tf)';
kinds = {'circle','ellipse','eight','lateral_sine'};
labels = {'Circle','Ellipse','Figure eight','Lateral sine'};
outdir = fullfile(base,'results_simulink','reference_trajectories');
if ~exist(outdir,'dir'), mkdir(outdir); end
overview = figure('Color','w','Visible','off','Position',[100 100 1150 900]);
tl = tiledlayout(overview,2,2,'TileSpacing','compact','Padding','compact');
for i = 1:numel(kinds)
    p.traj_type = kinds{i};
    q = zeros(numel(t),3);
    qdot = zeros(numel(t),3);
    qddot = zeros(numel(t),3);
    for j = 1:numel(t)
        [qj,vj,aj] = trajectory(t(j),p);
        q(j,:) = qj(:).';
        qdot(j,:) = vj(:).';
        qddot(j,:) = aj(:).';
    end
    % CSV preserves the exact reference signal and its analytic derivatives.
    T = table(t,q(:,1),q(:,2),q(:,3), ...
        qdot(:,1),qdot(:,2),qdot(:,3), ...
        qddot(:,1),qddot(:,2),qddot(:,3), ...
        'VariableNames',{'time_s','x_m','y_m','theta_rad', ...
        'vx_m_s','vy_m_s','omega_rad_s', ...
        'ax_m_s2','ay_m_s2','alpha_rad_s2'});
    writetable(T,fullfile(outdir,[kinds{i} '_reference.csv']));
    nexttile(tl); plot(q(:,1),q(:,2),'LineWidth',1.7);
    axis equal; grid on;
    xlabel('x_d (m)'); ylabel('y_d (m)'); title(labels{i});
    f = figure('Color','w','Visible','off','Position',[100 100 1050 650]);
    plot(q(:,1),q(:,2),'LineWidth',1.9); axis equal; grid on;
    xlabel('x_d (m)'); ylabel('y_d (m)');
    title([labels{i} ' - reference trajectory']);
    exportgraphics(f,fullfile(outdir,[kinds{i} '_reference_xy.png']),'Resolution',300);
    exportgraphics(f,fullfile(outdir,[kinds{i} '_reference_xy.pdf']),'ContentType','vector');
    close(f);
    f = figure('Color','w','Visible','off','Position',[100 100 1100 780]);
    tiledlayout(f,3,1,'TileSpacing','compact','Padding','compact');
    ylabels = {'x_d (m)','y_d (m)','theta_d (rad)'};
    for k = 1:3
        nexttile; plot(t,q(:,k),'LineWidth',1.5); grid on;
        ylabel(ylabels{k});
        if k == 3, xlabel('Time (s)'); end
    end
    exportgraphics(f,fullfile(outdir,[kinds{i} '_reference_states.png']),'Resolution',300);
    exportgraphics(f,fullfile(outdir,[kinds{i} '_reference_states.pdf']),'ContentType','vector');
    close(f);
end
title(tl,'Reference trajectories used by the MBC/PID benchmark');
exportgraphics(overview,fullfile(outdir,'all_4_reference_trajectories.png'),'Resolution',300);
exportgraphics(overview,fullfile(outdir,'all_4_reference_trajectories.pdf'),'ContentType','vector');
close(overview);
fprintf('Exported reference trajectory figures and CSVs to %s\n',outdir);
end
