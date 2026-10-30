function export_report_portrait_figures()
% Rerun the existing MATLAB benchmark and export 3-row portrait figures.
% MATLAB R2022b; run this function from anywhere in the repository.
base=fileparts(mfilename('fullpath'));
project=fileparts(base);
addpath(fullfile(project,'mecanum_extended_benchmark'));
out=fullfile(base,'chapter_4_matlab');
if ~exist(out,'dir'),mkdir(out);end
p0=parameters();
trajs={'circle','ellipse','eight','lateral_sine'};
conds={'nominal','payload_15kg','disturbance'};
for it=1:numel(trajs)
 p=p0;p.traj_type=trajs{it};
 [qstart,~,~]=trajectory(p.t0,p);
 p.q0=qstart+[0.20;-0.10;10*pi/180];
 cases=cell(1,3);
 for ic=1:3
  plant=p;
  if ic==2
   plant.m=15;plant.Iz=p.Iz*plant.m/p.m;
   plant.M=diag([plant.m,plant.m,plant.Iz]);
  end
  cases{ic}=simulate_case(p,plant,ic);
 end
 f=figure('Color','w','Units','centimeters','Position',[2,2,18,28]);
 tiledlayout(3,1,'Padding','compact','TileSpacing','compact');
 for ic=1:3
  R=cases{ic};nexttile;hold on;
  plot(R.qd(1,:),R.qd(2,:),'k--','LineWidth',1.7);
  plot(R.q_mbc(1,:),R.q_mbc(2,:),'Color',[.08,.42,.68],'LineWidth',1.3);
  plot(R.q_pid(1,:),R.q_pid(2,:),'Color',[.76,.29,.24],'LineWidth',1.3);
  grid on;axis equal;axis tight;
  xlabel('x (m)');ylabel('y (m)');title(strrep(conds{ic},'_',' '));
  legend('Reference','MBC','PID','Location','best');
 end
 exportgraphics(f,fullfile(out,['fig_4_paths_' trajs{it} '.png']),'Resolution',300);
 exportgraphics(f,fullfile(out,['fig_4_paths_' trajs{it} '.pdf']),'ContentType','vector');
 close(f);
 f=figure('Color','w','Units','centimeters','Position',[2,2,18,26]);
 tiledlayout(3,1,'Padding','compact','TileSpacing','compact');
 for ic=1:3
  R=cases{ic};nexttile;hold on;
  plot(R.t,R.ep_mbc,'Color',[.08,.42,.68],'LineWidth',1.4);
  plot(R.t,R.ep_pid,'Color',[.76,.29,.24],'LineWidth',1.4);
  if ic==3,xline(15,'k:');xline(20,'k:');end
  grid on;ylabel('e_p (m)');title(strrep(conds{ic},'_',' '));
  legend('MBC','PID','Location','best');
  if ic==3,xlabel('t (s)');end
 end
 exportgraphics(f,fullfile(out,['fig_4_errors_' trajs{it} '.png']),'Resolution',300);
 exportgraphics(f,fullfile(out,['fig_4_errors_' trajs{it} '.pdf']),'ContentType','vector');
 close(f);
end
fprintf('Exported figures to: %s\n',out);
end
