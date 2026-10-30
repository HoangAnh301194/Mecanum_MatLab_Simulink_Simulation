function all_results = run_all_simulink_benchmarks()
% Run four trajectories x three scenarios = 12 paired comparisons / 24 runs.
base = fileparts(mfilename('fullpath'));
addpath(base);
trajs = {'circle','ellipse','eight','lateral_sine'};
conds = {'nominal','payload_15kg','disturbance'};
rows = cell(12,1);k=0;
for i=1:4
    for j=1:3
        k=k+1;
        rows{k}=run_simulink_benchmark(trajs{i},conds{j});
    end
end
all_results = vertcat(rows{:});
out_dir = fullfile(base,'results_simulink');
writetable(all_results,fullfile(out_dir,'all_12_comparisons.csv'));
disp(all_results);
configure_simulink_benchmark('circle','nominal');
end
