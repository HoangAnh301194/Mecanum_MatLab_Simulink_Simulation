function m = compute_metrics(result,which,p,case_type)
% Metrics calculated from the SAME time samples and limits for MBC/PID.
% Always report position (m) and heading (deg) separately.
% Control effort is NORMALIZED by each force/torque channel's saturation.

switch lower(which)
    case 'mbc'
        err = result.error_mbc;
        pos = result.ep_mbc;
        tau = result.tau_mbc;
        wheel = result.wheel_mbc;
        nu = result.nu_mbc;
    case 'pid'
        err = result.error_pid;
        pos = result.ep_pid;
        tau = result.tau_pid;
        wheel = result.wheel_pid;
        nu = result.nu_pid;
    otherwise
        error('compute_metrics:controller','Controller must be mbc or pid.');
end

t = result.t;
idx = t >= 5;               % Exclude startup transient from primary accuracy metrics.
t2 = t(idx);
e2 = pos(idx);
angle2 = err(3,idx);

% Resolve time-synchronized position error into tangential/lateral axes.
% This is reference-tangent projection, not nearest-point path distance.
vxy = result.qd_dot(1:2,:);
unit_t = bsxfun(@rdivide,vxy,sqrt(sum(vxy.^2,1)));
unit_n = [-unit_t(2,:);unit_t(1,:)];
exy = err(1:2,:);
e_along = sum(unit_t.*exy,1);
e_cross = sum(unit_n.*exy,1);

m.rmse_pos_all_m = sqrt(mean(pos.^2));
m.rmse_pos_first5_m = sqrt(mean(pos(t < 5).^2));
m.rmse_pos_after10_m = sqrt(mean(pos(t >= 10).^2));
m.rmse_along_after5_m = sqrt(mean(e_along(idx).^2));
m.rmse_cross_after5_m = sqrt(mean(e_cross(idx).^2));
m.rmse_pos_after5_m = sqrt(mean(e2.^2));
m.rmse_yaw_all_deg = (180/pi)*sqrt(mean(err(3,:).^2));
m.rmse_yaw_after5_deg = (180/pi)*sqrt(mean(angle2.^2));
m.mae_pos_after5_m = mean(e2);
m.p95_pos_after5_m = empirical_percentile(e2,95);
m.max_pos_after5_m = max(e2);
m.iae_pos_after5_m_s = trapz(t2,e2);
m.final_pos_m = pos(end);
m.final_yaw_deg = abs(err(3,end))*180/pi;

% Equivalent normalized mean-square control utilization, dimensionless.
% The three axes have different physical units (N, N, N*m).
tau_norm = bsxfun(@rdivide,tau,p.tau_max(:));
m.control_effort_norm = trapz(t,sum(tau_norm.^2,1))/(t(end)-t(1));
m.saturation_pct = 100*mean(any(abs(tau_norm(:,1:end-1)) >= 0.999,1));
% Total variation per simulated second, all channels normalized.
m.control_variation_per_s = sum(sum(abs(diff(tau_norm,1,2))))/(t(end)-t(1));
m.peak_wheel_rad_s = max(abs(wheel(:)));
% Sum of the absolute generalized mechanical power, in joule-equivalent.
% This is NOT measured electrical energy consumption.
m.mechanical_work_proxy_J = trapz(t,sum(abs(tau.*nu),1));

% Calculate recovery only for a disturbed run, after disturbance ends.
% Must stay within both tolerances for 2 consecutive seconds.
m.recovery_s = NaN;
if case_type == 3
    e_theta_deg = abs(err(3,:))*180/pi;
    tol_pos = 0.10; tol_theta = 5; hold_s = 2;
    good = (pos <= tol_pos) & (e_theta_deg <= tol_theta);
    start_idx = find(t > 20,1,'first');
    count = ceil(hold_s/p.dt)+1;
    if ~isempty(start_idx)
        for k = start_idx:(numel(t)-count+1)
            if all(good(k:k+count-1))
                m.recovery_s = t(k)-20;
                break;
            end
        end
    end
end
end

function v = empirical_percentile(x,pct)
% Linear interpolation like conventional percentile estimates; no toolbox.
x = sort(x(:));
n = numel(x);
h = 1+(n-1)*pct/100;
lo = floor(h); hi = ceil(h);
v = x(lo)+(h-lo)*(x(hi)-x(lo));
end
