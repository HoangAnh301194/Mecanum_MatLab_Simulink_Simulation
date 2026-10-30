function [qd,qd_dot,qd_ddot] = trajectory(t,p)
% Four smooth references for planar Mecanum-robot benchmark.
% q = [x; y; theta], qd_dot and qd_ddot use WORLD coordinates.
% Select with p.traj_type: 'circle' (default), 'ellipse', 'eight', 'lateral_sine'.
%
% The angle for ellipse/eight is the instantaneous path tangent. All
% angular errors must be wrapped in the controller/metric functions.

if isfield(p,'traj_type')
    kind = char(p.traj_type);
else
    kind = 'circle';
end
w = p.wtraj;

switch lower(kind)
    case 'circle'
        % Original reference preserved exactly (baseline comparison).
        R = p.Rtraj;
        x = R*cos(w*t);           y = R*sin(w*t);
        vx = -R*w*sin(w*t);       vy = R*w*cos(w*t);
        ax = -R*w^2*cos(w*t);     ay = -R*w^2*sin(w*t);
        theta = w*t + pi/2;
        omega = w; alpha = 0;

    case 'ellipse'
        a = 2; b = 1.2;
        x = a*cos(w*t);           y = b*sin(w*t);
        vx = -a*w*sin(w*t);       vy = b*w*cos(w*t);
        ax = -a*w^2*cos(w*t);     ay = -b*w^2*sin(w*t);
        jx = a*w^3*sin(w*t);      jy = -b*w^3*cos(w*t);
        [theta,omega,alpha] = tangent_heading(vx,vy,ax,ay,jx,jy);

    case {'eight','figure8'}
        a = 2; b = 1.4;
        x = a*sin(w*t);           y = 0.5*b*sin(2*w*t);
        vx = a*w*cos(w*t);        vy = b*w*cos(2*w*t);
        ax = -a*w^2*sin(w*t);     ay = -2*b*w^2*sin(2*w*t);
        jx = -a*w^3*cos(w*t);     jy = -4*b*w^3*cos(2*w*t);
        [theta,omega,alpha] = tangent_heading(vx,vy,ax,ay,jx,jy);

    case {'lateral_sine','sine'}
        % Holonomic test: translation on both axes at fixed body heading.
        v = 0.35; A = 0.6; ws = 0.4;
        x = v*t;                  y = A*sin(ws*t);
        vx = v;                   vy = A*ws*cos(ws*t);
        ax = 0;                   ay = -A*ws^2*sin(ws*t);
        theta = 0; omega = 0; alpha = 0;

    otherwise
        error('trajectory:unknown', 'Unknown p.traj_type: %s',kind);
end

qd = [x;y;theta];
qd_dot = [vx;vy;omega];
qd_ddot = [ax;ay;alpha];
end

function [theta,omega,alpha] = tangent_heading(vx,vy,ax,ay,jx,jy)
% Analytic derivatives of atan2(vy,vx) avoiding finite-difference noise.
s2 = vx^2+vy^2;
if s2 <= 1e-12
    error('trajectory:zeroSpeed','Tangent heading undefined at zero speed.');
end
crossVA = vx*ay-vy*ax;
theta = atan2(vy,vx);
omega = crossVA/s2;
alpha = (vx*jy-vy*jx)/s2 - 2*crossVA*(vx*ax+vy*ay)/s2^2;
end
