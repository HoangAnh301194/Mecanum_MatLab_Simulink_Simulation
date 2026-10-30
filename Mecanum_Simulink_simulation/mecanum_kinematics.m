function wheel_speed = mecanum_kinematics(nu,p)
% =========================================================
% MECANUM INVERSE KINEMATICS
%
% nu = [vx; vy; omega]
%
% Output:
% wheel_speed = [w1; w2; w3; w4] rad/s
% =========================================================

r = p.r;
L = p.L;
W = p.W;

a = L + W;

Jm = (1/r)*[
     1  -1  -a;
     1   1   a;
     1   1  -a;
     1  -1   a
];

wheel_speed = Jm*nu;

end