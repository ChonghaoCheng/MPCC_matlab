function [A,B,c] = linearize_batch3D(xbar, ubar, dt)
% LINEARIZE_BATCH3D 批量线性化3D动力学
% 输入:
%   xbar - 名义状态轨迹 [nx, N+1]
%   ubar - 名义控制轨迹 [nu, N]
%   dt - 时间步长
% 输出:
%   A, B, c - 线性化矩阵和偏移项

nx = size(xbar,1); nu = size(ubar,1); N = size(ubar,2);
A = zeros(nx,nx,N); B = zeros(nx,nu,N); c = zeros(nx,N);

for k=1:N
    yaw = xbar(4,k); pit = xbar(5,k); v = xbar(6,k);
    cy = cos(yaw); sy = sin(yaw); cp = cos(pit); sp = sin(pit);

    A(:,:,k) = [ ...
        1, 0, 0, -dt*v*cp*sy, -dt*v*sp*cy,  dt*cp*cy;
        0, 1, 0,  dt*v*cp*cy, -dt*v*sp*sy,  dt*cp*sy;
        0, 0, 1,           0,   dt*v*cp,   dt*sp;
        0, 0, 0,           1,         0,       0;
        0, 0, 0,           0,         1,       0;
        0, 0, 0,           0,         0,       1];

    B(:,:,k) = [ ...
        0,                0,               0;
        0,                0,               0;
        0,                0,               0;
        0,              dt,               0;
        0,               0,              dt;
       dt,               0,               0];

    % 仿射修正
    xbar_next = [ ...
        xbar(1,k) + dt*xbar(6,k)*cos(xbar(5,k))*cos(xbar(4,k));
        xbar(2,k) + dt*xbar(6,k)*cos(xbar(5,k))*sin(xbar(4,k));
        xbar(3,k) + dt*xbar(6,k)*sin(xbar(5,k));
        wrapToPi(xbar(4,k) + dt*ubar(2,k));
        xbar(5,k) + dt*ubar(3,k);
        xbar(6,k) + dt*ubar(1,k)];
    
    c(:,k) = xbar_next - A(:,:,k)*xbar(:,k) - B(:,:,k)*ubar(:,k);
end

end

