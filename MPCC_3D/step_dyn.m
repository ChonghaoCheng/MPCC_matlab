function x_next = step_dyn(x, u, dt, v_max)
% STEP_DYN 3D点质量模型的动力学更新
% 输入:
%   x - 当前状态 [px,py,pz,yaw,pitch,v]
%   u - 控制输入 [a, w_yaw, w_pitch]
%   dt - 时间步长 [s]
%   v_max - 最大速度 [m/s]
% 输出:
%   x_next - 下一时刻状态

x_next = [ ...
    x(1) + dt*x(6)*cos(x(5))*cos(x(4));  % px+
    x(2) + dt*x(6)*cos(x(5))*sin(x(4));  % py+
    x(3) + dt*x(6)*sin(x(5));            % pz+
    wrapToPi(x(4) + dt*u(2));            % yaw+
    clamp_pitch(x(5) + dt*u(3));         % pitch+
    min(max(x(6) + dt*u(1), 0), v_max)]; % v+ (>=0)

end
