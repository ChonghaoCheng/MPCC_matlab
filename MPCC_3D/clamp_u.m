function u_clamped = clamp_u(u, a_max, w_yaw_max, w_pitch_max)
% CLAMP_U 限制控制输入在允许范围内
% 输入:
%   u - 控制输入 [a, w_yaw, w_pitch]
%   a_max - 最大加速度 [m/s^2]
%   w_yaw_max - 最大偏航角速度 [rad/s]
%   w_pitch_max - 最大俯仰角速度 [rad/s]
% 输出:
%   u_clamped - 限制后的控制输入

u_clamped = [ ...
    max(min(u(1), a_max), -a_max); ...
    max(min(u(2), w_yaw_max), -w_yaw_max); ...
    max(min(u(3), w_pitch_max), -w_pitch_max)];

end

