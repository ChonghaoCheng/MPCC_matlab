function [xref, uref] = sample_ref_path3D_time(t, N, dt, r, r1, r2, v_ref_of_s, geom3_at)
% SAMPLE_REF_PATH3D_TIME 根据时间采样参考路径（MPC用）
% 输入:
%   t - 当前时间 [s]
%   N - 预测步数
%   dt - 时间步长 [s]
%   r, r1, r2 - 路径函数
%   v_ref_of_s - 速度函数
%   geom3_at - 几何计算函数
% 输出:
%   xref - 参考状态序列 [nx, N+1]
%   uref - 前馈控制序列 [nu, N]

nx = 6; nu = 3;
xref = zeros(nx, N+1);
uref = zeros(nu, N);

% 基于时间计算参考轨迹
for k = 1:N+1
    t_k = t + (k-1)*dt;
    
    % 根据时间估计弧长（假设匀速运动）
    s_est = v_ref_of_s(0) * t_k;  % 简单估计
    
    % 获取几何信息
    [t_vec, n_vec, b_vec, kappa, yaw, pitch] = geom3_at(s_est, r, r1, r2);
    p = r(s_est);
    vref = v_ref_of_s(s_est);
    
    xref(:,k) = [p(1); p(2); p(3); yaw; pitch; vref];
    
    if k <= N
        % 前馈控制：保持零加速度，小的角速度
        uref(:,k) = [0; 0; 0];
    end
end

end



