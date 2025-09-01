function [t, n, kappa, psi, rt, rt2] = geom_at(s, r, r1, r2)
% GEOM_AT 计算路径在给定弧长s处的几何信息
%
% 输入:
%   s: 弧长参数
%   r: 位置函数 r(s)
%   r1: 一阶导数函数 r'(s)
%   r2: 二阶导数函数 r''(s)
%
% 输出:
%   t: 单位切向量（前进方向）
%   n: 单位法向量（垂直于前进方向）
%   kappa: 曲率
%   psi: 航向角
%   rt: 一阶导数 r'(s)
%   rt2: 二阶导数 r''(s)

% 计算导数
rt  = r1(s);
rt2 = r2(s);
L1  = norm(rt);

% 处理奇异情况
if L1 < 1e-9
    t = [1; 0];
else
    t = rt / L1;  % 单位切向量
end

% 法向量：将切向量顺时针旋转90度
n = [t(2); -t(1)];

% 曲率：kappa = (x'y'' - y'x'') / |r'|^3
kappa = (rt(1)*rt2(2) - rt(2)*rt2(1)) / (max(L1, 1e-9)^3);

% 航向角
psi = atan2(t(2), t(1));

end

function [xref, uref, svec] = sample_ref_path(s0, N, ds, r, r1, r2, v_ref_of_s)
% SAMPLE_REF_PATH 沿路径从s0起采样参考轨迹
%
% 输入:
%   s0: 起始弧长
%   N: 预测时域步数
%   ds: 弧长步长
%   r, r1, r2: 轨迹函数
%   v_ref_of_s: 参考速度函数
%
% 输出:
%   xref: 参考状态序列 [4 x (N+1)]
%   uref: 参考控制序列 [2 x N]
%   svec: 弧长序列 [1 x (N+1)]

svec = s0 + (0:N)*ds;
xref = zeros(4, N+1);
uref = zeros(2, N);

% 先计算所有参考点的角度
psi_raw = zeros(1, N+1);
for k = 1:N+1
    [t, n, kappa, psi] = geom_at(svec(k), r, r1, r2);
    psi_raw(k) = psi;
end

% 角度连续化：确保相邻角度差不超过 π
psi_continuous = psi_raw;
for k = 2:N+1
    angle_diff = psi_raw(k) - psi_continuous(k-1);
    % 如果角度差超过 π，说明有跳变，需要调整
    while abs(angle_diff) > pi
        if angle_diff > 0
            psi_continuous(k) = psi_continuous(k) - 2*pi;
        else
            psi_continuous(k) = psi_continuous(k) + 2*pi;
        end
        angle_diff = psi_continuous(k) - psi_continuous(k-1);
    end
end

% 使用连续化的角度构建参考轨迹
for k = 1:N+1
    [t, n, kappa, ~] = geom_at(svec(k), r, r1, r2);
    p = r(svec(k));
    vref = v_ref_of_s(svec(k));
    xref(:,k) = [p(1); p(2); psi_continuous(k); vref];
    
    if k <= N
        % 前馈近似：a_ff ≈ 0, omega_ff = kappa*v
        uref(:,k) = [0; kappa*vref];
    end
end

end
