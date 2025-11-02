function [t, n1, n2, kappa_t, p, rt, rt2] = geom3_at(s, r, r1, r2)
% GEOM3_AT 计算3D路径在弧长s处的几何信息
%
% 输入:
%   s:   弧长参数
%   r:   位置函数 r(s) -> R^3
%   r1:  一阶导数函数 r'(s) -> R^3
%   r2:  二阶导数函数 r''(s) -> R^3
%
% 输出:
%   t:       单位切向量 (3x1)，指向路径前进方向
%   n1, n2:  法平面的正交基 (3x1 each)，用于计算contouring error
%   kappa_t: 曲率大小 (标量)
%   p:       位置 r(s) (3x1)
%   rt:      r'(s) (3x1)
%   rt2:     r''(s) (3x1)
%
% 说明:
%   - 切向量 t = r'(s) / |r'(s)|，方向沿路径
%   - n1和n2构成法平面的正交基，满足 n1 ⊥ t, n2 ⊥ t, n2 = t × n1
%   - 曲率 kappa = |r' × r''| / |r'|^3

% 计算位置和导数
p   = r(s);
rt  = r1(s);
rt2 = r2(s);

% 切向量：单位化 r'(s)
L1 = norm(rt);
if L1 < 1e-12
    % 奇异情况：r'(s)为零，使用默认坐标系
    t = [1;0;0];
    n1 = [0;1;0];
    n2 = [0;0;1];
    kappa_t = 0;
    return;
end

t = rt / L1;  % 单位切向量

% 曲率：kappa = |r' × r''| / |r'|^3
cross_r = cross(rt, rt2);
kappa_t = norm(cross_r) / max(L1^3, 1e-12);

% 构造法平面的正交基
% n1: 取r''(s)在法平面上的投影作为第一个法向量
acc_perp = rt2 - (t' * rt2) * t;  % r''在法平面上的投影
if norm(acc_perp) > 1e-12
    n1 = acc_perp / norm(acc_perp);
else
    % 如果投影为零（直线），选择任意与t垂直的向量
    % 找到t的最大分量，选择不同的轴
    [~, idx] = max(abs(t));
    e = zeros(3,1); 
    e(mod(idx,3)+1) = 1;  % 选择不同的轴
    v = e - (t'*e)*t;     % 投影到法平面
    if norm(v) < 1e-12
        e = zeros(3,1); 
        e(mod(idx+1,3)+1) = 1;  % 再试另一个轴
        v = e - (t'*e)*t;
    end
    n1 = v / max(norm(v), 1e-12);
end

% n2: 通过叉积得到，形成右手坐标系 (t, n1, n2)
n2 = cross(t, n1);

% 归一化以确保数值稳定性
n1 = n1 / max(norm(n1), 1e-12);
n2 = n2 / max(norm(n2), 1e-12);

end








