function [t,n,b,kappa,yaw,pitch,rt,rt2] = geom3_at(s, r, r1, r2)
% GEOM3_AT 计算3D路径在弧长s处的Frenet三标架和几何量
% 输入:
%   s - 弧长参数
%   r - 位置函数 r(s)
%   r1 - 一阶导数函数 r'(s)
%   r2 - 二阶导数函数 r''(s)
% 输出:
%   t - 单位切向量
%   n - 单位法向量
%   b - 单位副法向量
%   kappa - 曲率
%   yaw - 期望航向角 [rad]
%   pitch - 期望俯仰角 [rad]
%   rt - 切向量（未归一化）
%   rt2 - 二阶切向量

rt  = r1(s);
rt2 = r2(s);
L1  = norm(rt);

if L1 < 1e-9
    t = [1;0;0]; 
else
    t = rt/L1; 
end

% normal from derivative: n ~ (d t / ds) normalized
dt_ds = (rt2*L1 - rt*(rt.'*rt2)/L1)/(L1^2 + eps);
Ln = norm(dt_ds);

if Ln < 1e-9
    % if curvature ~ 0, choose arbitrary n perpendicular to t
    tmp = [0;0;1]; 
    if abs(dot(tmp,t))>0.9
        tmp=[0;1;0]; 
    end
    n = (tmp - dot(tmp,t)*t); 
    n = n/(norm(n)+eps);
else
    n = dt_ds/Ln;
end

% binormal
b = cross(t,n); 
b = b/(norm(b)+eps);

% curvature
kappa = Ln; % in arc-length parameterization

% desired yaw, pitch from t (heading defined by t-direction)
% yaw = atan2(ty, tx), pitch = atan2(tz, sqrt(tx^2+ty^2))
yaw   = atan2(t(2), t(1));
pitch = atan2(t(3), sqrt(t(1)^2 + t(2)^2));

end

