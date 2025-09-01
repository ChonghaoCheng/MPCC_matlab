function [r, r1, r2, S_end, v_ref_of_s] = define_3d_trajectory(traj_type, v_ref_nom)
% DEFINE_3D_TRAJECTORY 定义3D轨迹
% 输入:
%   traj_type - 轨迹类型: 'line', 'helix', 'lissajous'
%   v_ref_nom - 标称期望速度 [m/s]
% 输出:
%   r - 位置函数 r(s) = [x(s); y(s); z(s)]
%   r1 - 一阶导数函数 r'(s)
%   r2 - 二阶导数函数 r''(s)
%   S_end - 轨迹总弧长 [m]
%   v_ref_of_s - 速度函数 v(s)

switch traj_type
    case 'line'
        % Straight line in 3D from (0,0,0) to (L,0,0)
        L = 40; S_end = L;
        r   = @(s)[ s; 0*s; 0*s ];
        r1  = @(s)[ 1; 0; 0 ] + 0*s;
        r2  = @(s)[ 0; 0; 0 ] + 0*s;

    case 'helix'
        % Helix: radius R, pitch H per revolution, angle phi = s/R
        R = 6; H = 1.5;  % pitch per radian along s/R -> z = (H/R)*s
        S_end = 3*2*pi*R;  % 3 turns
        r   = @(s)[ R*cos(s/R); R*sin(s/R); (H/R)*s ];
        r1  = @(s)[ -sin(s/R); cos(s/R); (H/R) ];
        r2  = @(s)[ -(1/R)*cos(s/R); -(1/R)*sin(s/R); 0 ];

    case 'lissajous'
        % 3D Lissajous-like curve parametrized by arclength ~ x=s, y=Asin(k s), z=B cos(m s)
        A=2; B=1.5; k=0.25; m=0.35; L=50; S_end=L;
        r   = @(s)[ s; A*sin(k*s); B*cos(m*s) ];
        r1  = @(s)[ 1; A*k*cos(k*s); -B*m*sin(m*s) ];
        r2  = @(s)[ 0; -A*k^2*sin(k*s); -B*m^2*cos(m*s) ];
    otherwise
        error('Unknown traj_type: %s. Use ''line'', ''helix'', or ''lissajous''', traj_type);
end

% desired speed profile vs (constant here)
v_ref_of_s = @(s) v_ref_nom;

end

