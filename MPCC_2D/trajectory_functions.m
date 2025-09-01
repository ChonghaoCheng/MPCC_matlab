function [r, r1, r2, S_end, R] = trajectory_functions(traj_type)
% TRAJECTORY_FUNCTIONS 定义不同的轨迹类型
%
% 输入:
%   traj_type: 轨迹类型 ('line' | 'circle' | 'sine')
%
% 输出:
%   r: 位置函数 r(s) = [x(s); y(s)]
%   r1: 一阶导数函数 r'(s)
%   r2: 二阶导数函数 r''(s)
%   S_end: 轨迹总长度
%   R: 圆形轨迹的半径（仅圆形轨迹有效）

switch traj_type
    case 'line'
        % 从(0,0)到(L,0)的直线
        L = 30;
        S_end = L;
        R = NaN; % 直线没有半径
        
        r   = @(s)[ s; 0*s ];
        r1  = @(s)[ 1; 0 ] + 0*s;   % first derivative wrt s
        r2  = @(s)[ 0; 0 ] + 0*s;   % second derivative
        
    case 'circle'
        % 半径为R的圆，一圈
        R = 8; 
        S_end = 2*pi*R;
        
        % arc-length s => angle phi = s/R
        r   = @(s)[ R*cos(s/R); R*sin(s/R) ];
        r1  = @(s)[ -sin(s/R);  cos(s/R) ];      % dr/ds (unit tangent)
        r2  = @(s)[ -(1/R)*cos(s/R); -(1/R)*sin(s/R) ]; % d^2r/ds^2
        
    case 'sine'
        % x方向的正弦波 y = A*sin(k*x)
        A = 2; k = 0.3; L = 40; 
        S_end = L;
        R = NaN; % 正弦波没有半径
        
        % 定义 x=s, y=A*sin(k*s)
        r   = @(s)[ s; A*sin(k*s) ];
        r1  = @(s)[ 1; A*k*cos(k*s) ];      % dr/ds (not unit length!)
        r2  = @(s)[ 0; -A*k^2*sin(k*s) ];
        
    otherwise
        error('Unknown traj_type: %s. Use ''line'', ''circle'', or ''sine''', traj_type);
end

end

