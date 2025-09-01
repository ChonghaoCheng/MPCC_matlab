function [Xn, A, B, c] = unified_dynamics(X, U, dt, v_max, state_type, compute_linearization)
% 统一的动力学函数，支持MPC和MPCC的不同状态表示
% 输入:
%   X - 状态向量
%   U - 控制输入
%   dt - 时间步长
%   v_max - 最大速度
%   state_type - 状态类型: 'mpc_theta', 'mpc_cos_sin', 'mpcc'
%   compute_linearization - 是否计算线性化 (可选，默认false)
% 输出:
%   Xn - 下一时刻状态
%   A - 状态矩阵 (如果compute_linearization=true)
%   B - 输入矩阵 (如果compute_linearization=true)
%   c - 常数项 (如果compute_linearization=true)

if nargin < 6
    compute_linearization = false;
end

% 根据状态类型确定状态维度和控制维度
switch state_type
    case 'mpc_theta'
        state_dim = 4;
        control_dim = 2;
        % 状态: [x; y; theta; v]
        x = X(1); y = X(2); th = X(3); v = X(4);
        a = U(1); w = U(2);
        
        % 非线性离散动力学
        Xn = [ x + v*cos(th)*dt;
               y + v*sin(th)*dt;
               wrapToPi(th + w*dt);
               min(max(v + a*dt, 0), v_max) ];
        
    case 'mpc_cos_sin'
        state_dim = 5;
        control_dim = 2;
        % 状态: [x; y; cos(theta); sin(theta); v]
        x = X(1); y = X(2); c = X(3); s = X(4); v = X(5);
        a = U(1); w = U(2);
        
        % 非线性离散动力学
        Xn = [ x + v*c*dt;
               y + v*s*dt;
               c - s*w*dt;
               s + c*w*dt;
               min(max(v + a*dt, 0), v_max) ];
        
        % 归一化cos和sin (保持单位长度)
        rn = hypot(Xn(3), Xn(4));
        if rn > 0
            Xn(3) = Xn(3) / rn;
            Xn(4) = Xn(4) / rn;
        end
        
    case 'mpcc'
        state_dim = 5;
        control_dim = 3;
        % 状态: [x; y; theta; v; progress]
        x = X(1); y = X(2); th = X(3); v = X(4); s = X(5);
        a = U(1); w = U(2); v_progress = U(3);
        
        % 非线性离散动力学
        Xn = [ x + v*cos(th)*dt;
               y + v*sin(th)*dt;
               wrapToPi(th + w*dt);
               min(max(v + a*dt, 0), v_max);
               s + v_progress*dt ];
        
    otherwise
        error('Unknown state_type: %s', state_type);
end

% 如果需要线性化
if compute_linearization
    [A, B, c] = compute_linearization_matrices(X, U, dt, state_type, Xn);
else
    A = []; B = []; c = [];
end

end

function [A, B, c] = compute_linearization_matrices(X, U, dt, state_type, Xn)
% 计算线性化矩阵的辅助函数

switch state_type
    case 'mpc_theta'
        % 4D状态: [x; y; theta; v]
        th = X(3); v = X(4);
        
        % 状态矩阵 A
        A = eye(4);
        A(1,3) = -dt*v*sin(th); A(1,4) = dt*cos(th);
        A(2,3) =  dt*v*cos(th); A(2,4) = dt*sin(th);
        
        % 输入矩阵 B
        B = zeros(4,2);
        B(3,2) = dt;  % dtheta/domega
        B(4,1) = dt;  % dv/da
        
    case 'mpc_cos_sin'
        % 5D状态: [x; y; cos(theta); sin(theta); v]
        c = X(3); s = X(4); v = X(5);
        w = U(2);
        
        % 状态矩阵 A
        A = eye(5);
        A(1,3) = v*dt; A(1,5) = c*dt;
        A(2,4) = v*dt; A(2,5) = s*dt;
        A(3,4) = -w*dt;
        A(4,3) =  w*dt;
        
        % 输入矩阵 B
        B = zeros(5,2);
        B(3,2) = -s*dt;
        B(4,2) =  c*dt;
        B(5,1) =  dt;
        
    case 'mpcc'
        % 5D状态: [x; y; theta; v; progress]
        th = X(3); v = X(4);
        
        % 状态矩阵 A
        A = eye(5);
        A(1,3) = -dt*v*sin(th); A(1,4) = dt*cos(th);
        A(2,3) =  dt*v*cos(th); A(2,4) = dt*sin(th);
        
        % 输入矩阵 B
        B = zeros(5,3);
        B(3,2) = dt;  % dtheta/domega
        B(4,1) = dt;  % dv/da
        B(5,3) = dt;  % dprogress/dv_progress
end

% 常数项 c (使用精确的离散化)
c = Xn - A*X - B*U;

end

