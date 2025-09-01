function [A, B, c] = unified_linearize_batch(xbar, ubar, dt, state_type)
% 统一的批量线性化函数，支持MPC和MPCC
% 输入:
%   xbar - 名义状态序列
%   ubar - 名义控制序列
%   dt - 时间步长
%   state_type - 状态类型: 'mpc_theta', 'mpc_cos_sin', 'mpcc'
% 输出:
%   A - 状态矩阵序列
%   B - 输入矩阵序列
%   c - 仿射项序列

N = size(ubar, 2);
switch state_type
    case 'mpc_theta'
        % 4D状态: [x; y; theta; v], 2D控制: [a; omega]
        A = zeros(4, 4, N);
        B = zeros(4, 2, N);
        c = zeros(4, N);
        
        for k = 1:N
            th = xbar(3, k);
            v  = xbar(4, k);
            
            % 线性化矩阵
            A(:,:,k) = [ ...
                1, 0, -dt*v*sin(th), dt*cos(th);
                0, 1,  dt*v*cos(th), dt*sin(th);
                0, 0,  1,             0;
                0, 0,  0,             1];
            
            B(:,:,k) = [ ...
                0,            0;
                0,            0;
                0,            dt;
                dt,           0];
            
            % 仿射校正项
            xbar_next = [ ...
                xbar(1,k) + dt*xbar(4,k)*cos(xbar(3,k)); 
                xbar(2,k) + dt*xbar(4,k)*sin(xbar(3,k));
                wrapToPi(xbar(3,k) + dt*ubar(2,k));
                xbar(4,k) + dt*ubar(1,k)];
            
            c(:,k) = xbar_next - A(:,:,k)*xbar(:,k) - B(:,:,k)*ubar(:,k);
        end
        
    case 'mpc_cos_sin'
        % 5D状态: [x; y; cos(theta); sin(theta); v], 2D控制: [a; omega]
        A = zeros(5, 5, N);
        B = zeros(5, 2, N);
        c = zeros(5, N);
        
        for k = 1:N
            c_ = xbar(3, k);
            s_ = xbar(4, k);
            v  = xbar(5, k);
            w  = ubar(2, k);
            
            % 线性化矩阵
            A(:,:,k) = [ ...
                1, 0, v*dt, 0, c_*dt;
                0, 1, 0, v*dt, s_*dt;
                0, 0, 1, -w*dt, 0;
                0, 0, w*dt, 1, 0;
                0, 0, 0, 0, 1];
            
            B(:,:,k) = [ ...
                0, 0;
                0, 0;
                0, -s_*dt;
                0, c_*dt;
                dt, 0];
            
            % 仿射校正项
            xbar_next = [ ...
                xbar(1,k) + dt*xbar(5,k)*xbar(3,k); 
                xbar(2,k) + dt*xbar(5,k)*xbar(4,k);
                xbar(3,k) - dt*ubar(2,k)*xbar(4,k);
                xbar(4,k) + dt*ubar(2,k)*xbar(3,k);
                xbar(5,k) + dt*ubar(1,k)];
            
            c(:,k) = xbar_next - A(:,:,k)*xbar(:,k) - B(:,:,k)*ubar(:,k);
        end
        
    case 'mpcc'
        % 5D状态: [x; y; theta; v; progress], 3D控制: [a; omega; v_progress]
        A = zeros(5, 5, N);
        B = zeros(5, 3, N);
        c = zeros(5, N);
        
        for k = 1:N
            th = xbar(3, k);
            v  = xbar(4, k);
            
            % 线性化矩阵
            A(:,:,k) = [ ...
                1, 0, -dt*v*sin(th), dt*cos(th), 0;
                0, 1,  dt*v*cos(th), dt*sin(th), 0;
                0, 0,  1,             0,         0;
                0, 0,  0,             1,         0;
                0, 0,  0,             0,         1];
            
            B(:,:,k) = [ ...
                0,            0,            0;
                0,            0,            0;
                0,            dt,           0;
                dt,           0,            0;
                0,            0,            dt];
            
            % 仿射校正项
            xbar_next = [ ...
                xbar(1,k) + dt*xbar(4,k)*cos(xbar(3,k)); 
                xbar(2,k) + dt*xbar(4,k)*sin(xbar(3,k));
                wrapToPi(xbar(3,k) + dt*ubar(2,k));
                xbar(4,k) + dt*ubar(1,k);
                xbar(5,k) + dt*ubar(3,k)];
            
            c(:,k) = xbar_next - A(:,:,k)*xbar(:,k) - B(:,:,k)*ubar(:,k);
        end
        
    otherwise
        error('Unknown state_type: %s', state_type);
end

end

