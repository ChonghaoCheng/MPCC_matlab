function [u_opt, xpred] = solve_tracking_mpc_3d(x0, N, xref, uref, Q, R, Rdu, u_last, dt, ...
                                                a_max, w_yaw_max, w_pitch_max, v_max)
% SOLVE_TRACKING_MPC_3D 求解3D跟踪MPC问题
% 输入:
%   x0 - 初始状态 [nx,1]
%   N - 预测步数
%   xref - 参考状态序列 [nx, N+1]
%   uref - 参考控制序列 [nu, N]
%   Q, R, Rdu - 权重矩阵
%   u_last - 上一时刻控制输入
%   dt - 时间步长
%   a_max, w_yaw_max, w_pitch_max - 控制约束
%   v_max - 最大速度
% 输出:
%   u_opt - 最优控制输入 [nu,1]
%   xpred - 预测状态序列 [nx, N+1]

nx = size(xref,1); nu = size(uref,1);

% 线性化
[A,B,c] = linearize_batch3D(xref(:,1:end-1), uref, dt);
[Ax,Bu,d] = condensed_dynamics(A,B,c);

xref_stack = reshape(xref(:,2:end), nx*N, 1);
Qb = kron(eye(N), Q);
Rb = kron(eye(N), R);

% delta-u块差分
D = kron(eye(N), eye(nu)) - kron(diag(ones(N-1,1),-1), eye(nu));
RduN = kron(eye(N), Rdu);

H = (Bu.'*Qb*Bu) + Rb + (D.'*RduN*D);
f = Bu.'*Qb*(Ax*x0 + d - xref_stack);

% delta-u偏移（考虑上一时刻控制）
u_offset = zeros(nu*N,1); 
u_offset(1:nu) = u_last;
f = f - (D.'*RduN*D)*u_offset;

% 边界约束
umin = repmat([-a_max; -w_yaw_max; -w_pitch_max], N,1);
umax = repmat([ a_max;  w_yaw_max;  w_pitch_max], N,1);

% 求解QP
opts = optimoptions('quadprog','Display','off');
u_seq = quadprog((H+H.')/2, f, [],[], [],[], umin, umax, [], opts);

if isempty(u_seq)
    u_seq = zeros(nu*N,1); 
end

u_opt = u_seq(1:nu);

% 预测展开（用于显示）
xk = x0; 
xpred = zeros(nx,N+1); 
xpred(:,1) = xk;

for k=1:N
    uk = u_seq((k-1)*nu+(1:nu));
    xk = [ ...
        xk(1) + dt*xk(6)*cos(xk(5))*cos(xk(4));
        xk(2) + dt*xk(6)*cos(xk(5))*sin(xk(4));
        xk(3) + dt*xk(6)*sin(xk(5));
        wrapToPi(xk(4) + dt*uk(2));
        clamp_pitch(xk(5) + dt*uk(3));
        min(max(xk(6) + dt*uk(1),0), v_max)];
    xpred(:,k+1) = xk;
end

end

