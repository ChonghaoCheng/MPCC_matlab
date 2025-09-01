function [u_opt, ds_opt, xpred] = solve_mpcc_3d(x0, s0, N, dt, r, r1, r2, v_ref_of_s, ...
    Wc, Wl, Wv, Wu, Wds, Wdu, u_last, v_max, a_max, w_yaw_max, w_pitch_max, geom3_at)
% SOLVE_MPCC_3D 求解3D MPCC问题
% 输入:
%   x0 - 初始状态 [nx,1]
%   s0 - 初始弧长
%   N - 预测步数
%   dt - 时间步长
%   r, r1, r2 - 路径函数
%   v_ref_of_s - 速度函数
%   Wc, Wl, Wv, Wu, Wds, Wdu - 权重参数
%   u_last - 上一时刻控制输入
%   v_max, a_max, w_yaw_max, w_pitch_max - 约束参数
%   geom3_at - 几何计算函数
% 输出:
%   u_opt - 最优控制输入 [nu,1]
%   ds_opt - 最优弧长增量 [1,1]
%   xpred - 预测状态序列 [nx, N+1]

nx = 6; nu = 3;

% ds 初猜：按速度
ds_guess = max(0.5, min(v_ref_of_s(s0), v_max))*dt; % [m]
svec = s0 + (0:N)*ds_guess;

% 名义轨迹（仅用于线性化）
xbar = zeros(nx, N+1); xbar(:,1) = x0;
ubar = zeros(nu, N);
for k = 1:N
    % 简单FF保持0，加速度/角速率全0
    ubar(:,k) = [0;0;0];
    xbar(:,k+1) = [ ...
        xbar(1,k) + dt*xbar(6,k)*cos(xbar(5,k))*cos(xbar(4,k));
        xbar(2,k) + dt*xbar(6,k)*cos(xbar(5,k))*sin(xbar(4,k));
        xbar(3,k) + dt*xbar(6,k)*sin(xbar(5,k));
        wrapToPi(xbar(4,k) + dt*ubar(2,k));
        xbar(5,k) + dt*ubar(3,k);
        min(max(xbar(6,k) + dt*ubar(1,k),0), v_max)];
end

% 线性化
[A,B,c] = linearize_batch3D(xbar(:,1:end-1), ubar, dt);
[Ax,Bu,d] = condensed_dynamics(A,B,c);

% MPCC 的参考（用于代价）：按路径 Frenet+速度
xref_mpcc = zeros(nx, N+1);
for k = 1:N+1
    sk = s0 + (k-1)*ds_guess;
    [t,n,b,kappa,yaw,pitch] = geom3_at(sk, r, r1, r2);
    p   = r(sk);
    vref = v_ref_of_s(sk);
    xref_mpcc(:,k) = [p(1); p(2); p(3); yaw; pitch; vref];
end
xref_stack = reshape(xref_mpcc(:,2:end), nx*N, 1);

% 状态代价：仅速度项（姿态/位置用 contour/lag 处理）
Qp = zeros(nx,nx); Qp(6,6) = Wv;   % penalize (v - v_ref)^2
Qb = kron(eye(N), Qp);

% 输入代价 + 平滑 du
D = kron(eye(N), eye(nu)) - kron(diag(ones(N-1,1),-1), eye(nu));
Ru = kron(eye(N), Wu) + (D.'*(Wdu*eye(nu*N))*D);

% 初始二次项（来自速度跟踪）
H_uu = Bu.'*Qb*Bu + Ru;
f_u  = Bu.'*Qb*(Ax*x0 + d - xref_stack);

% 位置投影矩阵：取 position 分量
Cpos = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0];

% 叠加 contour/lag 项
for k=1:N
    idxX = (k-1)*nx+(1:nx);
    Mx = Ax(idxX,:);   Mu = Bu(idxX,:);
    sk = s0 + (k-1)*ds_guess;

    [t_k, n_k, b_k, ~, ~, ~] = geom3_at(sk, r, r1, r2);
    p_ref = r(sk);

    % contouring: project on n and b
    % ec_n = n^T (p - p_ref), ec_b = b^T (p - p_ref)
    Jn_u = (n_k.'*Cpos)*Mu;  jn0 = (n_k.'*Cpos)*(Mx*x0 + d(idxX)) - n_k.'*p_ref;
    Jb_u = (b_k.'*Cpos)*Mu;  jb0 = (b_k.'*Cpos)*(Mx*x0 + d(idxX)) - b_k.'*p_ref;

    % lag: along tangent
    Jl_u = (t_k.'*Cpos)*Mu;  jl0 = (t_k.'*Cpos)*(Mx*x0 + d(idxX)) - t_k.'*p_ref;

    H_uu = H_uu + Wc*(Jn_u.'*Jn_u + Jb_u.'*Jb_u) + Wl*(Jl_u.'*Jl_u);
    f_u  = f_u  + Wc*(Jn_u.'*jn0  + Jb_u.'*jb0)   + Wl*(Jl_u.'*jl0);
end

% 变量 z = [U; ds]，仅对 ds 加二次项 + 线性项(ds - v_ref*dt)
H = blkdiag((H_uu+H_uu.')/2, Wds*N);
vref_now = v_ref_of_s(s0);
f = [f_u; -Wds*N*(vref_now*dt)];

% 边界
umin = repmat([-a_max; -w_yaw_max; -w_pitch_max], N,1);
umax = repmat([ a_max;  w_yaw_max;  w_pitch_max], N,1);
ds_min = 0.0; ds_max = v_max*dt;

lb = [umin; ds_min]; ub = [umax; ds_max];

opts = optimoptions('quadprog','Display','off');
z = quadprog(H, f, [],[], [],[], lb,ub, [], opts);

if isempty(z)
    u_opt = [0;0;0]; ds_opt = ds_guess;
else
    u_seq = z(1:nu*N);
    ds_opt = z(end);
    u_opt = u_seq(1:nu);
end

% 预测展开
xk = x0; xpred = zeros(nx,N+1); xpred(:,1) = xk;
for k=1:N
    uk = z((k-1)*nu+(1:nu));
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



