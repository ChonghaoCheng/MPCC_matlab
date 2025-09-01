function [u_opt, ds_opt, xpred] = solve_mpcc(x0, s0, N, dt, r, r1, r2, v_ref_nom, ...
    Wc, Wl, Wv, Wu, Wds, Wdu, u_last, v_max, a_max, w_max, v_progress_max)
% SOLVE_MPCC 求解MPCC问题
%
% 输入:
%   x0: 当前状态 [x; y; theta; v; progress]
%   s0: 当前弧长
%   N: 预测时域
%   dt: 时间步长
%   r, r1, r2: 轨迹函数
%   v_ref_nom: 标称参考速度
%   Wc, Wl, Wv, Wu, Wds, Wdu: 权重
%   u_last: 上一步控制 [a; omega; v_progress]
%   v_max, a_max, w_max: 约束
%
% 输出:
%   u_opt: 最优控制输入 [a; omega; v_progress]
%   ds_opt: 最优弧长增量
%   xpred: 预测轨迹

nx = 5; nu = 3;  % 新状态维度: [x,y,theta,v,progress], 新控制维度: [a,omega,v_progress]

% 选择每步的ds并沿s采样
ds_guess = max(0.4 * v_ref_nom * dt, min(v_ref_nom, v_max) * dt);
svec = s0 + (0:N)*ds_guess;

% 名义轨迹：使用零控制推出并跟随路径航向
xbar = zeros(nx, N+1); 
xbar(:,1) = x0;
ubar = zeros(nu, N);

for k = 1:N
    [t, n, kappa, psi] = geom_at(svec(k), r, r1, r2);
    vref = v_ref_nom;
    % 前馈控制：加速度=0，角速度=曲率*速度，进度速度=参考速度
    ubar(:,k) = [0; kappa*vref; vref]; 
    
    % 使用unified_dynamics进行状态预测
    xbar(:,k+1) = unified_dynamics(xbar(:,k), ubar(:,k), dt, v_max, 'mpcc', false);
end

% 参考状态
xref_mpcc = zeros(nx, N+1);
for k = 1:N+1
    sk = s0 + (k-1)*ds_guess;
    [t_k, ~, ~, psi_k] = geom_at(sk, r, r1, r2);
    pk = r(sk);
    vref = v_ref_nom;
    xref_mpcc(:,k) = [pk(1); pk(2); psi_k; vref; sk];  % 参考progress为弧长s
end

% 添加progress跟踪权重
Qp = zeros(nx, nx);
Qp(4,4) = Wv;  % 速度跟踪权重
Qp(5,5) = 0.05; % progress跟踪权重（较小，避免过度约束，允许灵活推进）
Qb = kron(eye(N), Qp);

xref_stack = reshape(xref_mpcc(:,2:end), nx*N, 1);

% 沿名义轨迹线性化
[A, B, c] = unified_linearize_batch(xbar(:,1:end-1), ubar, dt, 'mpcc');

% 构建压缩动力学：X = Ax*x0 + Bu*U + d
Ax = zeros(nx*N, nx);
Bu = zeros(nx*N, nu*N);
d = zeros(nx*N, 1);

for k = 1:N
    Ak1 = eye(nx);
    for j = 1:k
        Ak1 = A(:,:,j)*Ak1;
    end
    Ax((k-1)*nx+(1:nx),:) = Ak1;
    
    for j = 1:k
        Phi = eye(nx);
        for m = j+1:k
            Phi = A(:,:,m)*Phi;
        end
        Bu((k-1)*nx+(1:nx),(j-1)*nu+(1:nu)) = Phi*B(:,:,j);
    end
    
    cj = zeros(nx,1);
    for j = 1:k
        Phi = eye(nx);
        for m = j+1:k
            Phi = A(:,:,m)*Phi;
        end
        cj = cj + Phi*c(:,j);
    end
    d((k-1)*nx+(1:nx)) = cj;
end

% 构建轮廓/滞后代价线性化
H = zeros(nu*N); 
f = zeros(nu*N, 1);

% 增量控制项
D = kron(eye(N), eye(nu)) - kron(diag(ones(N-1,1),-1), eye(nu));
Ru = kron(eye(N), Wu) + (D.'*(Wdu*eye(nu*N))*D);

% 位置选择矩阵
Cpos = [1 0 0 0 0; 0 1 0 0 0];

% 进度ds决策 - 建模为地平线内共享的标量
H_uu = Bu.'*Qb*Bu + Ru;
f_u = Bu.'*Qb*(Ax*x0 + d - xref_stack);

% 添加轮廓/滞后项
for k = 1:N
    idxX = (k-1)*nx+(1:nx);
    Mx = Ax(idxX,:);
    Mu = Bu(idxX,:);
    
    % 位置残差相对于路径帧
    sk = s0 + (k-1)*ds_guess;
    [t_k, n_k, ~, ~] = geom_at(sk, r, r1, r2);
    p_ref = r(sk);
    
    % ec = n^T (p - p_ref), el = t^T (p - p_ref)
    Jc_u = (n_k.'*Cpos)*Mu;
    jc0 = (n_k.'*Cpos)*(Mx*x0 + d(idxX)) - n_k.'*p_ref;
    
    Jl_u = (t_k.'*Cpos)*Mu;
    jl0 = (t_k.'*Cpos)*(Mx*x0 + d(idxX)) - t_k.'*p_ref;
    
    % 添加到H,f
    H_uu = H_uu + Wc*(Jc_u.'*Jc_u) + Wl*(Jl_u.'*Jl_u);
    f_u = f_u + Wc*(Jc_u.'*jc0) + Wl*(Jl_u.'*jl0);
end

% 组装增广Hessian [U; ds]
H = blkdiag((H_uu+H_uu.')/2, Wds*N);
vref_now = v_ref_nom;
f = [f_u; -Wds*N*(vref_now*dt)];

% U和ds的约束
umin = repmat([-a_max; -w_max; 0], N, 1);  % v_progress >= 0
umax = repmat([a_max; w_max; v_progress_max], N, 1);  % 添加v_progress_max约束
ds_min = 0.2 * v_ref_nom * dt;  % 最小弧长增量（20%的标称速度）
ds_max = v_max * dt;            % 最大弧长增量

% 求解QP
Aineq = []; bineq = []; Aeq = []; beq = [];
lb = [umin; ds_min]; 
ub = [umax; ds_max];

opts = optimoptions('quadprog', 'Display', 'off');
z = quadprog(H, f, Aineq, bineq, Aeq, beq, lb, ub, [], opts);

if isempty(z)
    u_opt = [0; 0; 0]; 
    ds_opt = ds_guess;
else
    u_seq = z(1:nu*N);
    ds_opt = z(end);
    u_opt = u_seq(1:nu);
end

% 预测轨迹（用于显示）
xpred = zeros(nx, N+1);
xpred(:,1) = x0;

for k = 1:N
    uk = z((k-1)*nu+(1:nu));
    xpred(:,k+1) = unified_dynamics(xpred(:,k), uk, dt, v_max, 'mpcc', false);
end

end

