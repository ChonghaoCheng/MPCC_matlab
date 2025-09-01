function [u_opt, xpred] = solve_tracking_mpc(x0, N, xref, uref, Q, R, Rdu, u_last, dt, a_max, w_max, v_max)
% SOLVE_TRACKING_MPC 求解跟踪MPC问题
%
% 输入:
%   x0: 当前状态 [x; y; cos(theta); sin(theta); v] - 输入状态
%   N: 预测时域
%   xref: 参考状态序列 [x; y; cos(theta); sin(theta); v]
%   uref: 参考控制序列 [a; omega]
%   Q, R, Rdu: 权重矩阵
%   u_last: 上一步控制
%   dt: 时间步长
%   a_max, w_max, v_max: 约束
%
% 输出:
%   u_opt: 最优控制输入
%   xpred: 预测轨迹

    nx=4; nu=2;  % 内部使用4维状态: [x,y,theta,v]

    % 将输入状态转换为内部状态格式
    x0_internal = [x0(1); x0(2); atan2(x0(4), x0(3)); x0(5)];
    
    % 将参考状态转换为内部格式
    xref_internal = zeros(4, N+1);
    for k = 1:N+1
        c_th = xref(3, k);
        s_th = xref(4, k);
        xref_internal(:,k) = [xref(1,k); xref(2,k); atan2(s_th, c_th); xref(5,k)];
    end
    
    % 线性化
    [A,B,c] = unified_linearize_batch(xref_internal(:,1:end-1), uref, dt, 'mpc_theta');

    % Build condensed dynamics X = Ax*x0 + Bu*U + d
    Ax = zeros(nx*N, nx);   Bu = zeros(nx*N, nu*N);   d = zeros(nx*N,1);
    for k=1:N
        Ak = eye(nx);
        for j=1:k, Ak = A(:,:,j)*Ak; end
        Ax((k-1)*nx+(1:nx),:) = Ak;
        for j=1:k
            Phi = eye(nx);
            for m=j+1:k, Phi = A(:,:,m)*Phi; end
            Bu((k-1)*nx+(1:nx),(j-1)*nu+(1:nu)) = Phi*B(:,:,j);
        end
        cj = zeros(nx,1);
        for j=1:k
            Phi = eye(nx);
            for m=j+1:k, Phi = A(:,:,m)*Phi; end
            cj = cj + Phi*c(:,j);
        end
        d((k-1)*nx+(1:nx)) = cj;
    end

    % 调整权重矩阵以匹配内部状态维度
    Q_internal = diag([Q(1,1), Q(2,2), 8, Q(5,5)]);  % [x,y,theta,v] 权重
    Qb  = kron(eye(N), Q_internal);
    Rb  = kron(eye(N), R);
    D   = kron(eye(N), eye(nu)) - kron(diag(ones(N-1,1),-1), eye(nu));
    RduN= kron(eye(N), Rdu);

    H = (Bu.'*Qb*Bu) + Rb + (D.'*RduN*D);

    % Linear term
    xref_stack = reshape(xref_internal(:,2:end), nx*N, 1);
    X0 = Ax*x0_internal + d;
    e  = X0 - xref_stack;

    f = Bu.'*Qb*e;

    % delta-u offset around last control
    u_offset = zeros(nu*N,1); u_offset(1:nu) = u_last;
    f = f - (D.'*RduN*D)*u_offset;

    % bounds
    umin = repmat([-a_max; -w_max], N,1);
    umax = repmat([ a_max;  w_max], N,1);

    % solve QP
    opts = optimoptions('quadprog','Display','off');
    u_seq = quadprog( (H+H.')/2, f, [],[], [],[], umin, umax, [], opts);
    if isempty(u_seq), u_seq = zeros(nu*N,1); end
    u_opt = u_seq(1:nu);

    % rollout for visualization
    xk = x0_internal; xpred = zeros(4,N+1); xpred(:,1)=xk;
    for k=1:N
        uk = u_seq((k-1)*nu+(1:nu));
        % 使用unified_dynamics进行状态预测
        xk = unified_dynamics(xk, uk, dt, v_max, 'mpc_theta', false);
        xpred(:,k+1) = xk;
    end
    
    % 将预测结果转换回输出格式
    xpred_output = zeros(5, N+1);
    for k = 1:N+1
        xpred_output(:,k) = [xpred(1,k); xpred(2,k); cos(xpred(3,k)); sin(xpred(3,k)); xpred(4,k)];
    end
    xpred = xpred_output;

end

