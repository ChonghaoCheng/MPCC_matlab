clear; clc; close all;

%% 仿真参数
traj_type = 'circle';     % 'line' | 'circle' | 'sine'
Tf        = 80.0;       % max sim time [s]
dt        = 0.05;        % step [s]
N         = 10;          % horizon steps

%% 物理约束
v_max     = 2.0;         % max speed [m/s]
a_max     = 3.0;         % max accel [m/s^2]
w_max     = 2.5;         % max yaw rate [rad/s]
v_progress_max = 2.0;    % max progress velocity [m/s]
v_ref_nom = 1.5;         % nominal desired speed [m/s]
stop_pos_tol = 0.1;     % stopping tolerance to endpoint [m]

%% MPCC代价权重
Wc   = 50;               % contouring error weight
Wl   = 5;              % lag error weight
Wv   = 0.5;              % speed tracking weight
Wu   = diag([1e-3, 1e-3, 1e-3]); % control effort (a, omega, v_progress)
Wds  = 3.0;              % progress consistency
Wdu  = 1e-3;             % delta-u smoothness

%% MPC (tracking)权重
Qmpc = diag([15, 15, 5, 5, 0.3]);  % (x,y,cos(theta),sin(theta),v) - 改进的权重分配
Rmpc = diag([1e-3, 1e-3]);
Rmpc_du = diag([1e-3, 1e-3]);

% 设置随机种子
rng(1);

%% 噪声参数
enable_noise = true;      % 是否启用噪声
noise_type = 'gaussian';  % 'gaussian', 'uniform'
noise_std = 0.15;          % 高斯噪声标准差 [m]
noise_uniform_range = 0.15; % 均匀噪声范围 [m]


% 噪声强度调节（相对于轨迹尺寸）
noise_scale = 1.0;        % 噪声强度倍数


%% ========================= 轨迹定义 =========================
% 从轨迹函数文件获取轨迹定义
[r_ideal, r1_ideal, r2_ideal, S_end, R] = trajectory_functions(traj_type);

% 保存理想轨迹的终点位置（用于终止条件）
p_end_ideal = r_ideal(S_end);

% 添加轨迹噪声
if enable_noise
    noise_params.std = noise_std;
    noise_params.range = noise_uniform_range;
    [r, r1, r2] = add_trajectory_noise(r_ideal, r1_ideal, r2_ideal, noise_type, noise_params, S_end);
    fprintf('已添加%s噪声: ', noise_type);
    if strcmp(noise_type, 'gaussian')
        fprintf('标准差=%.3fm\n', noise_std);
    elseif strcmp(noise_type, 'uniform')
        fprintf('范围=±%.3fm\n', noise_uniform_range);
    end
else
    fprintf('未启用噪声\n');
    r = r_ideal;
    r1 = r1_ideal;
    r2 = r2_ideal;
end
if strcmp(traj_type, 'circle') && R > 0
    % 圆形轨迹：噪声相对于半径的比例
    noise_std = noise_std * noise_scale;
    noise_uniform_range = noise_uniform_range * noise_scale;
    % fprintf('圆形轨迹半径R=%.1fm，噪声强度已调整\n', R);
end


%% ========================= 初始状态与进度 =========================
s0 = 0.0;
p0 = r(s0);
[t0, ~, ~, psi0] = geom_at(s0, r, r1, r2);
x0 = [p0(1); p0(2); psi0; 0.0; s0];   % [x,y,theta,v,progress] - progress初始化为s0
x0_mpc = [p0(1); p0(2); cos(psi0); sin(psi0); 0.0];  % [x,y,cos(theta),sin(theta),v]
% 两个控制器的上一步控制
u_last_mpcc = [0; 0; 0];  % [a, omega, v_progress]
u_last_mpc  = [0; 0];     % [a, omega]

% 进度变量
s_mpcc = s0;  % MPCC使用的进度变量（弧长驱动）
t_mpc = 0.0;  % MPC使用的时间变量（时间驱动）

% 两个被控对象的副本（公平比较）
x_mpc  = x0_mpc; 
x_mpcc = x0;  % 使用新的5维状态向量

% 初始化完成状态标志
mpc_completed = false;
mpcc_completed = false;

% 记录完成时刻
mpc_completion_step = 0;
mpc_completion_time = 0;
mpcc_completion_step = 0;
mpcc_completion_time = 0;

%% ========================= 日志容器 =========================
T = 0:dt:Tf;
K = numel(T);

% 状态和控制日志
X_mpc   = nan(5, K);  U_mpc   = nan(2, K);
X_mpcc  = nan(5, K);  U_mpcc  = nan(3, K);  % MPCC: [x,y,theta,v,progress], [a,omega,v_progress]

% 进度日志
Smpcc   = nan(1, K);  Tmpc    = nan(1, K);

% 误差日志
ErrC    = nan(1, K);  ErrL    = nan(1, K);
ErrC_mpc = nan(1, K); ErrL_mpc = nan(1, K);
PosErr_mpc  = nan(1, K);
PosErr_mpcc = nan(1, K);

%% ========================= 可视化初始化 =========================
figure('Color', 'w', 'Name', 'MPCC vs MPC'); 
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

% 路径图
nexttile(1); hold on; axis equal; grid on; box on;
ss = linspace(0, S_end, 1000);
PP = arrayfun(@(s) r(s), ss, 'UniformOutput', false);
PP = cell2mat(PP);
if enable_noise
    plot(PP(1,:), PP(2,:), 'k--', 'LineWidth', 1.2, 'DisplayName', '带噪声路径');
else
    plot(PP(1,:), PP(2,:), 'k--', 'LineWidth', 1.2, 'DisplayName', '理想路径');
end 

% 轨迹线
h_mpc  = plot(NaN, NaN, '.-', 'LineWidth', 1.5);
h_mpcc = plot(NaN, NaN, '.-', 'LineWidth', 1.5);

% 预测轨迹
h_pred1 = plot(NaN, NaN, 'o-', 'MarkerSize', 3);
h_pred2 = plot(NaN, NaN, 'o-', 'MarkerSize', 3);

legend({'path', 'MPC', 'MPCC', 'MPC pred', 'MPCC pred'}, 'Location', 'best');
title('Trajectories'); xlabel('x [m]'); ylabel('y [m]');

% 控制输入图
nexttile(2); hold on; grid on; box on;
h_a1 = plot(NaN, NaN, '-'); h_w1 = plot(NaN, NaN, '-');
h_a2 = plot(NaN, NaN, '-'); h_w2 = plot(NaN, NaN, '-'); h_vp2 = plot(NaN, NaN, '-');
legend({'a (MPC)', '\omega (MPC)', 'a (MPCC)', '\omega (MPCC)', 'v_{progress} (MPCC)'}, 'Location', 'best');
title('Controls'); xlabel('t [s]'); ylabel('u');

% 位置误差图
nexttile(3); hold on; grid on; box on;
h_ep1 = plot(NaN, NaN, '-'); 
h_ep2 = plot(NaN, NaN, '-');
legend({'pos err MPC', 'pos err MPCC'}, 'Location', 'best');
title('Position error'); xlabel('t [s]'); ylabel('|p - p^*| [m]');

% MPCC轮廓/滞后误差图
nexttile(4); hold on; grid on; box on;
h_ec = plot(NaN, NaN, '-'); h_el = plot(NaN, NaN, '-');
h_ec_mpc = plot(NaN, NaN, '-'); h_el_mpc = plot(NaN, NaN, '-');
legend({'contouring (MPCC)', 'lag (MPCC)', 'contouring (MPC)', 'lag (MPC)'}, 'Location', 'best');
title('Contouring and Lag Errors'); xlabel('t [s]'); ylabel('m');

drawnow;




%% ========================= 主循环 =========================
k_end = K;

for k = 1:K
    t = T(k);

    % ==== MPC (tracking) ====
    % MPC基于时间推进，参考轨迹按时间采样
    t_mpc = t;  % 当前时间
    ds_track = v_ref_nom * dt;  % 固定弧长步长用于采样
    [xref, uref] = sample_ref_path_cs(t_mpc * v_ref_nom, N, ds_track, r, r1, r2, v_ref_nom);
    
    % ==== MPC控制 ====
    if ~mpc_completed
        % MPC未完成时，正常求解
        [u_mpc, xpred_mpc] = solve_tracking_mpc(x_mpc, N, xref, uref, Qmpc, Rmpc, Rmpc_du, u_last_mpc, dt, a_max, w_max, v_max);
        u_mpc = unified_clamp_control(u_mpc, a_max, w_max, [], 'mpc');
    else
        % MPC已完成，保持静止状态
        u_mpc = [0; 0];  % 加速度和角速度都设为0
        xpred_mpc = x_mpc;  % 预测状态保持当前状态
    end
    
    % 使用unified_dynamics更新MPC状态，保持5维状态格式
    if ~mpc_completed
        x_mpc = unified_dynamics(x_mpc, u_mpc, dt, v_max, 'mpc_cos_sin', false);
    else
        % MPC已完成，状态保持不变
        x_mpc = x_mpc;
    end
    
    % 改进的终点处理：当接近终点时，使用更智能的控制策略
    s_mpc_equiv = t_mpc * v_ref_nom;  % 等效弧长
    dist_to_end = norm(x_mpc(1:2) - p_end_ideal);  % 到终点的距离
    
    % if s_mpc_equiv >= S_end - 0.5 || dist_to_end < 0.5  % 距离终点0.5m时开始精细控制
    %     % 接近终点时，根据距离动态调整控制输入
    %     if dist_to_end < 0.2  % 非常接近终点
    %         u_mpc(1) = u_mpc(1) * 0.05;  % 大幅减小加速度
    %         u_mpc(2) = u_mpc(2) * 0.05;  % 大幅减小角速度
    %     elseif dist_to_end < 0.5  % 较接近终点
    %         u_mpc(1) = u_mpc(1) * 0.2;   % 减小加速度
    %         u_mpc(2) = u_mpc(2) * 0.2;   % 减小角速度
    %     else  % 接近终点
    %         u_mpc(1) = u_mpc(1) * 0.5;   % 适度减小加速度
    %         u_mpc(2) = u_mpc(2) * 0.5;   % 适度减小角速度
    %     end
    % end
    
    u_mpc = unified_clamp_control(u_mpc, a_max, w_max, [], 'mpc');
    
    u_last_mpc = u_mpc;

    % ==== MPCC ====
    [u_mpcc, ds_opt, xpred_mpcc] = solve_mpcc(x_mpcc, s_mpcc, N, dt, r, r1, r2, v_ref_nom, ...
        Wc, Wl, Wv, Wu, Wds, Wdu, u_last_mpcc, v_max, a_max, w_max, v_progress_max);
    u_mpcc = unified_clamp_control(u_mpcc, a_max, w_max, v_progress_max, 'mpcc');
    x_mpcc = unified_dynamics(x_mpcc, u_mpcc, dt, v_max, 'mpcc', false);
    
    % 使用MPCC的progress控制来推进弧长
    % progress状态现在由v_progress控制，s_mpcc用于参考轨迹采样
    s_mpcc = x_mpcc(5);  % 直接使用progress状态作为当前弧长
    
    % 确保progress不越界，并用于参考轨迹采样
    s_mpcc = min(max(s_mpcc, 0), S_end);
    
    % 改进的终点处理：当接近终点时，更精确地控制进度
    if s_mpcc >= S_end - 0.5  % 距离终点0.5m时开始精细控制
        % 接近终点时，减小progress速度，确保精确到达
        u_mpcc(3) = min(u_mpcc(3), 0.05 * v_ref_nom);
        
        % 如果非常接近终点，进一步减小速度
        if s_mpcc >= S_end - 0.1
            u_mpcc(3) = min(u_mpcc(3), 0.01 * v_ref_nom);
        end
        
        % 同时减小加速度和角速度，提高稳定性
        u_mpcc(1) = u_mpcc(1) * 0.5;  % 减小加速度
        u_mpcc(2) = u_mpcc(2) * 0.5;  % 减小角速度
    end
    
    % 如果progress已经超过终点，强制停止前进
    if s_mpcc >= S_end
        u_mpcc(3) = 0;  % 停止progress推进
        u_mpcc(1) = u_mpcc(1) * 0.1;  % 大幅减小加速度
        u_mpcc(2) = u_mpcc(2) * 0.1;  % 大幅减小角速度
    end
    
    u_last_mpcc = u_mpcc;

    % ==== 数据记录 ====
    X_mpc(:,k)  = x_mpc;  U_mpc(:,k)  = u_mpc;  Tmpc(k)  = t_mpc;
    X_mpcc(:,k) = x_mpcc; U_mpcc(:,k) = u_mpcc; Smpcc(k) = x_mpcc(5);  % 使用progress状态

    % 计算误差
    % MPC基于时间计算参考点
    s_mpc_equiv = t_mpc * v_ref_nom;  % 等效弧长
    pref_mpc  = r(min(s_mpc_equiv, S_end));
    pref_mpcc = r(min(x_mpcc(5), S_end));  % 使用progress状态作为弧长
    
    % MPCC位置误差（始终计算）
    PosErr_mpcc(k) = norm(X_mpcc(1:2,k) - pref_mpcc);
    
    % MPC位置误差（仅在未完成时计算）
    if ~mpc_completed
        PosErr_mpc(k) = norm(X_mpc(1:2,k) - pref_mpc);
    else
        % MPC已完成，位置误差设为NaN，不参与统计
        PosErr_mpc(k) = NaN;
    end
    
    % 调试信息：每100步输出一次位置信息
    if mod(k, 100) == 0
        fprintf('Step %d: MPC pos=(%.2f,%.2f), MPCC pos=(%.2f,%.2f), 理想终点=(%.2f,%.2f)\n', ...
                k, X_mpc(1,k), X_mpc(2,k), X_mpcc(1,k), X_mpcc(2,k), p_end_ideal(1), p_end_ideal(2));
        fprintf('  MPC等效弧长: %.2f, MPCC progress: %.2f, 总弧长: %.2f\n', ...
                s_mpc_equiv, x_mpcc(5), S_end);
        fprintf('  MPC到终点距离: %.3f, MPCC到终点距离: %.3f\n', ...
                norm(x_mpc(1:2)-p_end_ideal), norm(x_mpcc(1:2)-p_end_ideal));
    end
    
    % ==== 误差计算 ====
    % MPCC误差（始终计算）
    % 计算当前弧长处的路径切向量
    s_mpcc_current = x_mpcc(5);
    if s_mpcc_current >= S_end
        s_mpcc_current = S_end;
    end
    
    % 计算参考路径在当前弧长处的位置和切向量
    ref_pos_mpcc = r(s_mpcc_current);
    
    % 使用数值微分计算切向量
    ds = 0.01;
    if s_mpcc_current + ds <= S_end
        ref_pos_next = r(s_mpcc_current + ds);
        ref_tangent_mpcc = (ref_pos_next - ref_pos_mpcc) / norm(ref_pos_next - ref_pos_mpcc);
    else
        ref_pos_prev = r(s_mpcc_current - ds);
        ref_tangent_mpcc = (ref_pos_mpcc - ref_pos_prev) / norm(ref_pos_mpcc - ref_pos_prev);
    end
    
    % 计算当前位置到参考路径的误差向量
    error_vec_mpcc = x_mpcc(1:2) - ref_pos_mpcc;
    
    % 将误差分解为切向和法向分量
    % 切向分量：沿路径方向的误差（lag error）
    % 正值：车辆滞后于参考点，负值：车辆超前于参考点
    lag_error_mpcc = dot(error_vec_mpcc, ref_tangent_mpcc);
    
    % 法向分量：垂直于路径方向的误差（contouring error）
    % 始终为正值，表示到路径的垂直距离
    contouring_error_mpcc = norm(error_vec_mpcc - lag_error_mpcc * ref_tangent_mpcc);
    
    % 记录误差（lag error保持有符号，contouring error保持正值）
    ErrC(k) = contouring_error_mpcc;
    ErrL(k) = lag_error_mpcc;
    
    % 调试信息：显示lag error的符号含义
    if mod(k, 100) == 0
        if lag_error_mpcc > 0
            lag_status = '滞后';
        elseif lag_error_mpcc < 0
            lag_status = '超前';
        else
            lag_status = '同步';
        end
        fprintf('MPCC Lag Error: %.3f (%s), Contouring Error: %.3f\n', ...
                lag_error_mpcc, lag_status, contouring_error_mpcc);
    end
    
    % MPC误差（仅在未完成时计算）
    if ~mpc_completed
        % 计算MPC等效弧长处的路径切向量
        s_mpc_equiv = t_mpc * v_ref_nom;
        if s_mpc_equiv >= S_end
            s_mpc_equiv = S_end;
        end
        
        ref_pos_mpc = r(s_mpc_equiv);
        
        % 使用数值微分计算切向量
        if s_mpc_equiv + ds <= S_end
            ref_pos_next_mpc = r(s_mpc_equiv + ds);
            ref_tangent_mpc = (ref_pos_next_mpc - ref_pos_mpc) / norm(ref_pos_next_mpc - ref_pos_mpc);
        else
            ref_pos_prev_mpc = r(s_mpc_equiv - ds);
            ref_tangent_mpc = (ref_pos_mpc - ref_pos_prev_mpc) / norm(ref_pos_mpc - ref_pos_prev_mpc);
        end
        
        % 计算MPC位置到参考路径的误差向量
        error_vec_mpc = x_mpc(1:2) - ref_pos_mpc;
        
        % 将误差分解为切向和法向分量
        lag_error_mpc = dot(error_vec_mpc, ref_tangent_mpc);
        contouring_error_mpc = norm(error_vec_mpc - lag_error_mpc * ref_tangent_mpc);
        
        ErrC_mpc(k) = contouring_error_mpc;
        ErrL_mpc(k) = lag_error_mpc;
        
        % 调试信息：显示MPC的lag error符号含义
        if mod(k, 100) == 0
            if lag_error_mpc > 0
                lag_status_mpc = '滞后';
            elseif lag_error_mpc < 0
                lag_status_mpc = '超前';
            else
                lag_status_mpc = '同步';
            end
            fprintf('MPC Lag Error: %.3f (%s), Contouring Error: %.3f\n', ...
                    lag_error_mpc, lag_status_mpc, contouring_error_mpc);
        end
    else
        % MPC已完成，error设为NaN，不参与统计
        ErrC_mpc(k) = NaN;
        ErrL_mpc(k) = NaN;
    end
    
    % ==== 可视化更新 ====
    if mod(k, 2) == 1 || k == 1
        % 轨迹
        nexttile(1);
        set(h_mpc, 'XData', X_mpc(1,1:k), 'YData', X_mpc(2,1:k), 'Color', [0 0.45 0.74]);
        set(h_mpcc, 'XData', X_mpcc(1,1:k), 'YData', X_mpcc(2,1:k), 'Color', [0.85 0.33 0.1]);
        set(h_pred1, 'XData', xpred_mpc(1,:), 'YData', xpred_mpc(2,:), 'Color', [0 0.45 0.74]);
        set(h_pred2, 'XData', xpred_mpcc(1,:), 'YData', xpred_mpcc(2,:), 'Color', [0.85 0.33 0.1]);
        
        % 控制输入
        nexttile(2);
        set(h_a1, 'XData', T(1:k), 'YData', U_mpc(1,1:k), 'Color', [0 0.45 0.74]);
        set(h_w1, 'XData', T(1:k), 'YData', U_mpc(2,1:k), 'Color', [0 0.45 0.74], 'LineStyle', '--');
        set(h_a2, 'XData', T(1:k), 'YData', U_mpcc(1,1:k), 'Color', [0.85 0.33 0.1]);
        set(h_w2, 'XData', T(1:k), 'YData', U_mpcc(2,1:k), 'Color', [0.85 0.33 0.1], 'LineStyle', '--');
        set(h_vp2, 'XData', T(1:k), 'YData', U_mpcc(3,1:k), 'Color', [0.49 0.18 0.56], 'LineStyle', ':');
        
        % 位置误差
        nexttile(3);
        set(h_ep1, 'XData', T(1:k), 'YData', PosErr_mpc(1:k), 'Color', [0 0.45 0.74]);
        set(h_ep2, 'XData', T(1:k), 'YData', PosErr_mpcc(1:k), 'Color', [0.85 0.33 0.1]);
        
        % MPCC误差
        nexttile(4);
        set(h_ec, 'XData', T(1:k), 'YData', ErrC(1:k), 'Color', [0.85 0.33 0.1]);
        set(h_el, 'XData', T(1:k), 'YData', ErrL(1:k), 'Color', [0.49 0.18 0.56]);
        % MPC误差
        set(h_ec_mpc, 'XData', T(1:k), 'YData', ErrC_mpc(1:k), 'Color', [0 0.45 0.74]);
        set(h_el_mpc, 'XData', T(1:k), 'YData', ErrL_mpc(1:k), 'Color', [0 0.45 0.74], 'LineStyle', '--');
        
        drawnow limitrate;
    end

    % ==== 终止条件：到达终点或时间到 ====
    % 检查MPCC是否到达终点（使用更智能的条件）
    % 考虑噪声影响，使用多维度判断
    s_mpcc_current = x_mpcc(5);
    dist_to_end_mpcc = norm(x_mpcc(1:2) - p_end_ideal);
    
    % MPCC停止条件：综合考虑progress、位置和稳定性
    progress_condition = (s_mpcc_current >= S_end - 0.1);  % progress接近终点
    position_condition = (dist_to_end_mpcc < 1.0);         % 位置在合理范围内（考虑噪声）
    stability_condition = true;                            % 稳定性条件
    
    % 如果progress已经超过终点，检查是否稳定
    if s_mpcc_current >= S_end
        % 检查最近几步的位置变化，判断是否稳定
        if k >= 5
            recent_positions = X_mpcc(1:2, max(1, k-4):k);
            position_variance = var(recent_positions, 0, 2);  % 计算x和y方向的方差
            max_variance = max(position_variance);
            stability_condition = (max_variance < 0.01);  % 位置变化小于1cm认为稳定
        end
    end
    
    mpcc_finished = progress_condition && position_condition && stability_condition;
    
    % 检查MPC是否到达终点（使用更严格的条件）
    s_mpc_equiv = t_mpc * v_ref_nom;  % 等效弧长
    mpc_finished = (s_mpc_equiv >= S_end-0.1) && (norm(x_mpc(1:2)-p_end_ideal) < 0.1);
    
    % 记录MPC完成状态，但不立即停止
    if mpc_finished && ~mpc_completed
        mpc_completed = true;
        mpc_completion_step = k;
        mpc_completion_time = t;
        fprintf('MPC到达终点，步数=%d, 时间=%.1fs\n', k, t);
        fprintf('MPC位置: (%.2f, %.2f), 等效弧长: %.2f\n', x_mpc(1), x_mpc(2), s_mpc_equiv);
    end
    
    % 记录MPCC完成状态
    if mpcc_finished && ~mpcc_completed
        mpcc_completed = true;
        mpcc_completion_step = k;
        mpcc_completion_time = t;
        fprintf('MPCC到达终点，步数=%d, 时间=%.1fs\n', k, t);
        fprintf('MPCC位置: (%.2f, %.2f), progress: %.2f\n', x_mpcc(1), x_mpcc(2), x_mpcc(5));
    end
    
    % 调试信息：当MPCC接近终点时输出详细信息
    if x_mpcc(5) >= S_end - 0.5 && ~mpcc_completed
        fprintf('MPCC接近终点: progress=%.3f, 位置=(%.3f,%.3f), 到终点距离=%.3f\n', ...
                x_mpcc(5), x_mpcc(1), x_mpcc(2), norm(x_mpcc(1:2)-p_end_ideal));
    end
    
    % 只有当MPCC也到达终点，或者超时时，才结束仿真
    if mpcc_finished || t > Tf * 0.8
        k_end = k; 
        if mpcc_finished
            if mpc_completed
                fprintf('仿真完成: MPCC和MPC都到达终点，步数=%d, 时间=%.1fs\n', k, t);
            else
                fprintf('仿真完成: MPCC到达终点，MPC未完成，步数=%d, 时间=%.1fs\n', k, t);
            end
        else
            fprintf('超时保护：仿真时间%.1fs超过限制，强制结束\n', t);
        end
        break;
    end
end

%% ========================= 结果统计 =========================
% 截取有效数据
X_mpc  = X_mpc(:,1:k_end);   U_mpc  = U_mpc(:,1:k_end);
X_mpcc = X_mpcc(:,1:k_end);  U_mpcc = U_mpcc(:,1:k_end);
T      = T(1:k_end);

% 计算统计量
rmse = @(e) sqrt(mean(e.^2));
mae  = @(e) mean(abs(e));

% 计算沿记录弧长的参考点
% MPC基于时间计算参考点
Pref_mpc  = arrayfun(@(t) r(min(t*v_ref_nom, S_end)), Tmpc(1:k_end), 'UniformOutput', false); 
Pref_mpc  = cell2mat(Pref_mpc);
Pref_mpcc = arrayfun(@(s) r(min(s,S_end)), Smpcc(1:k_end), 'UniformOutput', false); 
Pref_mpcc = cell2mat(Pref_mpcc);

% 位置误差
% MPC位置误差只统计到完成时刻（排除NaN值）
if mpc_completion_step > 0
    % 获取MPC完成前的有效位置误差数据
    PosErr_mpc_valid = PosErr_mpc(1:mpc_completion_step);
    PosErr_mpc_valid = PosErr_mpc_valid(~isnan(PosErr_mpc_valid));
    
    % 计算对应的参考点
    Pref_mpc_valid = Pref_mpc(:,1:mpc_completion_step);
    
    % 计算有效的位置误差
    e_mpc = vecnorm(X_mpc(1:2,1:mpc_completion_step) - Pref_mpc_valid, 2, 1);
else
    % MPC未完成，统计所有数据（排除NaN值）
    PosErr_mpc_valid = PosErr_mpc(~isnan(PosErr_mpc));
    e_mpc = vecnorm(X_mpc(1:2,:) - Pref_mpc, 2, 1);
end

% MPCC位置误差统计到仿真结束
e_mpcc = vecnorm(X_mpcc(1:2,:) - Pref_mpcc, 2, 1);

% 轮廓误差和滞后误差统计
% MPC误差只统计到完成时刻（排除NaN值）
if mpc_completion_step > 0
    % 获取MPC完成前的有效误差数据
    ErrC_mpc_valid = ErrC_mpc(1:mpc_completion_step);
    ErrL_mpc_valid = ErrL_mpc(1:mpc_completion_step);
    
    % 确保没有NaN值
    ErrC_mpc_valid = ErrC_mpc_valid(~isnan(ErrC_mpc_valid));
    ErrL_mpc_valid = ErrL_mpc_valid(~isnan(ErrL_mpc_valid));
    
    ErrC_mpc_final = ErrC_mpc_valid;
    ErrL_mpc_final = ErrL_mpc_valid;
    fprintf('MPC误差统计: 步数1-%d, 时间0-%.1fs\n', mpc_completion_step, mpc_completion_time);
else
    % MPC未完成，统计所有数据（排除NaN值）
    ErrC_mpc_valid = ErrC_mpc(~isnan(ErrC_mpc));
    ErrL_mpc_valid = ErrL_mpc(~isnan(ErrL_mpc));
    
    ErrC_mpc_final = ErrC_mpc_valid;
    ErrL_mpc_final = ErrL_mpc_valid;
    fprintf('MPC误差统计: 步数1-%d, 时间0-%.1fs (未完成)\n', k_end, T(end));
end

% MPCC误差统计到仿真结束
ErrC_final = ErrC(1:k_end);
ErrL_final = ErrL(1:k_end);
fprintf('MPCC误差统计: 步数1-%d, 时间0-%.1fs\n', k_end, T(end));

% 创建统计表
stats = table( ...
    rmse(e_mpc).',  mae(e_mpc).',  max(e_mpc).', ...
    rmse(e_mpcc).', mae(e_mpcc).', max(e_mpcc).', ...
    'VariableNames', {'RMSE_MPC','MAE_MPC','Max_MPC','RMSE_MPCC','MAE_MPCC','Max_MPCC'});

% 创建轮廓误差和滞后误差统计表
stats_errors = table( ...
    rmse(ErrC_mpc_final).', mae(ErrC_mpc_final).', max(ErrC_mpc_final).', ...
    rmse(ErrL_mpc_final).', mae(ErrL_mpc_final).', max(ErrL_mpc_final).', ...
    rmse(ErrC_final).', mae(ErrC_final).', max(ErrC_final).', ...
    rmse(ErrL_final).', mae(ErrL_final).', max(ErrL_final).', ...
    'VariableNames', {'RMSE_C_MPC','MAE_C_MPC','Max_C_MPC','RMSE_L_MPC','MAE_L_MPC','Max_L_MPC', ...
                      'RMSE_C_MPCC','MAE_C_MPCC','Max_C_MPCC','RMSE_L_MPCC','MAE_L_MPCC','Max_L_MPCC'});

% 显示结果
disp('==== Final point to point error stats (m) ====');
disp(stats);

disp('==== Contouring and Lag Error stats (m) ====');
disp('MPC:');
fprintf('  Contouring Error - RMSE: %.4f, MAE: %.4f, Max: %.4f\n', ...
        rmse(ErrC_mpc_final), mae(ErrC_mpc_final), max(ErrC_mpc_final));
fprintf('  Lag Error - RMSE: %.4f, MAE: %.4f, Max: %.4f\n', ...
        rmse(ErrL_mpc_final), mae(ErrL_mpc_final), max(ErrL_mpc_final));

disp('MPCC:');
fprintf('  Contouring Error - RMSE: %.4f, MAE: %.4f, Max: %.4f\n', ...
        rmse(ErrC_final), mae(ErrC_final), max(ErrC_final));
fprintf('  Lag Error - RMSE: %.4f, MAE: %.4f, Max: %.4f\n', ...
        rmse(ErrL_final), mae(ErrL_final), max(ErrL_final));

% 计算平均误差
avg_contouring_mpc = mae(ErrC_mpc_final);
avg_lag_mpc = mae(ErrL_mpc_final);
avg_contouring_mpcc = mae(ErrC_final);
avg_lag_mpcc = mae(ErrL_final);

% fprintf('\n==== Average Errors Summary ====\n');
% fprintf('MPC  - Contouring: %.4fm, Lag: %.4fm\n', avg_contouring_mpc, avg_lag_mpc);
% fprintf('MPCC - Contouring: %.4fm, Lag: %.4fm\n', avg_contouring_mpcc, avg_lag_mpcc);
% 
% % 性能对比
% if avg_contouring_mpcc < avg_contouring_mpc
%     fprintf('MPCC在轮廓误差上表现更好 (%.1f%%)\n', (1-avg_contouring_mpcc/avg_contouring_mpc)*100);
% else
%     fprintf('MPC在轮廓误差上表现更好 (%.1f%%)\n', (1-avg_contouring_mpc/avg_contouring_mpcc)*100);
% end
% 
% if avg_lag_mpcc < avg_lag_mpc
%     fprintf('MPCC在滞后误差上表现更好 (%.1f%%)\n', (1-avg_lag_mpcc/avg_lag_mpc)*100);
% else
%     fprintf('MPC在滞后误差上表现更好 (%.1f%%)\n', (1-avg_lag_mpc/avg_lag_mpcc)*100);
% end


