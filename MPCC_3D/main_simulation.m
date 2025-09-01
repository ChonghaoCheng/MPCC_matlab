%% MPCC vs MPC on 3D trajectories (line / helix / lissajous)
% Author: you + ChatGPT
% Explanation in Chinese, comments in English.
% Requires Optimization Toolbox (quadprog).

clear; clc; close all;

%% ========================= 用户参数（可改） =========================
traj_type = 'lissajous';    % 'line' | 'helix' | 'lissajous'
Tf        = 100.0;       % max sim time [s]
dt        = 0.05;       % step [s]
N         = 20;         % horizon steps

v_max     = 3.0;        % max speed [m/s]
a_max     = 4.0;        % max accel [m/s^2]
w_yaw_max   = 2.5;      % max yaw rate [rad/s]
w_pitch_max = 2.0;      % max pitch rate [rad/s]
v_ref_nom = 1.5;        % nominal desired speed [m/s]

stop_pos_tol = 0.15;    % stopping tolerance to end arclength [m]

% MPCC cost weights
Wc   = 50;   % contouring error weight (normal + binormal)
Wl   = 1.0;  % lag error weight (tangential)
Wv   = 0.5;  % speed tracking weight
Wu   = diag([1e-3, 1e-3, 1e-3]); % control effort (a, yaw_rate, pitch_rate)
Wds  = 3.0;  % progress (ds) consistency
Wdu  = 1e-3; % delta-u smoothness

% MPC (tracking) weights（直接位置/姿态/速度跟踪）
% x = [px py pz yaw pitch v]
Qmpc = diag([15,15,15, 2,2, 0.5]); % state error weights
Rmpc = diag([1e-3,1e-3,1e-3]);     % input effort
Rmpc_du = diag([1e-3,1e-3,1e-3]);  % input rate change

% seed
rng(1);

%% ========================= 3D 轨迹定义 =========================
[r, r1, r2, S_end, v_ref_of_s] = define_3d_trajectory(traj_type, v_ref_nom);

%% ========================= 初始状态与进度 =========================
s0 = 0.0;
p0 = r(s0);
[t0,~,~,~,yaw0,pitch0] = geom3_at(s0, r, r1, r2);
x0 = [p0(1); p0(2); p0(3); yaw0; pitch0; 0.0]; % [px,py,pz,yaw,pitch,v]

u_last_mpcc = [0;0;0];  % [a, yaw_rate, pitch_rate]
u_last_mpc  = [0;0;0];

s_mpcc = s0;
s_mpc  = s0;

%% ========================= 日志容器 =========================
T  = 0:dt:Tf; K = numel(T);
nx = 6; nu = 3;

X_mpc   = nan(nx,K);  U_mpc   = nan(nu,K);
X_mpcc  = nan(nx,K);  U_mpcc  = nan(nu,K);
Smpcc   = nan(1,K);   Smpc    = nan(1,K);
ErrC    = nan(1,K);   ErrL    = nan(1,K);
PosErr_mpc  = nan(1,K);
PosErr_mpcc = nan(1,K);

x_mpc  = x0;
x_mpcc = x0;

%% ========================= 可视化初始化（3D+时序） =========================
[h_mpc, h_mpcc, h_pred1, h_pred2, h_a1, h_wy1, h_wp1, h_a2, h_wy2, h_wp2, h_ep1, h_ep2, h_ec, h_el] = initialize_visualization(S_end, r);

%% ========================= 主循环 =========================
k_end = K;
for k=1:K
    t = T(k);

    % ==== MPC (tracking) - 时间相关 ====
    [xref, uref] = sample_ref_path3D_time(t, N, dt, r, r1, r2, v_ref_of_s, @geom3_at);
    [u_mpc, xpred_mpc] = solve_tracking_mpc_3d(x_mpc, N, xref, uref, Qmpc, Rmpc, Rmpc_du, u_last_mpc, ...
                                               dt, a_max, w_yaw_max, w_pitch_max, v_max);
    u_mpc = clamp_u(u_mpc, a_max, w_yaw_max, w_pitch_max);
    x_mpc = step_dyn(x_mpc, u_mpc, dt, v_max);
    
    % 更新MPC的弧长进度（用于统计和可视化）
    s_mpc = update_arc_length(s_mpc, x_mpc, r, r1, r2);
    u_last_mpc = u_mpc;

    % ==== MPCC ====
    [u_mpcc, ds_opt, xpred_mpcc] = solve_mpcc_3d(x_mpcc, s_mpcc, N, dt, r, r1, r2, v_ref_of_s, ...
        Wc, Wl, Wv, Wu, Wds, Wdu, u_last_mpcc, v_max, a_max, w_yaw_max, w_pitch_max, @geom3_at);
    u_mpcc = clamp_u(u_mpcc, a_max, w_yaw_max, w_pitch_max);
    x_mpcc = step_dyn(x_mpcc, u_mpcc, dt, v_max);
    s_mpcc = min(s_mpcc + ds_opt, S_end);
    u_last_mpcc = u_mpcc;

    % ==== logging ====
    X_mpc(:,k)  = x_mpc;  U_mpc(:,k)  = u_mpc;  Smpc(k)  = s_mpc;
    X_mpcc(:,k) = x_mpcc; U_mpcc(:,k) = u_mpcc; Smpcc(k) = s_mpcc;

    % errors（位置误差）
    pref_mpc  = r(s_mpc);
    pref_mpcc = r(s_mpcc);
    PosErr_mpc(k)  = norm(X_mpc(1:3,k)  - pref_mpc);
    PosErr_mpcc(k) = norm(X_mpcc(1:3,k) - pref_mpcc);

    % MPCC contour / lag（展示）
    [t_k,n_k,b_k,~,~,~] = geom3_at(s_mpcc, r, r1, r2);
    pe = X_mpcc(1:3,k) - pref_mpcc;
    e_c = sqrt( (n_k.'*pe)^2 + (b_k.'*pe)^2 ); % magnitude in normal-binormal plane
    e_l = abs(t_k.'*pe);
    ErrC(k)=e_c; ErrL(k)=e_l;

    % ==== 可视化 ====
    if mod(k,2)==1 || k==1
        update_visualization(h_mpc, h_mpcc, h_pred1, h_pred2, h_a1, h_wy1, h_wp1, h_a2, h_wy2, h_wp2, ...
                           h_ep1, h_ep2, h_ec, h_el, X_mpc, X_mpcc, xpred_mpc, xpred_mpcc, U_mpc, U_mpcc, ...
                           PosErr_mpc, PosErr_mpcc, ErrC, ErrL, T, k);
    end

    % ==== 终止条件 ====
    if (s_mpcc >= S_end-1e-6) || (s_mpc >= S_end-1e-6)
        k_end = k; break;
    end
end

%% ========================= 结束统计 =========================
X_mpc  = X_mpc(:,1:k_end);   U_mpc  = U_mpc(:,1:k_end);
X_mpcc = X_mpcc(:,1:k_end);  U_mpcc = U_mpcc(:,1:k_end);
T      = T(1:k_end);         Smpc   = Smpc(1:k_end);  Smpcc = Smpcc(1:k_end);

% 计算统计结果
stats = calculate_final_stats(X_mpc, X_mpcc, Smpc, Smpcc, r, S_end);

disp('==== Final error stats (m) ====');
disp(stats);



