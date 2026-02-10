%% plot_arma_noise_demo.m
% 在一条“时间 t 参数化”的直线轨迹上加入扰动，并画出可视化结果。
%
% 假设/约定：
% - 理想轨迹：匀速直线，速度 v [m/s]，初始航向 yaw=0（沿 +x 方向）
% - 世界坐标系：x 前、y 左、z 上（右手系）
% - 扰动作用在“yaw/pitch”通道（更符合“短时间扰动→位置永久偏移”的物理直觉）
%   这里把扰动建模为“板子（board）的角速度扰动”在短时间出现一次，
%   扰动结束后角速度回到 0，但板子的姿态角保持在新的常值（永久保持新姿态）。
clear; clc; close all;

%% ========== 时间参数直线 ========== 
Tf = 60;              % [s]
dt = 0.02;            % [s]
t = (0:dt:Tf).';
N = numel(t);

v = 1.0;              % [m/s] nominal forward speed

% 理想：yaw=0，速度恒为 [v,0,0]
P_ideal = zeros(3, N);
for k = 2:N
    P_ideal(:,k) = P_ideal(:,k-1) + [v;0;0] * dt;
end

%% ========== 扰动“随机出现一段时间”的设置 ==========
% 我们用一个 burst mask 来表示“扰动出现的时间段”：
% - 只发生一次事件（一次 burst）
% - 事件持续 burst_duration 秒
% - 前 quiet_time 秒绝对不允许出现扰动
burst_duration = 2.0;     % [s] 扰动持续时间（一次）
quiet_time     = 5.0;     % [s] 初始静默期：前 quiet_time 秒不允许出现扰动
burst_len = max(1, round(burst_duration / dt));     % 持续步数

%% ========== 噪声模型：MA / AR / ARMA（作用在航向扰动上） ==========
% 说明：
% - 这里的“输入”是工程上的随机激励/扰动输入（random excitation），用于驱动扰动模型
% - 为了符合“扰动只出现一次”，我们令输入仅在 mask=1 的时间段非零
rng(1);                        % 固定随机种子，便于复现
sigma_omega_yaw = deg2rad(8.0);    % [rad/s] yaw 角速度扰动输入标准差（仅在 burst 时刻注入）
sigma_omega_pitch = deg2rad(5.0);  % [rad/s] pitch 角速度扰动输入标准差（仅在 burst 时刻注入）
sigma_eps_vy = 0.15;             % [m/s] y 方向“平移”速度扰动输入标准差（仅在 burst 时刻注入）
sigma_eps_vz = 0.10;             % [m/s] z 方向“平移”速度扰动输入标准差（仅在 burst 时刻注入）

% AR(1)/MA(1)/ARMA(1,1) 系数（|phi|<1）
phi_ar   = 0.98;   % AR(1)
theta_ma = 0.6;    % MA(1)
phi_a    = 0.95;   % ARMA(1,1)
theta_a  = 0.3;    % ARMA(1,1)

% 生成 burst mask（0/1）
mask = zeros(N,1);
k_quiet_end = min(N, round(quiet_time / dt) + 1);   % 包含 t=0 的点
% 只触发一次：在静默期之后随机选一个起点
k_latest_start = max(k_quiet_end + 1, N - burst_len + 1);
k0 = randi([k_quiet_end + 1, k_latest_start], 1, 1);
mask(k0:min(N, k0 + burst_len - 1)) = 1;
t_event = t(k0);

% 仅在 burst 期间注入角速度扰动输入（rad/s）
eps_omega_yaw = sigma_omega_yaw * randn(N,1) .* mask;
eps_omega_pitch = sigma_omega_pitch * randn(N,1) .* mask;
% 仅在 burst 期间注入“平移”扰动速度输入（m/s）
eps_vy = sigma_eps_vy * randn(N,1) .* mask;
eps_vz = sigma_eps_vz * randn(N,1) .* mask;

% 由输入生成角速度扰动序列（单位 rad/s）
% MA(1):  d_k = ε_k + θ ε_{k-1}
omega_yaw_ma = filter([1, theta_ma], 1, eps_omega_yaw);
% AR(1):  d_k = φ d_{k-1} + ε_k  <=>  filter(b=1, a=[1,-φ], ε)
omega_yaw_ar = filter(1, [1, -phi_ar], eps_omega_yaw);
% ARMA(1,1): d_k = φ d_{k-1} + ε_k + θ ε_{k-1}
omega_yaw_arma = filter([1, theta_a], [1, -phi_a], eps_omega_yaw);

omega_pitch_ma = filter([1, theta_ma], 1, eps_omega_pitch);
omega_pitch_ar = filter(1, [1, -phi_ar], eps_omega_pitch);
omega_pitch_arma = filter([1, theta_a], [1, -phi_a], eps_omega_pitch);

% “平移”扰动速度 dv_y(t), dv_z(t)（单位 m/s）
dvy_ma = filter([1, theta_ma], 1, eps_vy);
dvy_ar = filter(1, [1, -phi_ar], eps_vy);
dvy_arma = filter([1, theta_a], [1, -phi_a], eps_vy);

dvz_ma = filter([1, theta_ma], 1, eps_vz);
dvz_ar = filter(1, [1, -phi_ar], eps_vz);
dvz_arma = filter([1, theta_a], [1, -phi_a], eps_vz);

%% ========== 将角速度扰动积分为姿态角（永久保持新姿态） ==========
% yaw/pitch 角速度在 burst 后变为 0，因此 yaw/pitch 将保持在 burst 结束时的常值
yaw_ma = zeros(N,1);   yaw_ar = zeros(N,1);   yaw_arma = zeros(N,1);
pitch_ma = zeros(N,1); pitch_ar = zeros(N,1); pitch_arma = zeros(N,1);
for k = 2:N
    yaw_ma(k)   = yaw_ma(k-1)   + omega_yaw_ma(k)   * dt;
    yaw_ar(k)   = yaw_ar(k-1)   + omega_yaw_ar(k)   * dt;
    yaw_arma(k) = yaw_arma(k-1) + omega_yaw_arma(k) * dt;

    pitch_ma(k)   = pitch_ma(k-1)   + omega_pitch_ma(k)   * dt;
    pitch_ar(k)   = pitch_ar(k-1)   + omega_pitch_ar(k)   * dt;
    pitch_arma(k) = pitch_arma(k-1) + omega_pitch_arma(k) * dt;
end

%% ========== 无控制下的轨迹积分（位置是速度积分） ==========
% 对每种噪声模型都生成一条轨迹：板子姿态 yaw(t), pitch(t) 改变后，直线运动方向随之改变
P_ma   = zeros(3,N);
P_ar   = zeros(3,N);
P_arma = zeros(3,N);

dir_from_yaw_pitch = @(yaw, pitch) [
    cos(yaw) * cos(pitch);
    sin(yaw) * cos(pitch);
    sin(pitch) ];

for k = 2:N
    % MA
    yaw = yaw_ma(k);
    pitch = pitch_ma(k);
    v_world = v * dir_from_yaw_pitch(yaw, pitch);
    v_world = v_world + [0; dvy_ma(k); dvz_ma(k)];
    P_ma(:,k) = P_ma(:,k-1) + v_world * dt;

    % AR
    yaw = yaw_ar(k);
    pitch = pitch_ar(k);
    v_world = v * dir_from_yaw_pitch(yaw, pitch);
    v_world = v_world + [0; dvy_ar(k); dvz_ar(k)];
    P_ar(:,k) = P_ar(:,k-1) + v_world * dt;

    % ARMA
    yaw = yaw_arma(k);
    pitch = pitch_arma(k);
    v_world = v * dir_from_yaw_pitch(yaw, pitch);
    v_world = v_world + [0; dvy_arma(k); dvz_arma(k)];
    P_arma(:,k) = P_arma(:,k-1) + v_world * dt;
end

%% ========== 作图 ==========
figure('Color','w','Name','MA / AR / ARMA burst demo (no controller)');
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

% 轨迹对比（3D）
nexttile(1); hold on; grid on; box on; axis equal; view(3);
plot3(P_ideal(1,:), P_ideal(2,:), P_ideal(3,:), 'k--', 'LineWidth', 1.2, 'DisplayName', 'ideal');
plot3(P_ma(1,:),   P_ma(2,:),   P_ma(3,:),   'LineWidth', 1.5, 'DisplayName', 'MA(1) burst');
plot3(P_ar(1,:),   P_ar(2,:),   P_ar(3,:),   'LineWidth', 1.5, 'DisplayName', 'AR(1) burst');
plot3(P_arma(1,:), P_arma(2,:), P_arma(3,:), 'LineWidth', 1.5, 'DisplayName', 'ARMA(1,1) burst');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title(sprintf('Trajectory deviation (one event at t=%.2fs; board holds new attitude)', t_event));
legend('Location','best');

% yaw 角（注意：burst 后保持常值，不回到 0）
nexttile(2); hold on; grid on; box on;
plot(t, rad2deg(yaw_ma), 'LineWidth', 1.0, 'DisplayName', 'MA(1)');
plot(t, rad2deg(yaw_ar), 'LineWidth', 1.0, 'DisplayName', 'AR(1)');
plot(t, rad2deg(yaw_arma), 'LineWidth', 1.0, 'DisplayName', 'ARMA(1,1)');
ylabel('yaw [deg]'); xlabel('t [s]');
title('Board yaw angle (integrated from angular-rate disturbance)');
legend('Location','best');

% pitch 角（burst 后保持常值）
nexttile(3); hold on; grid on; box on;
plot(t, rad2deg(pitch_ma), 'LineWidth', 1.0, 'DisplayName', 'MA(1)');
plot(t, rad2deg(pitch_ar), 'LineWidth', 1.0, 'DisplayName', 'AR(1)');
plot(t, rad2deg(pitch_arma), 'LineWidth', 1.0, 'DisplayName', 'ARMA(1,1)');
ylabel('pitch [deg]'); xlabel('t [s]');
title('Board pitch angle (integrated from angular-rate disturbance)');
legend('Location','best');

% translation（平移）扰动速度：dv_y(t), dv_z(t)
nexttile(4); hold on; grid on; box on;
plot(t, dvy_ma, 'LineWidth', 1.0, 'DisplayName', 'dv_y MA(1)');
plot(t, dvy_ar, 'LineWidth', 1.0, 'DisplayName', 'dv_y AR(1)');
plot(t, dvy_arma, 'LineWidth', 1.0, 'DisplayName', 'dv_y ARMA(1,1)');
plot(t, dvz_ma, 'LineWidth', 1.0, 'LineStyle','--', 'DisplayName', 'dv_z MA(1)');
plot(t, dvz_ar, 'LineWidth', 1.0, 'LineStyle','--', 'DisplayName', 'dv_z AR(1)');
plot(t, dvz_arma, 'LineWidth', 1.0, 'LineStyle','--', 'DisplayName', 'dv_z ARMA(1,1)');
xlabel('t [s]'); ylabel('dv [m/s]');
title('Translation disturbance velocities (dv_y solid, dv_z dashed)');
legend('Location','best');

% 打印最终偏移量到命令行，便于精确查看
final_ma = P_ma(:,end) - P_ideal(:,end);
final_ar = P_ar(:,end) - P_ideal(:,end);
final_arma = P_arma(:,end) - P_ideal(:,end);
fprintf('Final offset (MA)   [m]: [%.3f, %.3f, %.3f]\\n', final_ma(1), final_ma(2), final_ma(3));
fprintf('Final offset (AR)   [m]: [%.3f, %.3f, %.3f]\\n', final_ar(1), final_ar(2), final_ar(3));
fprintf('Final offset (ARMA) [m]: [%.3f, %.3f, %.3f]\\n', final_arma(1), final_arma(2), final_arma(3));

%% ========== 额外图：显示“真实注入扰动输入大小”（不只是门控） ==========
figure('Color','w','Name','Injected disturbance inputs (actual magnitude)');
tiledlayout(2,1,'Padding','compact','TileSpacing','compact');

% rotation 输入（deg/s）
nexttile(1); hold on; grid on; box on;
yl1 = max([rad2deg(sigma_omega_yaw), rad2deg(sigma_omega_pitch)]) * 4;
plot(t, rad2deg(eps_omega_yaw),   'LineWidth', 1.0, 'DisplayName', 'input \omega_{yaw}(t)');
plot(t, rad2deg(eps_omega_pitch), 'LineWidth', 1.0, 'DisplayName', 'input \omega_{pitch}(t)');
ylabel('input [deg/s]'); xlabel('t [s]');
title('Rotation disturbance inputs (angular-rate excitation)');
legend('Location','best');
ylim([-yl1, yl1]);

% translation 输入（m/s）
nexttile(2); hold on; grid on; box on;
yl2 = max([sigma_eps_vy, sigma_eps_vz]) * 4;
plot(t, eps_vy, 'LineWidth', 1.0, 'DisplayName', 'input v_y(t)');
plot(t, eps_vz, 'LineWidth', 1.0, 'DisplayName', 'input v_z(t)');
ylabel('input [m/s]'); xlabel('t [s]');
title('Translation disturbance inputs (velocity excitation)');
legend('Location','best');
ylim([-yl2, yl2]);