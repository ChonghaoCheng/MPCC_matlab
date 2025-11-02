clear; clc; close all;

%% ========== CONFIGURATION ==========
%% Simulation parameters
traj_type = 'helix';           % 'line' | 'circle' | 'helix'
Tf = 60.0;                    % [s] Maximum simulation time (termination only)
dt = 0.01;                    % [s]
N = 10;                       % Horizon length

%% Trajectory timing
T_traj_desired = 20.0;        % [s] Desired time to complete trajectory
v_desired = 2;               % [m/s] Desired reference speed (empty = auto from T_traj_desired)
                              % If specified, trajectory length will be adjusted to match v_desired * T_traj_desired

%% Constraints
v_max = 2.5;                 % [m/s] Maximum linear velocity
w_max = 2.0;                 % [rad/s] Maximum angular velocity
v_progress_max = 2.5;         % [m/s] Maximum progress speed for MPCC

%% MPCC weights
Wc = 50;                      % Contouring error weight
Wl = 5;                       % Lag error weight
Qs = 1e-3;                    % Progress speed reward
Ru = diag([0,0,0,0,0,0, 0]);  % Control effort weights

%% MPC weights
Qmpc = diag([10,10,10, 5,5,5]);  % State error weights
Rmpc = diag([1e-3,1e-3,1e-3, 1e-3,1e-3,1e-3]);      % Control effort weights

%% Control smoothing weights
lambda_du_mpc = [0, 1e-3];              % [linear_vel, angular_vel] for MPC
lambda_du_mpcc = [0, 1e-3, 0];       % [linear_vel, angular_vel, progress_speed] for MPCC

%% Initial control strategy (for reducing initial oscillations)
init_control_mode = 'mixed';  % 'reference' | 'zero' | 'mixed'
                                 % 'reference': use trajectory reference velocities (smooth start)
                                 % 'zero': use zero linear/angular velocities, keep v_s (may reduce oscillations)
                                 % 'mixed': linear/angular from reference, but zero for u_last (for smoother QP nominal)

%% Noise settings
enable_noise = false;         % Toggle noise on/off
noise_type = 'rotation_only'; % 'gaussian' | 'rotation_only' | 'translation_only' | 'none'
noise_std = 0.1;              % [m] Gaussian noise standard deviation
noise_rot_deg = 5.0;          % [deg] Rotation amplitude for rotation_only
noise_trans_amp = 0.2;        % [m] Translation amplitude for translation_only
noise_trans_dir = [1; 0; 0];  % [3x1] Translation direction for translation_only
noise_period_sec = 5.0;       % [s] Period for time-triggered noise
noise_sdot_nom = [];          % [m/s] Time conversion speed (empty = use v_plan_mpc)

%% ========== INITIALIZATION ==========
%% Path generation with speed-controlled trajectory length
if ~isempty(v_desired)
    % User specified desired speed: adjust trajectory length to match
    % Check if desired speed is within constraints
    if v_desired > v_max
        warning('Desired speed %.4f m/s exceeds v_max (%.4f m/s). Using v_max instead.', v_desired, v_max);
        v_desired = v_max;
    end
    if v_desired > v_progress_max
        warning('Desired speed %.4f m/s exceeds v_progress_max (%.4f m/s). Using v_progress_max instead.', v_desired, v_progress_max);
        v_desired = min(v_desired, v_progress_max);
    end
    % Calculate target trajectory length based on desired speed and time
    S_target = v_desired * T_traj_desired;
    % Generate trajectory with adjusted length
    [r_ideal, r1_ideal, r2_ideal, S_end] = trajectory_functions_3d(traj_type, S_target);
    v_plan_mpc = v_desired;
    fprintf('Trajectory generated: length = %.2f m, speed = %.4f m/s, time = %.2f s\n', ...
        S_end, v_plan_mpc, T_traj_desired);
else
    % Auto mode: use trajectory's default length, compute speed from T_traj_desired
    [r_ideal, r1_ideal, r2_ideal, S_end] = trajectory_functions_3d(traj_type);
    v_plan_mpc = S_end / T_traj_desired;
    if v_plan_mpc > v_max
        warning('Computed speed %.4f m/s exceeds v_max (%.4f m/s). Using v_max instead.', v_plan_mpc, v_max);
        v_plan_mpc = v_max;
        fprintf('Actual completion time will be %.2f s (trajectory length: %.2f m)\n', ...
            S_end/v_plan_mpc, S_end);
    elseif v_plan_mpc > v_progress_max
        warning('Computed speed %.4f m/s exceeds v_progress_max (%.4f m/s). Using v_progress_max instead.', v_plan_mpc, v_progress_max);
        v_plan_mpc = min(v_plan_mpc, v_progress_max);
        fprintf('Actual completion time will be %.2f s (trajectory length: %.2f m)\n', ...
            S_end/v_plan_mpc, S_end);
    else
        fprintf('Trajectory: length = %.2f m, speed = %.4f m/s, time = %.2f s\n', ...
            S_end, v_plan_mpc, T_traj_desired);
    end
end
sdot_nom = v_plan_mpc;

%% Apply noise to trajectory
if enable_noise
    noise_params = struct();
    if strcmp(noise_type,'gaussian')
        noise_params.std = noise_std;
    elseif strcmp(noise_type,'rotation_only')
        if isempty(noise_sdot_nom)
            noise_params.sdot_nom = v_plan_mpc;
        else
            noise_params.sdot_nom = noise_sdot_nom;
        end
        noise_params.period_sec = noise_period_sec;
        noise_params.rot_deg = noise_rot_deg;
    elseif strcmp(noise_type,'translation_only')
        if isempty(noise_sdot_nom)
            noise_params.sdot_nom = v_plan_mpc;
        else
            noise_params.sdot_nom = noise_sdot_nom;
        end
        noise_params.period_sec = noise_period_sec;
        noise_params.trans_amp = noise_trans_amp;
        noise_params.trans_dir = noise_trans_dir;
    end
    [r, r1, r2] = add_trajectory_noise_3d(r_ideal, r1_ideal, r2_ideal, noise_type, noise_params, S_end);
else
    r = r_ideal; r1 = r1_ideal; r2 = r2_ideal;
end

%% Initial states
s0 = 0.0;
p0 = r(s0);
[t0, ~, ~, ~, ~] = geom3_at(s0, r, r1, r2);
yaw0 = atan2(t0(2), t0(1));
pitch0 = atan2(-t0(3), sqrt(max(t0(1)^2 + t0(2)^2, eps)));
roll0 = 0;
x0_mpcc = [p0(1); p0(2); p0(3); roll0; pitch0; yaw0; s0];
x0_mpc = [p0(1); p0(2); p0(3); roll0; pitch0; yaw0];

%% Initial velocities (aligned with reference trajectory)
ds_ref_init = v_plan_mpc * dt;
[xref_init, uref_init, ~] = sample_ref_path3d(s0, 1, ds_ref_init, r, r1, r2, v_plan_mpc);
v_ref_traj = uref_init(1:3, 1);
omega_ref_traj = uref_init(4:6, 1);

% Set initial control inputs based on strategy
switch lower(init_control_mode)
    case 'reference'
        % Use reference trajectory velocities (default: smooth start)
        u0_mpc = [v_ref_traj; omega_ref_traj];
        u0_mpcc = [v_ref_traj; omega_ref_traj; v_plan_mpc];
        u_last_mpc = u0_mpc;
        u_last_mpcc = u0_mpcc;
    case 'zero'
        % Zero linear/angular velocities, keep progress speed (may reduce initial oscillations)
        u0_mpc = zeros(6, 1);
        u0_mpcc = [zeros(6, 1); v_plan_mpc];  % Keep initial progress speed
        u_last_mpc = u0_mpc;
        u_last_mpcc = u0_mpcc;
    case 'mixed'
        % Use reference for actual control, but zero for u_last (smoother QP nominal trajectory)
        u0_mpc = [v_ref_traj; omega_ref_traj];
        u0_mpcc = [v_ref_traj; omega_ref_traj; v_plan_mpc];
        u_last_mpc = zeros(6, 1);  % Zero for smoother QP initialization
        u_last_mpcc = [zeros(6, 1); v_plan_mpc];  % Keep progress speed for MPCC
    otherwise
        error('Unknown init_control_mode: %s. Use ''reference'', ''zero'', or ''mixed''', init_control_mode);
end

%% Data logging
T = 0:dt:Tf; K = numel(T);
X_mpcc = nan(7,K); U_mpcc = nan(7,K); Smpcc = nan(1,K);
X_mpc = nan(6,K); U_mpc = nan(6,K);
ErrCtot = nan(1,K); ErrCtot_mpc = nan(1,K); ErrL = nan(1,K);
PosErr_mpcc = nan(1,K); PosErr_mpc = nan(1,K);
Sstar_mpcc = nan(1,K); Sstar_mpc = nan(1,K);

%% Visualization
figure('Color','w','Name','3D MPCC vs MPC');
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

nexttile(1); hold on; grid on; axis equal; box on; view(3);
ss = linspace(0, S_end, 1000);
PP = arrayfun(@(s) r(s), ss, 'UniformOutput', false); PP = cell2mat(PP);
plot3(PP(1,:), PP(2,:), PP(3,:), 'k--','LineWidth',1.2,'DisplayName','path');
h_mpc = plot3(NaN,NaN,NaN,'.-','LineWidth',1.5,'DisplayName','MPC','Color',[0 0.45 0.74]);
h_mpcc = plot3(NaN,NaN,NaN,'.-','LineWidth',1.5,'DisplayName','MPCC','Color',[0.85 0.33 0.1]);
legend('Location','best'); xlabel('x'); ylabel('y'); zlabel('z'); title('Trajectories');

nexttile(2); hold on; grid on; box on;
h_u_mpc = plot(NaN,NaN,'-','Color',[0 0.45 0.74]);
h_w_mpc = plot(NaN,NaN,'--','Color',[0 0.45 0.74]);
h_u_mpcc = plot(NaN,NaN,'-','Color',[0.85 0.33 0.1]);
h_w_mpcc = plot(NaN,NaN,'--','Color',[0.85 0.33 0.1]);
h_vs = plot(NaN,NaN,':','Color',[0.49 0.18 0.56]);
legend({'|v| MPC','|\omega| MPC','|v| MPCC','|\omega| MPCC','v_s MPCC'},'Location','best');
title('Control magnitudes'); xlabel('t [s]'); ylabel('magnitude');

nexttile(3); hold on; grid on; box on;
h_pos_mpc = plot(NaN,NaN,'-','Color',[0 0.45 0.74]);
h_pos_mpcc = plot(NaN,NaN,'-','Color',[0.85 0.33 0.1]);
legend({'|p-p^d| MPC','|p-p^d| MPCC'},'Location','best');
title('Position error'); xlabel('arc [m]'); ylabel('m');

nexttile(4); hold on; grid on; box on;
h_lag = plot(NaN,NaN,'-','Color',[0.49 0.18 0.56]);
h_contour = plot(NaN,NaN,'-','Color',[0.85 0.33 0.1]);
h_contour_mpc = plot(NaN,NaN,'--','Color',[0 0.45 0.74]);
legend({'lag (MPCC)','contour total (MPCC)','contour total (MPC)'},'Location','best');
title('Errors'); xlabel('arc [m]'); ylabel('m');
drawnow;

%% ========== SIMULATION ==========
x_mpc = x0_mpc; x_mpcc = x0_mpcc; s_mpcc = s0;
mpc_completed = false; mpc_completion_step = 0;
mpcc_completed = false; mpcc_completion_step = 0;
p_end_ideal = r_ideal(S_end);

% Record initial state
X_mpc(:,1) = x0_mpc; U_mpc(:,1) = u0_mpc;
X_mpcc(:,1) = x0_mpcc; U_mpcc(:,1) = u0_mpcc; Smpcc(1) = s0;

% Initial error calculation and diagnostic printing
[s_star_mpc_init, pref_mpc_init, ~, n1_mpc_init, n2_mpc_init] = closest_point_on_path_3d(x0_mpc(1:3), s0, r, r1, r2, S_end);
PosErr_mpc(1) = norm(x0_mpc(1:3) - pref_mpc_init);
err_m_init = x0_mpc(1:3) - pref_mpc_init;
ErrCtot_mpc(1) = sqrt(dot(n1_mpc_init,err_m_init)^2 + dot(n2_mpc_init,err_m_init)^2);
Sstar_mpc(1) = s_star_mpc_init;

[~, pref_mpcc_init, t_mpcc_init, n1_mpcc_init, n2_mpcc_init] = closest_point_on_path_3d(x0_mpcc(1:3), s0, r, r1, r2, S_end);
PosErr_mpcc(1) = norm(x0_mpcc(1:3) - pref_mpcc_init);
err_init = x0_mpcc(1:3) - pref_mpcc_init;
ErrL(1) = dot(t_mpcc_init, err_init);
ErrCtot(1) = sqrt(dot(n1_mpcc_init,err_init)^2 + dot(n2_mpcc_init,err_init)^2);

% Print initial state alignment diagnostics
fprintf('\n==== Initial State Alignment Diagnostics =====\n');
fprintf('Initial position: [%.4f, %.4f, %.4f]\n', p0(1), p0(2), p0(3));
fprintf('Reference position at s=0: [%.4f, %.4f, %.4f]\n', pref_mpcc_init(1), pref_mpcc_init(2), pref_mpcc_init(3));
fprintf('Position error: %.6f m (should be ~0)\n', PosErr_mpcc(1));
fprintf('Contour error: %.6f m (should be ~0)\n', ErrCtot(1));
fprintf('Lag error: %.6f m\n', ErrL(1));
fprintf('Initial control (MPCC):\n');
fprintf('  Linear velocity: [%.4f, %.4f, %.4f] m/s (ref: [%.4f, %.4f, %.4f])\n', ...
    u0_mpcc(1), u0_mpcc(2), u0_mpcc(3), v_ref_traj(1), v_ref_traj(2), v_ref_traj(3));
fprintf('  Angular velocity: [%.4f, %.4f, %.4f] rad/s (ref: [%.4f, %.4f, %.4f])\n', ...
    u0_mpcc(4), u0_mpcc(5), u0_mpcc(6), omega_ref_traj(1), omega_ref_traj(2), omega_ref_traj(3));
fprintf('  Progress speed: %.4f m/s (ref: %.4f)\n', u0_mpcc(7), v_plan_mpc);
fprintf('Initialization mode: %s\n', init_control_mode);
if PosErr_mpcc(1) > 0.01
    warning('Initial position error is large (%.6f m). Check trajectory alignment.', PosErr_mpcc(1));
end
fprintf('==============================================\n\n');

for k = 2:K
    tnow = T(k);

    % ==== MPC ====
    ds_ref = v_plan_mpc * dt;
    if ~mpc_completed
        s_ref_k = min(s0 + (k-1)*ds_ref, S_end);
        [xref, uref] = sample_ref_path3d(s_ref_k, N, ds_ref, r, r1, r2, v_plan_mpc);
        xref(:,1) = x_mpc;  % Ensure first reference matches current state
        [u_mpc, xpred_mpc] = solve_mpc_3d(x_mpc, N, xref, uref, Qmpc, Rmpc, dt, v_max, w_max, u_last_mpc, lambda_du_mpc);
        x_mpc = mpc_dynamics_3d(x_mpc, u_mpc, dt);
        u_last_mpc = u_mpc;
        dist_to_end_mpc = norm(x_mpc(1:3) - p_end_ideal);
        if (s_ref_k >= S_end - 0.1) && (dist_to_end_mpc < 0.1) && ~mpc_completed
            mpc_completed = true;
            mpc_completion_step = k;
        end
    end

    % ==== MPCC ====
    if ~mpcc_completed
        rem_s = max(0, S_end - s_mpcc);
        Qs_local = Qs; v_progress_cap = v_progress_max;
        if rem_s < 0.3
            Qs_local = 0;
            v_progress_cap = min(v_progress_max, rem_s / max(dt, eps));
        end
        [u_mpcc, ds_opt, xpred_mpcc] = solve_mpcc_3d(x_mpcc, s_mpcc, N, dt, r, r1, r2, ...
            Wc, Wl, Qs_local, Ru, u_last_mpcc, v_max, w_max, v_progress_cap, v_plan_mpc, lambda_du_mpcc);
        x_mpcc = mpcc_dynamics_3d(x_mpcc, u_mpcc, dt);
        u_last_mpcc = u_mpcc;
        s_mpcc = min(max(x_mpcc(7), 0), S_end);
        dist_to_end_mpcc = norm(x_mpcc(1:3) - p_end_ideal);
        if (s_mpcc >= S_end - 0.1) && (dist_to_end_mpcc < 0.1) && ~mpcc_completed
            mpcc_completed = true;
            mpcc_completion_step = k;
        end
    end

    % Logging
    if ~mpc_completed
        X_mpc(:,k) = x_mpc; U_mpc(:,k) = u_mpc;
        [s_star_mpc, pref_mpc, ~, n1_mpc, n2_mpc] = closest_point_on_path_3d(x_mpc(1:3), s_ref_k + ds_ref, r, r1, r2, S_end);
        PosErr_mpc(k) = norm(x_mpc(1:3) - pref_mpc);
        err_m = x_mpc(1:3) - pref_mpc;
        ErrCtot_mpc(k) = sqrt(dot(n1_mpc,err_m)^2 + dot(n2_mpc,err_m)^2);
        Sstar_mpc(k) = s_star_mpc;
    end
    if ~mpcc_completed
        X_mpcc(:,k) = x_mpcc; U_mpcc(:,k) = u_mpcc; Smpcc(k) = s_mpcc;
        [~, pref_mpcc, t_mpcc, n1_mpcc, n2_mpcc] = closest_point_on_path_3d(x_mpcc(1:3), s_mpcc, r, r1, r2, S_end);
        PosErr_mpcc(k) = norm(x_mpcc(1:3) - pref_mpcc);
        err = x_mpcc(1:3) - pref_mpcc;
        ErrL(k) = dot(t_mpcc, err);
        ErrCtot(k) = sqrt(dot(n1_mpcc,err)^2 + dot(n2_mpcc,err)^2);
    end

    % Plotting
    if mod(k,2)==1 || k==1
        nexttile(1);
        set(h_mpc, 'XData', X_mpc(1,1:k), 'YData', X_mpc(2,1:k), 'ZData', X_mpc(3,1:k));
        set(h_mpcc, 'XData', X_mpcc(1,1:k), 'YData', X_mpcc(2,1:k), 'ZData', X_mpcc(3,1:k));
        nexttile(2);
        set(h_u_mpc, 'XData', T(1:k), 'YData', vecnorm(U_mpc(1:3,1:k),2,1));
        set(h_w_mpc, 'XData', T(1:k), 'YData', vecnorm(U_mpc(4:6,1:k),2,1));
        set(h_u_mpcc, 'XData', T(1:k), 'YData', vecnorm(U_mpcc(1:3,1:k),2,1));
        set(h_w_mpcc, 'XData', T(1:k), 'YData', vecnorm(U_mpcc(4:6,1:k),2,1));
        set(h_vs, 'XData', T(1:k), 'YData', U_mpcc(7,1:k));
        nexttile(3);
        set(h_pos_mpc, 'XData', Sstar_mpc(1:k), 'YData', PosErr_mpc(1:k));
        set(h_pos_mpcc, 'XData', Smpcc(1:k), 'YData', PosErr_mpcc(1:k));
        nexttile(4);
        set(h_lag, 'XData', Smpcc(1:k), 'YData', ErrL(1:k));
        set(h_contour, 'XData', Smpcc(1:k), 'YData', ErrCtot(1,1:k));
        set(h_contour_mpc, 'XData', Sstar_mpc(1:k), 'YData', ErrCtot_mpc(1,1:k));
        drawnow limitrate;
    end

    % Termination
    if (mpc_completed || (s_ref_k >= S_end - 1e-3)) && (mpcc_completed || (s_mpcc >= S_end - 1e-3))
        break;
    end
end

%% ========== STATISTICS ==========
k_end = k;
rmse = @(e) sqrt(mean(e.^2));
mae = @(e) mean(abs(e));

% Position error
if mpc_completion_step > 0
    PosErr_mpc_valid = PosErr_mpc(1:mpc_completion_step);
else
    PosErr_mpc_valid = PosErr_mpc(1:k_end);
end
PosErr_mpc_valid = PosErr_mpc_valid(~isnan(PosErr_mpc_valid));

if mpcc_completion_step > 0
    PosErr_mpcc_valid = PosErr_mpcc(1:mpcc_completion_step);
else
    PosErr_mpcc_valid = PosErr_mpcc(1:k_end);
end
PosErr_mpcc_valid = PosErr_mpcc_valid(~isnan(PosErr_mpcc_valid));

fprintf('==== Position Error Stats (m) ====\n');
fprintf('MPC : RMSE=%.4f, MAE=%.4f, Max=%.4f\n', rmse(PosErr_mpc_valid), mae(PosErr_mpc_valid), max(PosErr_mpc_valid));
fprintf('MPCC: RMSE=%.4f, MAE=%.4f, Max=%.4f\n', rmse(PosErr_mpcc_valid), mae(PosErr_mpcc_valid), max(PosErr_mpcc_valid));

% MPCC lag/contour
if mpcc_completion_step > 0
    ErrL_valid = ErrL(1:mpcc_completion_step);
    ErrCtot_valid = ErrCtot(1:mpcc_completion_step);
else
    ErrL_valid = ErrL(1:k_end);
    ErrCtot_valid = ErrCtot(1:k_end);
end
ErrL_valid = ErrL_valid(~isnan(ErrL_valid));
ErrCtot_valid = ErrCtot_valid(~isnan(ErrCtot_valid));
fprintf('==== MPCC Lag/Contour Stats (m) ====\n');
fprintf('Lag      : RMSE=%.4f, MAE=%.4f, Max=%.4f\n', rmse(ErrL_valid), mae(ErrL_valid), max(ErrL_valid));
fprintf('Contour T: RMSE=%.4f, MAE=%.4f, Max=%.4f\n', rmse(ErrCtot_valid), mae(ErrCtot_valid), max(ErrCtot_valid));

% MPC contour
if mpc_completion_step > 0
    ErrCtot_mpc_valid = ErrCtot_mpc(1:mpc_completion_step);
else
    ErrCtot_mpc_valid = ErrCtot_mpc(1:k_end);
end
ErrCtot_mpc_valid = ErrCtot_mpc_valid(~isnan(ErrCtot_mpc_valid));
fprintf('==== MPC Contour Stats (m) ====\n');
fprintf('Contour T: RMSE=%.4f, MAE=%.4f, Max=%.4f\n', rmse(ErrCtot_mpc_valid), mae(ErrCtot_mpc_valid), max(ErrCtot_mpc_valid));
