clear; clc; close all;

%% Parameters 
traj_type = 'helix';   % 'line' | 'circle' | 'helix'
Tf        = 60.0;      % [s]
dt        = 0.01;      % [s]
N         = 10;        % horizon

%% Constraints
v_max      = 2.0;
w_max      = 2.0;
v_progress_max = 2;  % for MPCC   

%% MPCC weights
Wc = 100;
Wl = 0.5;
Qs = 1e-7;
Ru = diag([1e-3,1e-3,1e-3, 1e-3,1e-3,1e-3, 1e-6]);

%% MPC weights
Qmpc = diag([10,10,10, 5,5,5]);
Rmpc = diag([1e-3,1e-3,1e-3, 1e-3,1e-3,1e-3]);
 
%% Path
[r_ideal, r1_ideal, r2_ideal, S_end] = trajectory_functions_3d(traj_type);

%% Noise settings 
enable_noise = true;           % toggle noise on/off
noise_type = 'uniform_arc';        % 'gaussian' | 'uniform_time' | 'uniform_arc'
noise_std = 0.1;               % gaussian std [m]
noise_uniform_range = 0.1;     % uniform range [m]

if enable_noise
    noise_params = struct();
    if strcmp(noise_type,'gaussian')
        noise_params.std = noise_std;
    end
    [r, r1, r2] = add_trajectory_noise_3d(r_ideal, r1_ideal, r2_ideal, noise_type, noise_params, S_end);
else
    r = r_ideal; r1 = r1_ideal; r2 = r2_ideal;
end

%% Initial states
s0 = 0.0;
p0 = r(s0);
x0_mpcc = [p0(1); p0(2); p0(3); 0; 0; 0; s0];
x0_mpc  = [p0(1); p0(2); p0(3); 0; 0; 0];

u_last_mpcc = zeros(7,1);
u_last_mpc  = zeros(6,1);

v_plan_mpc = 2*S_end / max(Tf, eps);
sdot_nom = v_plan_mpc;

%% Logs
T = 0:dt:Tf; K = numel(T);
X_mpcc = nan(7,K); U_mpcc = nan(7,K); Smpcc = nan(1,K);
X_mpc  = nan(6,K); U_mpc  = nan(6,K);
ErrCtot = nan(1,K);        % MPCC contour total
ErrCtot_mpc = nan(1,K);    % MPC contour total
ErrL = nan(1,K);           % MPCC lag
PosErr_mpcc = nan(1,K); PosErr_mpc = nan(1,K);
Sstar_mpcc = nan(1,K); Sstar_mpc = nan(1,K);

%% Visualization
figure('Color','w','Name','3D MPCC vs MPC');
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

nexttile(1); hold on; grid on; axis equal; box on; view(3);
ss = linspace(0, S_end, 1000);
PP = arrayfun(@(s) r(s), ss, 'UniformOutput', false); PP = cell2mat(PP);
plot3(PP(1,:), PP(2,:), PP(3,:), 'k--','LineWidth',1.2,'DisplayName','path');
h_mpc  = plot3(NaN,NaN,NaN,'.-','LineWidth',1.5,'DisplayName','MPC','Color',[0 0.45 0.74]);
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
h_pos_mpc  = plot(NaN,NaN,'-','Color',[0 0.45 0.74]);
h_pos_mpcc = plot(NaN,NaN,'-','Color',[0.85 0.33 0.1]);
legend({'|p-p^d| MPC','|p-p^d| MPCC'},'Location','best');
title('Position error'); xlabel('t [s]'); ylabel('m');

nexttile(4); hold on; grid on; box on;
h_lag = plot(NaN,NaN,'-','Color',[0.49 0.18 0.56]);
h_contour = plot(NaN,NaN,'-','Color',[0.85 0.33 0.1]);
h_contour_mpc = plot(NaN,NaN,'--','Color',[0 0.45 0.74]);
legend({'lag (MPCC)','contour total (MPCC)','contour total (MPC)'},'Location','best');
title('Errors'); xlabel('arc [m]'); ylabel('m');

drawnow;

%% Main loop
x_mpc = x0_mpc; x_mpcc = x0_mpcc; s_mpcc = s0; s_mpc = s0;
% Completion gating
mpc_completed = false; mpc_completion_step = 0; mpc_completion_time = 0;
mpcc_completed = false; mpcc_completion_step = 0; mpcc_completion_time = 0;
% For distance-to-end, use ideal end point
p_end_ideal = r_ideal(S_end);
for k = 1:K
    tnow = T(k);

    % ==== MPC  ====
    ds_ref = v_plan_mpc*dt;  
    if ~mpc_completed
        s_ref_k = min(s0 + (k-1)*ds_ref, S_end);
        [xref, uref] = sample_ref_path3d(s_ref_k, N, ds_ref, r, r1, r2, v_plan_mpc);
        [u_mpc, xpred_mpc] = solve_mpc_3d(x_mpc, N, xref, uref, Qmpc, Rmpc, dt, v_max, w_max);
        x_mpc = mpc_dynamics_3d(x_mpc, u_mpc, dt);
        % Check completion 
        dist_to_end_mpc = norm(x_mpc(1:3) - p_end_ideal);
        mpc_finished = (s_ref_k >= S_end - 0.1) && (dist_to_end_mpc < 0.1);
        if mpc_finished && ~mpc_completed
            mpc_completed = true;
            mpc_completion_step = k; mpc_completion_time = tnow;
        end
    else
        % Stop MPC evolution and logging after completion
        U_mpc(:,k) = NaN; X_mpc(:,k) = NaN; ErrCtot_mpc(k) = NaN; PosErr_mpc(k) = NaN;
    end

    % ==== MPCC ====
    if ~mpcc_completed
        [u_mpcc, ds_opt, xpred_mpcc] = solve_mpcc_3d(x_mpcc, s_mpcc, N, dt, r, r1, r2, ...
            Wc, Wl, Qs, Ru, u_last_mpcc, v_max, w_max, v_progress_max, v_plan_mpc);
        x_mpcc = mpcc_dynamics_3d(x_mpcc, u_mpcc, dt);
        s_mpcc = min(max(x_mpcc(7), 0), S_end);
        % Check completion for MPCC
        dist_to_end_mpcc = norm(x_mpcc(1:3) - p_end_ideal);
        mpcc_finished = (s_mpcc >= S_end - 0.1) && (dist_to_end_mpcc < 0.1);
        if mpcc_finished && ~mpcc_completed
            mpcc_completed = true;
            mpcc_completion_step = k; mpcc_completion_time = tnow;
        end
    else
        % Stop MPCC evolution and logging after completion
        U_mpcc(:,k) = NaN; X_mpcc(:,k) = NaN; ErrCtot(k) = NaN; ErrL(k) = NaN; PosErr_mpcc(k) = NaN;
    end

    % Logs 
    if ~mpc_completed
        X_mpc(:,k)  = x_mpc;  U_mpc(:,k)  = u_mpc;
    end
    if ~mpcc_completed
        X_mpcc(:,k) = x_mpcc; U_mpcc(:,k) = u_mpcc; Smpcc(k) = s_mpcc;
    end

    % Errors using consistent closest-point projection
    if ~mpc_completed
        [s_star_mpc, pref_mpc, ~, n1_mpc, n2_mpc] = closest_point_on_path_3d(X_mpc(1:3,k), s_ref_k, r, r1, r2, S_end);
        PosErr_mpc(k)  = norm(X_mpc(1:3,k)  - pref_mpc);
        err_m = X_mpc(1:3,k) - pref_mpc;
        c1m = dot(n1_mpc, err_m); c2m = dot(n2_mpc, err_m);
        ErrCtot_mpc(k) = sqrt(c1m*c1m + c2m*c2m);
        Sstar_mpc(k) = s_star_mpc;
    end
    if ~mpcc_completed
        [~, pref_mpcc, t_mpcc, n1_mpcc, n2_mpcc] = closest_point_on_path_3d(X_mpcc(1:3,k), s_mpcc, r, r1, r2, S_end);
        PosErr_mpcc(k) = norm(X_mpcc(1:3,k) - pref_mpcc);
        err = X_mpcc(1:3,k) - pref_mpcc;
        ErrL(k)   = dot(t_mpcc, err);
        c1 = dot(n1_mpcc, err); c2 = dot(n2_mpcc, err);
        ErrCtot(k) = sqrt(c1*c1 + c2*c2);
    end

    % Plot updates
    if mod(k,2)==1 || k==1
        nexttile(1);
        set(h_mpc,  'XData', X_mpc(1,1:k),  'YData', X_mpc(2,1:k),  'ZData', X_mpc(3,1:k));
        set(h_mpcc, 'XData', X_mpcc(1,1:k), 'YData', X_mpcc(2,1:k), 'ZData', X_mpcc(3,1:k));

        nexttile(2);
        set(h_u_mpc,  'XData', T(1:k), 'YData', vecnorm(U_mpc(1:3,1:k),2,1));
        set(h_w_mpc,  'XData', T(1:k), 'YData', vecnorm(U_mpc(4:6,1:k),2,1));
        set(h_u_mpcc, 'XData', T(1:k), 'YData', vecnorm(U_mpcc(1:3,1:k),2,1));
        set(h_w_mpcc, 'XData', T(1:k), 'YData', vecnorm(U_mpcc(4:6,1:k),2,1));
        set(h_vs,     'XData', T(1:k), 'YData', U_mpcc(7,1:k));

        nexttile(3);
        set(h_pos_mpc,  'XData', Sstar_mpc(1:k), 'YData', PosErr_mpc(1:k));
        set(h_pos_mpcc, 'XData', Smpcc(1:k),     'YData', PosErr_mpcc(1:k));

        nexttile(4);
        set(h_lag, 'XData', Smpcc(1:k), 'YData', ErrL(1:k));
        set(h_contour,  'XData', Smpcc(1:k), 'YData', ErrCtot(1,1:k));
        set(h_contour_mpc,  'XData', Sstar_mpc(1:k), 'YData', ErrCtot_mpc(1,1:k));
        drawnow limitrate;
    end

    % Termination when both completed OR both s* reach end and stable
    if mpc_completed && mpcc_completed
        break;
    end
end


%% End-of-run stats
% Effective horizon
k_end = k;

rmse = @(e) sqrt(mean(e.^2));
mae  = @(e) mean(abs(e));

% Position error stats
PosErr_mpc_valid = PosErr_mpc(1:k_end);
if mpc_completion_step > 0
    PosErr_mpc_valid = PosErr_mpc(1:mpc_completion_step);
end
PosErr_mpc_valid = PosErr_mpc_valid(~isnan(PosErr_mpc_valid));

PosErr_mpcc_valid = PosErr_mpcc(1:k_end);
if mpcc_completion_step > 0
    PosErr_mpcc_valid = PosErr_mpcc(1:mpcc_completion_step);
end
PosErr_mpcc_valid = PosErr_mpcc_valid(~isnan(PosErr_mpcc_valid));

fprintf('==== Position Error Stats (m) ====''\n');
fprintf('MPC : RMSE=%.4f, MAE=%.4f, Max=%.4f\n', rmse(PosErr_mpc_valid), mae(PosErr_mpc_valid), max(PosErr_mpc_valid));
fprintf('MPCC: RMSE=%.4f, MAE=%.4f, Max=%.4f\n', rmse(PosErr_mpcc_valid), mae(PosErr_mpcc_valid), max(PosErr_mpcc_valid));

% MPCC lag/contour stats
ErrL_valid   = ErrL(1:k_end);
ErrCtot_valid = ErrCtot(1:k_end);
if mpcc_completion_step > 0
    ErrL_valid    = ErrL(1:mpcc_completion_step);
    ErrCtot_valid = ErrCtot(1:mpcc_completion_step);
end
ErrL_valid    = ErrL_valid(~isnan(ErrL_valid));
ErrCtot_valid = ErrCtot_valid(~isnan(ErrCtot_valid));
fprintf('==== MPCC Lag/Contour Stats (m) ====''\n');
fprintf('Lag      : RMSE=%.4f, MAE=%.4f, Max=%.4f\n', rmse(ErrL_valid), mae(ErrL_valid), max(ErrL_valid));
fprintf('Contour T: RMSE=%.4f, MAE=%.4f, Max=%.4f\n', rmse(ErrCtot_valid), mae(ErrCtot_valid), max(ErrCtot_valid));

% MPC contour stats 
ErrCtot_mpc_valid = ErrCtot_mpc(1:k_end);
if mpc_completion_step > 0
    ErrCtot_mpc_valid = ErrCtot_mpc(1:mpc_completion_step);
end
ErrCtot_mpc_valid = ErrCtot_mpc_valid(~isnan(ErrCtot_mpc_valid));
fprintf('==== MPC Contour Stats (m) ====''\n');
fprintf('Contour T: RMSE=%.4f, MAE=%.4f, Max=%.4f\n', rmse(ErrCtot_mpc_valid), mae(ErrCtot_mpc_valid), max(ErrCtot_mpc_valid));


 