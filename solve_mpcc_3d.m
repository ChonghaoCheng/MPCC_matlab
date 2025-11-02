function [u_opt, ds_opt, xpred] = solve_mpcc_3d(x0, s0, N, dt, r, r1, r2, ...
    Wc, Wl, Qs, Ru, u_last, v_lin_max, w_ang_max, v_progress_max, v_plan)
% SOLVE_MPCC_3D 
% State: [x;y;z; psi_x;psi_y;psi_z; s]
% Input: [v_x;v_y;v_z; omega_x;omega_y;omega_z; v_s]

nx = 7; nu = 7;

% Nominal progress rollout (fallback ds if solver fails)
ds_guess = v_progress_max * dt; 

% Nominal reference used for linearization
xbar = zeros(nx, N+1); xbar(:,1) = x0;
ubar = zeros(nu, N);
% Plan progress speed (optional), clamped to bounds
if nargin < 16 || isempty(v_plan)
    vs_nom = v_progress_max;
else
    vs_nom = min(v_progress_max, max(0, v_plan));
end
for k = 1:N
    % set linear and angular velocities to zero nominally
    ubar(:,k) = [0;0;0; 0;0;0; vs_nom];
    xbar(:,k+1) = mpcc_dynamics_3d(xbar(:,k), ubar(:,k), dt);
end

% Linearization
[A, B] = mpcc_linearize_3d(xbar(:,1:end-1), ubar, dt);

% Condensed dynamics X = Ax*x0 + Bu*U 
Ax = zeros(nx*N, nx);
Bu = zeros(nx*N, nu*N);
d  = zeros(nx*N, 1);
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
end

% Build C_e_bar and r_e_bar for 3D errors using nominal predicted progress sbar
% mpcc.tex definition: lag = t^T e, contour = ||(I - t t^T) e||
S_p = [eye(3) zeros(3,4)];

C_e_blocks = cell(1, N);
r_e_blocks = cell(1, N);
for k = 1:N
    sk = xbar(7,k); % use nominal progress from rollout
    [t_k, ~, ~, ~, p_ref, ~, ~] = geom3_at(sk, r, r1, r2);
    Pn = eye(3) - (t_k * t_k.'); % projection onto normal plane
    lag_row = (t_k.' * S_p);      % 1x7
    cont_mat = (Pn * S_p);        % 3x7 (rank-2)
    C_k = [ lag_row; cont_mat ];  % 4x7
    r_k = [ t_k.'*p_ref; (Pn * p_ref) ]; % 4x1
    C_e_blocks{k} = C_k;   % 4x7
    r_e_blocks{k} = r_k;   % 4x1
end
C_e_bar = blkdiag(C_e_blocks{:});          % (4N) x (nx*N)
r_e_bar = vertcat(r_e_blocks{:});          % (4N) x 1

% Weights per stage: diag([Wl, Wc, Wc, Wc]) 
Q_blocks = cell(1,N);
for k = 1:N
    Q_blocks{k} = diag([Wl, Wc, Wc, Wc]);
end
Q_blk = blkdiag(Q_blocks{:});              % (4N)x(4N)
R_blk = kron(eye(N), Ru);                  % (nu*N)x(nu*N)

% Condensed QP
X0 = Ax*x0 + d;
H = (Bu.' * (C_e_bar.' * Q_blk * C_e_bar) * Bu) + R_blk;
f = Bu.' * (C_e_bar.' * Q_blk * (C_e_bar * X0 - r_e_bar));


Ubar_stack = reshape(ubar, nu*N, 1);
f = f - R_blk * Ubar_stack;

% Progress reward -Qs * sum v_s * dt
if Qs > 0
    f_vs = zeros(nu*N,1);
    for k = 1:N
        idx_vs = (k-1)*nu + 7;
        f_vs(idx_vs) = -Qs * dt;
    end
    f = f + f_vs;
end

% Box constraints on controls
umin = repmat([ -v_lin_max; -v_lin_max; -v_lin_max; -w_ang_max; -w_ang_max; -w_ang_max; 0], N, 1);
umax = repmat([  v_lin_max;  v_lin_max;  v_lin_max;  w_ang_max;  w_ang_max;  w_ang_max; v_progress_max], N, 1);

opts = optimoptions('quadprog', 'Display', 'off');
H = (H + H.')/2;
u_seq = quadprog(H, f, [], [], [], [], umin, umax, [], opts);

if isempty(u_seq)
    u_opt = zeros(nu,1);
    ds_opt = ds_guess;
else
    u_opt = u_seq(1:nu);
    ds_opt = u_opt(7) * dt;
end

% Rollout prediction with either solution or nominal
xpred = zeros(nx, N+1);
xpred(:,1) = x0;
if ~isempty(u_seq)
    for k = 1:N
        uk = u_seq((k-1)*nu+(1:nu));
        xpred(:,k+1) = mpcc_dynamics_3d(xpred(:,k), uk, dt);
    end
else
    for k = 1:N
        xpred(:,k+1) = mpcc_dynamics_3d(xpred(:,k), ubar(:,k), dt);
    end
end

end





