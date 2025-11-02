function [u_opt, xpred] = solve_mpc_3d(x0, N, xref, uref, Q, R, dt, vmax_lin, wmax_ang, u_last, lambda_du)
% SOLVE_MPC_3D: 6D state/input condensed QP tracking MPC
% x0   : [6x1]
% xref : [6 x (N+1)] reference states
% uref : [6 x N]     reference inputs

nx = 6; nu = 6;
if nargin < 10 || isempty(u_last),    u_last = zeros(nu,1); end
if nargin < 11 || isempty(lambda_du), lambda_du = 0;        end

% Linearize around provided reference
[A, B] = mpc_linearize_3d(xref(:,1:end-1), uref, dt);

% Condensed dynamics X = Ax*x0 + Bu*U + d
Ax = zeros(nx*N, nx);  Bu = zeros(nx*N, nu*N);  d = zeros(nx*N,1);
for k=1:N
    Ak = eye(nx);
    for j=1:k, Ak = A(:,:,j)*Ak; end
    Ax((k-1)*nx+(1:nx),:) = Ak;
    for j=1:k
        Phi = eye(nx);
        for m=j+1:k, Phi = A(:,:,m)*Phi; end
        Bu((k-1)*nx+(1:nx),(j-1)*nu+(1:nu)) = Phi*B(:,:,j);
    end
end

Qb = kron(eye(N), Q);
Rb = kron(eye(N), R);

H = (Bu.' * Qb * Bu) + Rb;

xref_stack = reshape(xref(:,2:end), nx*N, 1);
uref_stack = reshape(uref(:,1:N), nu*N, 1);
X0 = Ax*x0 + d;
e  = X0 - xref_stack;
f  = Bu.' * Qb * e - Rb * uref_stack;

% Control increment smoothing with separate weights for linear/angular velocities
if lambda_du > 0
    Iu = eye(nu);
    D = kron(eye(N), Iu) - kron(diag(ones(N-1,1),-1), Iu);
    d0 = zeros(nu*N,1); d0(1:nu) = -u_last;
    % Weight matrix: [lambda_v, lambda_v, lambda_v, lambda_w, lambda_w, lambda_w]
    if isscalar(lambda_du)
        lambda_v = lambda_du; % linear velocities
        lambda_w = lambda_du; % angular velocities
    else
        lambda_v = lambda_du(1);
        lambda_w = lambda_du(2);
    end
    W_smooth = kron(eye(N), diag([lambda_v, lambda_v, lambda_v, lambda_w, lambda_w, lambda_w]));
    H = H + 2 * (D.' * W_smooth * D);
    f = f + 2 * (D.' * W_smooth * d0);
end

% Bounds on inputs
umin = repmat([ -vmax_lin; -vmax_lin; -vmax_lin; -wmax_ang; -wmax_ang; -wmax_ang], N,1);
umax = repmat([  vmax_lin;  vmax_lin;  vmax_lin;  wmax_ang;  wmax_ang;  wmax_ang], N,1);

opts = optimoptions('quadprog','Display','off','Algorithm','interior-point-convex');
u_seq = quadprog( (H+H.')/2, f, [], [], [], [], umin, umax, [], opts);
if isempty(u_seq)
    u_seq = uref_stack;
end
u_opt = u_seq(1:nu);

% Rollout
xk = x0; xpred = zeros(nx, N+1); xpred(:,1) = xk;
for k=1:N
    uk = u_seq((k-1)*nu+(1:nu));
    xk = mpc_dynamics_3d(xk, uk, dt);
    xpred(:,k+1) = xk;
end

end














