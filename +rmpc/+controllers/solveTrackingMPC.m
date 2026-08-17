function [uOpt, finalCost] = solveTrackingMPC(method, state0, ref, uPrev, uInit, cfg)
N = cfg.N;
nU = 6 * N;
assertFinite(ref.p, 'ref.p', method);
assertFinite(ref.q, 'ref.q', method);
assertFinite(ref.T, 'ref.T', method);
assertFinite(ref.rpy, 'ref.rpy', method);
assertFinite(uPrev, 'uPrev', method);
assertFinite(uInit, 'uInit', method);
assertFinite(cfg.lb, 'cfg.lb', method);
assertFinite(cfg.ub, 'cfg.ub', method);
switch method
    case 'euclidean'
        e0 = euclideanError(state0, ref, 1);
        Q = blkdiag(cfg.weights.Qp, cfg.weights.Qr);
        QTerminal = blkdiag(cfg.weights.QpTerminal, cfg.weights.QrTerminal);
        B = euclideanInputMap(ref, N);
    case 'riemannian'
        e0 = quaternionError(state0, ref, 1);
        Q = blkdiag(cfg.weights.Qp, cfg.weights.Qr);
        QTerminal = blkdiag(cfg.weights.QpTerminal, cfg.weights.QrTerminal);
        B = identityInputMap(N);
    case {'se3', 'lie'}
        e0 = se3Error(state0, ref, 1);
        Q = cfg.weights.Qse3;
        QTerminal = cfg.weights.Qse3Terminal;
        B = identityInputMap(N);
    otherwise
        error('Unknown tracking MPC method: %s', method);
end
uRef = rmpc.reference.referenceBodyTwists(ref, cfg.dt);
assertFinite(uRef, 'reference body twists', method);

S = predictionMatrix(B, N, cfg.dt);
b = zeros(6 * N, 1);
refCum = zeros(6, 1);
for j = 1:N
    refCum = refCum + B(:, :, j) * uRef(:, j);
    idx = blockIdx(j);
    b(idx) = e0 - cfg.dt * refCum;
end

Qbar = kron(eye(N), Q);
Qbar(blockIdx(N), blockIdx(N)) = QTerminal;
Rbar = kron(eye(N), cfg.weights.Ru);
D = deltaMatrix(N);
dPrev = [-uPrev; zeros(6 * (N - 1), 1)];
Rdbar = kron(eye(N), cfg.weights.Rdu);

H = 2 * (S' * Qbar * S + Rbar + D' * Rdbar * D);
f = 2 * (S' * Qbar * b + D' * Rdbar * dPrev);
H = 0.5 * (H + H') + 1e-10 * eye(nU);
assertFinite(H, 'quadprog H', method);
assertFinite(f, 'quadprog f', method);

[uOpt, finalCost, exitflag] = quadprog(H, f, [], [], [], [], cfg.lb, cfg.ub, uInit, cfg.opts);
if exitflag <= 0 || isempty(uOpt)
    warning('rmpc:quadprogTrackingFailed', ...
        'quadprog failed for %s MPC with exitflag %d. Reusing warm start.', method, exitflag);
    uOpt = min(max(uInit, cfg.lb), cfg.ub);
    finalCost = 0.5 * uOpt' * H * uOpt + f' * uOpt;
end
end

function e = euclideanError(state, ref, idx)
R = rmpc.geometry.quatToRotm(state.q);
e = [state.p - ref.p(:, idx);
     rmpc.utils.wrapToPiLocal(rmpc.geometry.rotmToRpy(R) - ref.rpy(:, idx))];
end

function e = quaternionError(state, ref, idx)
Rref = rmpc.geometry.quatToRotm(ref.q(:, idx));
e = [Rref' * (state.p - ref.p(:, idx));
     rmpc.geometry.quatLogError(ref.q(:, idx), state.q)];
end

function e = se3Error(state, ref, idx)
e = rmpc.geometry.logSE3(rmpc.geometry.invSE3(ref.T(:, :, idx)) * state.T);
end

function B = euclideanInputMap(ref, N)
B = zeros(6, 6, N);
for j = 1:N
    Rref = rmpc.geometry.quatToRotm(ref.q(:, j));
    B(:, :, j) = blkdiag(Rref, bodyOmegaToRpyRate(ref.rpy(:, j)));
end
end

function E = bodyOmegaToRpyRate(rpy)
roll = rpy(1);
pitch = rpy(2);
cp = cos(pitch);
if abs(cp) < 1e-6
    cp = sign(cp + (cp == 0)) * 1e-6;
end
E = [1, sin(roll) * tan(pitch), cos(roll) * tan(pitch);
     0, cos(roll),             -sin(roll);
     0, sin(roll) / cp,         cos(roll) / cp];
end

function B = identityInputMap(N)
B = repmat(eye(6), 1, 1, N);
end

function S = predictionMatrix(B, N, dt)
S = zeros(6 * N, 6 * N);
for row = 1:N
    for col = 1:row
        S(blockIdx(row), blockIdx(col)) = dt * B(:, :, col);
    end
end
end

function D = deltaMatrix(N)
D = eye(6 * N);
for j = 2:N
    D(blockIdx(j), blockIdx(j - 1)) = -eye(6);
end
end

function idx = blockIdx(j)
idx = (6 * (j - 1) + 1):(6 * j);
end

function assertFinite(x, name, method)
if any(~isfinite(x(:)))
    error('rmpc:nonFiniteTrackingMPC', ...
        '%s MPC has non-finite values in %s before quadprog.', method, name);
end
end
