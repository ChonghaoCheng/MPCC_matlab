function [uOpt, mpcc] = solveRiemannianMPCCTrackingQP(state, progress0, uPrev, mpcc, cfg)
N = cfg.N;
nZ = 7 * N;
progressStartPred = progressPrediction(progress0, mpcc.vprogressGuess, cfg, 'start');
progressEndPred = progressPrediction(progress0, mpcc.vprogressGuess, cfg, 'end');
e0 = rmpc.geometry.logSE3(rmpc.geometry.invSE3(rmpc.reference.referenceAtProgress(progress0, cfg.trajectory).T) * state.T);
[A, b] = errorPredictionModel(e0, progressStartPred, mpcc.vprogressGuess, cfg);
Qbar = riemannianStageWeights(progressEndPred, cfg);

H = 2 * (A' * Qbar * A);
f = 2 * (A' * Qbar * b);
[H, f] = addRegularization(H, f, uPrev, mpcc.vprogressPrev, progressStartPred, progress0, cfg);
[H, f] = addPathVelocityTracking(H, f, progressStartPred, mpcc, cfg);
H = 0.5 * (H + H') + 1e-10 * eye(nZ);

[Aineq, bineq] = progressConstraints(progress0, cfg);
progressLb = progressRateLowerBound(progress0, cfg);
lbz = [cfg.lb; progressLb];
ubz = [cfg.ub; cfg.mpcc.progressRateMax * ones(N, 1)];

[zOpt, cost, exitflag] = quadprog(H, f, Aineq, bineq, [], [], lbz, ubz, mpcc.zGuess, cfg.opts);
if exitflag <= 0 || isempty(zOpt)
    warning('rmpc:quadprogRiemannianMpccFailed', ...
        'quadprog failed for Riemannian MPCC with exitflag %d. Reusing warm start.', exitflag);
    zOpt = min(max(mpcc.zGuess, lbz), ubz);
    cost = 0.5 * zOpt' * H * zOpt + f' * zOpt;
end

uOpt = zOpt(1:6);
mpcc.zGuess = shiftMpccGuess(zOpt, N);
mpcc.vprogressGuess = mpcc.zGuess((6 * N + 1):end)';
mpcc.progressCurrent = min(progress0 + cfg.dt * zOpt(6 * N + 1), cfg.progressEnd);
mpcc.vprogressPrev = zOpt(6 * N + 1);
mpcc.costCurrent = cost;
end

function [A, b] = errorPredictionModel(e0, progressPred, vprogressGuess, cfg)
N = cfg.N;
A = zeros(6 * N, 7 * N);
b = zeros(6 * N, 1);
Aj = zeros(6, 7 * N);
bj = e0;
for j = 1:N
    row = blockIdx(j);
    tau = rmpc.reference.referenceTangentTwist(progressPred(j), cfg.trajectory);
    dsGuess = cfg.dt * vprogressGuess(j);
    F = rmpc.geometry.adjointSE3(rmpc.geometry.expSE3(-dsGuess * tau));

    Aj = F * Aj;
    bj = F * bj;
    Aj(:, blockIdx(j)) = Aj(:, blockIdx(j)) + cfg.dt * eye(6);
    Aj(:, 6 * N + j) = Aj(:, 6 * N + j) - cfg.dt * tau;

    A(row, :) = Aj;
    b(row) = bj;
end
end

function Qbar = riemannianStageWeights(progressPred, cfg)
N = cfg.N;
Qbar = zeros(6 * N);
for j = 1:N
    switch riemannianLagMode(cfg)
        case 'se3'
            Qe = fullSe3ContourWeight(progressPred(j), cfg);
        case 'position'
            Qe = positionContourWeight(progressPred(j), cfg);
        otherwise
            error('Unknown Riemannian MPCC lag mode: %s', riemannianLagMode(cfg));
    end
    if j == N
        Qe = 2.2 * Qe;
    end
    Qbar(blockIdx(j), blockIdx(j)) = Qe;
end
end

function mode = riemannianLagMode(cfg)
mode = 'position';
if isfield(cfg.mpcc, 'riemannianLagMode')
    mode = lower(char(cfg.mpcc.riemannianLagMode));
end
end

function Qe = fullSe3ContourWeight(progress, cfg)
M = cfg.mpcc.G;
tau = rmpc.reference.referenceTangentTwist(progress, cfg.trajectory);
denom = tau' * M * tau + 1e-12;
Plag = tau * (tau' * M) / denom;
Pc = eye(6) - Plag;
lagRow = (tau' * M) / sqrt(denom);
Qc = contourWeight(cfg.mpcc.Qc, 6);
Qe = Pc' * Qc * Pc + cfg.mpcc.Ql * (lagRow' * lagRow);
end

function Qe = positionContourWeight(progress, cfg)
tau = rmpc.reference.referenceTangentTwist(progress, cfg.trajectory);
tauPos = tau(1:3);
tauPos = tauPos / max(norm(tauPos), 1e-12);
Pc = eye(3) - tauPos * tauPos';
Qc = contourWeight(cfg.mpcc.Qc, 6);
QpContour = Qc(1:3, 1:3);
Qr = Qc(4:6, 4:6);
Qp = Pc' * QpContour * Pc + cfg.mpcc.Ql * (tauPos * tauPos');
Qe = blkdiag(Qp, Qr);
end

function Qc = contourWeight(rawQc, dim)
if isscalar(rawQc)
    Qc = rawQc * eye(dim);
    return;
end
if isequal(size(rawQc), [dim, dim])
    Qc = rawQc;
    return;
end
if size(rawQc, 1) >= dim && size(rawQc, 2) >= dim
    Qc = rawQc(1:dim, 1:dim);
    return;
end
error('mpcc.Qc must be scalar or %dx%d for Riemannian MPCC.', dim, dim);
end

function [H, f] = addRegularization(H, f, uPrev, vprogressPrev, progressPred, progress0, cfg)
N = cfg.N;
nZ = 7 * N;
Uidx = 1:(6 * N);
Vidx = (6 * N + 1):nZ;

H(Uidx, Uidx) = H(Uidx, Uidx) + 2 * kron(eye(N), cfg.mpcc.Ru);
Du = deltaMatrix(N, 6);
duPrev = [-uPrev; zeros(6 * (N - 1), 1)];
Rdu = kron(eye(N), cfg.mpcc.Rdu);
H(Uidx, Uidx) = H(Uidx, Uidx) + 2 * (Du' * Rdu * Du);
f(Uidx) = f(Uidx) + 2 * Du' * Rdu * duPrev;

H(Vidx, Vidx) = H(Vidx, Vidx) + 2 * cfg.mpcc.Rvprogress * eye(N);
f(Vidx) = f(Vidx) - 2 * cfg.mpcc.Rvprogress * cfg.mpcc.vprogressRef * ones(N, 1);
Dv = deltaMatrix(N, 1);
dvPrev = [-vprogressPrev; zeros(N - 1, 1)];
H(Vidx, Vidx) = H(Vidx, Vidx) + 2 * cfg.mpcc.Rdvprogress * (Dv' * Dv);
f(Vidx) = f(Vidx) + 2 * cfg.mpcc.Rdvprogress * Dv' * dvPrev;
f(Vidx) = f(Vidx) - cfg.mpcc.progressReward * ones(N, 1);

if cfg.mpcc.progressTrackingWeight > 0
    L = cfg.dt * tril(ones(N));
    nominalProgressErr0 = progress0 - min(progress0 + cfg.dt * cfg.mpcc.vprogressRef * (1:N)', cfg.progressEnd);
    H(Vidx, Vidx) = H(Vidx, Vidx) + 2 * cfg.mpcc.progressTrackingWeight * (L' * L);
    f(Vidx) = f(Vidx) + 2 * cfg.mpcc.progressTrackingWeight * L' * nominalProgressErr0;
end

if cfg.mpcc.progressTerminalWeight > 0
    c = cfg.dt * ones(N, 1);
    terminalTarget = min(progress0 + cfg.dt * cfg.mpcc.vprogressRef * N, cfg.progressEnd);
    terminalErr0 = progress0 - terminalTarget;
    H(Vidx, Vidx) = H(Vidx, Vidx) + 2 * cfg.mpcc.progressTerminalWeight * (c * c');
    f(Vidx) = f(Vidx) + 2 * cfg.mpcc.progressTerminalWeight * terminalErr0 * c;
end
end

function [H, f] = addPathVelocityTracking(H, f, progressPred, mpcc, cfg)
N = cfg.N;
if ~isfield(cfg.mpcc, 'RpathVelocity') || all(cfg.mpcc.RpathVelocity(:) == 0)
    return;
end
for j = 1:N
    idx = blockIdx(j);
    uRef = referenceControlAtProgress(mpcc.refU, progressPred(j), cfg);
    H(idx, idx) = H(idx, idx) + 2 * cfg.mpcc.RpathVelocity;
    f(idx) = f(idx) - 2 * cfg.mpcc.RpathVelocity * uRef;
end
end

function uRef = referenceControlAtProgress(refU, progress, cfg)
if isempty(refU)
    uRef = zeros(6, 1);
    return;
end
alpha = (min(max(progress, cfg.progressStart), cfg.progressEnd) - cfg.progressStart) / ...
    max(cfg.progressEnd - cfg.progressStart, 1e-12);
idx = 1 + floor(alpha * size(refU, 2));
idx = min(max(idx, 1), size(refU, 2));
uRef = refU(:, idx);
end

function [Aineq, bineq] = progressConstraints(progress0, cfg)
N = cfg.N;
Aineq = zeros(1, 7 * N);
Aineq(1, 6 * N + 1) = cfg.dt;
bineq = cfg.progressEnd + cfg.mpcc.progressUpperSlack - progress0;
end

function progressPred = progressPrediction(progress0, vprogressGuess, cfg, location)
progressEnd = progress0 + cfg.dt * cumsum(vprogressGuess(:)');
switch location
    case 'start'
        progressPred = [progress0, progressEnd(1:end - 1)];
    case 'end'
        progressPred = progressEnd;
    otherwise
        error('Unknown progress prediction location: %s', location);
end
progressPred = min(progressPred, cfg.progressEnd);
end

function lb = progressRateLowerBound(progress0, cfg)
remaining = cfg.progressEnd - progress0;
if remaining <= cfg.completionTol
    lb = zeros(cfg.N, 1);
elseif remaining < cfg.N * cfg.dt * cfg.mpcc.progressRateMin
    lb = zeros(cfg.N, 1);
else
    lb = cfg.mpcc.progressRateMin * ones(cfg.N, 1);
end
end

function zShift = shiftMpccGuess(z, N)
U = reshape(z(1:(6 * N)), 6, N);
V = z((6 * N + 1):end)';
U = [U(:, 2:end), U(:, end)];
V = [V(2:end), V(end)];
zShift = [U(:); V(:)];
end

function D = deltaMatrix(N, blockSize)
D = eye(blockSize * N);
for j = 2:N
    rows = ((j - 1) * blockSize + 1):(j * blockSize);
    cols = ((j - 2) * blockSize + 1):((j - 1) * blockSize);
    D(rows, cols) = -eye(blockSize);
end
end

function idx = blockIdx(j)
idx = (6 * (j - 1) + 1):(6 * j);
end
