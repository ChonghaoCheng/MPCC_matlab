function report = mathSanityCheck(cfg)
ref = rmpc.reference.makeReference(cfg.progressGrid, cfg.trajectory, cfg.dt);
U = rmpc.reference.referenceBodyTwists(ref, cfg.dt);

quatState.p = ref.p(:, 1);
quatState.q = ref.q(:, 1);
lieState.T = ref.T(:, :, 1);

n = size(U, 2);
quatPosErr = zeros(1, n);
quatAttErr = zeros(1, n);
liePosErr = zeros(1, n);
lieAttErr = zeros(1, n);
quatPathDiagnostic = zeros(1, n);
liePathDiagnostic = zeros(1, n);
maxControlRatio = 0;

for k = 1:n
    u = U(:, k);
    quatState = rmpc.simulation.stepQuatState(quatState, u, cfg.dt);
    lieState = rmpc.simulation.stepLieState(lieState, u, cfg.dt);

    qLie = rmpc.geometry.rotmToQuat(lieState.T(1:3, 1:3));
    quatPosErr(k) = norm(quatState.p - ref.p(:, k + 1));
    quatAttErr(k) = norm(rmpc.geometry.quatLogError(ref.q(:, k + 1), quatState.q));
    liePosErr(k) = norm(lieState.T(1:3, 4) - ref.p(:, k + 1));
    lieAttErr(k) = norm(rmpc.geometry.quatLogError(ref.q(:, k + 1), qLie));
    quatPathDiagnostic(k) = abs(rmpc.reference.pathDiagnosticDistance(quatState.p, cfg.trajectory));
    liePathDiagnostic(k) = abs(rmpc.reference.pathDiagnosticDistance(lieState.T(1:3, 4), cfg.trajectory));

    ratio = max(abs(u(1:3)) ./ cfg.limits.vMax);
    ratio = max(ratio, max(abs(u(4:6)) ./ cfg.limits.wMax));
    maxControlRatio = max(maxControlRatio, ratio);
end

report.maxQuatPosErr = max(quatPosErr);
report.maxQuatAttErr = max(quatAttErr);
report.maxLiePosErr = max(liePosErr);
report.maxLieAttErr = max(lieAttErr);
report.maxQuatPathDiagnostic = max(quatPathDiagnostic);
report.maxLiePathDiagnostic = max(liePathDiagnostic);
report.maxControlLimitRatio = maxControlRatio;
report.pass = report.maxQuatPosErr < 1e-10 && report.maxLiePosErr < 1e-10 && ...
    report.maxQuatAttErr < 1e-10 && report.maxLieAttErr < 1e-10 && maxControlRatio <= 1.0;
end
