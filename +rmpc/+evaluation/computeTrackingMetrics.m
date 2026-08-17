function metrics = computeTrackingMetrics(hist, ref, trajectory, progressEval)
n = size(hist.p, 2);
if nargin < 4 || isempty(progressEval)
    if isfield(hist, 'progress') && any(isfinite(hist.progress))
        progressEval = hist.progress;
    elseif isfield(ref, 'progress')
        progressEval = ref.progress;
    else
        progressEval = linspace(0, 1, n);
    end
end
progressEval = min(max(progressEval, 0), 1);
metrics.posErr = zeros(1, n);
metrics.tangentialErr = zeros(1, n);
metrics.attErr = zeros(1, n);
metrics.toolAxisAngle = zeros(1, n);
metrics.contourErr = zeros(1, n);
metrics.lagErr = zeros(1, n);
metrics.signedLagErr = zeros(1, n);
metrics.rotErr = zeros(1, n);
metrics.se3Err = zeros(1, n);
for i = 1:n
    refAtProgress = rmpc.reference.referenceAtProgress(progressEval(i), trajectory);
    ep = hist.p(:, i) - refAtProgress.p;
    metrics.posErr(i) = norm(ep);
    metrics.tangentialErr(i) = norm(rmpc.reference.tangentProjector(refAtProgress.p, trajectory) * ep);
    metrics.attErr(i) = norm(rmpc.geometry.quatLogError(refAtProgress.q, hist.q(:, i)));
    metrics.toolAxisAngle(i) = rmpc.reference.toolAxisAngle(hist.q(:, i), trajectory, hist.p(:, i));
    err = rmpc.evaluation.mpccTrackingErrors(hist.p(:, i), hist.q(:, i), progressEval(i), trajectory);
    metrics.contourErr(i) = err.contourNorm;
    metrics.lagErr(i) = err.lagAbs;
    metrics.signedLagErr(i) = err.lag;
    metrics.rotErr(i) = err.rotationNorm;
    metrics.se3Err(i) = err.se3Norm;
end
metrics.rmsPos = sqrt(mean(metrics.posErr .^ 2));
metrics.rmsTangential = sqrt(mean(metrics.tangentialErr .^ 2));
metrics.rmsAtt = sqrt(mean(metrics.attErr .^ 2));
metrics.rmsToolAxisAngle = sqrt(mean(metrics.toolAxisAngle .^ 2));
metrics.rmsContour = sqrt(mean(metrics.contourErr .^ 2));
metrics.rmsLag = sqrt(mean(metrics.lagErr .^ 2));
metrics.rmsRot = sqrt(mean(metrics.rotErr .^ 2));
metrics.rmsSe3 = sqrt(mean(metrics.se3Err .^ 2));
metrics.maxContour = max(metrics.contourErr);
metrics.maxLag = max(metrics.lagErr);
metrics.maxRot = max(metrics.rotErr);
metrics.maxSe3 = max(metrics.se3Err);
metrics.finalProgress = progressEval(end);
metrics.completionRatio = min(max(progressEval(end), 0), 1);
if isfield(hist, 'u') && ~isempty(hist.u)
    metrics.rmsControl = sqrt(mean(sum(hist.u .^ 2, 1)));
    if size(hist.u, 2) >= 2
        du = diff(hist.u, 1, 2);
        metrics.rmsControlRate = sqrt(mean(sum(du .^ 2, 1)));
    else
        metrics.rmsControlRate = 0;
    end
else
    metrics.rmsControl = 0;
    metrics.rmsControlRate = 0;
end
end
