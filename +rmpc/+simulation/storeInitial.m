function hist = storeInitial(hist, p, q, trajectory, progress, progressRef)
if nargin < 5
    progress = 0;
end
if nargin < 6
    progressRef = progress;
end
hist.p(:, 1) = p;
hist.pPre(:, 1) = p;
hist.q(:, 1) = q;
hist.qPre(:, 1) = q;
hist.toolAxisAngle(1) = rmpc.reference.toolAxisAngle(q, trajectory, p);
hist.progress(1) = progress;
hist.progressRef(1) = progressRef;
end
