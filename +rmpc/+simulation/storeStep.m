function hist = storeStep(hist, p, q, u, cInfo, k, trajectory, progress, progressRef, pPre, qPre, vprogress, vprogressRef)
if nargin < 8
    progress = NaN;
end
if nargin < 9
    progressRef = progress;
end
if nargin < 10 || isempty(pPre)
    pPre = p;
end
if nargin < 11 || isempty(qPre)
    qPre = q;
end
if nargin < 12 || isempty(vprogress)
    vprogress = NaN;
end
if nargin < 13 || isempty(vprogressRef)
    vprogressRef = NaN;
end
hist.p(:, k + 1) = p;
hist.pPre(:, k + 1) = pPre;
hist.q(:, k + 1) = q;
hist.qPre(:, k + 1) = qPre;
hist.u(:, k) = u;
hist.diagnostic(k) = cInfo.diagnostic;
hist.toolAxisAngle(k + 1) = rmpc.reference.toolAxisAngle(q, trajectory, p);
hist.progress(k + 1) = progress;
hist.progressRef(k + 1) = progressRef;
hist.vprogress(k) = vprogress;
hist.vprogressRef(k) = vprogressRef;
end
