function [refDisturbed, trajectoryDisturbed, disturbance] = applyDisturbanceProfile(refNominal, trajectoryNominal, profile, cfg)
% Apply the configured progress-indexed disturbance to the reference path.
%
% The nominal path is represented in the trajectory frame. The returned
% sampled trajectory is the world-frame path after incremental disturbances.
n = numel(refNominal.progress);
Tdist = zeros(4, 4, n);
disturbance = zeros(6, max(n - 1, 0));

Tdist(:, :, 1) = refNominal.T(:, :, 1);
state = rmpc.simulation.initDisturbanceState();

for k = 1:(n - 1)
    [d, state] = rmpc.simulation.sampleProgressDisturbance( ...
        profile, state, refNominal.progress(k), refNominal.progress(k + 1), cfg.enableDisturbance);
    Trel = rmpc.geometry.invSE3(refNominal.T(:, :, k)) * refNominal.T(:, :, k + 1);
    Tnext = Tdist(:, :, k) * Trel * rmpc.geometry.expSE3(d);
    Tnext(1:3, 1:3) = rmpc.geometry.projectSO3(Tnext(1:3, 1:3));
    Tdist(:, :, k + 1) = Tnext;
    disturbance(:, k) = d;
end

refDisturbed = refNominal;
refDisturbed.p = zeros(3, n);
refDisturbed.q = zeros(4, n);
refDisturbed.T = Tdist;
refDisturbed.rpy = zeros(3, n);
for k = 1:n
    R = Tdist(1:3, 1:3, k);
    refDisturbed.p(:, k) = Tdist(1:3, 4, k);
    refDisturbed.q(:, k) = rmpc.geometry.rotmToQuat(R);
    refDisturbed.rpy(:, k) = rmpc.geometry.rotmToRpy(R);
end
if n >= 2
    refDisturbed.v(:, 1:(n - 1)) = diff(refDisturbed.p, 1, 2) / cfg.dt;
    refDisturbed.v(:, n) = refDisturbed.v(:, n - 1);
end
refDisturbed.disturbance = disturbance;

trajectoryDisturbed = trajectoryNominal;
trajectoryDisturbed.pathType = 'sampled';
trajectoryDisturbed.pathParamStart = cfg.progressStart;
trajectoryDisturbed.pathParamEnd = cfg.progressEnd;
trajectoryDisturbed.sampledProgress = refNominal.progress;
trajectoryDisturbed.sampledT = Tdist;
[progressUnique, uniqueIdx] = unique(refNominal.progress, 'stable');
if numel(progressUnique) >= 2
    trajectoryDisturbed.sampledPositionPp = pchip(progressUnique, refDisturbed.p(:, uniqueIdx));
end
trajectoryDisturbed.nominalTrajectory = trajectoryNominal;
end
