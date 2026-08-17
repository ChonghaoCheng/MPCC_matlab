function xi = referenceTangentTwist(progress, trajectory)
h = tangentStep(trajectory);
progress1 = min(progress + h, 1);
if progress1 == progress
    progress0 = max(progress - h, 0);
    T0 = rmpc.reference.referenceAtProgress(progress0, trajectory).T;
    T1 = rmpc.reference.referenceAtProgress(progress, trajectory).T;
    hEff = progress - progress0;
else
    T0 = rmpc.reference.referenceAtProgress(progress, trajectory).T;
    T1 = rmpc.reference.referenceAtProgress(progress1, trajectory).T;
    hEff = progress1 - progress;
end
if hEff < 1e-12
    xi = zeros(6, 1);
    return;
end
xi = rmpc.geometry.logSE3(rmpc.geometry.invSE3(T0) * T1) / hEff;
end

function h = tangentStep(trajectory)
h = 1e-5;
if isfield(trajectory, 'sampledProgress')
    grid = trajectory.sampledProgress(:);
    if numel(grid) >= 2
        spacing = median(diff(grid));
        h = max(1e-5, 0.25 * spacing);
    end
end
end
