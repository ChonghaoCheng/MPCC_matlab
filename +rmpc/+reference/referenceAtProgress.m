function ref = referenceAtProgress(progress, trajectory)
progress = min(max(progress, 0), 1);
if isfield(trajectory, 'sampledT') && isfield(trajectory, 'sampledProgress')
    ref = sampledReferenceAtProgress(progress, trajectory);
    return;
end
theta = rmpc.reference.progressToPathParam(progress, trajectory);
ref = rmpc.reference.referenceAtTheta(theta, trajectory);
ref.progress = progress;
ref.pathParam = theta;
end

function ref = sampledReferenceAtProgress(progress, trajectory)
grid = trajectory.sampledProgress(:)';
progress = min(max(progress, grid(1)), grid(end));
idxHi = find(grid >= progress, 1, 'first');
if isempty(idxHi)
    idxHi = numel(grid);
end
if idxHi <= 1
    idxLo = 1;
    alpha = 0;
else
    idxLo = idxHi - 1;
    denom = max(grid(idxHi) - grid(idxLo), 1e-12);
    alpha = (progress - grid(idxLo)) / denom;
end

T0 = trajectory.sampledT(:, :, idxLo);
T1 = trajectory.sampledT(:, :, idxHi);
if isfield(trajectory, 'sampledPositionPp')
    p = ppval(trajectory.sampledPositionPp, progress);
    p = p(:);
else
    p = (1 - alpha) * T0(1:3, 4) + alpha * T1(1:3, 4);
end
q0 = rmpc.geometry.rotmToQuat(T0(1:3, 1:3));
q1 = rmpc.geometry.rotmToQuat(T1(1:3, 1:3));
q = quatSlerp(q0, q1, alpha);
R = rmpc.geometry.quatToRotm(q);

ref.p = p;
ref.R = R;
ref.T = rmpc.geometry.makeTransform(R, p);
ref.q = q;
ref.rpy = rmpc.geometry.rotmToRpy(R);
ref.progress = progress;
ref.pathParam = progress;
end

function q = quatSlerp(q0, q1, alpha)
q0 = rmpc.geometry.quatNormalize(q0);
q1 = rmpc.geometry.quatNormalize(q1);
dotq = q0' * q1;
if dotq < 0
    q1 = -q1;
    dotq = -dotq;
end
if dotq > 0.9995
    q = rmpc.geometry.quatNormalize((1 - alpha) * q0 + alpha * q1);
    return;
end
theta = acos(min(max(dotq, -1), 1));
s = sin(theta);
q = (sin((1 - alpha) * theta) / s) * q0 + (sin(alpha * theta) / s) * q1;
q = rmpc.geometry.quatNormalize(q);
end
