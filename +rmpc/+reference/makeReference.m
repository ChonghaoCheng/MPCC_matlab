function ref = makeReference(progress, trajectory, dt)
if nargin < 3 || isempty(dt)
    dt = 1;
end
n = numel(progress);
ref.p = zeros(3, n);
ref.v = zeros(3, n);
ref.q = zeros(4, n);
ref.T = zeros(4, 4, n);
ref.rpy = zeros(3, n);
ref.progress = progress;
ref.theta = zeros(1, n);
for i = 1:n
    pose = rmpc.reference.referenceAtProgress(progress(i), trajectory);
    ref.p(:, i) = pose.p;
    ref.q(:, i) = pose.q;
    ref.T(:, :, i) = pose.T;
    ref.rpy(:, i) = pose.rpy;
    ref.theta(i) = pose.pathParam;
end
if n >= 2
    ref.v(:, 1:(n - 1)) = diff(ref.p, 1, 2) / dt;
    ref.v(:, n) = ref.v(:, n - 1);
end
end
