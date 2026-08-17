function out = sliceReference(ref, idx)
out.p = ref.p(:, idx);
if isfield(ref, 'v')
    out.v = ref.v(:, idx);
end
out.q = ref.q(:, idx);
out.T = ref.T(:, :, idx);
out.rpy = ref.rpy(:, idx);
if isfield(ref, 'progress')
    out.progress = ref.progress(idx);
end
if isfield(ref, 'theta')
    out.theta = ref.theta(idx);
end
end
