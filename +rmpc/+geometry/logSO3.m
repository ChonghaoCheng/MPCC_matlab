function phi = logSO3(R)
R = rmpc.geometry.projectSO3(R);
c = (trace(R) - 1) / 2;
c = min(1, max(-1, c));
theta = acos(c);
if theta < 1e-8
    phi = rmpc.geometry.vee(0.5 * (R - R'));
elseif abs(pi - theta) < 1e-5
    A = (R + eye(3)) / 2;
    axis = sqrt(max(diag(A), 0));
    [~, idx] = max(axis);
    v = A(:, idx);
    if norm(v) < 1e-8, v = axis; end
    phi = theta * v / max(norm(v), 1e-12);
else
    phi = theta / (2 * sin(theta)) * rmpc.geometry.vee(R - R');
end
end
