function R = expSO3(phi)
theta = norm(phi);
Phi = rmpc.geometry.skew(phi);
if theta < 1e-8
    R = eye(3) + Phi + 0.5 * Phi * Phi;
else
    R = eye(3) + sin(theta) / theta * Phi + (1 - cos(theta)) / theta^2 * Phi * Phi;
end
end
