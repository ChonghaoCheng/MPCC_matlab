function xi = logSE3(T)
R = T(1:3, 1:3);
p = T(1:3, 4);
phi = rmpc.geometry.logSO3(R);
theta = norm(phi);
Phi = rmpc.geometry.skew(phi);
if theta < 1e-6
    Vinv = eye(3) - 0.5 * Phi + (1 / 12) * Phi * Phi;
else
    A = sin(theta) / theta;
    B = (1 - cos(theta)) / theta^2;
    if abs(B) < 1e-12
        Vinv = eye(3) - 0.5 * Phi + (1 / 12) * Phi * Phi;
    else
        Vinv = eye(3) - 0.5 * Phi + (1 / theta^2) * (1 - A / (2 * B)) * Phi * Phi;
    end
end
xi = [Vinv * p; phi];
end
