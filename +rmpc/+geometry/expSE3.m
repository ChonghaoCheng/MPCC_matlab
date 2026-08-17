function T = expSE3(xi)
rho = xi(1:3);
phi = xi(4:6);
theta = norm(phi);
Phi = rmpc.geometry.skew(phi);
R = rmpc.geometry.expSO3(phi);
if theta < 1e-8
    V = eye(3) + 0.5 * Phi + (1 / 6) * Phi * Phi;
else
    V = eye(3) + (1 - cos(theta)) / theta^2 * Phi + (theta - sin(theta)) / theta^3 * Phi * Phi;
end
T = rmpc.geometry.makeTransform(R, V * rho);
end
