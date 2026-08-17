function vBody = bodyVelocityForLocalDisplacement(localDisplacement, omegaBody, dt)
phi = dt * omegaBody;
theta = norm(phi);
Phi = rmpc.geometry.skew(phi);
if theta < 1e-8
    V = eye(3) + 0.5 * Phi + (1 / 6) * Phi * Phi;
else
    V = eye(3) + (1 - cos(theta)) / theta^2 * Phi + ...
        (theta - sin(theta)) / theta^3 * Phi * Phi;
end
rho = V \ localDisplacement;
vBody = rho / dt;
end
