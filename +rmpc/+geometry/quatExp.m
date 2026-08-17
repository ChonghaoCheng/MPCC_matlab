function q = quatExp(phi)
theta = norm(phi);
if theta < 1e-10
    q = [1; 0.5 * phi];
else
    axis = phi / theta;
    q = [cos(theta / 2); axis * sin(theta / 2)];
end
q = rmpc.geometry.quatNormalize(q);
end
