function a = toolAxisAngle(q, trajectory, p)
if nargin < 3
    p = [trajectory.radius; 0; 0];
end
toolAxis = rmpc.geometry.quatToRotm(q) * trajectory.toolAxisBody;
pathNormal = rmpc.reference.pathNormal(p, trajectory);
c = min(1, max(-1, dot(toolAxis, pathNormal)));
a = acos(c);
end
