function P = tangentProjector(p, trajectory)
pathNormal = rmpc.reference.pathNormal(p, trajectory);
P = eye(3) - pathNormal * pathNormal';
end
