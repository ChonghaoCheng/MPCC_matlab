function pathNormal = pathNormal(p, trajectory)
if nargin >= 2 && isfield(trajectory, 'pathType') && any(strcmpi(trajectory.pathType, {'line3d', 'line'}))
    direction = trajectory.lineDirection(:);
    direction = direction / max(norm(direction), 1e-12);
    pathNormal = trajectory.preferredNormal(:);
    pathNormal = pathNormal - direction * (direction' * pathNormal);
    if norm(pathNormal) < 1e-9
        pathNormal = [1; 0; 0] - direction * direction(1);
    end
    pathNormal = pathNormal / max(norm(pathNormal), 1e-12);
    return;
end
r = norm(p(1:2));
if r < 1e-9
    pathNormal = [1; 0; 0];
else
    pathNormal = [p(1) / r; p(2) / r; 0];
end
end
