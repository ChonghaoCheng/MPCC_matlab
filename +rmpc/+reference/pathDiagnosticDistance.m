function sd = pathDiagnosticDistance(p, trajectory)
if isfield(trajectory, 'pathType') && any(strcmpi(trajectory.pathType, {'line3d', 'line'}))
    direction = trajectory.lineDirection(:);
    direction = direction / max(norm(direction), 1e-12);
    rel = p - trajectory.lineP0(:);
    closest = trajectory.lineP0(:) + direction * (direction' * rel);
    sd = norm(p - closest);
    return;
end
sd = norm(p(1:2)) - trajectory.radius;
end
