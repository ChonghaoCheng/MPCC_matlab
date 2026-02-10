function [t, n1, n2, kappa_t, p, rt, rt2] = geom3_at(s, r, r1, r2)
% GEOM3_AT Compute 3D path geometry at arc-length s
% Inputs:
%   s:   arc length
%   r:   position function r(s) in R^3
%   r1:  first derivative r'(s)
%   r2:  second derivative r''(s)
% Outputs:
%   t:       unit tangent at s (3x1)
%   n1,n2:   an orthonormal basis of the normal plane at s (3x1 each)
%   kappa_t: curvature magnitude along t (scalar)
%   p:       position r(s)
%   rt:      r'(s)
%   rt2:     r''(s)

p   = r(s);
rt  = r1(s);
rt2 = r2(s);

L1 = norm(rt);
if L1 < 1e-12
    % Degenerate derivative; fall back to canonical frame
    t = [1;0;0];
    n1 = [0;1;0];
    n2 = [0;0;1];
    kappa_t = 0;
    return;
end

t = rt / L1;

% Curvature magnitude using |r' x r''| / |r'|^3
cross_r = cross(rt, rt2);
kappa_t = norm(cross_r) / max(L1^3, 1e-12);

% Construct a stable normal frame in the plane orthogonal to t
acc_perp = rt2 - (t' * rt2) * t;
if norm(acc_perp) > 1e-12
    n1 = acc_perp / norm(acc_perp);
else
    % Fallback: pick any vector not parallel to t
    [~, idx] = max(abs(t));
    e = zeros(3,1); e(mod(idx,3)+1) = 1; % choose a different axis
    v = e - (t'*e)*t;
    if norm(v) < 1e-12
        e = zeros(3,1); e(mod(idx+1,3)+1) = 1;
        v = e - (t'*e)*t;
    end
    n1 = v / max(norm(v), 1e-12);
end

n2 = cross(t, n1);
% Normalize for numerical robustness
n1 = n1 / max(norm(n1), 1e-12);
n2 = n2 / max(norm(n2), 1e-12);

end








