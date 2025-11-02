function [s_star, p_ref, t, n1, n2] = closest_point_on_path_3d(p, s0, r, r1, r2, S_end)
% CLOSET_POINT_ON_PATH_3D Find arc-length s* minimizing ||p - r(s)|| in 3D
% Robust two-stage grid search around initial guess s0.

if nargin < 6
    error('closest_point_on_path_3d requires p,s0,r,r1,r2,S_end');
end

% coarse search in a window around s0
win1 = 2.0;  % meters of arc-length window
s_lo = max(0, s0 - win1);
s_hi = min(S_end, s0 + win1);
Ns1 = 41;
s_grid = linspace(s_lo, s_hi, Ns1);
d2 = arrayfun(@(s) sum((p - r(s)).^2), s_grid);
[~, idx] = min(d2);
s_best = s_grid(idx);

%  refine in a smaller window around coarse best
step = max((s_hi - s_lo)/(Ns1-1), 1e-3);
win2 = max(5*step, 0.2);
s_lo2 = max(0, s_best - win2);
s_hi2 = min(S_end, s_best + win2);
Ns2 = 51;
s_grid2 = linspace(s_lo2, s_hi2, Ns2);
d2b = arrayfun(@(s) sum((p - r(s)).^2), s_grid2);
[~, idx2] = min(d2b);
s_star = s_grid2(idx2);

% Outputs at s*
[t, n1, n2, ~, p_ref] = geom3_at(s_star, r, r1, r2);

end











