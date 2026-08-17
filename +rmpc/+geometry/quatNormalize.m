function q = quatNormalize(q)
q = q / max(norm(q), 1e-12);
if q(1) < 0, q = -q; end
end
