function e = quatLogError(qRef, q)
qErr = rmpc.geometry.quatMultiply(rmpc.geometry.quatConj(qRef), q);
if qErr(1) < 0, qErr = -qErr; end
v = qErr(2:4);
nv = norm(v);
w = min(1, max(-1, qErr(1)));
if nv < 1e-10
    e = 2 * v;
else
    e = 2 * atan2(nv, w) * v / nv;
end
end
