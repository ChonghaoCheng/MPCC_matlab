function err = mpccTrackingErrors(p, q, progress, trajectory)
ref = rmpc.reference.referenceAtProgress(progress, trajectory);
R = rmpc.geometry.quatToRotm(q);

ep = p - ref.p;
tauTwist = rmpc.reference.referenceTangentTwist(progress, trajectory);
tau = ref.R * tauTwist(1:3);
tau = tau / max(norm(tau), 1e-12);

err.lag = tau' * ep;
err.contour = ep - tau * err.lag;
err.contourNorm = norm(err.contour);
err.lagAbs = abs(err.lag);
err.rotation = rmpc.geometry.logSO3(ref.R' * R);
err.rotationNorm = norm(err.rotation);
err.positionNorm = norm(ep);
err.se3 = rmpc.geometry.logSE3(rmpc.geometry.invSE3(ref.T) * rmpc.geometry.makeTransform(R, p));
err.se3Norm = norm(err.se3);
end
