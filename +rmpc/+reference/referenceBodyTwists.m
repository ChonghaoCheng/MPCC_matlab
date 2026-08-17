function U = referenceBodyTwists(ref, dt)
n = size(ref.p, 2) - 1;
U = zeros(6, n);
for k = 1:n
    U(:, k) = rmpc.geometry.logSE3(rmpc.geometry.invSE3(ref.T(:, :, k)) * ref.T(:, :, k + 1)) / dt;
end
end
