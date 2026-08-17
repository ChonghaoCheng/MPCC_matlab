function uShift = shiftWarmStart(u, N, uAppend)
U = reshape(u, 6, N);
if nargin < 3
    uAppend = U(:, end);
end
U = [U(:, 2:end), uAppend];
uShift = U(:);
end
