function U = referenceTranslationControls(ref)
n = size(ref.p, 2) - 1;
U = zeros(6, n);
if ~isfield(ref, 'v')
    error('rmpc:missingReferenceVelocity', ...
        'Reference must contain ref.v. Build it with makeReference(..., dt).');
end
for k = 1:n
    Rref = rmpc.geometry.quatToRotm(ref.q(:, k));
    U(1:3, k) = Rref' * ref.v(:, k);
end
end
