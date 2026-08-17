function R = projectSO3(R)
[U, ~, V] = svd(R);
R = U * V';
if det(R) < 0
    U(:, 3) = -U(:, 3);
    R = U * V';
end
end
