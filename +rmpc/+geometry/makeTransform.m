function T = makeTransform(R, p)
T = eye(4);
T(1:3, 1:3) = R;
T(1:3, 4) = p;
end
