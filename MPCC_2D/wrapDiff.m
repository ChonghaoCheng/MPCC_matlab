function d = wrapDiff(a, b)
% wrapped angle difference a-b into (-pi, pi]
    d = atan2(sin(a - b), cos(a - b));
end