function u = saturateControl(u, limits)
u(1:3) = min(max(u(1:3), -limits.vMax), limits.vMax);
u(4:6) = min(max(u(4:6), -limits.wMax), limits.wMax);
end
