function p = clamp_pitch(p)
% CLAMP_PITCH 限制俯仰角在合理范围内
% 输入/输出:
%   p - 俯仰角 [rad]

p = max(min(p, pi/2-1e-3), -pi/2+1e-3);

end

