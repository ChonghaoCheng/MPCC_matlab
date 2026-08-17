function theta = progressToPathParam(progress, trajectory)
[thetaStart, thetaEnd] = pathParamBounds(trajectory);
theta = thetaStart + progress .* (thetaEnd - thetaStart);
end

function [thetaStart, thetaEnd] = pathParamBounds(trajectory)
if isfield(trajectory, 'pathParamStart')
    thetaStart = trajectory.pathParamStart;
else
    thetaStart = 0;
end
if isfield(trajectory, 'pathParamEnd')
    thetaEnd = trajectory.pathParamEnd;
else
    thetaEnd = 2 * pi;
end
end
