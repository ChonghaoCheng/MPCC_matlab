function ref = referenceAtTheta(theta, trajectory)
pathType = 'spiral';
if isfield(trajectory, 'pathType') && ~isempty(trajectory.pathType)
    pathType = trajectory.pathType;
end

switch lower(pathType)
    case {'spiral', 'helix'}
        [p, tangent, normal] = spiralPoseData(theta, trajectory);
    case {'line3d', 'line'}
        [p, tangent, normal] = linePoseData(theta, trajectory);
    case {'sine3d', 'sine'}
        [p, tangent, normal] = sinePoseData(theta, trajectory);
    case {'flower3d', 'flower'}
        [p, tangent, normal] = flowerPoseData(theta, trajectory);
    case {'lissajous3d', 'lissajous'}
        [p, tangent, normal] = lissajousPoseData(theta, trajectory);
    case {'trefoil3d', 'trefoil'}
        [p, tangent, normal] = trefoilPoseData(theta, trajectory);
    case {'wavy_spiral', 'wavyspiral'}
        [p, tangent, normal] = wavySpiralPoseData(theta, trajectory);
    otherwise
        error('Unknown reference path type: %s', pathType);
end

R = frameFromTangentNormal(tangent, normal);
ref.p = p;
ref.R = R;
ref.T = rmpc.geometry.makeTransform(R, p);
ref.q = rmpc.geometry.rotmToQuat(R);
ref.rpy = rmpc.geometry.rotmToRpy(R);
end

function [p, tangent, normal] = spiralPoseData(theta, trajectory)
p = [trajectory.radius * cos(theta);
     trajectory.radius * sin(theta);
     trajectory.z0 + trajectory.pitch * theta];
tangent = [-trajectory.radius * sin(theta);
            trajectory.radius * cos(theta);
            trajectory.pitch];
normal = [cos(theta); sin(theta); 0];
end

function [p, tangent, normal] = linePoseData(theta, trajectory)
direction = trajectory.lineDirection(:);
direction = direction / max(norm(direction), 1e-12);
scale = trajectory.lineScale;
p = trajectory.lineP0(:) + scale * theta * direction;
tangent = scale * direction;

normal = trajectory.preferredNormal(:);
normal = normal - direction * (direction' * normal);
if norm(normal) < 1e-9
    normal = [1; 0; 0] - direction * direction(1);
end
normal = normal / max(norm(normal), 1e-12);
end

function [p, tangent, normal] = sinePoseData(theta, trajectory)
x = trajectory.sineP0(1) + trajectory.sineLengthScale * theta;
y = trajectory.sineP0(2) + trajectory.sineAmpY * sin(trajectory.sineFreqY * theta);
z = trajectory.sineP0(3) + trajectory.sineAmpZ * sin(trajectory.sineFreqZ * theta + trajectory.sinePhaseZ);
p = [x; y; z];

tangent = [trajectory.sineLengthScale;
           trajectory.sineAmpY * trajectory.sineFreqY * cos(trajectory.sineFreqY * theta);
           trajectory.sineAmpZ * trajectory.sineFreqZ * cos(trajectory.sineFreqZ * theta + trajectory.sinePhaseZ)];
normal = trajectory.preferredNormal(:);
end

function [p, tangent, normal] = flowerPoseData(theta, trajectory)
petals = trajectory.flowerPetals;
r = trajectory.flowerBaseRadius + trajectory.flowerAmp * cos(petals * theta);
dr = -trajectory.flowerAmp * petals * sin(petals * theta);

p = [r * cos(theta);
     r * sin(theta);
     trajectory.flowerZ0 + trajectory.flowerPitch * theta];

tangent = [dr * cos(theta) - r * sin(theta);
           dr * sin(theta) + r * cos(theta);
           trajectory.flowerPitch];

radial = [cos(theta); sin(theta); 0];
normal = radial;
end

function [p, tangent, normal] = lissajousPoseData(theta, trajectory)
amp = trajectory.lissajousAmp(:);
freq = trajectory.lissajousFreq(:);
phase = trajectory.lissajousPhase(:);
center = trajectory.lissajousCenter(:);

arg = freq * theta + phase;
p = center + amp .* sin(arg);
tangent = amp .* freq .* cos(arg);
normal = externalPointingNormal(p, tangent, center);
end

function [p, tangent, normal] = trefoilPoseData(theta, trajectory)
s = trajectory.trefoilScale;
zScale = trajectory.trefoilZScale;
center = trajectory.trefoilCenter(:);

x = s * (sin(theta) + 2 * sin(2 * theta));
y = s * (cos(theta) - 2 * cos(2 * theta));
z = zScale * (-sin(3 * theta));
p = center + [x; y; z];

dx = s * (cos(theta) + 4 * cos(2 * theta));
dy = s * (-sin(theta) + 4 * sin(2 * theta));
dz = zScale * (-3 * cos(3 * theta));
tangent = [dx; dy; dz];
normal = externalPointingNormal(p, tangent, center);
end

function [p, tangent, normal] = wavySpiralPoseData(theta, trajectory)
r = trajectory.wavyRadius + trajectory.wavyRadiusAmp * sin(trajectory.wavyRadiusFreq * theta);
dr = trajectory.wavyRadiusAmp * trajectory.wavyRadiusFreq * cos(trajectory.wavyRadiusFreq * theta);
z = trajectory.wavyZ0 + trajectory.wavyPitch * theta + trajectory.wavyZAmp * sin(trajectory.wavyZFreq * theta);
dz = trajectory.wavyPitch + trajectory.wavyZAmp * trajectory.wavyZFreq * cos(trajectory.wavyZFreq * theta);

p = [r * cos(theta); r * sin(theta); z];
tangent = [dr * cos(theta) - r * sin(theta);
           dr * sin(theta) + r * cos(theta);
           dz];
normal = [cos(theta); sin(theta); 0];
end

function normal = externalPointingNormal(p, tangent, center)
normal = p - center(:);
normal = normal - tangent * ((tangent' * normal) / max(tangent' * tangent, 1e-12));
if norm(normal) < 1e-9
    normal = fallbackNormal(tangent / max(norm(tangent), 1e-12));
end
normal = normal / max(norm(normal), 1e-12);
end

function R = frameFromTangentNormal(tangent, normal)
xAxis = tangent / max(norm(tangent), 1e-12);
zAxis = normal - xAxis * (xAxis' * normal);
if norm(zAxis) < 1e-9
    zAxis = fallbackNormal(xAxis);
end
zAxis = zAxis / max(norm(zAxis), 1e-12);
yAxis = cross(zAxis, xAxis);
yAxis = yAxis / max(norm(yAxis), 1e-12);
R = [xAxis, yAxis, zAxis];
end

function normal = fallbackNormal(tangent)
candidates = eye(3);
[~, idx] = min(abs(candidates' * tangent));
normal = candidates(:, idx) - tangent * (tangent' * candidates(:, idx));
end
