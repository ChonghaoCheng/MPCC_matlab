function profile = makeDisturbanceProfile(cfg)
params = cfg.disturbance;
profile.type = params.type;
profile.eventProgress = [];
profile.bursts = {};
profile.burstProgressStep = cfg.referenceProgressRate * cfg.dt;

switch disturbanceGenerator(params)
    case 'progress_arma21'
        profile.errorType = disturbanceType(params);
        eventEnd = cfg.progressEnd - endExclusionProgress(params);
        eventEnd = max(cfg.progressStart, eventEnd - params.excludeEndTol);
        eventProgress = params.progressInterval:params.progressInterval:eventEnd;
        eventProgress = eventProgress(eventProgress < eventEnd);
        profile.eventProgress = eventProgress;
        for i = 1:numel(eventProgress)
            profile.bursts{i} = disturbanceSign(params, i) * makeArmaBurst(params); %#ok<AGROW>
        end
    case 'none'
        profile.errorType = 'none';
        return;
    otherwise
        error('Unknown disturbance type: %s', params.type);
end
end

function margin = endExclusionProgress(params)
if isfield(params, 'endExclusionProgress')
    margin = max(0, params.endExclusionProgress);
else
    margin = 0;
end
end

function burst = makeArmaBurst(params)
burstSteps = params.burstSteps;
sigma = [params.transSigma(:); params.rotSigma(:)];
limits = [params.maxTrans(:); params.maxRot(:)];
mask = disturbanceMask(params);
innov = sigma .* randn(6, burstSteps);
burst = zeros(6, burstSteps);
epsPrev = zeros(6, 1);
yPrev1 = zeros(6, 1);
yPrev2 = zeros(6, 1);

for k = 1:burstSteps
    eps = innov(:, k);
    [armaA, armaB] = armaCoefficients(params);
    burst(:, k) = armaA(1) * yPrev1 + armaA(2) * yPrev2 + eps + armaB * epsPrev;
    burst(:, k) = min(max(burst(:, k), -limits), limits) .* mask;
    yPrev2 = yPrev1;
    yPrev1 = burst(:, k);
    epsPrev = eps;
end

burst = burst .* burstWindow(burstSteps, params);
end

function s = disturbanceSign(params, eventIdx)
pattern = 'alternating';
if isfield(params, 'signPattern')
    pattern = lower(char(params.signPattern));
end
switch pattern
    case 'alternating'
        s = 2 * mod(eventIdx, 2) - 1;
    case {'same', 'positive'}
        s = 1;
    case 'negative'
        s = -1;
    case 'random'
        s = 2 * (rand > 0.5) - 1;
    otherwise
        error('Unknown disturbance signPattern: %s', pattern);
end
end

function mask = disturbanceMask(params)
switch disturbanceType(params)
    case 'arma'
        mask = ones(6, 1);
    case 'translation_only'
        mask = [ones(3, 1); zeros(3, 1)];
    case 'rotation_only'
        mask = [zeros(3, 1); ones(3, 1)];
    case 'translation_rotation'
        mask = ones(6, 1);
    otherwise
        error('Unknown disturbance type: %s', params.type);
end
end

function type = disturbanceType(params)
type = lower(char(params.type));
switch type
    case {'arma', 'progress_arma21'}
        type = 'arma';
    case {'translation', 'trans', 'translation_only'}
        type = 'translation_only';
    case {'rotation', 'rot', 'rotation_only'}
        type = 'rotation_only';
    case {'both', 'translation_rotation', 'translation_and_rotation'}
        type = 'translation_rotation';
    otherwise
        error('Unknown disturbance type: %s', type);
end
end

function generator = disturbanceGenerator(params)
type = lower(char(params.type));
switch type
    case {'none'}
        generator = 'none';
    case {'arma', 'progress_arma21', 'translation', 'trans', 'translation_only', ...
            'rotation', 'rot', 'rotation_only', ...
            'translation_rotation', 'translation_and_rotation'}
        generator = 'progress_arma21';
    otherwise
        error('Unknown disturbance type: %s', params.type);
end
end

function [armaA, armaB] = armaCoefficients(params)
if isfield(params, 'armaA')
    armaA = params.armaA;
else
    armaA = [0.05, 0.02];
end
if isfield(params, 'armaB')
    armaB = params.armaB;
else
    armaB = 0.05;
end
end

function w = burstWindow(n, params)
idx = 1:n;
windowType = 'sine';
if isfield(params, 'burstWindow')
    windowType = lower(char(params.burstWindow));
end
switch windowType
    case 'sine'
        w = sin(pi * idx / (n + 1));
    case 'hann'
        if n == 1
            w = 1;
        else
            w = 0.5 - 0.5 * cos(2 * pi * (idx - 1) / (n - 1));
        end
    case {'rect', 'boxcar', 'none'}
        w = ones(1, n);
    otherwise
        error('Unknown disturbance burstWindow: %s', windowType);
end
end
