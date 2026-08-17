function disturbance = makeDisturbance(progressGrid, cfg)
nSteps = min(cfg.nSteps, numel(progressGrid) - 1);
progressGrid = progressGrid(1:(nSteps + 1));
disturbance = zeros(6, nSteps);
if nSteps <= 0
    return;
end

params = cfg.disturbance;
switch disturbanceGenerator(params)
    case 'progress_arma21'
        eventSteps = progressEventSteps(progressGrid, params, cfg);
        for i = 1:numel(eventSteps)
            signFlip = 2 * mod(i, 2) - 1;
            disturbance = addArmaBurst(disturbance, eventSteps(i), signFlip, params);
        end
    case 'none'
        return;
    otherwise
        error('Unknown disturbance type: %s', params.type);
end
end

function eventSteps = progressEventSteps(progressGrid, params, cfg)
eventEnd = cfg.progressEnd - endExclusionProgress(params);
eventEnd = max(cfg.progressStart, eventEnd - params.excludeEndTol);
eventProgress = params.progressInterval:params.progressInterval:eventEnd;
eventProgress = eventProgress(eventProgress < eventEnd);
eventSteps = zeros(1, numel(eventProgress));
for i = 1:numel(eventProgress)
    idx = find(progressGrid(1:end - 1) < eventProgress(i) & ...
        progressGrid(2:end) >= eventProgress(i), 1, 'first');
    if isempty(idx)
        idx = find(progressGrid(1:end - 1) >= eventProgress(i), 1, 'first');
    end
    eventSteps(i) = idx;
end
eventSteps = unique(eventSteps(isfinite(eventSteps) & eventSteps > 0), 'stable');
end

function margin = endExclusionProgress(params)
if isfield(params, 'endExclusionProgress')
    margin = max(0, params.endExclusionProgress);
else
    margin = 0;
end
end

function disturbance = addArmaBurst(disturbance, startStep, signFlip, params)
nSteps = size(disturbance, 2);
burstSteps = min(params.burstSteps, nSteps - startStep + 1);
if burstSteps <= 0
    return;
end

sigma = [params.transSigma(:); params.rotSigma(:)];
limits = [params.maxTrans(:); params.maxRot(:)];
mask = disturbanceMask(params);
innov = sigma .* randn(6, burstSteps);
y = zeros(6, burstSteps);
epsPrev = zeros(6, 1);
yPrev1 = zeros(6, 1);
yPrev2 = zeros(6, 1);

for k = 1:burstSteps
    eps = innov(:, k);
    [armaA, armaB] = armaCoefficients(params);
    y(:, k) = armaA(1) * yPrev1 + armaA(2) * yPrev2 + eps + armaB * epsPrev;
    y(:, k) = min(max(y(:, k), -limits), limits) .* mask;
    yPrev2 = yPrev1;
    yPrev1 = y(:, k);
    epsPrev = eps;
end

window = burstWindow(burstSteps);
cols = startStep:(startStep + burstSteps - 1);
disturbance(:, cols) = disturbance(:, cols) + signFlip * y .* window;
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

function w = burstWindow(n)
idx = 1:n;
w = sin(pi * idx / (n + 1));
end
