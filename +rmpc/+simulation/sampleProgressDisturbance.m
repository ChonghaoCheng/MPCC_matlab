function [d, state] = sampleProgressDisturbance(profile, state, progressPrev, progressNext, enabled)
d = zeros(6, 1);
if ~enabled || isempty(profile.eventProgress)
    return;
end

if ~isfield(profile, 'burstProgressStep') || profile.burstProgressStep <= 0
    return;
end

progress = max(progressPrev, progressNext);
for i = 1:numel(profile.eventProgress)
    burst = profile.bursts{i};
    rel = (progress - profile.eventProgress(i)) / profile.burstProgressStep;
    sampleIdx = floor(rel) + 1;
    if sampleIdx >= 1 && sampleIdx <= size(burst, 2)
        d = d + burst(:, sampleIdx);
    end
end
end
