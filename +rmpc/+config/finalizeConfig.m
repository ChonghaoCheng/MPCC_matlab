function cfg = finalizeConfig(cfg)
trajectory = cfg.trajectory;

if isfield(cfg, 'referenceOmegaNominal') && ~isempty(cfg.referenceOmegaNominal)
    cfg.referenceOmega = cfg.referenceOmegaNominal;
else
    cfg.referenceOmegaNominal = cfg.referenceOmega;
end
cfg.referenceProgressRate = cfg.referenceOmega / ...
    max(abs(trajectory.pathParamEnd - trajectory.pathParamStart), 1e-12);
if isfield(cfg, 'referenceMaxControlUtilization') && cfg.referenceMaxControlUtilization > 0
    util = estimateReferenceControlUtilization(cfg);
    if util > cfg.referenceMaxControlUtilization
        scale = cfg.referenceMaxControlUtilization / max(util, 1e-12);
        cfg.referenceOmega = cfg.referenceOmega * scale;
        cfg.referenceProgressRate = cfg.referenceProgressRate * scale;
    end
end
cfg.referenceControlUtilization = estimateReferenceControlUtilization(cfg);
cfg.nominalSimTime = (cfg.progressEnd - cfg.progressStart) / cfg.referenceProgressRate;
cfg.maxSimTime = cfg.maxOverrunFactor * cfg.nominalSimTime;
cfg.simTime = cfg.maxSimTime;
cfg.nSteps = ceil(cfg.maxSimTime / cfg.dt);
cfg.nominalSteps = ceil(cfg.nominalSimTime / cfg.dt);
cfg.tGrid = (0:(cfg.nSteps + cfg.N)) * cfg.dt;
cfg.progressGrid = min(cfg.progressStart + cfg.referenceProgressRate * cfg.tGrid, cfg.progressEnd);

cfg.lb = repmat([-cfg.limits.vMax; -cfg.limits.wMax], cfg.N, 1);
cfg.ub = repmat([ cfg.limits.vMax;  cfg.limits.wMax], cfg.N, 1);

cfg.mpcc.vprogressRef = cfg.referenceProgressRate;
cfg.mpcc.progressRateMin = 0.25 / max(abs(trajectory.pathParamEnd - trajectory.pathParamStart), 1e-12);
cfg.mpcc.progressRateMax = 1.15 / max(abs(trajectory.pathParamEnd - trajectory.pathParamStart), 1e-12);

cfg.controllers = rmpc.controllers.controllerSpecs(cfg.controllerKeys);
cfg.names = {cfg.controllers.name};
cfg.colors = lines(numel(cfg.names));
end

function util = estimateReferenceControlUtilization(cfg)
progress = linspace(cfg.progressStart, cfg.progressEnd, 401);
lim = [cfg.limits.vMax(:); cfg.limits.wMax(:)];
util = 0;
for i = 1:numel(progress)
    uRef = rmpc.reference.referenceTangentTwist(progress(i), cfg.trajectory) * cfg.referenceProgressRate;
    util = max(util, max(abs(uRef) ./ max(lim, 1e-12)));
end
end
