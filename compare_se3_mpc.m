%% compare_se3_mpc
% Pure SE(3) helix trajectory tracking benchmark.
%
% Select compared controllers in +rmpc/+config/defaultConfig.m via
% cfg.controllerKeys. Each controller definition maps to its own solver file
% through +rmpc/+controllers/controllerSpecs.m.

clearvars -except cfgOverride; clc; close all;

if exist('quadprog', 'file') ~= 2
    error('This example requires quadprog from MATLAB Optimization Toolbox.');
end

rng(2);
cfg = rmpc.config.defaultConfig();
if exist('cfgOverride', 'var') && ~isempty(cfgOverride)
    cfg = mergeConfig(cfg, cfgOverride);
    cfg = rmpc.config.finalizeConfig(cfg);
end

if cfg.runMathSanity
    sanity = rmpc.evaluation.mathSanityCheck(cfg);
    rmpc.evaluation.printSanityReport(sanity);
    if ~sanity.pass
        error('Math sanity check failed. Fix model/reference consistency before running comparisons.');
    end
end

trajectoryNominal = cfg.trajectory;
refNominal = rmpc.reference.makeReference(cfg.progressGrid, trajectoryNominal, cfg.dt);
disturbanceProfile = rmpc.simulation.makeDisturbanceProfile(cfg);
if disturbanceTargetsReference(cfg)
    [ref, cfg.trajectory, referenceDisturbance] = rmpc.reference.applyDisturbanceProfile( ...
        refNominal, trajectoryNominal, disturbanceProfile, cfg);
else
    ref = refNominal;
    referenceDisturbance = zeros(6, max(size(ref.p, 2) - 1, 0));
end
zeroDisturbance = zeros(6, cfg.nSteps);
refUTracking = rmpc.reference.referenceBodyTwists(ref, cfg.dt);
refUMpcc = refUTracking;

p0 = ref.p(:, 1);
q0 = ref.q(:, 1);
T0 = rmpc.geometry.makeTransform(rmpc.geometry.quatToRotm(q0), p0);
controllers = initControllers(cfg.controllers, p0, q0, T0, refUTracking, refUMpcc, cfg, zeroDisturbance);

viz = rmpc.evaluation.initRealtimeViews(cfg.names, cfg.colors, ref, cfg.dt, cfg.nSteps, cfg.enableRealtime);

fprintf('Running %d pure trajectory-tracking steps for %d controllers...\n', cfg.nSteps, numel(controllers));

for k = 1:cfg.nSteps
    refHorizon = rmpc.reference.sliceReference(ref, k:(k + cfg.N));
    for i = 1:numel(controllers)
        controllers(i) = runControllerStep(controllers(i), ref, refHorizon, refUTracking, disturbanceProfile, k, cfg);
    end

    histsLive = {controllers.hist};
    rmpc.evaluation.updateRealtimeViews(viz, histsLive, ref, cfg.trajectory, cfg.dt, k, cfg.enableRealtime);

    if mod(k, 10) == 0 || k == cfg.nSteps
        fprintf('  step %d / %d\n', k, cfg.nSteps);
    end

    if k >= cfg.nominalSteps && allMpccCompleted(controllers, cfg, k)
        fprintf('  all MPCC controllers completed at step %d / %d\n', k, cfg.nSteps);
        break;
    end
end

lastStep = k;
refEval = rmpc.reference.sliceReference(ref, 1:(lastStep + 1));
hists = {controllers.hist};
metrics = cell(size(hists));
for i = 1:numel(hists)
    hists{i} = trimHistory(hists{i}, lastStep, cfg);
    metrics{i} = rmpc.evaluation.computeTrackingMetrics(hists{i}, refEval, cfg.trajectory, refEval.progress);
    metrics{i}.pathFollowing = rmpc.evaluation.computeTrackingMetrics(hists{i}, refEval, cfg.trajectory, hists{i}.progress);
end

if disturbanceTargetsReference(cfg)
    disturbanceEval = referenceDisturbance(:, 1:lastStep);
else
    disturbanceEval = aggregateDisturbance(hists, lastStep);
end
rmpc.evaluation.printTrackingMetrics(cfg.names, hists, metrics, disturbanceEval);
rmpc.evaluation.plotTrackingSummary(cfg.names, cfg.colors, hists, metrics, refEval, disturbanceEval, ...
    cfg.dt, cfg.trajectory, disturbanceProfile, trajectoryNominal);

function controllers = initControllers(specs, p0, q0, T0, refUTracking, refUMpcc, cfg, disturbance)
template = struct('key', '', 'name', '', 'stateType', '', 'kind', '', 'solver', '', ...
    'state', struct(), 'hist', struct(), 'uPrev', zeros(6, 1), 'uGuess', [], ...
    'pidState', struct(), 'mpcc', struct(), 'disturbanceState', struct());
controllers = repmat(template, 1, numel(specs));

for i = 1:numel(specs)
    controllers(i).key = specs(i).key;
    controllers(i).name = specs(i).name;
    controllers(i).stateType = specs(i).stateType;
    controllers(i).kind = specs(i).kind;
    controllers(i).solver = specs(i).solver;
    controllers(i).hist = rmpc.simulation.storeInitial( ...
        rmpc.simulation.initHistory(cfg.nSteps, disturbance), p0, q0, cfg.trajectory, cfg.progressStart, cfg.progressStart);
    controllers(i).uPrev = zeros(6, 1);
    controllers(i).disturbanceState = rmpc.simulation.initDisturbanceState();

    switch specs(i).stateType
        case 'quat'
            controllers(i).state.p = p0;
            controllers(i).state.q = q0;
        case 'lie'
            controllers(i).state.T = T0;
        otherwise
            error('Unknown controller state type: %s', specs(i).stateType);
    end

    switch specs(i).kind
        case 'pid'
            controllers(i).pidState = rmpc.controllers.initPidState();
        case 'mpc'
            controllers(i).uGuess = reshape(refUTracking(:, 1:cfg.N), [], 1);
        case 'mpcc'
            controllers(i).mpcc = rmpc.controllers.initMpccState(refUMpcc, cfg);
        otherwise
            error('Unknown controller kind: %s', specs(i).kind);
    end
end
end

function ctrl = runControllerStep(ctrl, ref, refHorizon, refUTracking, disturbanceProfile, k, cfg)
zeroDiagnostic.diagnostic = 0;
cost = 0;

switch ctrl.kind
    case 'pid'
        solver = str2func(['rmpc.controllers.' ctrl.solver]);
        [u, ctrl.pidState] = solver(ctrl.state, ref, k, ctrl.pidState, cfg);

    case 'mpc'
        solver = str2func(['rmpc.controllers.' ctrl.solver]);
        [u, ctrl.uGuess, cost] = solver(ctrl.state, refHorizon, ctrl.uPrev, ctrl.uGuess, cfg);
        uAppend = refUTracking(:, min(k + cfg.N, size(refUTracking, 2)));
        ctrl.uGuess = rmpc.utils.shiftWarmStart(ctrl.uGuess, cfg.N, uAppend);

    case 'mpcc'
        solver = str2func(['rmpc.controllers.' ctrl.solver]);
        [u, ctrl.mpcc] = solver(ctrl.state, ctrl.mpcc.progress(k), ctrl.uPrev, ctrl.mpcc, cfg);
        ctrl.mpcc.progress(k + 1) = ctrl.mpcc.progressCurrent;
        cost = ctrl.mpcc.costCurrent;

    otherwise
        error('Unknown controller kind: %s', ctrl.kind);
end

ctrl.uPrev = u;
ctrl.hist.cost(k) = cost;
progressRef = ref.progress(k + 1);
progressPrev = ctrl.hist.progress(k);
progressRefPrev = ctrl.hist.progressRef(k);
if strcmp(ctrl.kind, 'mpcc')
    progress = ctrl.mpcc.progress(k + 1);
else
    progress = progressRef;
end
vprogress = (progress - progressPrev) / cfg.dt;
vprogressRef = (progressRef - progressRefPrev) / cfg.dt;
if disturbanceTargetsState(cfg)
    [disturbance, ctrl.disturbanceState] = rmpc.simulation.sampleProgressDisturbance( ...
        disturbanceProfile, ctrl.disturbanceState, progressPrev, progress, cfg.enableDisturbance);
else
    disturbance = zeros(6, 1);
end
ctrl.hist.disturbance(:, k) = disturbance;

switch ctrl.stateType
    case 'quat'
        statePre = rmpc.simulation.stepQuatState(ctrl.state, u, cfg.dt);
        ctrl.state = rmpc.simulation.applyQuatDisturbance(statePre, disturbance);
        ctrl.hist = rmpc.simulation.storeStep(ctrl.hist, ctrl.state.p, ctrl.state.q, u, zeroDiagnostic, ...
            k, cfg.trajectory, progress, progressRef, statePre.p, statePre.q, vprogress, vprogressRef);
    case 'lie'
        statePre = rmpc.simulation.stepLieState(ctrl.state, u, cfg.dt);
        ctrl.state = rmpc.simulation.applyLieDisturbance(statePre, disturbance);
        qPre = rmpc.geometry.rotmToQuat(statePre.T(1:3, 1:3));
        ctrl.hist = rmpc.simulation.storeStep(ctrl.hist, ctrl.state.T(1:3, 4), ...
            rmpc.geometry.rotmToQuat(ctrl.state.T(1:3, 1:3)), u, zeroDiagnostic, ...
            k, cfg.trajectory, progress, progressRef, statePre.T(1:3, 4), qPre, vprogress, vprogressRef);
    otherwise
        error('Unknown controller state type: %s', ctrl.stateType);
end

if ~ctrl.hist.completed && progress >= cfg.progressEnd - cfg.completionTol
    ctrl.hist.completed = true;
    ctrl.hist.finishStep = k;
    ctrl.hist.finishTime = k * cfg.dt;
end
end

function tf = disturbanceTargetsReference(cfg)
tf = strcmpi(disturbanceTarget(cfg), 'reference');
end

function tf = disturbanceTargetsState(cfg)
tf = strcmpi(disturbanceTarget(cfg), 'state');
end

function target = disturbanceTarget(cfg)
target = 'state';
if isfield(cfg, 'disturbance') && isfield(cfg.disturbance, 'target')
    target = lower(char(cfg.disturbance.target));
end
switch target
    case {'reference', 'state'}
        return;
    otherwise
        error('Unknown disturbance target: %s', target);
end
end

function done = allMpccCompleted(controllers, cfg, k)
hasMpcc = false;
done = true;
for i = 1:numel(controllers)
    if strcmp(controllers(i).kind, 'mpcc')
        hasMpcc = true;
        done = done && controllers(i).mpcc.progress(k + 1) >= cfg.progressEnd - cfg.completionTol;
    end
end
done = hasMpcc && done;
end

function disturbance = aggregateDisturbance(hists, lastStep)
disturbance = zeros(6, lastStep * numel(hists));
for i = 1:numel(hists)
    cols = ((i - 1) * lastStep + 1):(i * lastStep);
    disturbance(:, cols) = hists{i}.disturbance(:, 1:lastStep);
end
end

function hist = trimHistory(hist, lastStep, cfg)
stateIdx = 1:(lastStep + 1);
ctrlIdx = 1:lastStep;
hist.p = hist.p(:, stateIdx);
hist.pPre = hist.pPre(:, stateIdx);
hist.q = hist.q(:, stateIdx);
hist.qPre = hist.qPre(:, stateIdx);
hist.u = hist.u(:, ctrlIdx);
hist.cost = hist.cost(ctrlIdx);
hist.diagnostic = hist.diagnostic(ctrlIdx);
hist.toolAxisAngle = hist.toolAxisAngle(stateIdx);
hist.progress = hist.progress(stateIdx);
hist.progressRef = hist.progressRef(stateIdx);
hist.vprogress = hist.vprogress(ctrlIdx);
hist.vprogressRef = hist.vprogressRef(ctrlIdx);
hist.disturbance = hist.disturbance(:, ctrlIdx);
hist.thetaFinal = rmpc.reference.progressToPathParam(hist.progress(end), cfg.trajectory);
if ~hist.completed && hist.progress(end) >= cfg.progressEnd - cfg.completionTol
    hist.completed = true;
    hist.finishStep = lastStep;
    hist.finishTime = lastStep * cfg.dt;
end
if hist.completed
    hist.completionRatio = 1;
else
    hist.completionRatio = min(max(hist.progress(end), cfg.progressStart), cfg.progressEnd);
end
end

function cfg = mergeConfig(cfg, override)
fields = fieldnames(override);
for i = 1:numel(fields)
    name = fields{i};
    if isstruct(override.(name)) && isfield(cfg, name) && isstruct(cfg.(name))
        cfg.(name) = mergeConfig(cfg.(name), override.(name));
    else
        cfg.(name) = override.(name);
    end
end
end
