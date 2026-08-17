function cfg = defaultConfig()
cfg.dt = 0.010;
cfg.N = 10;
cfg.referenceOmega = 0.85;
cfg.referenceOmegaNominal = cfg.referenceOmega;
cfg.referenceMaxControlUtilization = 0.45;

% Reference path geometry. Supported path types:
%   'spiral' : cylinder helix parameterized by theta.
%   'line3d' : 3D straight line parameterized by theta.
%   'sine3d' : spatial sine wave parameterized by theta.
%   'flower3d' : rising flower/rose curve parameterized by theta.
%   'lissajous3d' : bounded 3D Lissajous curve with continuously changing tangent.
%   'trefoil3d' : closed knot-like spatial curve.
%   'wavy_spiral' : helix with radial and vertical modulation.
% For all path types the reference frame x-axis follows the path tangent, and
% the tool z-axis is perpendicular to the path tangent.
trajectory.pathType = 'wavy_spiral';
trajectory.pathParamStart = 0;
trajectory.pathParamEnd = 2 * pi;
trajectory.radius = 1.15;
trajectory.z0 = 0.15;
trajectory.pitch = 0.12;
trajectory.lineP0 = [-0.85; -0.45; 0.20];
trajectory.lineDirection = [1.0; 0.55; 0.35];
trajectory.lineScale = 0.22;
trajectory.sineP0 = [-0.85; -0.55; 0.25];
trajectory.sineLengthScale = 0.14;
trajectory.sineAmpY = 0.18;
trajectory.sineAmpZ = 0.108;
trajectory.sineFreqY = 0.75;
trajectory.sineFreqZ = 0.4875;
trajectory.sinePhaseZ = pi / 4;
trajectory.flowerBaseRadius = 1.40;
trajectory.flowerAmp = 0.02;
trajectory.flowerPetals = 2;
trajectory.flowerZ0 = 0.15;
trajectory.flowerPitch = 0.075;
trajectory.lissajousCenter = [0; 0; 0.45];
trajectory.lissajousAmp = [0.85; 0.55; 0.35];
trajectory.lissajousFreq = [2.0; 3.0; 4.0];
trajectory.lissajousPhase = [0.0; pi / 3; pi / 5];
trajectory.trefoilScale = 0.16;
trajectory.trefoilZScale = 0.20;
trajectory.trefoilCenter = [0; 0; 0.45];
trajectory.wavyRadius = 0.95;
trajectory.wavyRadiusAmp = 0.22;
trajectory.wavyRadiusFreq = 3.0;
trajectory.wavyZ0 = 0.10;
trajectory.wavyPitch = 0.10;
trajectory.wavyZAmp = 0.16;
trajectory.wavyZFreq = 2.0;
trajectory.preferredNormal = [0; 0; 1];
trajectory.toolAxisBody = [0; 0; 1];
cfg.trajectory = trajectory;

cfg.progressStart = 0;
cfg.progressEnd = 1;
cfg.completionTol = 1e-3;
cfg.maxOverrunFactor = 1.30;
weights.Qp = diag([450 450 250]);
weights.Qt = diag([450 450 0]);
weights.Qs = 250;
weights.Qr = diag([220 220 120]);
weights.Qse3 = diag([450 450 250 220 220 120]);
weights.Ru = 1e-5 * eye(6);
weights.Rdu = 1e-5 * eye(6);
weights.QpTerminal = 2.2 * weights.Qp;
weights.QtTerminal = 2.2 * weights.Qt;
weights.QsTerminal = 2.2 * weights.Qs;
weights.QrTerminal = 2.2 * weights.Qr;
weights.Qse3Terminal = 2.2 * weights.Qse3;
cfg.weights = weights;

pid.Kp = diag([1.45 1.25 1.30 9.0 9.0 7.0]);
pid.Ki = diag([0.04 0.04 0.04 0.20 0.20 0.15]);
pid.Kd = diag([0.16 0.14 0.16 0.35 0.35 0.28]);
cfg.pid = pid;

limits.vMax = [1.45; 0.34; 0.28];
limits.wMax = [0.90; 0.90; 1.10];
cfg.limits = limits;

cfg.opts = optimoptions('quadprog', ...
    'Display', 'none', ...
    'Algorithm', 'interior-point-convex', ...
    'OptimalityTolerance', 1e-8, ...
    'StepTolerance', 1e-10, ...
    'ConstraintTolerance', 1e-9);

cfg.controllerKeys = {'euclidean_mpcc', 'riemannian_mpcc'};
cfg.enableRealtime = false;
cfg.enableDisturbance = true;
cfg.runMathSanity = false;

disturbance.type = 'progress_arma21';  % 'progress_arma21', 'translation_only', 'rotation_only', 'translation_rotation', or 'none'
disturbance.target = 'reference';  % 'reference' deforms the world-frame path; 'state' applies external perturbations.
disturbance.progressInterval = 0.20;
disturbance.excludeEndTol = 1e-9;
disturbance.endExclusionProgress = 0.12;
disturbance.burstSteps = 7;
disturbance.armaA = [0.65, 0.22];
disturbance.armaB = 0.35;
disturbance.burstWindow = 'sine';  % 'sine', 'hann', or 'rect'.
disturbance.signPattern = 'alternating';  % 'alternating', 'same', or 'random'.
disturbance.transSigma = [0.018; 0.014; 0.010];
disturbance.rotSigma = deg2rad([1.0; 0.2; 1.5]);
disturbance.maxTrans = [0.060; 0.050; 0.040];
disturbance.maxRot = deg2rad([4.0; 3.0; 3.0]);
cfg.disturbance = disturbance;

mpcc.G = diag([450 450 250 10 10 10]);
mpcc.Qc = diag([600 600 350 600 600 350]);
mpcc.Ql = 300;
mpcc.riemannianLagMode = 'position';  % 'position' matches path contour metrics; 'se3' uses full twist projection.
mpcc.Ru = 1e-5 * eye(6);
mpcc.Rdu = 1e-5 * eye(6);
mpcc.RpathVelocity = 1e-5 * eye(6);

% MPCC progress handling uses normalized path progress s in [0, 1].
%   + Rvprogress * (vprogress - vprogressRef)^2 regularizes progress speed.
%   - progressReward * vprogress rewards forward progress in the minimization.
mpcc.Rvprogress = 80;
mpcc.vprogressRef = 0;
mpcc.Rdvprogress = 0.5;
mpcc.progressReward = 0.01;
mpcc.progressRateMin = 0;
mpcc.progressRateMax = 0;
mpcc.progressTrackingWeight = 0;
mpcc.progressTerminalWeight = 0;
mpcc.progressUpperSlack = 1e-4;
cfg.mpcc = mpcc;
cfg = rmpc.config.finalizeConfig(cfg);
end
