function plotTrackingSummary(names, colors, hists, metrics, refEval, disturbance, dt, trajectory, disturbanceProfile, trajectoryFrame)
if nargin < 10 || isempty(trajectoryFrame)
    trajectoryFrame = trajectory;
end
timeState = (0:(size(refEval.p, 2) - 1)) * dt;

figure('Name', 'Tracking parameters over time/progress', 'Color', 'w', ...
    'Units', 'centimeters', 'Position', [3 3 18 10]);
tiledlayout(2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot3(refEval.p(1, :), refEval.p(2, :), refEval.p(3, :), 'k--', 'LineWidth', 1.5);
hold on;
for i = 1:numel(names)
    plot3(hists{i}.p(1, :), hists{i}.p(2, :), hists{i}.p(3, :), 'Color', colors(i, :), 'LineWidth', 1.2);
end
grid on; axis equal;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
legend(['reference', names], 'Location', 'best');
title('Trajectory tracking');
stylePaperAxes(gca);

nexttile;
for i = 1:numel(names)
    contourErr = metrics{i}.contourErr;
    xProgress = refEval.progress;
    if isfield(metrics{i}, 'pathFollowing')
        contourErr = metrics{i}.pathFollowing.contourErr;
        xProgress = hists{i}.progress;
    end
    plot(xProgress, contourErr, 'Color', colors(i, :), 'LineWidth', 1.2);
    hold on;
end
grid on;
xlabel('progress s'); ylabel('contouring error [m]');
legend(names, 'Location', 'best');
title('3D path contour error');
stylePaperAxes(gca);

nexttile;
for i = 1:numel(names)
    plot(timeState, hists{i}.progress, 'Color', colors(i, :), 'LineWidth', 1.2);
    hold on;
end
yline(1, 'k--', 'LineWidth', 1.0);
grid on;
xlabel('time [s]'); ylabel('progress s');
legend(names, 'Location', 'best');
title('Path progress');
stylePaperAxes(gca);

nexttile;
for i = 1:numel(names)
    x = hists{i}.progress(1:end - 1);
    y = hists{i}.vprogress;
    valid = isfinite(x) & isfinite(y);
    plot(x(valid), y(valid), 'Color', colors(i, :), 'LineWidth', 1.2);
    hold on;
    if isfield(hists{i}, 'vprogressRef') && i == 1
        yRef = hists{i}.vprogressRef;
        validRef = isfinite(x) & isfinite(yRef);
        plot(x(validRef), yRef(validRef), 'k--', 'LineWidth', 1.0);
    end
end
grid on;
xlabel('progress s'); ylabel('progress velocity ds/dt');
legend([names, {'reference'}], 'Location', 'best');
title('Progress velocity');
stylePaperAxes(gca);

nexttile;
for i = 1:numel(names)
    rotErr = metrics{i}.rotErr;
    xProgress = refEval.progress;
    if isfield(metrics{i}, 'pathFollowing')
        rotErr = metrics{i}.pathFollowing.rotErr;
        xProgress = hists{i}.progress;
    end
    plot(xProgress, rad2deg(rotErr), 'Color', colors(i, :), 'LineWidth', 1.2);
    hold on;
end
grid on;
xlabel('progress s'); ylabel('rotation error [deg]');
legend(names, 'Location', 'best');
title('Rotation error');
stylePaperAxes(gca);

if nargin >= 8 && ~isempty(hists) && isfield(hists{1}, 'pPre')
    plotUndisturbedTrajectoryFrame(names, colors, hists, trajectoryFrame);
    if nargin < 9
        disturbanceProfile = [];
    end
    plotDisturbedWorldTrajectories(names, colors, hists, refEval, disturbance);
end
end

function plotUndisturbedTrajectoryFrame(names, colors, hists, trajectory)
figure('Name', 'Undisturbed trajectories in trajectory frame', 'Color', 'w', ...
    'Units', 'centimeters', 'Position', [3 3 16 12]);
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
hold on;
for i = 1:numel(names)
    localPre = localPositionError(hists{i}.pPre, hists{i}.progress, trajectory);
    plot3(localPre(1, :), localPre(2, :), localPre(3, :), ...
        'Color', colors(i, :), 'LineWidth', 1.2);
end
plot3(0, 0, 0, 'k.', 'MarkerSize', 14);
grid on; axis equal;
xlabel('local tangent x [m]');
ylabel('local normal y [m]');
zlabel('local normal z [m]');
legend([names, {'reference'}], 'Location', 'best');
title('Pre-disturbance trajectory-frame error');
stylePaperAxes(gca);

componentLabels = {'local tangent error [m]', 'local normal-y error [m]', 'local normal-z error [m]'};
for component = 1:3
    nexttile;
    hold on;
    for i = 1:numel(names)
        localPre = localPositionError(hists{i}.pPre, hists{i}.progress, trajectory);
        plot(hists{i}.progress, localPre(component, :), ...
            'Color', colors(i, :), 'LineWidth', 1.2);
    end
    yline(0, 'k--', 'LineWidth', 0.8);
    grid on;
    xlabel('progress s');
    ylabel(componentLabels{component});
    if component == 1
        legend(names, 'Location', 'best');
    end
    stylePaperAxes(gca);
end
end

function plotDisturbedWorldTrajectories(names, colors, hists, refEval, disturbance)
figure('Name', 'Disturbed reference and actual controller trajectories', 'Color', 'w', ...
    'Units', 'centimeters', 'Position', [3 3 15 11]);
ax = axes;
hold(ax, 'on');
plot3(ax, refEval.p(1, :), refEval.p(2, :), refEval.p(3, :), ...
    '-', 'Color', [0.70 0.05 0.05], 'LineWidth', 2.2);
activeDisturbance = disturbanceActivity(disturbance, size(refEval.p, 2));
eventIdx = findEventMarkerIndices(activeDisturbance);
if ~isempty(eventIdx)
    scatter3(ax, refEval.p(1, eventIdx), refEval.p(2, eventIdx), ...
        refEval.p(3, eventIdx), 26, [0.70 0.05 0.05], 'filled', ...
        'MarkerEdgeColor', 'w', 'LineWidth', 0.5);
end
legendEntries = {'disturbed reference'};
for i = 1:numel(names)
    plot3(ax, hists{i}.p(1, :), hists{i}.p(2, :), hists{i}.p(3, :), ...
        '-', 'Color', colors(i, :), 'LineWidth', 1.6);
    legendEntries{end + 1} = [names{i} ' actual']; %#ok<AGROW>
end
grid on; axis equal;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
legend(legendEntries, 'Location', 'northeast');
view(42, 24);
camlight(ax, 'headlight');
stylePaperAxes(ax);
end

function active = disturbanceActivity(disturbance, nRef)
active = false(1, nRef);
if isempty(disturbance)
    return;
end
n = min(size(disturbance, 2), nRef - 1);
if n <= 0
    return;
end
active(2:(n + 1)) = vecnorm(disturbance(:, 1:n), 2, 1) > 0;
end

function idx = findEventMarkerIndices(activeDisturbance)
starts = find(diff([false, activeDisturbance]) == 1);
idx = starts;
end

function localErr = localPositionError(p, progress, trajectory)
n = size(p, 2);
localErr = zeros(3, n);
for k = 1:n
    s = min(max(progress(k), 0), 1);
    ref = rmpc.reference.referenceAtProgress(s, trajectory);
    localErr(:, k) = ref.R' * (p(:, k) - ref.p);
end
end

function stylePaperAxes(ax)
set(ax, 'FontName', 'Arial', 'FontSize', 9, 'LineWidth', 0.9, ...
    'TickDir', 'out', 'Box', 'off');
ax.GridAlpha = 0.18;
ax.MinorGridAlpha = 0.08;
end
