function viz = initRealtimeViews(names, colors, ref, dt, nSteps, enableRealtime)
viz = struct([]);
if ~enableRealtime
    return;
end

timeState = (0:nSteps) * dt;
timeCtrl = (0:(nSteps - 1)) * dt;
xRange = [min(ref.p(1, :)) - 0.2, max(ref.p(1, :)) + 0.2];
yRange = [min(ref.p(2, :)) - 0.2, max(ref.p(2, :)) + 0.2];

for i = 1:numel(names)
    fig = figure('Name', ['Realtime trajectory view - ' names{i}], 'Color', 'w');
    tiledlayout(fig, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    axTraj = nexttile;
    plot(axTraj, ref.p(1, :), ref.p(2, :), 'k--', 'LineWidth', 1.2);
    hold(axTraj, 'on');
    viz(i).traj = plot(axTraj, nan, nan, 'Color', colors(i, :), 'LineWidth', 1.8);
    viz(i).marker = plot(axTraj, nan, nan, 'o', 'MarkerFaceColor', colors(i, :), 'MarkerEdgeColor', colors(i, :));
    grid(axTraj, 'on'); axis(axTraj, 'equal');
    xlim(axTraj, xRange); ylim(axTraj, yRange);
    xlabel(axTraj, 'x [m]'); ylabel(axTraj, 'y [m]');
    title(axTraj, ['Trajectory path: ' names{i}]);
    legend(axTraj, 'reference', names{i}, 'Location', 'best');

    axTang = nexttile;
    viz(i).tangent = plot(axTang, nan, nan, 'Color', colors(i, :), 'LineWidth', 1.4);
    grid(axTang, 'on'); xlim(axTang, [timeState(1), timeState(end)]);
    xlabel(axTang, 'time [s]'); ylabel(axTang, 'projected error [m]');
    title(axTang, 'Projected tracking');

    axTool = nexttile;
    viz(i).toolAxis = plot(axTool, nan, nan, 'Color', colors(i, :), 'LineWidth', 1.4);
    grid(axTool, 'on'); xlim(axTool, [timeState(1), timeState(end)]);
    xlabel(axTool, 'time [s]'); ylabel(axTool, 'tool-axis error [deg]');
    title(axTool, 'Tool-axis alignment');

    axPen = nexttile;
    viz(i).diagnostic = stairs(axPen, nan, nan, 'Color', colors(i, :), 'LineWidth', 1.4);
    grid(axPen, 'on'); xlim(axPen, [timeCtrl(1), timeCtrl(end)]);
    xlabel(axPen, 'time [s]'); ylabel(axPen, 'diagnostic [mm]');
    title(axPen, 'Trajectory diagnostic');

    viz(i).fig = fig;
    viz(i).name = names{i};
end
drawnow;
end
