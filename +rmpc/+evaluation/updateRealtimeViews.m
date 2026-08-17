function updateRealtimeViews(viz, hists, ref, trajectory, dt, k, enableRealtime)
if ~enableRealtime || isempty(viz)
    return;
end

stateIdx = 1:(k + 1);
timeState = (0:k) * dt;
timeCtrl = (0:(k - 1)) * dt;
for i = 1:numel(viz)
    hist = hists{i};
    tangentialErr = zeros(1, numel(stateIdx));
    for j = stateIdx
        ep = hist.p(:, j) - ref.p(:, j);
        tangentialErr(j) = norm(rmpc.reference.tangentProjector(ref.p(:, j), trajectory) * ep);
    end
    set(viz(i).traj, 'XData', hist.p(1, stateIdx), 'YData', hist.p(2, stateIdx));
    set(viz(i).marker, 'XData', hist.p(1, k + 1), 'YData', hist.p(2, k + 1));
    set(viz(i).tangent, 'XData', timeState, 'YData', tangentialErr);
    set(viz(i).toolAxis, 'XData', timeState, 'YData', rad2deg(hist.toolAxisAngle(stateIdx)));
    set(viz(i).diagnostic, 'XData', timeCtrl, 'YData', 1000 * hist.diagnostic(1:k));
end
drawnow limitrate;
end
