function printTrackingMetrics(names, hists, metrics, disturbance)
fprintf('\nMPCC path-following metrics\n');
fprintf('Controller              RMS contour [m]  RMS lag [m]  RMS rot [deg]  RMS SE3  final s  complete %%  finish [s]  RMS |u|  RMS |du|  mean opt cost\n');
for i = 1:numel(names)
    validCost = hists{i}.cost(isfinite(hists{i}.cost) & hists{i}.cost ~= 0);
    c = mean(validCost);
    if isnan(c), c = 0; end
    finishTime = hists{i}.finishTime;
    if isnan(finishTime)
        finishTime = Inf;
    end
    pathContour = metrics{i}.rmsContour;
    pathLag = metrics{i}.rmsLag;
    pathRot = metrics{i}.rmsRot;
    pathSe3 = metrics{i}.rmsSe3;
    pathControl = metrics{i}.rmsControl;
    pathControlRate = metrics{i}.rmsControlRate;
    if isfield(metrics{i}, 'pathFollowing')
        pathContour = metrics{i}.pathFollowing.rmsContour;
        pathLag = metrics{i}.pathFollowing.rmsLag;
        pathRot = metrics{i}.pathFollowing.rmsRot;
        pathSe3 = metrics{i}.pathFollowing.rmsSe3;
        pathControl = metrics{i}.pathFollowing.rmsControl;
        pathControlRate = metrics{i}.pathFollowing.rmsControlRate;
    end
    fprintf('%-22s %15.4e  %11.4e  %13.4e  %7.4e  %7.4f  %10.2f  %10.3f  %8.4f  %8.4f  %13.4e\n', ...
        names{i}, pathContour, pathLag, rad2deg(pathRot), pathSe3, ...
        hists{i}.progress(end), ...
        100 * hists{i}.completionRatio, finishTime, pathControl, pathControlRate, c);
end
fprintf('Disturbance RMS         %11.4e  %12s  %13.4e\n', ...
    sqrt(mean(vecnorm(disturbance(1:3, :), 2, 1) .^ 2)), '', ...
    rad2deg(sqrt(mean(vecnorm(disturbance(4:6, :), 2, 1) .^ 2))));
end
