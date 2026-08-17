function printSanityReport(report)
fprintf('\nMath sanity check: reference feedforward rollout\n');
fprintf('  quat max position error:   %.3e m\n', report.maxQuatPosErr);
fprintf('  quat max attitude error:   %.3e rad\n', report.maxQuatAttErr);
fprintf('  lie max position error:    %.3e m\n', report.maxLiePosErr);
fprintf('  lie max attitude error:    %.3e rad\n', report.maxLieAttErr);
fprintf('  quat max path diagnostic:  %.3e m\n', report.maxQuatPathDiagnostic);
fprintf('  lie max path diagnostic:   %.3e m\n', report.maxLiePathDiagnostic);
fprintf('  max reference/control lim: %.3f\n', report.maxControlLimitRatio);
if report.pass
    fprintf('  status: PASS\n');
else
    fprintf('  status: FAIL\n');
end
end
