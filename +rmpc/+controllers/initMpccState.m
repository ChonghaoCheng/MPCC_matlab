function mpcc = initMpccState(refU, cfg)
mpcc.refU = refU;
mpcc.progress = zeros(1, cfg.nSteps + 1);
mpcc.progress(1) = cfg.progressStart;
mpcc.vprogressGuess = cfg.mpcc.vprogressRef * ones(1, cfg.N);
mpcc.vprogressPrev = cfg.mpcc.vprogressRef;
mpcc.uGuess = reshape(refU(:, 1:cfg.N), [], 1);
mpcc.zGuess = [mpcc.uGuess; mpcc.vprogressGuess(:)];
mpcc.cost = zeros(1, cfg.nSteps);
end
