function [u, uGuess, cost] = solveRiemannianMPCControl(state, refHorizon, uPrev, uGuess, cfg)
[uGuess, cost] = rmpc.controllers.solveTrackingMPC('se3', state, refHorizon, uPrev, uGuess, cfg);
u = uGuess(1:6);
end
