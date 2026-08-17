function [u, uGuess, cost] = solveEuclideanMPCControl(state, refHorizon, uPrev, uGuess, cfg)
[uGuess, cost] = rmpc.controllers.solveTrackingMPC('euclidean', state, refHorizon, uPrev, uGuess, cfg);
u = uGuess(1:6);
end
