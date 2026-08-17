function [u, uGuess, cost] = solveQuaternionMPCControl(state, refHorizon, uPrev, uGuess, cfg)
[uGuess, cost] = rmpc.controllers.solveTrackingMPC('riemannian', state, refHorizon, uPrev, uGuess, cfg);
u = uGuess(1:6);
end
