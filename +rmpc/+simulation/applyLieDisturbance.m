function stateNext = applyLieDisturbance(state, d)
stateNext.T = state.T * rmpc.geometry.expSE3(d);
stateNext.T(1:3, 1:3) = rmpc.geometry.projectSO3(stateNext.T(1:3, 1:3));
end
