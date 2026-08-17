function stateNext = stepLieState(state, u, dt)
stateNext.T = state.T * rmpc.geometry.expSE3(dt * u);
stateNext.T(1:3, 1:3) = rmpc.geometry.projectSO3(stateNext.T(1:3, 1:3));
end
