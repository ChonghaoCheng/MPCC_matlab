function stateNext = stepQuatState(state, u, dt)
T = rmpc.geometry.makeTransform(rmpc.geometry.quatToRotm(state.q), state.p);
TNext = T * rmpc.geometry.expSE3(dt * u);
stateNext.p = TNext(1:3, 4);
stateNext.q = rmpc.geometry.rotmToQuat(TNext(1:3, 1:3));
end
