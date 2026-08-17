function stateNext = applyQuatDisturbance(state, d)
T = rmpc.geometry.makeTransform(rmpc.geometry.quatToRotm(state.q), state.p);
TNext = T * rmpc.geometry.expSE3(d);
TNext(1:3, 1:3) = rmpc.geometry.projectSO3(TNext(1:3, 1:3));
stateNext.p = TNext(1:3, 4);
stateNext.q = rmpc.geometry.rotmToQuat(TNext(1:3, 1:3));
end
