function pidState = initPidState()
pidState.eInt = zeros(6, 1);
pidState.ePrev = zeros(6, 1);
pidState.zAdm = 0;
pidState.zAdmDot = 0;
pidState.penLast = 0;
end
