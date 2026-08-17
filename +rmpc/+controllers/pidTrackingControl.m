function [u, pidState] = pidTrackingControl(state, ref, k, pidState, cfg)
T = rmpc.geometry.makeTransform(rmpc.geometry.quatToRotm(state.q), state.p);
TRef = ref.T(:, :, k);
e = rmpc.geometry.logSE3(rmpc.geometry.invSE3(TRef) * T);
eDot = (e - pidState.ePrev) / cfg.dt;
pidState.eInt = rmpc.utils.clamp(pidState.eInt + e * cfg.dt, -0.45, 0.45);
pidState.ePrev = e;

Kp = cfg.pid.Kp;
Ki = cfg.pid.Ki;
Kd = cfg.pid.Kd;
feedback = Kp * e + Ki * pidState.eInt + Kd * eDot;
uRef = rmpc.geometry.logSE3(rmpc.geometry.invSE3(ref.T(:, :, k)) * ref.T(:, :, k + 1)) / cfg.dt;
u = uRef - feedback;
u = rmpc.utils.saturateControl(u, cfg.limits);
end
