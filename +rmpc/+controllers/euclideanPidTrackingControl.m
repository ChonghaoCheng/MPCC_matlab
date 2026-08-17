function [u, pidState] = euclideanPidTrackingControl(state, ref, k, pidState, cfg)
R = rmpc.geometry.quatToRotm(state.q);

rpy = rmpc.geometry.rotmToRpy(R);
rpyRef = ref.rpy(:, k);

e = [state.p - ref.p(:, k);
     rmpc.utils.wrapToPiLocal(rpy - rpyRef)];
eDot = (e - pidState.ePrev) / cfg.dt;
pidState.eInt = rmpc.utils.clamp(pidState.eInt + e * cfg.dt, -0.45, 0.45);
pidState.ePrev = e;

Kp = cfg.pid.Kp;
Ki = cfg.pid.Ki;
Kd = cfg.pid.Kd;

uRef = referenceBodyTwist(ref, k, cfg.dt);
Rref = rmpc.geometry.quatToRotm(ref.q(:, k));
rpyRateRef = bodyOmegaToRpyRate(rpyRef, uRef(4:6));
cmd = [Rref * uRef(1:3);
       rpyRateRef] ...
    - Kp * e - Ki * pidState.eInt - Kd * eDot;

omegaBody = rpyRateToBodyOmega(rpy, cmd(4:6));
localDisplacement = R' * (cfg.dt * cmd(1:3));
vBody = rmpc.geometry.bodyVelocityForLocalDisplacement(localDisplacement, omegaBody, cfg.dt);
u = [vBody; omegaBody];
u = rmpc.utils.saturateControl(u, cfg.limits);
end

function uRef = referenceBodyTwist(ref, k, dt)
uRef = rmpc.geometry.logSE3(rmpc.geometry.invSE3(ref.T(:, :, k)) * ref.T(:, :, k + 1)) / dt;
end

function rpyRate = bodyOmegaToRpyRate(rpy, omega)
roll = rpy(1);
pitch = rpy(2);
cp = cos(pitch);
if abs(cp) < 1e-6
    cp = sign(cp + (cp == 0)) * 1e-6;
end
E = [1, sin(roll) * tan(pitch), cos(roll) * tan(pitch);
     0, cos(roll),             -sin(roll);
     0, sin(roll) / cp,         cos(roll) / cp];
rpyRate = E * omega;
end

function omega = rpyRateToBodyOmega(rpy, rpyRate)
roll = rpy(1);
pitch = rpy(2);
cp = cos(pitch);
if abs(cp) < 1e-6
    cp = sign(cp + (cp == 0)) * 1e-6;
end
E = [1, sin(roll) * tan(pitch), cos(roll) * tan(pitch);
     0, cos(roll),             -sin(roll);
     0, sin(roll) / cp,         cos(roll) / cp];
omega = E \ rpyRate;
end
