function [xref, uref, svec] = sample_ref_path3d(s0, N, ds, r, r1, r2, sdot_nom)
% SAMPLE_REF_PATH3D: sample 3D reference states/inputs from path by arc length
% Returns xref (6x(N+1)) and uref (6xN) for MPC tracking
% State: [x;y;z; psi_x;psi_y;psi_z]
% Input: [v_x;v_y;v_z; omega_x;omega_y;omega_z]

svec = s0 + (0:N) * ds;
xref = zeros(6, N+1);
uref = zeros(6, N);

for k = 1:N+1
    [t, ~, ~, ~, p] = geom3_at(svec(k), r, r1, r2);
  
    yaw   = atan2(t(2), t(1));
    pitch = atan2(-t(3), sqrt(t(1)^2 + t(2)^2));
    roll  = 0;
    xref(:,k) = [p; roll; pitch; yaw];
end

% Reference velocities: finite-diff of position and angles
for k = 1:N
    p_now = xref(1:3,k); p_next = xref(1:3,k+1);
    dp = (p_next - p_now) / max(ds, 1e-9); % ~ d/ds
    % convert to per time by assuming ds/dt ~ 1 (unit speed). You can rescale outside.
    v_ref = dp;  % treat as nominal Cartesian velocities

    ang_now = xref(4:6,k); ang_next = xref(4:6,k+1);
    % unwrap yaw (3rd angle in [roll; pitch; yaw]) for continuity
    ang_next(3) = ang_now(3) + atan2(sin(ang_next(3)-ang_now(3)), cos(ang_next(3)-ang_now(3)));
    dang_ds = (ang_next - ang_now) / max(ds, 1e-9);

    % Convert derivatives wrt s to per-time rates using nominal sdot
    if nargin < 7 || isempty(sdot_nom)
        sdot_nom = 1.0;
    end
    v_ref = v_ref * sdot_nom;
    dang = dang_ds * sdot_nom;

    uref(:,k) = [v_ref; dang];
end

end


