function Xn = mpcc_dynamics_3d(X, U, dt)
% MPCC 3D kinematic dynamics with progress
% State X = [x;y;z; psi_x;psi_y;psi_z; s]
% Input U = [v_x;v_y;v_z; omega_x;omega_y;omega_z; v_s]

x = X(1); y = X(2); z = X(3);
psix = X(4); psiy = X(5); psiz = X(6);
s = X(7);

vx = U(1); vy = U(2); vz = U(3);
wx = U(4); wy = U(5); wz = U(6);
vs = U(7);

Xn = [ x + dt * vx;
       y + dt * vy;
       z + dt * vz;
       psix + dt * wx;
       psiy + dt * wy;
       psiz + dt * wz;
       s + dt * vs ];

end





