function Xn = mpc_dynamics_3d(X, U, dt)
% 3D MPC kinematic dynamics (no progress state)
% State X = [x;y;z; psi_x;psi_y;psi_z]
% Input U = [v_x;v_y;v_z; omega_x;omega_y;omega_z]

x = X(1); y = X(2); z = X(3);
psix = X(4); psiy = X(5); psiz = X(6);

vx = U(1); vy = U(2); vz = U(3);
wx = U(4); wy = U(5); wz = U(6);

Xn = [ x + dt * vx;
       y + dt * vy;
       z + dt * vz;
       psix + dt * wx;
       psiy + dt * wy;
       psiz + dt * wz ];

end











