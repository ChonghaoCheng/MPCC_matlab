function [A, B] = mpc_linearize_3d(xbar, ubar, dt)
% Linearization for 3D MPC (state 6, input 6)
% x = [x;y;z; psi_x;psi_y;psi_z]
% u = [v_x;v_y;v_z; omega_x;omega_y;omega_z]

N = size(ubar,2);
A = zeros(6,6,N);
B = zeros(6,6,N);

for k = 1:N
    A(:,:,k) = [
        1 0 0 0 0 0;
        0 1 0 0 0 0;
        0 0 1 0 0 0;
        0 0 0 1 0 0;
        0 0 0 0 1 0;
        0 0 0 0 0 1];
    B(:,:,k) = [
        dt 0  0  0  0  0;
        0  dt 0  0  0  0;
        0  0  dt 0  0  0;
        0  0  0  dt 0  0;
        0  0  0  0  dt 0;
        0  0  0  0  0  dt];
end

end














