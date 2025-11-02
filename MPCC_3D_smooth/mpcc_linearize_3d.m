function [A, B, c] = mpcc_linearize_3d(xbar, ubar, dt)
% Linearization for 3D MPCC (state 7, input 7)
% x = [x;y;z; psi_x;psi_y;psi_z; s]
% u = [v_x;v_y;v_z; omega_x;omega_y;omega_z; v_s]

N = size(ubar,2);
A = zeros(7,7,N);
B = zeros(7,7,N);
c = zeros(7,N);

for k = 1:N
    A(:,:,k) = [
        1 0 0 0 0 0 0;
        0 1 0 0 0 0 0;
        0 0 1 0 0 0 0;
        0 0 0 1 0 0 0;
        0 0 0 0 1 0 0;
        0 0 0 0 0 1 0;
        0 0 0 0 0 0 1];
    B(:,:,k) = [
        dt 0  0  0  0  0  0;
        0  dt 0  0  0  0  0;
        0  0  dt 0  0  0  0;
        0  0  0  dt 0  0  0;
        0  0  0  0  dt 0  0;
        0  0  0  0  0  dt 0;
        0  0  0  0  0  0  dt];
    xnext = mpcc_dynamics_3d(xbar(:,k), ubar(:,k), dt);
    c(:,k) = xnext - A(:,:,k)*xbar(:,k) - B(:,:,k)*ubar(:,k);
end

end














