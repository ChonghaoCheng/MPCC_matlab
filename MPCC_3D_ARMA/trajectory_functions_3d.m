function [r, r1, r2, S_end, params] = trajectory_functions_3d(traj_type)
% TRAJECTORY_FUNCTIONS_3D Provide 3D reference path and derivatives by arc length s
% Outputs:
%   r(s)  -> R^3 position
%   r1(s) -> first derivative wrt s
%   r2(s) -> second derivative wrt s
%   S_end -> terminal arc length

if nargin < 1
    traj_type = 'helix';
end

switch lower(traj_type)
    case 'line'
        % Straight line from origin along x, length L
        L = 20;
        params = struct('L', L);
        r = @(s) [s; 0*s; 0*s];
        r1 = @(s) [1; 0; 0];
        r2 = @(s) [0; 0; 0];
        S_end = L;

    case 'circle'
        % Circle in xy-plane, radius R, one lap
        R = 5;
        params = struct('R', R);
        % Arc length s maps to angle theta = s/R
        r = @(s) [ R*cos(s/R);
                   R*sin(s/R);
                   0*s ];
        r1 = @(s) [ -sin(s/R);
                     cos(s/R);
                     0];
        r2 = @(s) [ -(1/R)*cos(s/R);
                    -(1/R)*sin(s/R);
                     0];
        S_end = 2*pi*R;

    case 'helix'
        % Helix of radius R, height H over multiple turns (longer path)
        R = 3; turns = 3; H = 6*turns;  % keep pitch per turn same as 1-turn case
        params = struct('R', R, 'H', H, 'turns', turns);
        % True arc-length parameterization of helix with constant speed
        % Let base angle theta(s) = s / sqrt(R^2 + (H/(2pi))^2) / R
        pitch = H / (2*pi*turns);
        LperTheta = sqrt(R^2 + pitch^2);
        theta_of_s = @(s) s / LperTheta / R * R; % simplifies to s / LperTheta
        r = @(s) [ R*cos(theta_of_s(s));
                   R*sin(theta_of_s(s));
                   pitch*theta_of_s(s) ];
        r1 = @(s) [ -R*sin(theta_of_s(s)) * (1/LperTheta);
                     R*cos(theta_of_s(s)) * (1/LperTheta);
                     pitch * (1/LperTheta) ];
        r2 = @(s) [ -R*cos(theta_of_s(s)) * (1/LperTheta)^2 * (1/R)*R;
                    -R*sin(theta_of_s(s)) * (1/LperTheta)^2 * (1/R)*R;
                     0 ];
        S_end = 2*pi*turns * LperTheta;

    otherwise
        error('Unknown 3D trajectory type: %s', traj_type);
end

end


