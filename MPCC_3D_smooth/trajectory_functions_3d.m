function [r, r1, r2, S_end, params] = trajectory_functions_3d(traj_type, S_target)
% TRAJECTORY_FUNCTIONS_3D Provide 3D reference path and derivatives by arc length s
% Inputs:
%   traj_type: 'line' | 'circle' | 'helix'
%   S_target: (optional) Target trajectory length [m]. If provided, trajectory
%             parameters will be adjusted to match this length.
% Outputs:
%   r(s)  -> R^3 position
%   r1(s) -> first derivative wrt s
%   r2(s) -> second derivative wrt s
%   S_end -> terminal arc length

if nargin < 1
    traj_type = 'helix';
end
if nargin < 2
    S_target = [];
end

switch lower(traj_type)
    case 'line'
        % Straight line from origin along x, length L
        if isempty(S_target)
            L = 20;  % Default length
        else
            L = S_target;  % Use target length directly
        end
        params = struct('L', L);
        r = @(s) [s; 0*s; 0*s];
        r1 = @(s) [1; 0; 0];
        r2 = @(s) [0; 0; 0];
        S_end = L;

    case 'circle'
        % Circle in xy-plane, radius R
        if isempty(S_target)
            R = 5;  % Default radius
            turns = 1;  % One full circle
        else
            % Adjust radius or number of turns to match target length
            % Keep default radius, adjust number of turns
            R = 5;
            turns = S_target / (2*pi*R);
        end
        params = struct('R', R, 'turns', turns);
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
        S_end = 2*pi*R * turns;

    case 'helix'
        % Helix of radius R, height H over multiple turns
        R = 3;  % Keep default radius
        pitch_per_turn = 6;  % Height per turn (default: 6m per turn)
        
        if isempty(S_target)
            turns = 3;  % Default number of turns
        else
            % Calculate number of turns needed to reach target length
            % S_end = 2*pi*turns * sqrt(R^2 + pitch^2)
            % where pitch = pitch_per_turn / (2*pi) = pitch_per_turn / (2*pi)
            % Actually: LperTheta = sqrt(R^2 + (H/(2*pi*turns))^2)
            % S_end = 2*pi*turns * LperTheta = 2*pi*turns * sqrt(R^2 + (pitch_per_turn/(2*pi))^2)
            LperTurn = sqrt(R^2 + (pitch_per_turn/(2*pi))^2) * 2*pi;
            turns = max(0.1, S_target / LperTurn);  % At least 0.1 turns
        end
        
        H = pitch_per_turn * turns;
        params = struct('R', R, 'H', H, 'turns', turns);
        
        % True arc-length parameterization of helix with constant speed
        pitch = H / (2*pi*turns);
        LperTheta = sqrt(R^2 + pitch^2);
        theta_of_s = @(s) s / LperTheta;
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


