function [r_noisy, r1_noisy, r2_noisy] = add_trajectory_noise_3d(r, r1, r2, noise_type, noise_params, S_end)
% ADD_TRAJECTORY_NOISE_3D Add spatial noise to a 3D reference path r(s)
% Inputs:
%   r, r1, r2: original trajectory function handles (R^3)
%   noise_type: 'gaussian' | 'uniform' | 'uniform_time' | 'uniform_arc' | 'ar' | 'ma' | 'arma' | 'none'
%   noise_params: struct with fields depending on type
%       gaussian: .std (meters)
%       uniform:  .range (meters)
%       ar:       .phi (1xp), .std_eps (m), optional .smooth_win
%       ma:       .theta (1xq), .std_eps (m), optional .smooth_win
%       arma:     .phi (1xp), .theta (1xq), .std_eps (m), optional .smooth_win
%   S_end: total arc length
% Outputs:
%   r_noisy, r1_noisy, r2_noisy: noisy trajectory and derivatives

switch lower(noise_type)
    case 'gaussian'
        ss = linspace(0, S_end, 1000);
        nx = noise_params.std * randn(1, numel(ss));
        ny = noise_params.std * randn(1, numel(ss));
        nz = noise_params.std * randn(1, numel(ss));

        % Smooth to keep derivatives well-behaved
        win = 50;
        nx_s = smoothdata(nx, 'gaussian', win);
        ny_s = smoothdata(ny, 'gaussian', win);
        nz_s = smoothdata(nz, 'gaussian', win);

        % Position noise via interpolation
        r_noisy = @(s) r(s) + [
            interp1(ss, nx_s, s, 'pchip', 'extrap');
            interp1(ss, ny_s, s, 'pchip', 'extrap');
            interp1(ss, nz_s, s, 'pchip', 'extrap')];

        % Derivative noise from spatial gradient wrt s (reduced gain)
        dnx = gradient(nx_s, ss); dny = gradient(ny_s, ss); dnz = gradient(nz_s, ss);
        ddnx = gradient(dnx, ss); ddny = gradient(dny, ss); ddnz = gradient(dnz, ss);

        r1_noisy = @(s) r1(s) + 0.1 * [
            interp1(ss, dnx, s, 'pchip', 'extrap');
            interp1(ss, dny, s, 'pchip', 'extrap');
            interp1(ss, dnz, s, 'pchip', 'extrap')];

        r2_noisy = @(s) r2(s) + 0.01 * [
            interp1(ss, ddnx, s, 'pchip', 'extrap');
            interp1(ss, ddny, s, 'pchip', 'extrap');
            interp1(ss, ddnz, s, 'pchip', 'extrap')];

    case {'ar','ma','arma'}
        % Spatially-correlated noise along arc-length s (meters).
        % Interpretation:
        %   e[k] ~ N(0, std_eps^2) are i.i.d innovations in meters.
        %   Noise sequence n[k] follows AR/MA/ARMA recursion in discrete index k,
        %   where k corresponds to uniform samples of s in [0, S_end].
        if ~isfield(noise_params,'std_eps')
            error('%s noise requires noise_params.std_eps (meters)', lower(noise_type));
        end
        if any(strcmpi(noise_type, {'ar','arma'})) && ~isfield(noise_params,'phi')
            error('%s noise requires noise_params.phi', lower(noise_type));
        end
        if any(strcmpi(noise_type, {'ma','arma'})) && ~isfield(noise_params,'theta')
            error('%s noise requires noise_params.theta', lower(noise_type));
        end

        ss = linspace(0, S_end, 1000);
        if isfield(noise_params,'phi'),   phi = noise_params.phi(:).'; else, phi = []; end
        if isfield(noise_params,'theta'), th  = noise_params.theta(:).'; else, th = []; end

        epsx = noise_params.std_eps * randn(1, numel(ss));
        epsy = noise_params.std_eps * randn(1, numel(ss));
        epsz = noise_params.std_eps * randn(1, numel(ss));

        a = [1, -phi];      % AR denominator
        b = [1,  th];       % MA numerator
        nx = filter(b, a, epsx);
        ny = filter(b, a, epsy);
        nz = filter(b, a, epsz);

        % Optional smoothing to keep derivatives well-behaved
        if isfield(noise_params,'smooth_win')
            win = max(0, round(noise_params.smooth_win));
        else
            win = 25;
        end
        if win > 1
            nx = smoothdata(nx, 'gaussian', win);
            ny = smoothdata(ny, 'gaussian', win);
            nz = smoothdata(nz, 'gaussian', win);
        end

        r_noisy = @(s) r(s) + [
            interp1(ss, nx, s, 'pchip', 'extrap');
            interp1(ss, ny, s, 'pchip', 'extrap');
            interp1(ss, nz, s, 'pchip', 'extrap')];

        dnx = gradient(nx, ss); dny = gradient(ny, ss); dnz = gradient(nz, ss);
        ddnx = gradient(dnx, ss); ddny = gradient(dny, ss); ddnz = gradient(dnz, ss);

        r1_noisy = @(s) r1(s) + 0.1 * [
            interp1(ss, dnx, s, 'pchip', 'extrap');
            interp1(ss, dny, s, 'pchip', 'extrap');
            interp1(ss, dnz, s, 'pchip', 'extrap')];
        r2_noisy = @(s) r2(s) + 0.01 * [
            interp1(ss, ddnx, s, 'pchip', 'extrap');
            interp1(ss, ddny, s, 'pchip', 'extrap');
            interp1(ss, ddnz, s, 'pchip', 'extrap')];

    case 'uniform'
        ss = linspace(0, S_end, 1000);
        nx = noise_params.range * (2*rand(1, numel(ss)) - 1);
        ny = noise_params.range * (2*rand(1, numel(ss)) - 1);
        nz = noise_params.range * (2*rand(1, numel(ss)) - 1);

        r_noisy = @(s) r(s) + [
            interp1(ss, nx, s, 'linear', 'extrap');
            interp1(ss, ny, s, 'linear', 'extrap');
            interp1(ss, nz, s, 'linear', 'extrap')];

        % Light derivative noise
        rnd1 = @(N) 2*rand(1,N)-1;
        r1_noisy = @(s) r1(s) + noise_params.range * 0.05 * [
            interp1(ss, rnd1(numel(ss)), s, 'linear', 'extrap');
            interp1(ss, rnd1(numel(ss)), s, 'linear', 'extrap');
            interp1(ss, rnd1(numel(ss)), s, 'linear', 'extrap')];
        r2_noisy = @(s) r2(s) + noise_params.range * 0.01 * [
            interp1(ss, rnd1(numel(ss)), s, 'linear', 'extrap');
            interp1(ss, rnd1(numel(ss)), s, 'linear', 'extrap');
            interp1(ss, rnd1(numel(ss)), s, 'linear', 'extrap')];

    case 'uniform_time'
        % Time-triggered piecewise rigid perturbations every period_sec
        % Params: .sdot_nom (m/s), .period_sec, .rot_deg, .trans_amp
        if ~isfield(noise_params,'sdot_nom'),   noise_params.sdot_nom = 1.0; end
        if ~isfield(noise_params,'period_sec'), noise_params.period_sec = 10.0; end
        if ~isfield(noise_params,'rot_deg'),    noise_params.rot_deg = 0; end   % about z-axis
        if ~isfield(noise_params,'trans_amp'),  noise_params.trans_amp = 0.2; end % meters

        sdot_nom = noise_params.sdot_nom;
        Tper = noise_params.period_sec;
        rot_rad = noise_params.rot_deg * pi/180;
        Atrans = noise_params.trans_amp;

        % Deterministic pseudo-random transform per time window index k
        angle_k = @(k) rot_rad * sin(12.9898*k + 0.5);
        trans_k = @(k) Atrans * [
            sin(78.233*k + 0.3);
            cos(39.425*k + 0.7);
            sin(17.130*k + 1.1)];

        Rz = @(th) [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];

        % Map arc-length to (nominal) time
        t_of_s = @(s) s / max(sdot_nom, 1e-9);
        k_of_s = @(s) floor(t_of_s(s) / Tper);

        r_noisy = @(s) (Rz(angle_k(k_of_s(s))) * r(s) + trans_k(k_of_s(s)));
        r1_noisy = @(s) (Rz(angle_k(k_of_s(s))) * r1(s));
        r2_noisy = @(s) (Rz(angle_k(k_of_s(s))) * r2(s));

    case 'uniform_arc'
        % Arc-length-triggered piecewise rigid perturbations every period_arc (meters)
        % Params: .period_arc (m), .rot_deg (deg), .trans_amp (m)
        if ~isfield(noise_params,'period_arc'), noise_params.period_arc = 6.0; end
        if ~isfield(noise_params,'rot_deg'),    noise_params.rot_deg    = 0.00;   end
        if ~isfield(noise_params,'trans_amp'),  noise_params.trans_amp  = 0.05; end

        Sparc  = max(noise_params.period_arc, 1e-6);
        rot_rad = noise_params.rot_deg * pi/180;
        Atrans = noise_params.trans_amp;

        % Deterministic pseudo-random transform per arc window index k
        angle_k = @(k) rot_rad * sin(12.9898*k + 0.5);
        trans_k = @(k) Atrans * [
            sin(78.233*k + 0.3);
            cos(39.425*k + 0.7);
            sin(17.130*k + 1.1)];

        Rz = @(th) [cos(th) -sin(th) 0; sin(th) cos(th) 0; 0 0 1];
        k_of_s = @(s) floor(s / Sparc);

        r_noisy = @(s) (Rz(angle_k(k_of_s(s))) * r(s)  + trans_k(k_of_s(s)));
        r1_noisy = @(s) (Rz(angle_k(k_of_s(s))) * r1(s));
        r2_noisy = @(s) (Rz(angle_k(k_of_s(s))) * r2(s));

    case 'none'
        r_noisy = r; r1_noisy = r1; r2_noisy = r2;

    otherwise
        error('Unknown noise type: %s. Use ''gaussian'', ''uniform'', ''uniform_time'', ''uniform_arc'', ''ar'', ''ma'', ''arma'', or ''none''', noise_type);
end

end






