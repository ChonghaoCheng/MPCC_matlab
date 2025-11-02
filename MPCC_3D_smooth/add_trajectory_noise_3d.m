function [r_noisy, r1_noisy, r2_noisy] = add_trajectory_noise_3d(r, r1, r2, noise_type, noise_params, S_end)
% ADD_TRAJECTORY_NOISE_3D 为3D参考路径添加空间噪声
%
% 输入:
%   r, r1, r2: 原始轨迹函数句柄 (R^3)
%   noise_type: 'gaussian' | 'rotation_only' | 'translation_only' | 'none'
%   noise_params: 结构体，包含噪声参数
%       gaussian:        .std (米) - 高斯噪声标准差
%       rotation_only:   .sdot_nom (m/s), .period_sec, .rot_deg
%       translation_only: .sdot_nom (m/s), .period_sec, .trans_amp, .trans_dir
%   S_end: 总弧长
%
% 输出:
%   r_noisy, r1_noisy, r2_noisy: 带噪声的轨迹及其导数

switch lower(noise_type)
    case 'gaussian'
        % Gaussian spatial noise
        if ~isfield(noise_params,'std')
            error('gaussian noise requires noise_params.std');
        end
        
        ss = linspace(0, S_end, 1000);
        nx = noise_params.std * randn(1, numel(ss));
        ny = noise_params.std * randn(1, numel(ss));
        nz = noise_params.std * randn(1, numel(ss));

        win = 50;
        nx_s = smoothdata(nx, 'gaussian', win);
        ny_s = smoothdata(ny, 'gaussian', win);
        nz_s = smoothdata(nz, 'gaussian', win);

        r_noisy = @(s) r(s) + [
            interp1(ss, nx_s, s, 'pchip', 'extrap');
            interp1(ss, ny_s, s, 'pchip', 'extrap');
            interp1(ss, nz_s, s, 'pchip', 'extrap')];

        dnx = gradient(nx_s, ss); 
        dny = gradient(ny_s, ss); 
        dnz = gradient(nz_s, ss);
        ddnx = gradient(dnx, ss); 
        ddny = gradient(dny, ss); 
        ddnz = gradient(dnz, ss);

        r1_noisy = @(s) r1(s) + 0.1 * [
            interp1(ss, dnx, s, 'pchip', 'extrap');
            interp1(ss, dny, s, 'pchip', 'extrap');
            interp1(ss, dnz, s, 'pchip', 'extrap')];

        r2_noisy = @(s) r2(s) + 0.01 * [
            interp1(ss, ddnx, s, 'pchip', 'extrap');
            interp1(ss, ddny, s, 'pchip', 'extrap');
            interp1(ss, ddnz, s, 'pchip', 'extrap')];

    case 'rotation_only'
        % Rotation-only: 时间触发的分段旋转（绕z轴）
        if ~isfield(noise_params,'sdot_nom'),   noise_params.sdot_nom = 1.0; end
        if ~isfield(noise_params,'period_sec'), noise_params.period_sec = 10.0; end
        if ~isfield(noise_params,'rot_deg'),    noise_params.rot_deg = 5.0; end

        sdot_nom = noise_params.sdot_nom;
        Tper = noise_params.period_sec;
        rot_rad = noise_params.rot_deg * pi/180;

        % 直接在匿名函数内部计算所有逻辑
        r_noisy = @(s) apply_rotation_z(r(s), s, sdot_nom, Tper, rot_rad);
        r1_noisy = @(s) apply_rotation_z(r1(s), s, sdot_nom, Tper, rot_rad);
        r2_noisy = @(s) apply_rotation_z(r2(s), s, sdot_nom, Tper, rot_rad);

    case 'translation_only'
        % Translation-only: 时间触发的分段平移
        if ~isfield(noise_params,'sdot_nom'),   noise_params.sdot_nom = 1.0; end
        if ~isfield(noise_params,'period_sec'), noise_params.period_sec = 10.0; end
        if ~isfield(noise_params,'trans_amp'),  noise_params.trans_amp = 0.2; end
        if ~isfield(noise_params,'trans_dir'),  noise_params.trans_dir = [1; 0; 0]; end

        sdot_nom = noise_params.sdot_nom;
        Tper = noise_params.period_sec;
        Atrans = noise_params.trans_amp;
        trans_dir = noise_params.trans_dir / max(norm(noise_params.trans_dir), eps);

        % 直接在匿名函数内部计算所有逻辑
        r_noisy = @(s) apply_translation(r(s), s, sdot_nom, Tper, Atrans, trans_dir);
        r1_noisy = @(s) r1(s);  % 平移不改变导数
        r2_noisy = @(s) r2(s);  % 平移不改变导数

    case 'none'
        r_noisy = r; 
        r1_noisy = r1; 
        r2_noisy = r2;

    otherwise
        error('Unknown noise type: %s. Use ''gaussian'', ''rotation_only'', ''translation_only'', or ''none''', noise_type);
end

end

% Helper functions (defined at file level, not nested)
function p_out = apply_rotation_z(p_in, s, sdot_nom, Tper, rot_rad)
    % 计算当前时间窗口索引
    t = s / max(sdot_nom, 1e-9);
    k = floor(t / Tper);
    % 根据窗口索引计算旋转角度
    theta = rot_rad * sin(12.9898*k + 0.5);
    % 构建绕z轴旋转矩阵
    Rz = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
    % 应用旋转
    p_out = Rz * p_in;
end

function p_out = apply_translation(p_in, s, sdot_nom, Tper, Atrans, trans_dir)
    % 计算当前时间窗口索引
    t = s / max(sdot_nom, 1e-9);
    k = floor(t / Tper);
    % 交替符号：+1, -1, +1, -1, ...
    sign_k = (-1)^k;
    % 计算平移向量并应用
    p_out = p_in + Atrans * trans_dir * sign_k;
end

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
        error('Unknown noise type: %s. Use ''gaussian'', ''uniform'', or ''none''', noise_type);
end

end






