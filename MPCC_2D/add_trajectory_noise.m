function [r_noisy, r1_noisy, r2_noisy] = add_trajectory_noise(r, r1, r2, noise_type, noise_params, S_end)
% ADD_TRAJECTORY_NOISE 给轨迹添加位置噪声
%
% 输入:
%   r, r1, r2: 原始轨迹函数
%   noise_type: 噪声类型 ('gaussian', 'uniform', 'none')
%   noise_params: 噪声参数结构体
%   S_end: 轨迹总长度
%
% 输出:
%   r_noisy, r1_noisy, r2_noisy: 带噪声的轨迹函数

switch noise_type
    case 'gaussian'
        % 高斯噪声：为每个弧长点生成固定的噪声
        ss = linspace(0, S_end, 1000);  % 采样点
        noise_x = noise_params.std * randn(1, length(ss));
        noise_y = noise_params.std * randn(1, length(ss));
        
        % 使用平滑插值确保导数连续性
        noise_x_smooth = smoothdata(noise_x, 'gaussian', 50);  % 高斯平滑
        noise_y_smooth = smoothdata(noise_y, 'gaussian', 50);
        
        % 创建带噪声的轨迹函数
        r_noisy = @(s) r(s) + [interp1(ss, noise_x_smooth, s, 'pchip', 'extrap'); 
                                interp1(ss, noise_y_smooth, s, 'pchip', 'extrap')];
        
        % 导数噪声（使用数值微分）
        r1_noisy = @(s) r1(s) + [interp1(ss, gradient(noise_x_smooth, ss), s, 'pchip', 'extrap');
                                  interp1(ss, gradient(noise_y_smooth, ss), s, 'pchip', 'extrap')] * 0.1;
        
        % 二阶导数噪声（更小）
        r2_noisy = @(s) r2(s) + [interp1(ss, gradient(gradient(noise_x_smooth, ss), ss), s, 'pchip', 'extrap');
                                   interp1(ss, gradient(gradient(noise_y_smooth, ss), ss), s, 'pchip', 'extrap')] * 0.01;
        
    case 'uniform'
        % 均匀噪声：为每个弧长点生成固定的噪声
        ss = linspace(0, S_end, 1000);
        noise_x = noise_params.range * (2*rand(1, length(ss)) - 1);
        noise_y = noise_params.range * (2*rand(1, length(ss)) - 1);
        
        r_noisy = @(s) r(s) + [interp1(ss, noise_x, s, 'linear', 'extrap'); 
                                interp1(ss, noise_y, s, 'linear', 'extrap')];
        
        r1_noisy = @(s) r1(s) + noise_params.range * 0.05 * [interp1(ss, 2*rand(1,length(ss))-1, s, 'linear', 'extrap');
                                                               interp1(ss, 2*rand(1,length(ss))-1, s, 'linear', 'extrap')];
        
        r2_noisy = @(s) r2(s) + noise_params.range * 0.01 * [interp1(ss, 2*rand(1,length(ss))-1, s, 'linear', 'extrap');
                                                                interp1(ss, 2*rand(1,length(ss))-1, s, 'linear', 'extrap')];
        
    case 'none'
        % 无噪声
        r_noisy = r;
        r1_noisy = r1;
        r2_noisy = r2;
        
    otherwise
        error('Unknown noise type: %s. Use ''gaussian'', ''uniform'', or ''none''', noise_type);
end

end
