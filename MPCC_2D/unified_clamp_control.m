function u_clamped = unified_clamp_control(u, a_max, w_max, v_progress_max, control_type)
% 统一的控制输入约束函数，支持MPC和MPCC
% 输入:
%   u - 原始控制输入
%   a_max - 最大加速度
%   w_max - 最大角速度
%   v_progress_max - 最大进度速度 (仅MPCC需要)
%   control_type - 控制类型: 'mpc' 或 'mpcc'
% 输出:
%   u_clamped - 约束后的控制输入

switch control_type
    case 'mpc'
        % MPC控制: [a; omega]
        if length(u) ~= 2
            error('MPC控制输入必须是2维向量 [a; omega]');
        end
        
        u_clamped = [ ...
            min(max(u(1), -a_max), a_max);    % 约束加速度 [-a_max, a_max]
            min(max(u(2), -w_max), w_max)];   % 约束角速度 [-w_max, w_max]
        
    case 'mpcc'
        % MPCC控制: [a; omega; v_progress]
        if length(u) ~= 3
            error('MPCC控制输入必须是3维向量 [a; omega; v_progress]');
        end
        
        u_clamped = [ ...
            min(max(u(1), -a_max), a_max);           % 约束加速度 [-a_max, a_max]
            min(max(u(2), -w_max), w_max);           % 约束角速度 [-w_max, w_max]
            min(max(u(3), 0), v_progress_max)];      % 约束进度速度 [0, v_progress_max]
        
    otherwise
        error('Unknown control_type: %s', control_type);
end

end

