function stats = calculate_final_stats(X_mpc, X_mpcc, Smpc, Smpcc, r, S_end)
% CALCULATE_FINAL_STATS 计算最终统计结果
% 输入:
%   X_mpc, X_mpcc - MPC和MPCC的状态轨迹
%   Smpc, Smpcc - MPC和MPCC的弧长进度
%   r - 位置函数
%   S_end - 轨迹总弧长
% 输出:
%   stats - 统计结果表格

rmse = @(e) sqrt(mean(e.^2));
mae  = @(e) mean(abs(e));

% 计算参考位置
Pref_mpc  = cell2mat(arrayfun(@(s) r(min(s,S_end)), Smpc,  'UniformOutput', false));
Pref_mpcc = cell2mat(arrayfun(@(s) r(min(s,S_end)), Smpcc, 'UniformOutput', false));

% 计算位置误差
e_mpc  = vecnorm(X_mpc(1:3,:)  - Pref_mpc,  2,1);
e_mpcc = vecnorm(X_mpcc(1:3,:) - Pref_mpcc, 2,1);

% 创建统计表格
stats = table( ...
    rmse(e_mpc).',  mae(e_mpc).',  max(e_mpc).', ...
    rmse(e_mpcc).', mae(e_mpcc).', max(e_mpcc).', ...
    'VariableNames', {'RMSE_MPC','MAE_MPC','Max_MPC','RMSE_MPCC','MAE_MPCC','Max_MPCC'});

end



