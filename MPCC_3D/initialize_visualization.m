function [h_mpc, h_mpcc, h_pred1, h_pred2, h_a1, h_wy1, h_wp1, h_a2, h_wy2, h_wp2, h_ep1, h_ep2, h_ec, h_el] = initialize_visualization(S_end, r)
% INITIALIZE_VISUALIZATION 初始化可视化界面
% 输入:
%   S_end - 轨迹总弧长
%   r - 位置函数
% 输出:
%   各种图形句柄

figure('Color','w','Name','3D MPCC vs MPC'); 
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

% 3D path plot
nexttile(1); hold on; grid on; box on; axis vis3d; view(40,25);
ss = linspace(0,S_end,1200);
PP = cell2mat(arrayfun(@(s) r(s), ss, 'UniformOutput', false));
plot3(PP(1,:), PP(2,:), PP(3,:), 'k--','LineWidth',1.2);
h_mpc  = plot3(NaN,NaN,NaN,'-','LineWidth',1.5);
h_mpcc = plot3(NaN,NaN,NaN,'-','LineWidth',1.5);
h_pred1 = plot3(NaN,NaN,NaN,'o-','MarkerSize',3);
h_pred2 = plot3(NaN,NaN,NaN,'o-','MarkerSize',3);
legend({'path','MPC','MPCC','MPC pred','MPCC pred'},'Location','best');
title('3D Trajectories'); xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');

% controls
nexttile(2); hold on; grid on; box on;
h_a1 = plot(NaN,NaN,'-'); h_wy1 = plot(NaN,NaN,'-'); h_wp1 = plot(NaN,NaN,'-');
h_a2 = plot(NaN,NaN,'-'); h_wy2 = plot(NaN,NaN,'-'); h_wp2 = plot(NaN,NaN,'-');
legend({'a (MPC)','\omega_y (MPC)','\omega_p (MPC)', 'a (MPCC)','\omega_y (MPCC)','\omega_p (MPCC)'},'Location','best');
title('Controls'); xlabel('t [s]'); ylabel('u');

% position error
nexttile(3); hold on; grid on; box on;
h_ep1 = plot(NaN,NaN,'-'); 
h_ep2 = plot(NaN,NaN,'-');
legend({'pos err MPC','pos err MPCC'},'Location','best');
title('Position error'); xlabel('t [s]'); ylabel('|p - p^*| [m]');

% MPCC contour/lag
nexttile(4); hold on; grid on; box on;
h_ec = plot(NaN,NaN,'-'); h_el = plot(NaN,NaN,'-');
legend({'contouring (MPCC)','lag (MPCC)'},'Location','best');
title('MPCC errors'); xlabel('t [s]'); ylabel('m');

drawnow;

end



