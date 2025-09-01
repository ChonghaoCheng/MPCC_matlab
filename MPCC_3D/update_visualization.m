function update_visualization(h_mpc, h_mpcc, h_pred1, h_pred2, h_a1, h_wy1, h_wp1, h_a2, h_wy2, h_wp2, ...
                           h_ep1, h_ep2, h_ec, h_el, X_mpc, X_mpcc, xpred_mpc, xpred_mpcc, U_mpc, U_mpcc, ...
                           PosErr_mpc, PosErr_mpcc, ErrC, ErrL, T, k)
% UPDATE_VISUALIZATION 更新可视化显示
% 输入:
%   各种图形句柄和数据

% 3D trajectories
nexttile(1);
set(h_mpc, 'XData', X_mpc(1,1:k),  'YData', X_mpc(2,1:k),  'ZData', X_mpc(3,1:k),  'Color',[0 0.45 0.74]);
set(h_mpcc,'XData', X_mpcc(1,1:k), 'YData', X_mpcc(2,1:k), 'ZData', X_mpcc(3,1:k), 'Color',[0.85 0.33 0.1]);
set(h_pred1,'XData', xpred_mpc(1,:),  'YData', xpred_mpc(2,:),  'ZData', xpred_mpc(3,:),  'Color',[0 0.45 0.74]);
set(h_pred2,'XData', xpred_mpcc(1,:),'YData', xpred_mpcc(2,:),'ZData', xpred_mpcc(3,:),'Color',[0.85 0.33 0.1]);
drawnow limitrate;

% controls
nexttile(2);
set(h_a1,'XData',T(1:k),'YData',U_mpc(1,1:k),'Color',[0 0.45 0.74]);
set(h_wy1,'XData',T(1:k),'YData',U_mpc(2,1:k),'Color',[0 0.45 0.74],'LineStyle','--');
set(h_wp1,'XData',T(1:k),'YData',U_mpc(3,1:k),'Color',[0 0.45 0.74],'LineStyle',':');
set(h_a2,'XData',T(1:k),'YData',U_mpcc(1,1:k),'Color',[0.85 0.33 0.1]);
set(h_wy2,'XData',T(1:k),'YData',U_mpcc(2,1:k),'Color',[0.85 0.33 0.1],'LineStyle','--');
set(h_wp2,'XData',T(1:k),'YData',U_mpcc(3,1:k),'Color',[0.85 0.33 0.1],'LineStyle',':');

% pos error
nexttile(3);
set(h_ep1,'XData',T(1:k),'YData',PosErr_mpc(1:k),'Color',[0 0.45 0.74]);
set(h_ep2,'XData',T(1:k),'YData',PosErr_mpcc(1:k),'Color',[0.85 0.33 0.1]);

% MPCC e_c/e_l
nexttile(4);
set(h_ec,'XData',T(1:k),'YData',ErrC(1:k),'Color',[0.85 0.33 0.1]);
set(h_el,'XData',T(1:k),'YData',ErrL(1:k),'Color',[0.49 0.18 0.56]);
drawnow limitrate;

end

