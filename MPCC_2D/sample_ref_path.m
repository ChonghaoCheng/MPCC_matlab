function [xref, uref, svec] = sample_ref_path(s0, N, ds, r, r1, r2, v_ref_nom, theta0)
% SAMPLE_REF_PATH 沿路径从s0起采样参考轨迹
%
% 输入:
%   s0: 起始弧长
%   N: 预测时域步数
%   ds: 弧长步长
%   r, r1, r2: 轨迹函数
%   v_ref_nom: 标称参考速度
%
% 输出:
%   xref: 参考状态序列 [4 x (N+1)]
%   uref: 参考控制序列 [2 x N]
%   svec: 弧长序列 [1 x (N+1)]

    svec = s0 + (0:N)*ds;
    xref = zeros(4, N+1);
    uref = zeros(2, N);
    psi_raw = zeros(1, N+1);

    for k = 1:N+1
        [~,~,kappa,psi] = geom_at(svec(k), r, r1, r2);
        p    = r(svec(k));
        vref = v_ref_nom;
        psi_raw(k) = psi;
        xref(:,k)  = [p(1); p(2); psi; vref];
        if k <= N
            uref(:,k) = [0; kappa*vref]; % omega_ff = kappa*v
        end
    end

    % unwrap reference heading relative to current theta0
    xref(3,:) = unwrap_heading_seq(psi_raw, theta0);
end
