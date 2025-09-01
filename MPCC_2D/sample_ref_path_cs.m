function [xref, uref] = sample_ref_path_cs(s0, N, ds, r, r1, r2, v_ref)
% xref: 5x(N+1) = [x_r; y_r; c_r; s_r; v_r]
% uref: 2xN   = [a_r; omega_r]  （可全零或按曲率给个名义值）
    xref = zeros(5, N+1);
    uref = zeros(2, N);
    s = s0;
    for k = 1:N+1
        p  = r(min(s, inf));
        t  = r1(min(s, inf));    % 切向（单位）
        psi = atan2(t(2), t(1));
        xref(:,k) = [p(1); p(2); cos(psi); sin(psi); v_ref];
        if k<=N
            % 名义控制（可选）：a=0，omega=曲率*v_ref
            n  = r2(min(s, inf));                % 法向
            kappa = (t(1)*n(2)-t(2)*n(1));       % 曲率（若你的 r1,r2 已单位化，则近似）
            uref(:,k) = [0; kappa*v_ref];
        end
        s = s + ds;
    end
end
