function [Ax,Bu,d] = condensed_dynamics(A,B,c)
% CONDENSED_DYNAMICS 构建凝聚动力学矩阵
% 输入:
%   A, B, c - 线性化矩阵和偏移项
% 输出:
%   Ax, Bu, d - 凝聚矩阵

nx = size(A,1); nu = size(B,2); N = size(B,3);
Ax = zeros(nx*N, nx); Bu = zeros(nx*N, nu*N); d = zeros(nx*N,1);

for k=1:N
    Ak1 = eye(nx);
    for j=1:k
        Ak1 = A(:,:,j)*Ak1;
    end
    Ax((k-1)*nx+(1:nx),:) = Ak1;
    
    for j=1:k
        Phi = eye(nx);
        for m=j+1:k
            Phi = A(:,:,m)*Phi;
        end
        Bu((k-1)*nx+(1:nx),(j-1)*nu+(1:nu)) = Phi*B(:,:,j);
    end
    
    cj = zeros(nx,1);
    for j=1:k
        Phi = eye(nx);
        for m=j+1:k
            Phi = A(:,:,m)*Phi;
        end
        cj = cj + Phi*c(:,j);
    end
    d((k-1)*nx+(1:nx)) = cj;
end

end

