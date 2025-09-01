function s_new = update_arc_length(s_old, x, r, r1, r2)
% UPDATE_ARC_LENGTH 根据当前位置更新弧长进度
% 输入:
%   s_old - 当前弧长
%   x - 当前状态 [px,py,pz,yaw,pitch,v]
%   r - 位置函数
%   r1 - 一阶导数函数
% 输出:
%   s_new - 新的弧长进度

% 找到距离当前位置最近的路径点
p_current = x(1:3);
s_search = linspace(max(0, s_old-5), min(s_old+5, 100), 1000); % 搜索范围
distances = zeros(size(s_search));

for i = 1:length(s_search)
    p_ref = r(s_search(i));
    distances(i) = norm(p_current - p_ref);
end

[~, idx] = min(distances);
s_new = s_search(idx);

end

