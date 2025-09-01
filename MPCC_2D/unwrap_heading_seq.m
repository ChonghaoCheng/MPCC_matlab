function seq_out = unwrap_heading_seq(seq_in, start_angle)
% Unwrap heading sequence anchored at start_angle (continuous reference).
% 以 start_angle 为锚，把 psi 序列展开成连续角度，避免 ±pi 跳变。
    seq_out = seq_in;
    if isempty(seq_in), return; end
    seq_out(1) = start_angle + wrapDiff(seq_in(1), start_angle);
    for k = 2:numel(seq_in)
        prev = seq_out(k-1);
        raw  = seq_in(k);
        seq_out(k) = prev + wrapDiff(raw, prev);
    end
end