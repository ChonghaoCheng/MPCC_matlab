function Ad = adjointSE3(T)
R = T(1:3, 1:3);
p = T(1:3, 4);
Ad = [R, rmpc.geometry.skew(p) * R;
      zeros(3), R];
end
