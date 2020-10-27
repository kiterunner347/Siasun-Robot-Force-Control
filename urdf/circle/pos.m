function p = pos(theta)
%输入关节角，输出各关节位置（基系表示）
a = [0.26667 2/3 0 0 0 0];
d = [0.293333 0 0 0.871333 0 0.10667+0.3];
alpha = [-pi/2 0 -pi/2 pi/2 -pi/2 0];
T = eye(4);
p = zeros(3,6);

for i = 1:6
    T = T*Tr(theta(i),d(i),a(i),alpha(i));
    p(:,i) = T(1:3,4);
end

end
