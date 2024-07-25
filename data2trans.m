
function [T] = data2trans(A)
    t = A(1:3);
    q = A(4:7); % 四元数
    R = quat2rotm(q); % 四元数转旋转矩阵
    T = [R, t'; 0, 0, 0, 1]; % 构建齐次变换矩阵
end