
function [T] = data2trans(A)
    t = A(1:3);
    R = mathtrans().quat2rotm(A(4:7));  % 接收qx qy qz qw
    T = [R, t'; 0, 0, 0, 1]; % 构建齐次变换矩阵

end