
function [T] = data2trans(A)
    t = A(1:3);
    q = [A(7), A(4:6)]; % 四元数matlab格式： qw qx qy qz  ros格式：qx qy qz qw
%     R = quat2rotm(q); % 四元数转旋转矩阵
    R = mathtrans().quat2rotm(A(4:7));  % 接收qx qy qz qw
    T = [R, t'; 0, 0, 0, 1]; % 构建齐次变换矩阵

end