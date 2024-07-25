
% 测试过了
function R = quat2rotm(q)
    % 四元数转旋转矩阵
    q = q / norm(q); % 归一化四元数
    qw = q(4);
    qx = q(1);
    qy = q(2);
    qz = q(3);

    R = [1-2*(qy^2+qz^2), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw);
         2*(qx*qy+qz*qw), 1-2*(qx^2+qz^2), 2*(qy*qz-qx*qw);
         2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx^2+qy^2)];
end