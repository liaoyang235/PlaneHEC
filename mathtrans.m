function mathtrans=mathtrans
    mathtrans.data2trans=@data2trans;
    mathtrans.rotm2quat=@rotm2quat;
    mathtrans.quat2rotm=@quat2rotm;
end


function [T] = data2trans(A)
    t = A(1:3);
    q = A(4:7); % 四元数
    R = quat2rotm(q); % 四元数转旋转矩阵
    T = [R, t'; 0, 0, 0, 1]; % 构建齐次变换矩阵
end

function q = rotm2quat(R)
    % 检查输入是否为3x3矩阵
    if ~all(size(R) == [3, 3])
        error('输入必须是3x3矩阵');
    end

    % 计算四元数
    tr = trace(R);
    if tr > 0
        S = sqrt(tr + 1.0) * 2; % S=4*qw
        qw = 0.25 * S;
        qx = (R(3, 2) - R(2, 3)) / S;
        qy = (R(1, 3) - R(3, 1)) / S;
        qz = (R(2, 1) - R(1, 2)) / S;
    elseif (R(1, 1) > R(2, 2)) && (R(1, 1) > R(3, 3))
        S = sqrt(1.0 + R(1, 1) - R(2, 2) - R(3, 3)) * 2; % S=4*qx
        qw = (R(3, 2) - R(2, 3)) / S;
        qx = 0.25 * S;
        qy = (R(1, 2) + R(2, 1)) / S;
        qz = (R(1, 3) + R(3, 1)) / S;
    elseif R(2, 2) > R(3, 3)
        S = sqrt(1.0 + R(2, 2) - R(1, 1) - R(3, 3)) * 2; % S=4*qy
        qw = (R(1, 3) - R(3, 1)) / S;
        qx = (R(1, 2) + R(2, 1)) / S;
        qy = 0.25 * S;
        qz = (R(2, 3) + R(3, 2)) / S;
    else
        S = sqrt(1.0 + R(3, 3) - R(1, 1) - R(2, 2)) * 2; % S=4*qz
        qw = (R(2, 1) - R(1, 2)) / S;
        qx = (R(1, 3) + R(3, 1)) / S;
        qy = (R(2, 3) + R(3, 2)) / S;
        qz = 0.25 * S;
    end

    q = [qw, qx, qy, qz];
end


function R = quat2rotm(q)       %qx qy qz qw
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