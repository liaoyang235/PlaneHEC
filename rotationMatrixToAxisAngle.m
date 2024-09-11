function [axis, angle] = rotationMatrixToAxisAngle(R)
    % 从旋转矩阵计算旋转轴和旋转角
    % 输入: R - 3x3的旋转矩阵
    % 输出: axis - 旋转轴（单位向量）
    %       angle - 旋转角（弧度）

    % 计算旋转角
    angle = acos((trace(R) - 1) / 2);
    
    % 计算旋转轴
    rx = R(3, 2) - R(2, 3);
    ry = R(1, 3) - R(3, 1);
    rz = R(2, 1) - R(1, 2);
    axis = [rx; ry; rz];
    axis = axis / norm(axis);
end

% % 示例用法
% R = [0.866, -0.5, 0;
%      0.5, 0.866, 0;
%      0, 0, 1];
% 
% [axis, angle] = rotationMatrixToAxisAngle(R);
% disp('旋转轴:');
% disp(axis);
% disp('旋转角（弧度）:');
% disp(angle);