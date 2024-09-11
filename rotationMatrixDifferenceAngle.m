function angle = rotationMatrixDifferenceAngle(R1, R2)
    % 计算两个旋转矩阵之间相差的角度
    % 输入: R1, R2 - 3x3的旋转矩阵
    % 输出: angle - 两个旋转矩阵之间相差的角度（弧度）

    % 计算相对旋转矩阵
    R_diff = R1' * R2;
    
    % 计算相差的角度
    angle = acos((trace(R_diff) - 1) / 2);
end
