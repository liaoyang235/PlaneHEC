function angle = quaternionDifferenceAngle(q1, q2)
    % 计算两个四元数之间相差的角度
    % 输入: q1, q2 - 四元数，格式为 [w, x, y, z]
    % 输出: angle - 两个四元数之间相差的角度（弧度）

    % 计算四元数的点积
    dot_product = dot(q1, q2);
    
    % 确保点积在[-1, 1]范围内，以避免数值误差
    dot_product = max(min(dot_product, 1), -1);
    
    % 计算相差的角度
    angle = 2 * acos(dot_product);
end
