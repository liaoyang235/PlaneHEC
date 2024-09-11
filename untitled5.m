
% 示例用法
q1 = [0.7071, 0.7071, 0, 0]; % 90度绕X轴旋转
q2 = [0.7071, 0, 0.7071, 0]; % 90度绕Y轴旋转

angle = quaternionDifferenceAngle(q1, q2);
disp('两个四元数之间相差的角度（弧度）:');
disp(angle);
angle = angle * 180 / pi


angle = quaternionDifferenceAngle1(q1, q2);
disp('两个四元数之间相差的角度（弧度）:');
disp(angle);
angle = angle * 180 / pi

