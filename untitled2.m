R = [1,0,0;0,1,0;0,0,1];
q = rotm2quat(R)

inv_A1 = inv_A(:,:,11);
x = inv_A1 * [0.5,0.5,0.5,1]'

% 定义一个四元数
q = [0.7071, 0.7071, 0, 0]; % 这是一个代表90度绕X轴旋转的四元数

% 将四元数转换为旋转矩阵
R = quat2rotm(q);

% 输出旋转矩阵
disp('Rotation Matrix:');
disp(R);

% 将旋转矩阵转换回四元数
q_converted = rotm2quat(R);

% 输出转换回的四元数
disp('Converted Quaternion:');
disp(q_converted);