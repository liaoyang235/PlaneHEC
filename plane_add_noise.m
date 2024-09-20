function [ M ] = plane_add_noise( M0, rotation_noise, translation_noise)
% 平移误差以毫米为单位，旋转误差以deg为单位
    error_t = translation_noise * 0.001;
    error_r = rotation_noise * pi / 180;


    n = M0(1:3);
    d = M0(4);

    % 生成旋转矩阵
    theta = acos(dot(n, [0; 0; 1]) / norm(n)); % 计算旋转角度
    k = cross([0; 0; 1], n); % 计算旋转轴
    k = k / norm(k); % 归一化旋转轴
    K = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0]; % 反对称矩阵
    R = eye(3) + sin(theta) * K + (1 - cos(theta)) * K^2; % 旋转矩阵

    % 应用旋转矩阵到平面法向量
    n_rotated = R * [0; 0; 1];
    M(1:3) = n_rotated;


    rotation_noise_vector = tan([error_r * randn(2, 1); 0]) % 生成旋转噪声
    translation_noise_vector = error_t * randn(1, 1) % 生成平移噪声
    n1 = R * rotation_noise_vector + n';
    n1 = n1 / norm(n1);

    d1 = d + translation_noise_vector;
    M = [n1', d1]
end
