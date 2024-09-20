

clear;
 clc;
all_data = importdata('data/test_plane.csv');

pose_data = importdata('data/test_plane_pose.csv');

% 初始化变换矩阵X
X = eye(4);

% 提取位移向量和四元数
translation = pose_data(1:3);
quaternion = [pose_data(7), pose_data(4:6)];

% 将四元数转换为旋转矩阵
rotation_matrix = quat2rotm(quaternion);

% 构建变换矩阵X
X(1:3, 1:3) = rotation_matrix;
X(1:3, 4) = translation;

% 打印变换矩阵X
disp('变换矩阵X:');
disp(X);


data = all_data;

[numRows, numCols] = size(data);
M = [];
inv_A = [];
for i=1:numRows
    M = [M; data2plane(data(i, 1:3))];
    data2trans(data(i, 4:10));
    inv_A = cat(3, inv_A,data2trans(data(i, 4:10)));
end


% inv_A0 = inv_A(:,:,1);
% %     inv_A(1:3, 4, :)
% inv_A0(1:3, 4) = mean(inv_A(1:3, 4, :), 3);
% 
% for i=1:numRows %做这一步转换到相对位置
%     inv_A(:,:,i) = inv(inv_A0) * inv_A(:,:,i);
% end



Y = [];
for i = 1:length(inv_A)
    Y = [Y; M(i,:) * X * inv(inv_A(:,:,i))];
end

Y

real_Y = mean(Y);
real_Y(1:3) = real_Y(1:3) / norm(real_Y(1:3));
S_y = std(Y);  %计算标准差,除以的是（N-1）

N = numRows;
RMS = sqrt(S_y(4).^2* (N-1) / N) * 1000

r_avg = real_Y(1:3);
error_ri = zeros(N, 1);
for i = 1:N
    % 计算法向量之间的夹角
    cos_theta = dot(r_avg, Y(i, 1:3)) / (norm(r_avg) * norm(Y(i, 1:3)));
    angle = acosd(cos_theta); % 夹角，单位为度
    error_ri(i) = angle;
end

error_q = mean(error_ri);
error_R = sqrt(sum(error_ri.^2)/N)



