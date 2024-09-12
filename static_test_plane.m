

clear;
 clc;
all_data = importdata('data/static_plane2.csv');

data = all_data;

[numRows, numCols] = size(data);
M = [];
inv_A = [];
for i=1:numRows
    M = [M; data2plane(data(i, 1:3))];
    data2trans(data(i, 4:10));
    inv_A = cat(3, inv_A,data2trans(data(i, 4:10)));
end

M

real_M = mean(M);
real_M(1:3) = real_M(1:3) / norm(real_M(1:3));
S_M = std(M);  %计算标准差,除以的是（N-1）

N = numRows;
RMS = sqrt(S_M(4).^2* (N-1) / N) * 1000

r_avg = real_M(1:3);
error_ri = zeros(N, 1);
for i = 1:N
    % 计算法向量之间的夹角
    cos_theta = dot(r_avg, M(i, 1:3)) / (norm(r_avg) * norm(M(i, 1:3)));
    angle = acosd(cos_theta); % 夹角，单位为度
    error_ri(i) = angle;
end

error_q = mean(error_ri);
error_R = sqrt(sum(error_ri.^2)/N)
