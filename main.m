

clear;
% clc;
data = importdata('results.csv');

[numRows, numCols] = size(data);
M = [];
A = [];
for i=1:numRows
    M = [M; data2plane(data(i, 1:3))];
    A = cat(3, A, data2trans(data(i, 4:10)));
end

X = YMXA(M, A);

save('X2.mat', 'X'); % 保存X到output.mat文件

for i = 1:10
    Y = M(i,:) * X * inv(A(:,:,i))
end


X2=[  0.08337526616917447,  0.06401080299139758  ,-0.006832672285502246 ,-0.6550601093990662,-0.2599869457029329,-0.6485481063780865, 0.2875558986554467]
X2 = data2trans(X2)
invx2 = inv(X2)

for i = 1:10
    Y = M(i,:) * X2 * inv(A(:,:,i))
end



function [M] = data2plane(A)
    M = [A(1), A(2), -1, A(3)];
    M = M / norm(M(1:3));
end

