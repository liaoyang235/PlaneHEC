

clear;
 clc;
data = importdata('results_8.2.csv');
% data = importdata('results.csv');

% 这个T用于改变平面的朝向和位置，可以观察不同平面下算法的效果
% 如果不需要可以设置为单位阵
% T = eye(4);
t_step = 0.2;
t_02_r_error = [];
t_02_t_error = [];
for t = 0:t_step:15

    random_Rv = rand(3,1);
    random_R = rodrigues(random_Rv/norm(random_Rv));
    random_t = rand(3,1);
    random_t = random_t/norm(random_t)*t
    T = [random_R(1,1:3),random_t(1,1);random_R(2,1:3),random_t(2,1);random_R(3,1:3),random_t(3,1);0,0,0,1];
    
    [numRows, numCols] = size(data);
    M = [];
    inv_A = [];
    for i=1:numRows
        M = [M; data2plane(data(i, 1:3))];
        data2trans(data(i, 4:10));
        inv_A = cat(3, inv_A, inv(T)*data2trans(data(i, 4:10)));
    end
    
    
    data_sta = 1;
    data_end = 30;
    
    X = YMXA(M(data_sta:data_end, :), inv_A(:,:,data_sta:data_end))
    
    former_x = X;
    X = iteration(X,M(data_sta:data_end, :),inv_A(:,:,data_sta:data_end),100)
    
    qX = rotm2quat(X(1:3,1:3))
    
    % save('X2.mat', 'X'); % 保存X到output.mat文件
    
    Y = [];
    
    for i = 1:length(inv_A)
        Y = [Y; M(i,:) * X * inv(inv_A(:,:,i))];
    end
    
    real_Y = mean(Y);
    real_Y = real_Y/norm(real_Y(1:3));
    
    [R_error,t_error]  = error_calc(M(data_sta:data_end, :), inv_A(:,:,data_sta:data_end),former_x,real_Y)
    [R_error,t_error]  = error_calc(M(data_sta:data_end, :), inv_A(:,:,data_sta:data_end),X,real_Y)

    t_02_r_error = [t_02_r_error,R_error];
    t_02_t_error = [t_02_t_error,t_error];

    S_y = std(Y);  %计算标准差

end

