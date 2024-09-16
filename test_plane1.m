

clear;
clc;

% all_data = importdata('data/results_8.2.csv');       %30
% all_data = importdata('data/results_9.11.csv');       %26
all_data = importdata('data/results_8.30.1234.csv');    %161组数据
% all_data = importdata('data/results_8.22.csv');       %103


% all_data = all_data(1:90,:)

groupSize = 4;

RMSs = [];
error_Rs = [];

for it=1:50
    % 随机抽取30个数据

    numRows = size(all_data, 1);

    randomIndices = randperm(numRows, groupSize);
    group1 = all_data(randomIndices, :);
    
    % 剩下的数据作为另一组
    remainingIndices = setdiff(1:numRows, randomIndices);
    group2 = all_data(remainingIndices, :);
    
    
    
    data = group1;
    
    [numRows, numCols] = size(data);
    M = [];
    inv_A = [];
    for i=1:numRows
        M = [M; data2plane(data(i, 1:3))];
        data2trans(data(i, 4:10));
        inv_A = cat(3, inv_A,data2trans(data(i, 4:10)));
    end
    
    inv_A0 = inv_A(:,:,1);
    inv_A0(1:3, 4) = mean(inv_A(1:3, 4, :), 3);
    
    for i=1:numRows %做这一步转换到相对位置
        inv_A(:,:,i) = inv(inv_A0) * inv_A(:,:,i);
    end
    
    data_sta = 1;
    data_end = numRows;
    
    X = YMXA(M(data_sta:data_end, :), inv_A(:,:,data_sta:data_end))
    
    Y = [];
    for i = 1:length(inv_A)
        Y = [Y; M(i,:) * X * inv(inv_A(:,:,i))];
    end
    
    Y
    
    qX = rotm2quat(X(1:3,1:3)); %四元数wxyz
    end_result_former = [X(1:3,4)',qX(2:4), qX(1)];
    fprintf('%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f\n', end_result_former)
    
    former_x = X;
    
    
    real_Y = mean(Y);
    real_Y(1:3) = real_Y(1:3) / norm(real_Y(1:3));
    S_former_y = std(Y);  %计算标准差,除以的是（N-1）
    
    
    
    % 使用这个iteration函数对之前的结果进行优化
    % 第一个参数是已经得到的闭式解（也可以用单位矩阵从头开始优化，但相当容易陷入局部最优）
    % 第二、三个参数是数据，第四个参数是迭代次数。一般100次就可以收敛了，收敛情况请见iteration.m中的delta_x大小
    [X, num_it] = iteration(X,M(data_sta:data_end, :),inv_A(:,:,data_sta:data_end),100);
    
    qX = rotm2quat(X(1:3,1:3)); %四元数wxyz
    
    end_result = [X(1:3,4)',qX(2:4), qX(1)];
    fprintf('%.7f,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f\n', end_result)
    % save('X2.mat', 'X'); % 保存X到output.mat文件
    
    Y = [];
    
    for i = 1:length(inv_A)
        Y = [Y; M(i,:) * X * inv(inv_A(:,:,i))];
    end
    
    Y
    
    % 懒得优化Y了，直接取了均值
    real_Y = mean(Y);
    % real_Y = real_Y/norm(real_Y(1:3));
    real_Y(1:3) = real_Y(1:3) / norm(real_Y(1:3));
    
    % 对迭代前和迭代后的X分别求旋转误差和平移误差，推导见公式推导4.jpg
    % 这计算的是相对误差。事实上可视化效果不是很好的结果误差也可能很小
    % 不过也难说，毕竟手眼标定误差确实不好算，点云配准论文用的也是点云距离的标准差
    % R_error是误差角度的均方，t_error是误差距离的均方
    % 误差没有特别大的优化，大概还行吧
    [R_error,t_error]  = error_calc(M(data_sta:data_end, :), inv_A(:,:,data_sta:data_end),former_x,real_Y)
    [R_error,t_error]  = error_calc(M(data_sta:data_end, :), inv_A(:,:,data_sta:data_end),X,real_Y)
    
    % 之后可以计算重投影误差，由于需要处理图像所以用python写了，请见rrmse.py
    % 测试了两组，误差分别是12.59，31.24（px）
    % 相较于别的方法差很多。我觉得是因为别的方法直接用标定板和RGB图像进行标定所以重投影误差小。我们算重投影误差是重新采数据计算的，和标定过程没关系，比较吃亏
    
    
    % variance_Y = var(Y)  %计算方差
    S_y = std(Y)  %计算标准差,除以的是（N-1）
    
    
    
    
    data = group2;
    
    [numRows, numCols] = size(data);
    M = [];
    inv_A = [];
    for i=1:numRows
        M = [M; data2plane(data(i, 1:3))];
        data2trans(data(i, 4:10));
        inv_A = cat(3, inv_A,data2trans(data(i, 4:10)));
    end
    
    
    for i=1:numRows %做这一步转换到相对位置
        inv_A(:,:,i) = inv(inv_A0) * inv_A(:,:,i);
    end
    
    
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
    RMSs = [RMSs, RMS];
    error_Rs = [error_Rs, error_R];
end

mean_rms = mean(RMSs)
mean_error_R = mean(error_Rs)
