

clear;
 clc;
all_data = importdata('data/results_8.2.csv');       %30
% all_data = importdata('data/results_8.30.1234.csv');
% all_data = importdata('data/results_8.22.csv');


% all_data = all_data(1:90,:)

% 随机划分，5个为一组
groupSize = 10;
numRows = size(all_data, 1);
numGroups = floor(numRows / groupSize);
indices = randperm(numRows);
% indices = 1:numRows;
dataGroups = cell(1, numGroups);


for i = 1:numGroups
    startIdx = (i-1) * groupSize + 1;
    endIdx = min(i * groupSize, numRows);

    dataGroups{i} = all_data(indices(startIdx:endIdx), :);
end

% % 将数据按类似"123123123”报数的方式分成10组，每组包含报相同数字的数据行。
% numGroups = 10;
% numRows = size(all_data, 1);
% dataGroups = cell(1, numGroups);
% 
% for i = 1:numGroups
%     dataGroups{i} = all_data(i:numGroups:end, :);
% end



all_result = [];
all_S_y = [];
all_real_y = [];

all_former_result = [];
all_former_S_y = [];
all_former_real_y = [];
num_its = [];

tic;


for group_num = 1:numGroups
    data = dataGroups{group_num};

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
    all_former_real_y = [all_former_real_y; real_Y];
    all_former_S_y = [all_former_S_y; S_former_y];
    all_former_result = [all_former_result; end_result_former];



    % 使用这个iteration函数对之前的结果进行优化
    % 第一个参数是已经得到的闭式解（也可以用单位矩阵从头开始优化，但相当容易陷入局部最优）
    % 第二、三个参数是数据，第四个参数是迭代次数。一般100次就可以收敛了，收敛情况请见iteration.m中的delta_x大小
    [X, num_it] = iteration(X,M(data_sta:data_end, :),inv_A(:,:,data_sta:data_end),100);
    num_its = [num_its; num_it];

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
    if t_error < 100
        all_real_y = [all_real_y; real_Y];
        all_S_y = [all_S_y; S_y];
        all_result = [all_result; end_result];
    end
end

toc;

all_former_result;
N = size(all_former_result, 1);
S_all_former_result = std(all_former_result);

S_all_former_result = sqrt(S_all_former_result.^2* (N-1) / N) * 1000  %单位mm
error_former_t = sqrt(sum(S_all_former_result(1:3).^2))

error_former_txy = sqrt(sum(S_all_former_result(1:2).^2))
error_former_tz = S_all_former_result(3)

Q = all_former_result(:,4:7);
Q_avg = avg_quaternion_markley(Q)';
T = all_former_result(:,1:3);
T_avg = mean(T);

error_qi = zeros(size(Q, 1), 1);
for i = 1:size(Q, 1)
    error_qi(i) = quaternionDifferenceAngle(Q(i, :), Q_avg) * 180 / pi;
end
error_former_q = mean(error_qi);        %绝对值的均值，单位为°
error_former_R = sqrt(sum(error_qi.^2)/size(Q, 1))     %平方和均值，单位为°





all_result;
N = size(all_result, 1);
S_all_result = std(all_result);

S_all_result = sqrt(S_all_result.^2* (N-1) / N) * 1000  %单位mm
error_t = sqrt(sum(S_all_result(1:3).^2))

error_txy = sqrt(sum(S_all_result(1:2).^2))
error_tz = S_all_result(3)

Q = all_result(:,4:7);
Q_avg = avg_quaternion_markley(Q)';
T = all_result(:,1:3);
T_avg = mean(T);

error_qi = zeros(size(Q, 1), 1);
for i = 1:size(Q, 1)
    error_qi(i) = quaternionDifferenceAngle(Q(i, :), Q_avg) * 180 / pi;
end
error_q = mean(error_qi);        %绝对值的均值，单位为°
error_R = sqrt(sum(error_qi.^2)/size(Q, 1))     %平方和均值，单位为°

% 计算num_its的均值，标准差，最大值，最小值
mean_num_its = mean(num_its);
std_num_its = std(num_its);
max_num_its = max(num_its);
min_num_its = min(num_its);

fprintf('迭代次数的均值: %.5f\n', mean_num_its);
fprintf('迭代次数的标准差: %.5f\n', std_num_its);
fprintf('迭代次数的最大值: %d\n', max_num_its);
fprintf('迭代次数的最小值: %d\n', min_num_its);



