function [T, num_iterations] = iteration(Tx,M,iinvA,ite)
    Rx = Tx(1:3,1:3);
    tx = Tx(1:3,4);
    A = pageinv(iinvA);
    zeros = [0;0;0];
    delta_x = [0;0;0;0;0;0];
    [numRows, numCols] = size(M);
    last_norm_delta_x = 1000;
    target = [];
    for k = 1:ite
        g = [];
        J = [];
        z1 = [];
        z2 = [];
        z3 = [];
        z4 = [];
        
        Multiple_factor = 1;
        target_k = 0;

        for i=1:numRows-1
            for j = i+1 : numRows
                Ai = A(:,:,i);
                alphai = A(1:3,1,i);
                betai = A(1:3,2,i);
                gammai = A(1:3,3,i);
                deltai = A(1:3,4,i);
                Aj = A(:,:,j);
                alphaj = A(1:3,1,j);
                betaj = A(1:3,2,j);
                gammaj = A(1:3,3,j);
                deltaj = A(1:3,4,j);
                % 这是待优化的函数，表示了两个MXA计算出的Y的差距，大小是1x4
                % 事实上是损失，目标是降为0
                calc_g = M(i,:)*Tx*Ai-M(j,:)*Tx*Aj;
                calc_g(4) = Multiple_factor * calc_g(4);

%                 calc_g(5) = calc_g(4);
%                 calc_g(6) = calc_g(4);


                target_k = target_k + sum(calc_g.^2);

                z1 = [z1,calc_g(1)];
                z2 = [z2,calc_g(2)];
                z3 = [z3,calc_g(3)];
                z4 = [z1,calc_g(4)];
                todis = ["z1: mean ",mean(z1)," std ",std(z1);
                    "z2: mean ",mean(z2)," std ",std(z2);
                    "z3: mean ",mean(z3)," std ",std(z3);
                    "z4: mean ",mean(z4)," std ",std(z4);];
                % disp(todis);
                % 合并后大小是1x(4n)，n是循环次数
                g = [g,calc_g];
                % 这是对待优化函数求导得到的雅可比矩阵，大小是4x6
                % 推导请见公式推导3.jpg
                calc_J = [M(i,:)*dong(Tx,alphai)-M(j,:)*dong(Tx,alphaj)-M(i,:)*dong(Tx,zeros)+M(j,:)*dong(Tx,zeros);
                    M(i,:)*dong(Tx,betai)-M(j,:)*dong(Tx,betaj)-M(i,:)*dong(Tx,zeros)+M(j,:)*dong(Tx,zeros);
                    M(i,:)*dong(Tx,gammai)-M(j,:)*dong(Tx,gammaj)-M(i,:)*dong(Tx,zeros)+M(j,:)*dong(Tx,zeros);
                    M(i,:)*dong(Tx,deltai)-M(j,:)*dong(Tx,deltaj)];
                % 合并后大小是(4n)x6
                calc_J(4,:) = Multiple_factor * calc_J(4,:);

%                 calc_J(5) = calc_J(4);
%                 calc_J(6) = calc_J(4);
                
                J = [J;calc_J];
            end
        end
        target = [target; target_k];
        % 转置后大小是(4n)x1
        g = g';
        % 根据正规方程求梯度。需要注意的是，梯度前三位是位移，后三位是旋转矩阵的李代数，见p85 4.3.5节
        % 其中0.01是学习率，可以自行设定

        delta_x = - 1*inv(J'*J)*J'*g;
        norm_delta_x = norm(delta_x);
        

        if norm_delta_x > last_norm_delta_x
            num_iterations = k;
            disp(['迭代在第 ', num2str(k), ' 次停止，因为 delta_x 变大了']);
            last_norm_delta_x
            break;
        end

%         if norm_delta_x < 1e-15
%             disp(['迭代在第 ', num2str(k), ' 次停止，因为 delta_x 很小了']);
%             break;
%         end


        last_norm_delta_x = norm_delta_x;

        Tx = [lie_to_rotate(delta_x(4:6,1)),delta_x(1:3,1);0,0,0,1]*Tx;

    end
    T = Tx;
end

% 这个函数将李代数扩展为反对称矩阵，见p75式(4.7)
% 本来想命名为skew_symmetry的，但是太长了懒得改
function [S] = skew(p)
    a1 = p(1,1);
    a2 = p(2,1);
    a3 = p(3,1);
    S = [0,-a3,a2;a3,0,-a1;-a2,a1,0];
end

% 这个函数将李代数转换为旋转矩阵，见p79式(4.22)
function [R] = lie_to_rotate(p)
    theta = norm(p);
    a = p/norm(p);
    R = cos(theta)*eye(3)+(1-cos(theta))*a*a'+sin(theta)*skew(a);
end

% 这个函数在se(3)上对李代数求导，见p86。命名来源于注释"咚"
% 请注意，返回的导数是一个4x6的矩阵
function [se3] = dong(T,p)
    se3 = [eye(3),-skew(T(1:3,1:3)*p+T(1:3,4));0,0,0,0,0,0];
end
    