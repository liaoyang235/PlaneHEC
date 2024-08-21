function [T] = iteration2(Tx,M,iinvA,ite)
    RxT = Tx(1:3,1:3)';
    txT = Tx(1:3,4)';
    [numRows, numCols] = size(M);
    for k = 1:ite
        g = [];
        J = [];
        for i=1:numRows-1
            for j = i+1 : numRows
                tAiT = iinvA(1:3,4,i)';
                tAjT = iinvA(1:3,4,j)';
                piT = M(i,1:3)';
                pjT = M(j,1:3)';
                di = M(i,4);
                dj = M(j,4);
                calc_g = tAiT*RxT*piT+txT*piT+di-tAjT*RxT*pjT-txT*pjT-dj;
                g = [g,calc_g];
                calc_J = piT'-pjT';
                J = [J,calc_J'];
            end
        end
        mean(g)
        std(g)
        g = g';
        J = J';
        delta_x = -inv(J'*J)*J'*g
        txT = txT + delta_x(1:3,1)';
    end
    T = [RxT',txT';0,0,0,1];
end

function [S] = skew(p)
    a1 = p(1,1);
    a2 = p(2,1);
    a3 = p(3,1);
    S = [0,-a3,a2;a3,0,-a1;-a2,a1,0];
end

function [R] = lie_to_rotate(p)
    theta = norm(p);
    a = p/norm(p);
    R = cos(theta)*eye(3)+(1-cos(theta))*a*a'+sin(theta)*skew(a);
end
    