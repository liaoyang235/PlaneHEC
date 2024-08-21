function [R_error,t_error] = error_calc(M,invA,X,Y)
    n = length(M);
    total_theta = 0;
    total_t = 0;
    A = pageinv(invA);

    
    for i = 1:n

        % 求两个向量之间的旋转角
        a = M(i,1:3)';
        b = (Y(1:3)*invA(1:3,1:3,i)*inv(X(1:3,1:3)))';
        theta = acos((a'*b) / ( norm(a)*norm(b) ));
        
        total_theta = total_theta + power(theta*180/pi,2);
        c = cross(b, a);
        Rv = c / norm(c) * theta;
        R = rodrigues(Rv);

        % 求距离误差的最小范数解
        d = M(i,4);
        p = M(i,1:3);
        b = Y(1,4) - d - p*R*X(1:3,1:3)*A(1:3,4,i)-p*X(1:3,4);
        t = norm(p'*inv(p*p')*b)*1000;
        total_t = total_t + t*t;
    end

    R_error = total_theta/n;
    t_error = total_t/n;
end

function [R] = rodrigues(Rv)
    theta = norm(Rv);
    Rv = Rv/theta;
    a1 = Rv(1,1);
    a2 = Rv(2,1);
    a3 = Rv(3,1);
    S = [0,-a3,a2;a3,0,-a1;-a2,a1,0];
    R = cos(theta)*eye(3)+(1-cos(theta))*S*S'+ sin(theta)*S;
end

function Ang2 = rotm2eul(R)
    x = atan2(R(3,2),R(3,3));
    y = atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2));
    z = atan2(R(2,1),R(1,1));
    Ang2 = [x y z];
end