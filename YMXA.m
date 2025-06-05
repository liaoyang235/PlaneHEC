function [ X ] = YMXA( M,iinv_A )
%AXXB 使用了K积方法，计算手眼标定问题中的Y=MXA问题，其中Y，M是1*4平面表示，X，A是齐次变换矩阵
%     iinv_A = iinv_A(:,:,1:11);
    A = pageinv(iinv_A);
    n=length(iinv_A);
    K=[];

%     for i=1:n-1
%         R1=iinv_A(1:3,1:3,i);
%         R2=iinv_A(1:3,1:3,i+1);
%         m1=M(i, 1:3);
%         m2=M(i+1, 1:3);
%         K=[K;kron(R1,m1)-kron(R2,m2)];
%     end

    for i=1:n-1
        for j = i+1 : n
            R1=iinv_A(1:3,1:3,i);
            R2=iinv_A(1:3,1:3,j);
            m1=M(i, 1:3);
            m2=M(j, 1:3);
            K=[K;kron(R1,m1)-kron(R2,m2)];
        end
    end
    [U,S,V]=svd(K);
    Rerror = S(9,9);
    col=length(V);
    Vx=[V(1:3,col),V(4:6,col),V(7:9,col)];
    RX=Vx / nthroot(det(Vx), 3);   %求它的立方根

    %SVD分解实现旋转矩阵正交化
    [Ux,Sx,Vx]=svd(RX);
    RX=Ux*Vx';
    
    RX=sign(det(RX))*RX;

    qX = rotm2quat(RX(1:3,1:3));
    qX1 = mathtrans().rotm2quat(RX(1:3,1:3));

    a1 = K * [RX(1:3,1);RX(1:3,2);RX(1:3,3)];

    % X = RX
    
    % Ax=b
    At = [];
    bt = [];
%     for i=1:n-1
%         m1=M(i, 1:3);
%         m2=M(i+1, 1:3);
%         At = [At; m1-m2];
%         
%         t1=A(1:3,4,i);
%         t2=A(1:3,4,i+1);
%         d1=M(i, 4);
%         d2=M(i+1, 4);
%         bt =[bt; -m1 * RX * t1 + m2 * RX * t2 - d1 + d2];
%     end
    for i=1:n-1
        for j = i+1:n
            m1=M(i, 1:3);
            m2=M(j, 1:3);
            At = [At; m1-m2];
            
            t1=A(1:3,4,i);
            t2=A(1:3,4,j);
            d1=M(i, 4);
            d2=M(j, 4);
            bt =[bt; -m1 * RX * t1 + m2 * RX * t2 - d1 + d2];
        end
    end

    
    tx=At\bt;
    
    err = At*tx - bt;

    X=[RX,tx;0,0,0,1];
end


%     [ Error,err,average_err ] = AXXB_Error( A(1:3,1:3,:),B(1:3,1:3,:),RX );
%     
%     At=[];
%     bt=[];
%     for i=1:n
%         RA=A(1:3,1:3,i);
%         tB=B(1:3,4,i);
%         tA=A(1:3,4,i);
%         At=[At;I-RA];
%         bt=[bt;tA-RX*tB];
%     end
%     tx=At\bt;
%     X=[RX,tx;0,0,0,1];
%     [ Error,err,average_err ] = AXXB_Error( A,B,X );




function [ Error,err,average_err ] = AXXB_Error( A,B,X )
%AXXB_Error 用于简单得到标定的误差
    n=length(A);
    sz=size(A);
    Error=zeros(sz(1),sz(1),n);
    err=zeros(n,1);
    average_err=0;
    Y1=zeros(sz(1),sz(1),n);
    Y2=zeros(sz(1),sz(1),n);
    for i=1:n
        Y1(:,:,i)=A(:,:,i)*X;
        Y2(:,:,i)=X*B(:,:,i);
        Error(:,:,i)=Y1(:,:,i)-Y2(:,:,i);
        err(i)=norm(Error(:,:,i));
        average_err=average_err+err(i);
    end
    average_err=average_err/length(err);
end
