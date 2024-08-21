
function [M] = data2plane(A)
    M = [A(1), A(2), -1, A(3)];
    M = M / norm(M(1:3));

end
