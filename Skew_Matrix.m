% Skew_Matrix: alternative implementation to produce skew-symmetric matrix for vector M

function [skew_M]=Skew_Matrix(M)
skew_M=[0 -M(3) M(2)        % row 1
    ;M(3) 0 -M(1)           % row 2
    ;-M(2) M(1) 0];         % row 3
end
