% Skew: produce skew-symmetric matrix for vector M such that Skew(M)*v = M x v

function [skew_M]=Skew(M)
skew_M=[0      -M(3)    M(2)      % first row
        M(3)    0      -M(1)      % second row
       -M(2)    M(1)    0];       % third row
end
