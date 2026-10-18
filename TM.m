% TM: compute direction cosine matrix (dcm) and kinematic matrix H from Euler angles
% Euler is a 3-element vector (assumed order [phi, theta, psi])
% dcm is 3x3 rotation matrix; H maps Euler rates to body angular velocity.

function [dcm,H]=TM(Euler)

% Direction cosine matrix using Z-Y-X (or similar) Euler angle composition as implemented
dcm=[cos(Euler(2))*cos(Euler(3))       (sin(Euler(1))*sin(Euler(2))*cos(Euler(3)))-(cos(Euler(1))*sin(Euler(3)))      (cos(Euler(1))*sin(Euler(2))*cos(Euler(3)))+(sin(Euler(1))*sin(Euler(3)))
     cos(Euler(2))*sin(Euler(3))       (sin(Euler(1))*sin(Euler(2))*sin(Euler(3)))+(cos(Euler(1))*cos(Euler(3)))      (cos(Euler(1))*sin(Euler(2))*sin(Euler(3)))-(sin(Euler(1))*cos(Euler(3)))
     -sin(Euler(2))                     sin(Euler(1))*cos(Euler(2))                                                   cos(Euler(1))*cos(Euler(2))];

% Kinematic matrix H that maps Euler-angle rates to body angular velocity (3x3)
% OM_body = A * (H * th_dot)  (or in the code they use th_dot = H \ (A \ OM))
H=[1         0               -sin(Euler(2))
   0    cos(Euler(1))   sin(Euler(1))*cos(Euler(2))
   0    -sin(Euler(1))  cos(Euler(1))*cos(Euler(2))];

end
