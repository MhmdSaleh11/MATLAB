% rigidbody: constructs 13x13 rigid-body transfer matrix U_RB for a single rigid body
% Inputs:
%   m  - mass
%   lc - vector from body origin to center of mass (local coordinates)
%   lo - vector from body origin to joint offset (local coordinates)
%   thr- Euler angles for the body (3x1)
%   OM - body angular velocity vector (3x1)
%   j  - vector of diagonal inertia entries (1x3)
%   fC - external force (3x1)
%   mC - external moment (3x1)

% Output:
%   U_RB - 13x13 rigid-body transfer matrix

function[U_RB]=rigidbody(m,lc,lo,thr,OM,j,fC,mC)

[A,H]=TM(thr);               % get DCM (A) and kinematic matrix (H) for Euler angles
r_IC=A*lc;                   % center-of-mass position in inertial frame (body->inertial)
r_Io=A*lo;                   % joint offset position in inertial frame
r_IO_s=Skew(r_Io);           % skew-symmetric matrix for r_Io (for cross products)
r_IC_s=Skew(r_IC);           % skew-symmetric for r_IC

th_dot=(H\ (A\OM));          % Euler-angle rates computed from OM (th_dot = H^{-1} * (A^{-1} * OM))
OMs=Skew(OM);                % skew matrix of angular velocity OM
omg=A'*OM;                   % angular velocity expressed in body frame (A' * OM)
omgs=Skew(omg);              % skew matrix of body-frame angular velocity
J=diag(j);                   % inertia matrix (3x3 diagonal from vector j)

% Build blocks used in U_RB construction using kinematic/dynamic relations:

E1=-r_IO_s;                  % used to relate translational to rotational effects
E2=OMs*OMs*r_Io;             % centripetal / coriolis related term for joint offset
E3=-m*eye(3);                % -m * I (3x3)
E4=m*r_IC_s;                 % m * skew(r_IC) => couples COM to moments
E5=fC-m*OMs*OMs*r_IC;        % net force term including gravity/inertial centripetal
E6=r_IO_s;                   % r_IO_s again used in other block
E7=E4;                       % alias (used to build block)
E8=A*J*A';                   % inertia in inertial coordinates (A * J * A')
E9=-mC-r_IC_s*fC+A*omgs*J*omg; % net moment term including inertia coupling

% Assemble the 13x13 rigid-body transfer matrix using block rows/columns

U_RB=[eye(3)        E1            zeros(3)      zeros(3)      E2
      zeros(3)      eye(3)        zeros(3)      zeros(3)      zeros(3,1)
      E6*E3+E7      E6*E4+E8      eye(3)        E6            E6*E5+E9
      E3            E4            zeros(3)      eye(3)        E5
      zeros(1,3)    zeros(1,3)    zeros(1,3)    zeros(1,3)    1];

end
