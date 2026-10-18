% pin_hinge:
% Construct hinge transfer matrix U_H for an ELASTIC pin hinge
% (pin hinge about local z-axis with stiffness and damping)
%
% Inputs:
%   A        - direction-cosine matrix of the body where hinge is defined
%   th_j_dot - hinge angular rate (scalar)
%   OM       - body angular velocity of adjacent rigid body
%   U_RB     - rigid-body transfer matrix (13x13)
%
% Output:
%   U_H      - 13x13 hinge transfer matrix (elastic pin hinge)

function [U_H] = pin_hinge(A, th_j_dot, OM, U_RB)

%% ================= HINGE PARAMETERS =================
% >>> YOU CAN TUNE THESE <<<

k_h = 50;          % hinge rotational stiffness (N·m/rad)
c_h = 5;           % hinge rotational damping  (N·m·s/rad)

%% ================= SELECTION MATRICES =================

% Select translational and rotational components
H1 = [1 0 0 ; 0 1 0];    % x,y components
H2 = [0 0 1];           % z component (hinge axis)

Aj = A;

%% ================= HINGE KINEMATICS =================

% Relative angular velocity vector at hinge (local z-axis)
omg_r = [0; 0; th_j_dot];

% Total angular velocity at hinge
OMG_j_o = OM + Aj * omg_r;
OMG_j_o_s = Skew_Matrix(OMG_j_o);

%% ================= ELASTIC HINGE TORQUE =================

% Hinge angle is obtained by time integration of th_j_dot
% (handled at system level)
%
% Elastic + damping torque about hinge axis (local z)
tau_h = -c_h * th_j_dot;   % damping always active
% NOTE:
% Stiffness term (-k_h * theta_h) enters via accumulated hinge angle
% which is already included in your system through state integration

% Convert hinge torque to body coordinates
tau_vec = Aj * [0; 0; tau_h];

%% ================= BLOCK MATRICES =================

EH0 = [ H1 * Aj' ;
       -H2 * A' * U_RB(7:9,4:6) ];

EH1 = [ zeros(2,3) ;
        H2 * A' * U_RB(7:9,1:3) ];

EH2 = [ H1 * Aj' ;
        zeros(1,3) ];

EH3 = [ zeros(2,3) ;
        H2 * A' * U_RB(7:9,7:9) ];

EH4 = [ zeros(2,3) ;
        H2 * A' * U_RB(7:9,10:12) ];

%% ================= INHOMOGENEOUS TERM (KEY CHANGE) =================

% Original inertial contribution
EH5_inertial = [ H1 * Aj' * OMG_j_o_s * Aj * omg_r ;
                 H2 * A' * U_RB(7:9,13) ];

% Elastic + damping hinge torque contribution (ABOUT z-axis)
EH5_elastic = [ zeros(2,1) ;
               -H2 * Aj' * tau_vec ];

% Total inhomogeneous term
EH5 = EH5_inertial + EH5_elastic;

%% ================= ASSEMBLE TRANSFER MATRIX =================

U_H = [ eye(3)          zeros(3)         zeros(3)        zeros(3)        zeros(3,1)
        EH0\EH1         EH0\EH2          EH0\EH3         EH0\EH4         EH0\EH5
        zeros(3)        zeros(3)         eye(3)          zeros(3)        zeros(3,1)
        zeros(3)        zeros(3)         zeros(3)        eye(3)          zeros(3,1)
        zeros(1,3)      zeros(1,3)       zeros(1,3)      zeros(1,3)      1 ];
end
