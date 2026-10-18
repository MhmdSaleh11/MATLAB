function [DOT] = pen_4seg(t,z)

% pen_4seg:
% 3 rigid hose segments + rigid drogue
% Elastic pin hinges
% Aerodynamic forces + aerodynamic moment damping

%% ================= LOAD CFD DATABASE (ONLY ONCE) =================

persistent F Warned

if isempty(F)

    S = load('CFD_database.mat');

    F = S.F;

    Warned = false;

end

%% ================= CONSTANT PARAMETERS =================

% ---------------- Mass properties ----------------
m  = 0.75;        % hose segment mass (kg)
mD = 5;         % drogue mass (kg)
g  = 9.81;

% ---------------- Geometry ----------------
Lseg = 0.3;
LD   = 0.6;

lo  = [Lseg; 0; 0];
lc  = [Lseg/2; 0; 0];

loD = [LD; 0; 0];
lcD = [0.2; 0; 0];  % Drouge center of mass is shifted because the parachute part is lighter than the rest of the drogue body

% ---------------- Inertia ----------------
j = [0, 0.0225, 0.0225];
jD = j * (mD/m);

%% ================= HINGE PROPERTIES =================

k_h = 55;      % hinge stiffness (N·m/rad)
c_h = 7;       % hinge damping   (N·m·s/rad)

%% ================= STATE EXTRACTION =================

thr1 = z(1:3);
thr2 = z(7:9);
thr3 = z(13:15);
thrD = z(19:21);

uu=t
%% ================= CURRENT CFD ANGLES =================

S1_CFD = rad2deg(thr1(3)) - 180;
S2_CFD = rad2deg(thr2(3)) - 180;

%% ================= CFD DATABASE CHECK =================

if (~Warned) && ...
   (S1_CFD < F.S1min || S1_CFD > F.S1max || ...
    S2_CFD < F.S2min || S2_CFD > F.S2max)

    warning('CFD database exceeded: S1 = %.2f°, S2 = %.2f°', ...
            S1_CFD,S2_CFD);

    Warned = true;

end
%% ================= STATE EXTRACTION Cont. =================

OM1 = z(4:6);
OM2 = z(10:12);
OM3 = z(16:18);
OMD = z(22:24);

theta_j1 = z(25);
theta_j2 = z(26);
theta_j3 = z(27);

th_j_dot1 = z(28);
th_j_dot2 = z(29);
th_j_dot3 = z(30);

uu =t

%% ================= KINEMATICS =================

[A1,H1] = TM(thr1);
[A2,H2] = TM(thr2);
[A3,H3] = TM(thr3);
[AD,HD] = TM(thrD);

th_dot1 = H1 \ (A1 \ OM1);
th_dot2 = H2 \ (A2 \ OM2);
th_dot3 = H3 \ (A3 \ OM3);
th_dotD = HD \ (AD \ OMD);

omg_r1 = [0;0;th_j_dot1];
omg_r2 = [0;0;th_j_dot2];
omg_r3 = [0;0;th_j_dot3];

%% ================= AERODYNAMIC FORCES (UNCHANGED) =================
%% ================= REFERENCE FLIGHT CONDITION =================

Vref   = 132.2324;      % CFD reference velocity (m/s)
rhoref = 0.654;         % CFD reference density (kg/m^3)

%% ================= CURRENT FLIGHT CONDITION =================

Vinf = 132.2324;        % <-- change this for other flight speeds
rho  = 0.654;           % <-- change this for other altitudes

%% ================= DYNAMIC PRESSURE SCALING =================

scale = (rho*Vinf^2)/(rhoref*Vref^2);

Vair = [-Vinf;0;0];
%% 

CD1 = 0.0063333964; CL1 = 0.0158364624; A1s = 0.1101591;
CD2 = 0.0086287046; CL2 = 0.0110916234; A2s = 0.1101591;
CD3 = 0.0009158995; CL3 = 0.0030320726; A3s = 0.1101591;

CDD = 2.4232466;
CLD = -0.07087066;
ADa = 0.1101591;

Vrel = -Vair;
Vmag = norm(Vrel) + eps;

eD = -Vrel / Vmag;
eL = [-eD(2); eD(1); 0];

%% ======== CFD INTERPOLATED FORCES ========

% Segment 1
Drag1 = scale * F.Drag1(S1_CFD,S2_CFD);
Lift1 = scale * F.Lift1(S1_CFD,S2_CFD);

% Segment 2
Drag2 = scale * F.Drag2(S1_CFD,S2_CFD);
Lift2 = scale * F.Lift2(S1_CFD,S2_CFD);

% Segment 3
Drag3 = scale * F.Drag3(S1_CFD,S2_CFD);
Lift3 = scale * F.Lift3(S1_CFD,S2_CFD);

% Drogue
DragD = scale * F.DragD(S1_CFD,S2_CFD);
LiftD = scale * F.LiftD(S1_CFD,S2_CFD);

persistent firstPrint

if isempty(firstPrint)

    fprintf('\n================ INITIAL CFD FORCES ================\n');
    fprintf('Velocity Scale = %.4f\n',scale);
    fprintf('Angles:\n');
    fprintf('S1 = %.2f deg\n',S1_CFD);
    fprintf('S2 = %.2f deg\n\n',S2_CFD);

    fprintf('Segment 1 : Drag = %10.4f   Lift = %10.4f\n',Drag1,Lift1);
    fprintf('Segment 2 : Drag = %10.4f   Lift = %10.4f\n',Drag2,Lift2);
    fprintf('Segment 3 : Drag = %10.4f   Lift = %10.4f\n',Drag3,Lift3);
    fprintf('Drogue    : Drag = %10.4f   Lift = %10.4f\n',DragD,LiftD);

    fprintf('===================================================\n\n');

    firstPrint = true;

end

Faero1 = Drag1*eD + Lift1*eL;
Faero2 = Drag2*eD + Lift2*eL;
Faero3 = Drag3*eD + Lift3*eL;
FaeroD = DragD*eD + LiftD*eL;

%% ================= AERODYNAMIC MOMENT DAMPING (NEW) =================

eta_aero = 5;   % <<< damping acceleration factor (try 3–10)

c_aero1 = eta_aero * 0.5 * rho * A1s * Vinf * Lseg^2;
c_aero2 = eta_aero * 0.5 * rho * A2s * Vinf * Lseg^2;
c_aero3 = eta_aero * 0.5 * rho * A3s * Vinf * Lseg^2;
c_aeroD = eta_aero * 0.5 * rho * ADa * Vinf * LD^2;

Maero1 = -c_aero1 * OM1;
Maero2 = -c_aero2 * OM2;
Maero3 = -c_aero3 * OM3;
MaeroD = -c_aeroD * OMD;

%% ================= TOTAL FORCES =================

fC1 = [0;-m*g;0]  + Faero1;
fC2 = [0;-m*g;0]  + Faero2;
fC3 = [0;-m*g;0]  + Faero3;
fCD = [0;-mD*g;0] + FaeroD;


%% ================= TRANSFER MATRICES =================

U_RB1 = rigidbody(m,  lc,  lo,  thr1, OM1, j,  fC1, Maero1);
U_RB2 = rigidbody(m,  lc,  lo,  thr2, OM2, j,  fC2, Maero2);
U_RB3 = rigidbody(m,  lc,  lo,  thr3, OM3, j,  fC3, Maero3);
U_RBD = rigidbody(mD, lcD, loD, thrD, OMD, jD, fCD, MaeroD);

U_H1 = pin_hinge(A1, th_j_dot1, OM1, U_RB1);
U_H2 = pin_hinge(A2, th_j_dot2, OM2, U_RB2);
U_H3 = pin_hinge(A3, th_j_dot3, OM3, U_RB3);

%% ================= SYSTEM SOLUTION =================

syms omgz0 mx0 my0 qx0 qy0 qz0 ...
     xdd5 ydd5 zdd5 omgx5 omgy5 omgz5

Uall = U_RBD * U_RB3 * U_H3 * U_RB2 * U_H2 * U_RB1 * U_H1;

z0 = [0;0;0; 0;0;omgz0; mx0;my0;0; qx0;qy0;qz0; 1];
z5 = [xdd5; ydd5; zdd5; omgx5; omgy5; omgz5; 0;0;0; 0;0;0; 1];

[AA,BB] = equationsToMatrix(Uall*z0 == z5, ...
 [omgz0,mx0,my0,qx0,qy0,qz0,xdd5,ydd5,zdd5,omgx5,omgy5,omgz5]);

X = linsolve(double(AA), double(BB));

z0 = [0;0;0; 0;0;X(1); X(2);X(3);0; X(4);X(5);X(6); 1];

z1 = U_RB1 * U_H1 * z0;
z2 = U_RB2 * U_H2 * z1;
z3 = U_RB3 * U_H3 * z2;
z4 = U_RBD * z3;

%% ================= HINGE ACCELERATIONS =================

Hj = [0 0 1];

thjdd1 = Hj*A1'*z1(4:6) - Hj*A1'*z0(4:6) ...
       - k_h*theta_j1 ...
       - c_h*th_j_dot1;

thjdd2 = Hj*A2'*z2(4:6) - Hj*A2'*z1(4:6) ...
       - k_h*theta_j2 ...
       - c_h*th_j_dot2;

thjdd3 = Hj*A3'*z3(4:6) - Hj*A3'*z2(4:6) ...
       - k_h*theta_j3 ...
       - c_h*th_j_dot3;

%% ================= ASSEMBLE STATE DERIVATIVE =================

DOT = [ ...
    th_dot1; z1(4:6); ...
    th_dot2; z2(4:6); ...
    th_dot3; z3(4:6); ...
    th_dotD; z4(4:6); ...
    th_j_dot1; th_j_dot2; th_j_dot3; ...
    thjdd1; thjdd2; thjdd3; ...
    z1(1:3); z2(1:3) ];

end