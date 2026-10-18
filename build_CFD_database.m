%% ==========================================================
% Build CFD interpolation database
% Run ONLY ONCE
%% ==========================================================

clear
clc

%% Read Excel files

Drag = readtable('cfd matlab map (drag).xlsx');
Lift = readtable('cfd matlab map (lift).xlsx');

%% Angle vectors

S1 = unique(Drag{:,2});
S2 = unique(Drag{:,3});

%% Allocate matrices

Drag1 = zeros(length(S1),length(S2));
Drag2 = Drag1;
Drag3 = Drag1;
DragD = Drag1;

Lift1 = Drag1;
Lift2 = Drag1;
Lift3 = Drag1;
LiftD = Drag1;

%% Fill matrices

for i = 1:height(Drag)

    r = find(S1==Drag{i,2});
    c = find(S2==Drag{i,3});

    Drag1(r,c)=Drag{i,6};
    Drag2(r,c)=Drag{i,7};
    Drag3(r,c)=Drag{i,8};
    DragD(r,c)=Drag{i,9};

    Lift1(r,c)=Lift{i,6};
    Lift2(r,c)=Lift{i,7};
    Lift3(r,c)=Lift{i,8};
    LiftD(r,c)=Lift{i,9};

end

%% Build interpolants

F.Drag1 = griddedInterpolant({S1,S2},Drag1,'linear','linear');
F.Drag2 = griddedInterpolant({S1,S2},Drag2,'linear','linear');
F.Drag3 = griddedInterpolant({S1,S2},Drag3,'linear','linear');
F.DragD = griddedInterpolant({S1,S2},DragD,'linear','linear');

F.Lift1 = griddedInterpolant({S1,S2},Lift1,'linear','linear');
F.Lift2 = griddedInterpolant({S1,S2},Lift2,'linear','linear');
F.Lift3 = griddedInterpolant({S1,S2},Lift3,'linear','linear');
F.LiftD = griddedInterpolant({S1,S2},LiftD,'linear','linear');

%% Store valid CFD angle limits

F.S1min = min(S1);
F.S1max = max(S1);

F.S2min = min(S2);
F.S2max = max(S2);

%% Save

save CFD_database.mat F

disp('CFD database successfully created.')