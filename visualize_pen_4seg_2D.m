%% 2D Visualization script for pen_4seg results
% Produces a 2D animation and optional MP4 video in the X-Y plane

clc
close all

%% --- Ensure simulation data exists ---
if ~exist('t','var') || ~exist('z','var')
    if exist('pen4seg_results.mat','file')
        load('pen4seg_results.mat','t','z')
        disp('Loaded t and z from pen4seg_results.mat')
    else
        error(['t and z not found.\n', ...
               'Run the main script or load pen4seg_results.mat']);
    end
end

%% Geometry parameters
L1 = 0.3;    % length of segment 1 (m)
L2 = 0.3;    % length of segment 2 (m)
L3 = 0.3;    % length of segment 3 (m)

LD = 2*L3;        % drogue length (twice a segment length)
Ds = LD/3;        % small side length of trapezoid (attached to segment 3)

%% Figure setup (2D)
figure('Color','w')
axis equal
grid on
hold on

xlabel('X (m)')
ylabel('Y (m)')
xlim([-2 .5])
ylim([-1 0.7])

% Ground / reference line
plot([-5 5],[0 0],'k--')

%% Plot handles
h1  = plot([0 0],[0 0],'b','LineWidth',3);    % segment 1
h2  = plot([0 0],[0 0],'r','LineWidth',3);    % segment 2
h3  = plot([0 0],[0 0],'g','LineWidth',3);    % segment 3

hJ1 = plot(0,0,'ko','MarkerFaceColor','k');   % hinge between segment 1 and 2
hJ2 = plot(0,0,'ko','MarkerFaceColor','k');   % hinge between segment 2 and 3

% Drogue patch (trapezoid)
hD = patch('XData',nan,'YData',nan,'FaceColor',[0.6 0.6 0.6]);

%% Video writer (optional)
makeVideo = true;

if makeVideo
    v = VideoWriter('pen_4seg_motion_2D.mp4','MPEG-4');
    v.FrameRate = 30;
    open(v);
end

%% Animation loop
for k = 1:length(t)

    % --- Extract Euler angles ---
    th1 = z(k,1:3).';
    th2 = z(k,7:9).';
    th3 = z(k,13:15).';
    thD = z(k,19:21).';     % drogue orientation

    % --- Rotation matrices ---
    [A1,~] = TM(th1);
    [A2,~] = TM(th2);
    [A3,~] = TM(th3);
    [AD,~] = TM(thD);

    % --- Positions (planar projection X-Y) ---
    O0 = [0;0;0];                 % base point

    P1 = O0 + A1*[L1;0;0];        % end of segment 1
    P2 = P1 + A2*[L2;0;0];        % end of segment 2
    P3 = P2 + A3*[L3;0;0];        % end of segment 3 (drogue attachment point)

    % --- Update segments ---
    set(h1,'XData',[O0(1) P1(1)], ...
           'YData',[O0(2) P1(2)]);

    set(h2,'XData',[P1(1) P2(1)], ...
           'YData',[P1(2) P2(2)]);

    set(h3,'XData',[P2(1) P3(1)], ...
           'YData',[P2(2) P3(2)]);

    % --- Update hinge markers ---
    set(hJ1,'XData',P1(1),'YData',P1(2));   % hinge 1–2
    set(hJ2,'XData',P2(1),'YData',P2(2));   % hinge 2–3

    %% --- Drogue trapezoid geometry (local frame) ---
    % Small side is attached to segment 3, large side is downstream

    % Local coordinates of trapezoid corners (small side centered at origin)
Dloc = [ ...
    0     -Ds/2;       % small side, bottom
    0      Ds/2;       % small side, top
    LD     LD/2;       % large side, top
    LD    -LD/2 ]';    % large side, bottom


    % Rotate and translate drogue to global frame
    Dglob = AD(1:2,1:2) * Dloc + P3(1:2);

    % Update drogue patch
    set(hD,'XData',Dglob(1,:), ...
           'YData',Dglob(2,:));

    title(sprintf('2D Motion (X-Y Plane),  t = %.2f s', t(k)))

    drawnow

    % --- Save frame ---
    if makeVideo
        writeVideo(v, getframe(gcf));
    end
end

%% Close video
if makeVideo
    close(v);
    disp('2D video saved: pen_4seg_motion_2D.mp4');
end
