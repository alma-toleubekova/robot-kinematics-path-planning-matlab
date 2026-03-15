%% 3D Workbench Visualisation (UR3 + UR10e)
clear; close all; clc;

%% Workspace dimensions (m)
W = 1.945;    % Y-axis span
H = 1.250;    % X-axis span

%% Object locations (m)
UR3_c   = [0.40, 0.37];
UR10_c  = [1.725, 0.20];

pos3_c  = [0.505, 0.945];
pos4_c  = [1.055, 0.405];

shield_c = [1.24, 0.72];

%% Object dimensions
robot_dim  = [0.16 0.16];
holder_dim = [0.15 0.09];
pcb_dim    = [0.085 0.055];
holder_h   = 0.14;
pcb_th     = 0.01;

shield_w   = 0.70;
shield_t   = 0.04;
shield_h   = 0.50;

%% Figure setup
figure('Color','w','Position',[100 100 1100 800]);
ax = axes; hold(ax,'on'); grid(ax,'on');

xlabel('Y (m)'); ylabel('X (m)'); zlabel('Z (m)');
title('3D Workbench with UR3 and UR10e (Stick Models)');
set(ax,'YDir','reverse');
view(45,25); axis equal;

xlim([0 W]); ylim([0 H]); zlim([0 0.9]);

drawRect3D = @(c,d,r,col,z,h) draw_rectangle3D(ax,c,d,r,col,z,h);

%% ------------------------------------------------------------------------
%                             Workbench Objects
%% ------------------------------------------------------------------------

% UR3 Base
drawRect3D(UR3_c, robot_dim, pi/2, [0.6 0.9 1], 0, 0.02);
text(UR3_c(1), UR3_c(2), 0.03, 'UR3');

% UR10e Base
drawRect3D(UR10_c, robot_dim, -pi/2, [1 0.85 0.85], 0, 0.02);
text(UR10_c(1), UR10_c(2), 0.03, 'UR10e');

% PCB Holders + PCBs
drawRect3D(pos3_c, holder_dim, 0,       [0.7 1 0.7], 0, holder_h);
drawRect3D(pos3_c, pcb_dim,    0,       [1 0.9 0.6], holder_h, pcb_th);
text(pos3_c(1), pos3_c(2), holder_h+0.03, 'Position 3');

drawRect3D(pos4_c, holder_dim, pi/2,    [0.7 1 0.7], 0, holder_h);
drawRect3D(pos4_c, pcb_dim,    pi/2,    [1 0.9 0.6], holder_h, pcb_th);
text(pos4_c(1), pos4_c(2), holder_h+0.03, 'Position 4');

% Soldering Shield
drawRect3D(shield_c, [shield_w shield_t], 0, [0.9 0.85 0.55], 0, shield_h);
text(shield_c(1), shield_c(2), shield_h+0.05, 'Shield');

% Workbench outline
plot3([0 W W 0 0], [0 0 H H 0], [0 0 0 0 0], 'k-', 'LineWidth',2);

%% ------------------------------------------------------------------------
%                          UR3 Stick Model (visual)
%% ------------------------------------------------------------------------
L3 = [0.152; 0.112; 0.244; 0.112; 0.213; 0.112; 0.085; 0.082; 0.152];
DIR3 = [
    0 0 1; 1 0 0; 0 0 1; -1 0 0; 0 0 1; 1 0 0; 0 0 1; 1 0 0; 1 0 0
];

P3 = zeros(3,length(L3)+1);
P3(:,1) = [UR3_c(1); UR3_c(2); 0];

for i=1:length(L3)
    P3(:,i+1) = P3(:,i) + DIR3(i,:)' * L3(i);
end

plot3(P3(1,1:end-2), P3(2,1:end-2), P3(3,1:end-2), 'b.-','LineWidth',4);
plot3(P3(1,end-2:end), P3(2,end-2:end), P3(3,end-2:end), 'g.-','LineWidth',5);
text(P3(1,end), P3(2,end), P3(3,end)+0.03, 'UR3 TCP', 'Color','g');

%% ------------------------------------------------------------------------
%                        UR10e Stick Model (visual)
%% ------------------------------------------------------------------------
L10 = [0.180; 0.1741; 0.6127; 0.1741; 0.5716; 0.1741; 0.1198; 0.1165; 0.280];
DIR10 = [
     0 0 1; -1 0 0; 0 0 1; 1 0 0; 0 0 1; -1 0 0; 0 0 1; -1 0 0; -1 0 0
];

P10 = zeros(3,length(L10)+1);
P10(:,1) = [UR10_c(1); UR10_c(2); 0];

for i=1:length(L10)
    P10(:,i+1) = P10(:,i) + DIR10(i,:)' * L10(i);
end

plot3(P10(1,1:end-2), P10(2,1:end-2), P10(3,1:end-2), 'r.-','LineWidth',4);
plot3(P10(1,end-2:end), P10(2,end-2:end), P10(3,end-2:end), 'g.-','LineWidth',5);
text(P10(1,end), P10(2,end), P10(3,end)+0.03, 'UR10 TCP', 'Color','g');

%% ------------------------------------------------------------------------
function draw_rectangle3D(ax, center, dims, rot, color, baseZ, height)
    w = dims(1); h = dims(2);

    R = [-w/2 -h/2; w/2 -h/2; w/2 h/2; -w/2 h/2]';
    Rm = [cos(rot) -sin(rot); sin(rot) cos(rot)];
    Rr = Rm * R;

    X = Rr(1,:) + center(1);
    Y = Rr(2,:) + center(2);
    Zb = baseZ; Zt = baseZ + height;

    for i=1:4
        j = mod(i,4)+1;
        patch(ax,[X(i) X(j) X(j) X(i)], ...
                 [Y(i) Y(j) Y(j) Y(i)], ...
                 [Zb Zb Zt Zt], color, ...
                 'FaceAlpha',0.5,'EdgeColor','k');
    end

    patch(ax, X, Y, Zt*ones(1,4), color, 'FaceAlpha',0.8,'EdgeColor','k');
end
