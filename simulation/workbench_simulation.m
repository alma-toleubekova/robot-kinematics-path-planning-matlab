function workbench_simulation(mode, varargin)
% WORKBENCH_SIMULATION
%   Visualises:
%       - Workcell layout      : workbench_simulation('workcell')
%       - Static robots        : workbench_simulation('static', ur3, q3, ur10, q10)
%       - Full animation       : workbench_simulation('animate', ur3, traj3, ur10, traj10, t3, t10)
%
%   World coordinates: METERS
%   Robot FK: MILLIMETERS → converted to meters for plotting
%
%   Coordinate convention:
%       axis X (horizontal)  : world Y (→ right)
%       axis Y (horizontal)  : world X (→ down)
%       axis Z (vertical)    : world Z (↑ up)

mode  = lower(mode);
const = get_constants();

switch mode
    case 'workcell'
        fig = figure('Color','w','Position',[100 100 1100 800]);
        ax  = init_axes(fig, const);
        draw_workcell(ax, const);
        title(ax,'Workbench Layout','FontSize',14,'FontWeight','bold');

    case 'static'
        % Usage: workbench_simulation('static', ur3, q3, ur10e, q10)
        ur3   = varargin{1};  q3  = varargin{2};
        ur10e = varargin{3};  q10 = varargin{4};

        fig = figure('Color','w','Position',[100 100 1100 800]);
        ax  = init_axes(fig, const);
        draw_workcell(ax, const);

        % === VISUAL BASE ROTATIONS (do NOT touch IK) ===
        ur3_vis  = ur3;
        ur10_vis = ur10e;
        % UR3 base as given by your DH model (faces towards screen)
        ur3_vis.theta_offset(1)  = ur3.theta_offset(1);
        % UR10e rotated -90° around base so it faces UR3
        ur10_vis.theta_offset(1) = ur10e.theta_offset(1) - pi/2;

        draw_robot(ax, ur3_vis,  q3,  [0 0.4 0.8]);
        draw_robot(ax, ur10_vis, q10, [0.9 0.2 0]);

        title(ax,'Workbench With Robots','FontSize',14,'FontWeight','bold');

    case 'animate'
        % Usage: workbench_simulation('animate', ur3, traj3, ur10, traj10, t3, t10)
        ur3    = varargin{1}; traj3  = varargin{2};
        ur10e  = varargin{3}; traj10 = varargin{4};
        t3     = varargin{5}; t10    = varargin{6};

        fig = figure('Color','w','Position',[100 100 1100 800]);
        ax  = init_axes(fig, const);
        draw_workcell(ax, const);

        animate_robots(ax, const, ur3, traj3, ur10e, traj10, t3, t10);

    otherwise
        error('Unknown mode "%s". Use: workcell | static | animate', mode);
end
end

%% =====================================================================
%                            CONSTANTS
% =====================================================================
function const = get_constants()

const.W = 1.945;          % Y
const.H = 1.250;          % X

% Base centres [Y, X] in meters
const.UR3_c   = [0.37  0.40];
const.UR10_c  = [1.725 0.20];

% PCB holder centres [Y, X]
const.pos3_c  = [0.505 0.945];
const.pos4_c  = [1.055 0.405];

% Screen centre [Y, X]
const.shield_c = [1.24 0.72];

% Object dimensions
const.robot_dim  = [0.16 0.16];
const.holder_dim = [0.15 0.09];
const.pcb_dim    = [0.085 0.055];

const.holder_h = 0.14;
const.pcb_th   = 0.01;

const.shield_w = 0.70;
const.shield_t = 0.04;
const.shield_h = 0.50;
end

%% =====================================================================
%                              AXES
% =====================================================================
function ax = init_axes(fig, const)
ax = axes('Parent',fig); hold(ax,'on'); grid(ax,'on');

xlabel(ax,'Y (m) \rightarrow right');
ylabel(ax,'X (m) \rightarrow down');
zlabel(ax,'Z (m) \rightarrow up');

xlim(ax,[0 const.W]);
ylim(ax,[0 const.H]);
zlim(ax,[0 1.6]);

set(ax,'YDir','reverse');    % X axis downwards
view(ax,45,25);
axis(ax,'equal');
end

%% =====================================================================
%                           WORKCELL DRAW
% =====================================================================
function draw_workcell(ax, const)

drawRect3D = @(c,d,r,col,z,h) draw_rectangle3D(ax,c,d,r,col,z,h);

% Workbench outline
plot3(ax,[0 const.W const.W 0 0],[0 0 const.H const.H 0],[0 0 0 0 0], ...
      'k','LineWidth',2);

% UR3 base
drawRect3D(const.UR3_c,  const.robot_dim,  pi/2,[0.6 0.9 1],0,0.02);
text(const.UR3_c(1),const.UR3_c(2)+0.03,0.03,'UR3','Parent',ax, ...
     'HorizontalAlignment','center');

% UR10e base
drawRect3D(const.UR10_c, const.robot_dim, -pi/2,[1 0.85 0.85],0,0.02);
text(const.UR10_c(1),const.UR10_c(2)+0.03,0.03,'UR10e','Parent',ax, ...
     'HorizontalAlignment','center');

% POS3 holder + PCB (horizontal)
drawRect3D(const.pos3_c,const.holder_dim,0,[0.7 1 0.7],0,const.holder_h);
draw_pcb_oriented(ax, const.pos3_c, const.pcb_dim, ...
                  0, const.holder_h, const.pcb_th, [1 0.9 0.6]);
text(const.pos3_c(1),const.pos3_c(2),const.holder_h+0.03,'POS 3', ...
     'Parent',ax,'HorizontalAlignment','center');

% POS4 holder + PCB (rotated 90°)
drawRect3D(const.pos4_c,const.holder_dim,pi/2,[0.7 1 0.7],0,const.holder_h);
draw_pcb_oriented(ax, const.pos4_c, const.pcb_dim, ...
                  pi/2, const.holder_h, const.pcb_th, [1 0.9 0.6]);
text(const.pos4_c(1),const.pos4_c(2),const.holder_h+0.03,'POS 4', ...
     'Parent',ax,'HorizontalAlignment','center');

% Screen
drawRect3D(const.shield_c,[const.shield_w const.shield_t],0,[0.9 0.85 0.55],0,const.shield_h);
text(const.shield_c(1),const.shield_c(2),const.shield_h+0.05,'SCREEN', ...
     'Parent',ax,'HorizontalAlignment','center');
end

%% =====================================================================
%                             ANIMATION
% =====================================================================
function animate_robots(ax, const, ur3, traj3, ur10e, traj10, t3, t10)

% --- Visual-only base rotations (robots face each other) ---
ur3_vis  = ur3;
ur10_vis = ur10e;
ur3_vis.theta_offset(1)  = ur3.theta_offset(1);          % as per DH
ur10_vis.theta_offset(1) = ur10e.theta_offset(1); % turn towards UR3

T3  = t3(:);
T10 = t10(:);
T3_end  = T3(end);
T10_end = T10(end);
t_total = T3_end + T10_end;   % UR10e starts after UR3 finishes

% --- PCB grip timing from UR3 gripper trajectory ---
g = traj3.gripper(:);
idx_on  = find(g > 0.5, 1, 'first'); % first time gripper closes
idx_off = find(g > 0.5, 1, 'last');  % last time gripper closed

if isempty(idx_on)
    t_on  = Inf;
    t_off = -Inf;
else
    t_on  = T3(idx_on);
    t_off = T3(idx_off);
end

fps = 30;
dt  = 1/fps;

h3   = [];
h10  = [];
hpcb = [];

for t = 0:dt:t_total

    if ~ishandle(ax)
        break;
    end

    % Delete previous robot & PCB graphics (layout stays)
    if ~isempty(h3),   delete(h3);   h3   = []; end
    if ~isempty(h10),  delete(h10);  h10  = []; end
    if ~isempty(hpcb)
        delete(hpcb(ishandle(hpcb)));
        hpcb = [];
    end

    % ------------------ PHASE SELECTION ------------------
    if t <= T3_end
        % ========= PHASE 1: UR3 active, UR10e at home =========
        q3  = interp_q(traj3.q,  T3,  t);
        q10 = traj10.q(1,:);          % UR10e home

        % UR3 TCP in meters (world Y, X, Z)
        [~, joints3_mm] = forward_kinematics(q3, ur3);
        ee3_m = joints3_mm(end,:) / 1000;    % [Y X Z]

        % --- PCB STATE MACHINE (UR3) ---
        if t < t_on
            % PCB resting on POS3 (horizontal)
            pcb_mode = "POS3";
        elseif t <= t_off
            % PCB held by UR3 TCP (horizontal, moves with tool)
            pcb_mode = "HELD";
        else
            % PCB placed on POS4 (vertical)
            pcb_mode = "POS4";
        end

    else
        % ========= PHASE 2: UR10e active, UR3 holds final pose =========
        t_rel = t - T3_end;

        q3  = traj3.q(end,:);        % UR3 stays at final place pose
        if t_rel <= T10_end
            q10 = interp_q(traj10.q, T10, t_rel);
        else
            q10 = traj10.q(end,:);
        end

        % In phase 2 the PCB is already on POS4
        pcb_mode = "POS4";
        ee3_m = [];  %#ok<NASGU>
    end

    % ------------------ DRAW ROBOTS ------------------
    h3  = draw_robot(ax, ur3_vis,  q3,  [0 0.4 0.8]);
    h10 = draw_robot(ax, ur10_vis, q10, [0.9 0.2 0]);

    % ------------------ DRAW PCB ------------------
    switch pcb_mode
        case "POS3"
            % On holder at POS3, horizontal (same as layout)
            hpcb = draw_pcb_oriented(ax, const.pos3_c, const.pcb_dim, ...
                                     0, const.holder_h, const.pcb_th, [1 0.9 0.6]);

        case "HELD"
            % PCB attached to UR3 TCP, horizontal, centered under TCP
            y = ee3_m(1);    % world Y
            x = ee3_m(2);    % world X
            z = ee3_m(3);    % world Z

            center2D = [y, x];
            baseZ    = z - const.pcb_th;  % so TCP is slightly above PCB top

            hpcb = draw_pcb_oriented(ax, center2D, const.pcb_dim, ...
                                     0, baseZ, const.pcb_th, [1 0.9 0.6]);

        case "POS4"
            % On holder at POS4, rotated by +90°, as in layout
            hpcb = draw_pcb_oriented(ax, const.pos4_c, const.pcb_dim, ...
                                     pi/2, const.holder_h, const.pcb_th, [1 0.9 0.6]);
    end

    % ------------------ UI ------------------
    title(ax, sprintf('t = %.2f s', t), ...
          'FontSize', 12, 'FontWeight', 'bold');
    drawnow;
end
end

%% =====================================================================
%                           DRAW ROBOT
% =====================================================================
function h = draw_robot(ax, robot, q, color)
[~, joints_mm] = forward_kinematics(q, robot);

% ===== CORRECT TCP OFFSET ALONG LAST LINK DIRECTION (UR10) =====
if isfield(robot,'name') && strcmpi(robot.name,'UR10e')

    % Tool length (mm)
    Ltool = 160;   % adjust only this if needed (140–180)

    % Direction of the last link (from joint 6 to TCP)
    v = joints_mm(end,:) - joints_mm(end-1,:);
    v = v / norm(v);   % unit direction vector

    % Apply offset ALONG tool axis
    joints_mm(end,:) = joints_mm(end,:) + Ltool * v;

end
% ===============================================================

j = joints_mm/1000;
h = plot3(ax,j(:,1),j(:,2),j(:,3),'o-','Color',color,'LineWidth',3,'MarkerFaceColor',color);
end

%% =====================================================================
%                         LINEAR INTERP
% =====================================================================
function q = interp_q(Q,T,t)
t = max(T(1),min(t,T(end)));    % clamp time
q = zeros(1,size(Q,2));
for i = 1:size(Q,2)
    q(i) = interp1(T, Q(:,i), t, 'linear');
end
end

%% =====================================================================
%                           PRISM GEOMETRY
% =====================================================================
function draw_rectangle3D(ax,c,d,r,col,z,h)
% c = [Yc, Xc], d = [width, depth] in meters
w = d(1); 
l = d(2);

% Rectangle in its own frame
R = [-w/2 -l/2;
      w/2 -l/2;
      w/2  l/2;
     -w/2  l/2]';

% In-plane rotation
Rm = [cos(r) -sin(r); sin(r) cos(r)];
Rr = Rm * R;

X = Rr(1,:) + c(1);   % world Y
Y = Rr(2,:) + c(2);   % world X

Zb = z;
Zt = z + h;

for i = 1:4
    j = mod(i,4)+1;
    patch(ax, ...
        [X(i) X(j) X(j) X(i)], ...
        [Y(i) Y(j) Y(j) Y(i)], ...
        [Zb  Zb  Zt  Zt], ...
        col,'FaceAlpha',0.5,'EdgeColor','k');
end

patch(ax, X, Y, ones(1,4)*Zt, col, ...
      'FaceAlpha',0.8,'EdgeColor','k');
end

%% =====================================================================
%                        ORIENTED PCB DRAWING
% =====================================================================
function hpcb = draw_pcb_oriented(ax, center2D, dims, rot, baseZ, thickness, color)
% center2D : [Yc, Xc] in meters (planar center)
% dims     : [width, depth] in meters
% rot      : yaw in radians (0 = along Y, +pi/2 = along X)
% baseZ    : bottom Z in meters
% thickness: PCB thickness
% color    : [r g b]

w = dims(1);
l = dims(2);

% Rectangle in local coordinates (centered)
R = [-w/2 -l/2;
      w/2 -l/2;
      w/2  l/2;
     -w/2  l/2]';

% Rotate in the table plane
Rm = [cos(rot) -sin(rot); sin(rot) cos(rot)];
Rr = Rm * R;

X = Rr(1,:) + center2D(1);   % world Y
Y = Rr(2,:) + center2D(2);   % world X

Zb = baseZ;
Zt = baseZ + thickness;

p1 = patch(ax, X, Y, Zb*ones(1,4), color, ...
           'FaceAlpha',0.95,'EdgeColor','k');
p2 = patch(ax, X, Y, Zt*ones(1,4), color, ...
           'FaceAlpha',0.95,'EdgeColor','k');

hpcb = [p1; p2];
end
