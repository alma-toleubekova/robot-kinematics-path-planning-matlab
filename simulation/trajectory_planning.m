function [traj, t] = trajectory_planning(robot_type, operation, dt)
% Generate joint-space trajectories for UR3 pick-and-place
% or UR10e soldering. Produces smooth quintic paths.

if nargin < 3
    dt = 0.01;   % default time step
end

robot_type = upper(robot_type);

switch robot_type
    case 'UR3'
        [traj, t] = ur3_pick_place(dt);

    case 'UR10E'
        [traj, t] = ur10e_soldering(dt);

    otherwise
        error('Unknown robot type: %s', robot_type);
end
end

%%
function [traj, t] = ur3_pick_place(dt)
fprintf('Generating UR3 pick-and-place trajectory...\n');

robot = parameters('UR3');

% Predefined IK targets
home = positions('UR3','Home');
pos3 = positions('UR3','Pos3');
pos4 = positions('UR3','Pos4');

q_home = home.q;
q_pick_seed  = pos3.q;
q_place_seed = pos4.q;

target3 = pos3.target;
target4 = pos4.target;

% Refine using orientation-aware IK
q_pick  = solve_ik_ur3(robot, target3, q_pick_seed);
q_place = solve_ik_ur3(robot, target4, q_place_seed);

% Lift above PCB
lift_z = 400;
target3_above = target3; target3_above(3) = target3(3) + lift_z;
target4_above = target4; target4_above(3) = target4(3) + lift_z;

q_above_pick  = solve_ik_ur3(robot, target3_above, q_pick);
q_above_place = solve_ik_ur3(robot, target4_above, q_place);

% Mid-height detour (screen avoidance)
target_detour    = target3_above;
target_detour(1) = target3_above(1);
target_detour(2) = mean([target3_above(2), target4_above(2)]);
q_detour = solve_ik_ur3(robot, target_detour, q_above_pick);

% Waypoint list
waypoints = [
    q_home;
    q_above_pick;
    q_pick;
    q_above_pick;
    q_detour;
    q_above_place;
    q_place;
    q_above_place;
    q_home
];

segment_times = [1.5; 0.8; 0.6; 1.4; 1.4; 0.8; 0.6; 1.5];

% Gripper schedule (0=open,1=closed)
gripper = [0;0;1;1;1;1;0;0;0];

phase_names = {
    'Approach POS3';
    'Descend to POS3';
    'Lift from POS3';
    'Detour';
    'Move towards POS4';
    'Descend to POS4';
    'Release';
    'Return home'
};

[traj, t] = compute_quintic_trajectory( ...
    waypoints, segment_times, gripper, phase_names, dt);

fprintf('UR3 trajectory ready (%.2f s)\n\n', t(end));
end

%% 
function [traj, t] = ur10e_soldering(dt)
fprintf('Generating UR10e soldering trajectory...\n');

robot = parameters('UR10e');

% Pre-calibrated pin targets
p1 = positions('UR10e','P1');
p2 = positions('UR10e','P2');
p3 = positions('UR10e','P3');
p4 = positions('UR10e','P4');
home = positions('UR10e','Home');

q_home = home.q;

T1 = p1.target; T2 = p2.target; T3 = p3.target; T4 = p4.target;

% Approach 120 mm vertically above each point
lift = 120;
T1a = T1; T1a(3)=T1(3)+lift;
T2a = T2; T2a(3)=T2(3)+lift;
T3a = T3; T3a(3)=T3(3)+lift;
T4a = T4; T4a(3)=T4(3)+lift;

% IK with vertical-tool constraint
q1a = solve_ik_ur10e(robot, T1a, q_home);
q1  = solve_ik_ur10e(robot, T1,  q1a);

q2a = solve_ik_ur10e(robot, T2a, q1);
q2  = solve_ik_ur10e(robot, T2,  q2a);

q3a = solve_ik_ur10e(robot, T3a, q2);
q3  = solve_ik_ur10e(robot, T3,  q3a);

q4a = solve_ik_ur10e(robot, T4a, q3);
q4  = solve_ik_ur10e(robot, T4,  q4a);

waypoints = [
    q_home;
    q1a; q1; q1; q1a;
    q2a; q2; q2; q2a;
    q3a; q3; q3; q3a;
    q4a; q4; q4; q4a;
    q_home
];

segment_times = [
    1.5;
    0.5;0.6;0.4;0.5;
    0.5;0.6;0.4;0.5;
    0.5;0.6;0.4;0.5;
    0.5;0.6;0.4;1.5
];

tool = [
    0;
    0;1;1;0;
    0;1;1;0;
    0;1;1;0;
    0;1;1;0;
    0
];

phase_names = repmat({'Soldering'}, length(segment_times), 1);

[traj, t] = compute_quintic_trajectory( ...
    waypoints, segment_times, tool, phase_names, dt);

fprintf('UR10e trajectory ready (%.2f s)\n', t(end));
end

%% ------------------------------------------------------------------------
function [traj, t] = compute_quintic_trajectory(waypoints, segment_times, ...
                                                tool_states, phase_labels, dt)
% Smooth joint trajectory generation using quintic polynomials

n_joints = size(waypoints,2);
q_all = [];
qd_all = [];
qdd_all = [];
tool_all = [];
phase_all = {};
t_all = [];
t_offset = 0;

for seg = 1:length(segment_times)
    T  = segment_times(seg);
    q0 = waypoints(seg,:);
    qf = waypoints(seg+1,:);
    
    t_seg = (0:dt:T)';
    n_points = length(t_seg);
    
    q_seg   = zeros(n_points, n_joints);
    qd_seg  = zeros(n_points, n_joints);
    qdd_seg = zeros(n_points, n_joints);
    
    for j = 1:n_joints
        [q_seg(:,j), qd_seg(:,j), qdd_seg(:,j)] = ...
            quintic_polynomial(q0(j), qf(j), T, t_seg);
    end
    
    % Tool state interpolation (step at midpoint)
    tool_seg = ones(n_points,1)*tool_states(seg);
    if tool_states(seg) ~= tool_states(seg+1)
        mid_idx = round(n_points/2);
        tool_seg(mid_idx:end) = tool_states(seg+1);
    end
    
    % Phase label
    phase_seg = repmat(phase_labels(seg), n_points, 1);
    
    % Append without duplicate points
    if seg == 1
        q_all   = [q_all;   q_seg];
        qd_all  = [qd_all;  qd_seg];
        qdd_all = [qdd_all; qdd_seg];
        tool_all  = [tool_all;  tool_seg];
        phase_all = [phase_all; phase_seg];
        t_all   = [t_all;   t_seg + t_offset];
    else
        q_all   = [q_all;   q_seg(2:end,:)];
        qd_all  = [qd_all;  qd_seg(2:end,:)];
        qdd_all = [qdd_all; qdd_seg(2:end,:)];
        tool_all  = [tool_all;  tool_seg(2:end)];
        phase_all = [phase_all; phase_seg(2:end)];
        t_all   = [t_all;   t_seg(2:end) + t_offset];
    end
    
    t_offset = t_offset + T;
end

traj.q   = q_all;
traj.qd  = qd_all;
traj.qdd = qdd_all;
traj.gripper = tool_all;
traj.phase   = phase_all;

t = t_all;
end

%%
function [q, qd, qdd] = quintic_polynomial(q0, qf, T, t)
% Quintic polynomial for smooth rest-to-rest joint motion

a0 = q0; a1 = 0; a2 = 0;
a3 = 10*(qf-q0)/T^3;
a4 = -15*(qf-q0)/T^4;
a5 =  6*(qf-q0)/T^5;

q   = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
qd  =      3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;
qdd =      6*a3*t    +12*a4*t.^2 +20*a5*t.^3;
end

%%
function q_sol = solve_ik_ur3(robot, target_W, q0)
% target_W : [Y X Z] in mm (WORLD frame)
% q0       : initial guess (1x6)
%
% Cost:
%   - position error in mm (scaled)
%   - orientation error: TCP z-axis vs world z-axis
%     → makes TCP "horizontal" (tool axis vertical)

    cost = @(q) ik_cost(q, robot, target_W);

    opts = optimset('Display','off', ...
                    'TolX',1e-5, 'TolFun',1e-6, ...
                    'MaxFunEvals',2000, 'MaxIter',1000);

    q_sol = fminsearch(cost, q0, opts);

    % Wrap to [-pi, pi] for neatness
    q_sol = atan2(sin(q_sol), cos(q_sol));
end

function J = ik_cost(q, robot, target_W)
    q = q(:).';

    % Forward kinematics
    [T_ee, joints_mm] = forward_kinematics(q, robot);

    % 1) POSITION ERROR (mm)
    tcp_W = joints_mm(end,:);   % [Y X Z] mm
    e_pos = tcp_W - target_W;   % mm

    e_pos_scaled = e_pos / 100;      % scale to cm
    pos_cost = e_pos_scaled * e_pos_scaled.';

    % 2) ORIENTATION ERROR
    % TCP Z must be HORIZONTAL
    R = T_ee(1:3,1:3);
    z_tcp = R(:,3);          % TCP Z axis
    z_world = [0;0;1];

    % Penalize being VERTICAL
    % If dot = 1 → BAD
    % If dot = 0 → PERFECT
    dot_val = z_tcp.' * z_world;

    orient_cost = dot_val^2;   % force dot → 0

    % 3) WEIGHTING
    w_pos    = 1.0;
    w_orient = 5.0;   % strong horizontal enforcement

    J = w_pos*pos_cost + w_orient*orient_cost;
end