clear; clc; close all;
%% MAIN

ur3   = parameters('UR3');
ur10e = parameters('UR10e');

% Trajectory planning
fprintf('Trajectory planning...\n');

[traj_ur3,  t_ur3]  = trajectory_planning('UR3',  'pick_place');
[traj_ur10e, t_ur10e] = trajectory_planning('UR10e','soldering');

% Animation
fprintf('\nRunning animated simulation...\n');
workbench_simulation('animate', ur3, traj_ur3, ur10e, traj_ur10e, t_ur3, t_ur10e);

% Joint space trajectories
figure('Name','Joint Space Trajectories');

subplot(2,3,1);
plot(t_ur3, rad2deg(traj_ur3.q(:,1:3)));
grid on; title('UR3 Joint Positions'); xlabel('Time (s)'); ylabel('Deg');

subplot(2,3,2);
plot(t_ur3, rad2deg(traj_ur3.qd(:,1:3)));
grid on; title('UR3 Joint Velocities'); xlabel('Time (s)');

subplot(2,3,3);
plot(t_ur3, rad2deg(traj_ur3.qdd(:,1:3)));
grid on; title('UR3 Joint Accelerations'); xlabel('Time (s)');

subplot(2,3,4);
plot(t_ur10e, rad2deg(traj_ur10e.q(:,1:3)));
grid on; title('UR10e Joint Positions'); xlabel('Time (s)');

subplot(2,3,5);
plot(t_ur10e, rad2deg(traj_ur10e.qd(:,1:3)));
grid on; title('UR10e Joint Velocities'); xlabel('Time (s)');

subplot(2,3,6);
plot(t_ur10e, rad2deg(traj_ur10e.qdd(:,1:3)));
grid on; title('UR10e Joint Accelerations'); xlabel('Time (s)');

% PD controller
options_normal.enable_disturbance = false;
options_error.enable_disturbance  = true;

options_error.disturbance_time = 2.5;
options_error.disturbance_mag  = 0.1;

result_ur3_normal = pd_controller(traj_ur3, t_ur3, ur3, options_normal);
result_ur3_error  = pd_controller(traj_ur3, t_ur3, ur3, options_error);

options_error.disturbance_time = 3.0;
options_error.disturbance_mag  = 0.08;

result_ur10e_normal = pd_controller(traj_ur10e, t_ur10e, ur10e, options_normal);
result_ur10e_error  = pd_controller(traj_ur10e, t_ur10e, ur10e, options_error);

% Controller error handling
figure('Name','Controller Error Handling');

subplot(2,2,1);
plot(t_ur3, rad2deg(result_ur3_normal.error_norm),'b','LineWidth',1.5); hold on;
plot(t_ur3, rad2deg(result_ur3_error.error_norm),'r','LineWidth',1.5);
xline(2.5,'k--');
grid on; title('UR3 Tracking Error'); legend('Normal','With Disturbance');

subplot(2,2,2);
plot(t_ur3, result_ur3_error.control_effort(:,1:3),'LineWidth',1.2);
xline(2.5,'k--');
grid on; title('UR3 Control Effort');

subplot(2,2,3);
plot(t_ur10e, rad2deg(result_ur10e_normal.error_norm),'b','LineWidth',1.5); hold on;
plot(t_ur10e, rad2deg(result_ur10e_error.error_norm),'r','LineWidth',1.5);
xline(3.0,'k--');
grid on; title('UR10e Tracking Error'); legend('Normal','With Disturbance');

subplot(2,2,4);
plot(t_ur10e, result_ur10e_error.control_effort(:,1:3),'LineWidth',1.2);
xline(3.0,'k--');
grid on; title('UR10e Control Effort');

% Error recovery
figure('Name','Detailed Error Recovery');

t_zoom = t_ur3 >= 2.0 & t_ur3 <= 4.5;
subplot(1,2,1);
plot(t_ur3(t_zoom), rad2deg(result_ur3_error.q_desired(t_zoom,2)),'b','LineWidth',2); hold on;
plot(t_ur3(t_zoom), rad2deg(result_ur3_error.q_actual(t_zoom,2)),'r--','LineWidth',2);
xline(2.5,'k--');
grid on; title('UR3 Joint 2 Recovery'); legend('Desired','Actual');

t_zoom = t_ur10e >= 2.5 & t_ur10e <= 5.0;
subplot(1,2,2);
plot(t_ur10e(t_zoom), rad2deg(result_ur10e_error.q_desired(t_zoom,2)),'b','LineWidth',2); hold on;
plot(t_ur10e(t_zoom), rad2deg(result_ur10e_error.q_actual(t_zoom,2)),'r--','LineWidth',2);
xline(3.0,'k--');
grid on; title('UR10e Joint 2 Recovery'); legend('Desired','Actual');

% Singularity analysis
singularity_ur3   = analyze_singularity(traj_ur3.q, ur3);
singularity_ur10e = analyze_singularity(traj_ur10e.q, ur10e);

figure('Name','Singularity Analysis');

subplot(1,2,1);
plot(t_ur3, singularity_ur3.manipulability,'LineWidth',1.5);
yline(singularity_ur3.threshold,'r--');
grid on; title('UR3 Manipulability');

subplot(1,2,2);
plot(t_ur10e, singularity_ur10e.manipulability,'LineWidth',1.5);
yline(singularity_ur10e.threshold,'r--');
grid on; title('UR10e Manipulability');

fprintf('\nSimulation completed successfully.\n');

%% Helper functions
function result = analyze_singularity(q_traj, robot)
    n = size(q_traj,1);
    manipulability = zeros(n,1);

    for i = 1:n
        J = compute_jacobian(q_traj(i,:), robot);
        manipulability(i) = sqrt(abs(det(J*J')));
    end

    threshold = 0.001;

    result.manipulability = manipulability;
    result.min_manip = min(manipulability);
    result.max_manip = max(manipulability);
    result.threshold = threshold;
    result.near_singular = manipulability < threshold;
end

function J = compute_jacobian(q, robot)
    delta = 1e-6;
    n = length(q);

    [T0, ~] = forward_kinematics(q, robot);
    p0 = T0(1:3, 4);

    J = zeros(3, n); 

    for i = 1:n
        q_plus = q;
        q_plus(i) = q_plus(i) + delta;

        [T_plus, ~] = forward_kinematics(q_plus, robot);
        p_plus = T_plus(1:3, 4);

        J(:, i) = (p_plus - p0) / delta;
    end
end
