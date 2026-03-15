function [result] = pd_controller(traj, t, robot, options)
%PD_CONTROLLER Simple PD joint space controller for trajectory tracking

if nargin < 4
    options = struct();
end

% Controller gains
Kp = get_option(options, 'Kp', [50,50,50,30,30,30]);
Kd = get_option(options, 'Kd', [10,10,10,6,6,6]);

% Disturbance parameters
enable_disturbance = get_option(options, 'enable_disturbance', true);
disturbance_time   = get_option(options, 'disturbance_time', 3.0);
disturbance_mag    = get_option(options, 'disturbance_mag', 0.05);

n_points = length(t);
dt = t(2) - t(1);

% Desired trajectory
q_des  = traj.q;
qd_des = traj.qd;

q_actual  = zeros(n_points, 6);
qd_actual = zeros(n_points, 6);
control_effort = zeros(n_points, 6);
error_history  = zeros(n_points, 6);
error_norm     = zeros(n_points, 1);

disturbance_times = [];
disturbance_applied = zeros(n_points, 1);

% Start at initial desired position
q_actual(1,:)  = q_des(1,:);
qd_actual(1,:) = qd_des(1,:);

fprintf('Running PD Control Simulation...\n');
fprintf('  Kp=[%s], Kd=[%s]\n', num2str(Kp), num2str(Kd));
if enable_disturbance
    fprintf('  Disturbance enabled at t=%.2f s, magnitude=%.3f rad\n', ...
            disturbance_time, disturbance_mag);
else
    fprintf('  Disturbance disabled\n');
end

for k = 2:n_points
    q_curr  = q_actual(k-1,:)';
    qd_curr = qd_actual(k-1,:)';
    
    q_d  = q_des(k,:)';
    qd_d = qd_des(k,:)';
    
    % Disturbance injection
    if enable_disturbance && abs(t(k) - disturbance_time) < dt
        disturbance = disturbance_mag * randn(6,1);
        q_curr = q_curr + disturbance;
        disturbance_times = [disturbance_times; t(k)];
        disturbance_applied(k) = 1;
        fprintf('  *** Disturbance injected at t=%.2f s ***\n', t(k));
    end
    
    % PD control law: tau = Kp*error_pos + Kd*error_vel
    error_pos = q_d - q_curr;
    error_vel = qd_d - qd_curr;
    
    tau = (Kp .* error_pos' + Kd .* error_vel')';
    
    qdd_curr = tau;
    
    % Euler integration
    qd_new = qd_curr + qdd_curr * dt;
    q_new  = q_curr + qd_curr * dt + 0.5 * qdd_curr * dt^2;
    
    q_actual(k,:)  = q_new';
    qd_actual(k,:) = qd_new';
    control_effort(k,:) = tau';
    error_history(k,:)  = error_pos';
    error_norm(k)       = norm(error_pos);
end

% Compute statistics
max_error   = max(error_norm);
mean_error  = mean(error_norm);
final_error = error_norm(end);

threshold = 0.01*max_error;
settling_idx = find(error_norm > threshold, 1, 'last');
settling_time = isempty(settling_idx) * 0 + ~isempty(settling_idx) * t(settling_idx);

fprintf('Simulation complete.\n');
fprintf(' Max error: %.4f rad (%.2f deg)\n', max_error, rad2deg(max_error));
fprintf(' Mean error: %.4f rad (%.2f deg)\n', mean_error, rad2deg(mean_error));
fprintf(' Final error: %.4f rad (%.2f deg)\n', final_error, rad2deg(final_error));

% Package results
result.q_actual  = q_actual;
result.qd_actual = qd_actual;
result.q_desired = q_des;
result.error     = error_history;
result.error_norm= error_norm;
result.control_effort = control_effort;
result.disturbance_times = disturbance_times;
result.disturbance_applied = disturbance_applied;
result.t = t;

result.stats.max_error   = max_error;
result.stats.mean_error  = mean_error;
result.stats.final_error = final_error;
result.stats.settling_time = settling_time;

result.params.Kp = Kp;
result.params.Kd = Kd;

end

%%
function value = get_option(options, field, default)
    if isfield(options, field)
        value = options.(field);
    else
        value = default;
    end
end
