function q_sol = solve_ik_ur10e(robot, target_W, q0, lambda)
% Numerical IK solver for the UR10e (position + vertical tool constraint)

if nargin < 4 || isempty(lambda)
    lambda = 1e-3;
end

cost = @(q) ik_cost_ur10e(q, robot, target_W, q0, lambda);

opts = optimset( ...
    'Display','off', ...
    'TolX',1e-6, 'TolFun',1e-8, ...
    'MaxFunEvals',4000, 'MaxIter',2000);

q_sol = fminsearch(cost, q0, opts);

% Wrap angles to [-pi, pi] for consistency
q_sol = atan2(sin(q_sol), cos(q_sol));
end
