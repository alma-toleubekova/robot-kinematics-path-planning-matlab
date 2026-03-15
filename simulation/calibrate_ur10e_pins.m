clear; clc;

% Load UR10e model (link lengths, offsets, base transform)
robot = parameters('UR10e');

% Analytical IK solutions used as initial guesses (deg → rad)
q_deg_old = [
    -62.21  -5.49  -110.83   26.32   90   0;
    -61.15  -3.09  -113.45   26.54   90   0;
    -60.89  -2.53  -114.03   26.56   90   0;
    -60.84  -5.92  -110.35   26.26   90   0
];
q0_all = deg2rad(q_deg_old);

% Pin coordinates in WORLD frame [Y X Z] (mm)
Zpin = 140;  
pins_W = [
    1002.775  368.297  Zpin;
    1029.191  368.297  Zpin;
    1035.287  368.297  Zpin;
    1002.775  386.839  Zpin
];

lambda = 1e-3;

q_new   = zeros(4,6);
tcp_new = zeros(4,3);

fprintf('=== Calibrating UR10e pin poses ===\n\n');

for i = 1:4
    target_W = pins_W(i,:);
    q0       = q0_all(i,:);

    % Numerical IK refinement
    q_sol = solve_ik_ur10e(robot, target_W, q0, lambda);
    q_new(i,:) = q_sol;

    % Forward kinematics check
    [~, joints_mm] = forward_kinematics(q_sol, robot);
    tcp_W = joints_mm(end,:);
    tcp_new(i,:) = tcp_W;

    fprintf('P%d:\n', i);
    fprintf('  q_deg  = [%.2f %.2f %.2f %.2f %.2f %.2f]\n', rad2deg(q_sol));
    fprintf('  TCP mm = [%.1f %.1f %.1f]\n', tcp_W);
    fprintf('  Error  = [%.1f %.1f %.1f] mm\n\n', tcp_W - target_W);
end

fprintf('Insert these q_deg values into positions.m (UR10e section).\n');
