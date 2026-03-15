function [T_ee, joint_positions] = forward_kinematics(q, robot)
%FORWARD_KINEMATICS Computes FK using Modified DH parameters (mm units)

if numel(q) ~= 6
    error('Joint vector q must have 6 elements.');
end
q = q(:)';

% Base transform (WORLD → ROBOT BASE)
T = eye(4);
T(1:3,4) = robot.base(:);

joint_positions = zeros(7,3);
joint_positions(1,:) = robot.base(:)';

% Forward kinematics chain
for i = 1:6
    theta_i = q(i) + robot.theta_offset(i);
    A_i = dh_table(theta_i, robot.d(i), robot.a(i), robot.alpha(i));
    T = T * A_i;
    joint_positions(i+1,:) = T(1:3,4)';
end

T_ee = T;
end
