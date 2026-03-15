function J = ik_cost_ur10e(q, robot, target_W, q_ref, lambda)
% Cost function for UR10e soldering IK:
%   - Position accuracy
%   - Tool axis vertical (Z-down)
%   - Joint regularization for smoother solutions

q = q(:).';

[T_ee, joints_mm] = forward_kinematics(q, robot);
tcp_W = joints_mm(end,:);

e_pos = tcp_W - target_W;

z_tool = T_ee(1:3,3);
orient_err = 1 - abs(z_tool' * [0;0;-1]);

e_reg = q - q_ref;

% Final cost
J = ...
      (e_pos * e_pos.') ...
    + 1e6 * (orient_err^2) ...
    + lambda * (e_reg * e_reg.');
end
