function robot = parameters(robot_type)
%PARAMETERS Return DH parameters and configuration for UR3 or UR10e

robot_type = upper(robot_type);

switch robot_type
    case 'UR3'
        robot = define_ur3();
    case 'UR10E'
        robot = define_ur10e();
    otherwise
        error('Unknown robot type. Use UR3 or UR10e.');
end
end

%% ------------------------------------------------------------------------
function robot = define_ur3()
robot.name = 'UR3';

robot.alpha = [0 -pi/2 0 0 pi/2 -pi/2];
robot.a     = [0 0 244 213 0 0];
robot.d     = [152 0 0 112 85 234];
robot.theta_offset = [0 -pi/2 0 pi/2 0 0];

robot.base = [370 400 0];  % WORLD frame (mm)

robot.joint_limits = repmat([-2*pi 2*pi],6,1);
robot.max_velocity = [pi pi pi 2*pi 2*pi 2*pi];
robot.max_acceleration = [4*pi 4*pi 4*pi 8*pi 8*pi 8*pi];

robot.payload = 3.0;
robot.reach   = 500;
end

%% ------------------------------------------------------------------------
function robot = define_ur10e()
robot.name = 'UR10e';

robot.alpha = [0 pi/2 0 0 -pi/2 pi/2];
robot.a     = [0 0 612.7 571.6 0 0];
robot.d     = [180 0 0 174.1 119.8 396.5];
robot.theta_offset = [0 pi/2 0 -pi/2 0 0];

robot.base = [1695 200 0];  % WORLD frame (mm)

robot.joint_limits = repmat([-2*pi 2*pi],6,1);
robot.max_velocity = [2*pi/3 2*pi/3 pi pi pi pi];
robot.max_acceleration = [2.5 2.5 5 5 5 5];

robot.payload = 12.5;
robot.reach   = 1300;
end
