function config = positions(robot_type, position_name)

robot_type    = upper(robot_type);
position_name = upper(position_name);

switch robot_type
    case 'UR3'
        config = ur3_positions(position_name);
    case 'UR10E'
        config = ur10e_positions(position_name);
    otherwise
        error('Unknown robot type.');
end
end

%%
function config = ur3_positions(pos_name)

robot = parameters('UR3');   % uses your current DH + base (mm)

% --- Bench / holder geometry in WORLD frame (mm, [Y X Z]) ---
UR3_base_W = [370, 400, 0];            % from parameters (mm)

pos3_W = [505, 945, 150];              % POS3 centre (Y, X, Z) mm
pos4_W = [1055, 405, 150];             % POS4 centre (Y, X, Z) mm
                                       % 150 mm ≈ holder height + pcb thickness

switch upper(pos_name)

    case 'HOME'
        % Upright looking roughly towards POS4 (same as you used)
        config.q     = zeros(1,6);
        config.q_deg = zeros(1,6);
        config.target      = [];
        config.description = 'UR3 Home Position (upright)';

    case 'POS3'
        % --------- Numeric IK to reach PCB at POS3 ----------
        target_W = pos3_W;              % already in WORLD coordinates

        % Initial guess: something "reasonable" over POS3
        q0_deg = [10, 40, 80, 50, 90, 0];
        q0     = deg2rad(q0_deg);

        q_sol  = solve_ik_ur3(robot, target_W, q0);

        config.q     = q_sol(:).';
        config.q_deg = rad2deg(config.q);
        config.target      = target_W;
        config.description = 'UR3 Pick-up Position (PCB at POS3)';

    case 'POS4'
        % --------- Numeric IK to place PCB at POS4 ----------
        target_W = pos4_W;

        q0_deg = [80, 60, 30, 80, 80, 0];   % start from a "forward" pose
        q0     = deg2rad(q0_deg);

        q_sol  = solve_ik_ur3(robot, target_W, q0);

        config.q     = q_sol(:).';
        config.q_deg = rad2deg(config.q);
        config.target      = target_W;
        config.description = 'UR3 Place Position (PCB at POS4)';

    otherwise
        error('Unknown UR3 position: %s. Valid: Home, Pos3, Pos4', pos_name);
end
end



%%
function config = ur10e_positions(pos_name)
%UR10E_POSITIONS Predefined joint angles & targets for UR10e soldering

    % === VISUAL CALIBRATION SHIFTS ===
    % These only affect where we *aim* in the world, not the joint angles.
    Y_SHIFT =  25;   % mm  (toward UR10e base as tuned by you)
    Z_SHIFT =  150;   % mm  (lift above 140 mm so it doesn't solder the floor)

    switch upper(pos_name)

        case 'HOME'
            config.q_deg = [0, 0, 0, 0, 0, 0];
            config.q     = deg2rad(config.q_deg);
            config.target = [];
            config.description = 'UR10e Home';

        case 'P1'
            % Your IK solution for pin 1 (in degrees)
            config.q_deg = [-62.2084008671, -5.4850116729, -110.8300949769, ...
                             26.3151066498,  90.0,           0.0];
            config.q = deg2rad(config.q_deg);

            base_target = [1002.775, 368.297, 140];    % original [Y X Z] mm
            base_target(1) = base_target(1) + Y_SHIFT; % small Y tweak
            base_target(3) = base_target(3) + Z_SHIFT; % lift above PCB
            config.target = base_target;

            config.description = 'UR10e Soldering Point 1';

        case 'P2'
            config.q_deg = [-61.1485204833, -3.0863661353, -113.4451913150, ...
                             26.5397316125,  90.0,           0.0];
            config.q = deg2rad(config.q_deg);

            base_target = [1029.191, 368.297, 140];
            base_target(1) = base_target(1) + Y_SHIFT;
            base_target(3) = base_target(3) + Z_SHIFT;
            config.target = base_target;

            config.description = 'UR10e Soldering Point 2';

        case 'P3'
            config.q_deg = [-60.8925859854, -2.5324038307, -114.0297718635, ...
                             26.5621756942,  90.0,           0.0];
            config.q = deg2rad(config.q_deg);

            base_target = [1035.287, 368.297, 140];
            base_target(1) = base_target(1) + Y_SHIFT;
            base_target(3) = base_target(3) + Z_SHIFT;
            config.target = base_target;

            config.description = 'UR10e Soldering Point 3';

        case 'P4'
            config.q_deg = [-60.8426031727, -5.9155280924, -110.3484554073, ...
                             26.2639834997,  90.0,           0.0];
            config.q = deg2rad(config.q_deg);

            base_target = [1002.775, 386.839, 140];
            base_target(1) = base_target(1) + Y_SHIFT;
            base_target(3) = base_target(3) + Z_SHIFT;
            config.target = base_target;

            config.description = 'UR10e Soldering Point 4';

        otherwise
            error('Unknown UR10e position: %s', pos_name);
    end
end



%% ------------------------------------------------------------------------
function q_sol = solve_ik_ur3(robot, target_W, q0)
% Numeric IK: minimise position error between TCP and target in WORLD frame
% target_W is [Y X Z] in mm

    % Simple cost: position error of end-effector
    cost = @(q) ik_cost(q, robot, target_W);

    opts = optimset('Display','off', ...
                    'TolX',1e-5, 'TolFun',1e-6, ...
                    'MaxFunEvals',2000, 'MaxIter',1000);

    % Use fminsearch (built-in MATLAB, no toolboxes)
    q_sol = fminsearch(cost, q0, opts);

    % Wrap angles to [-pi, pi] for neatness
    q_sol = atan2(sin(q_sol), cos(q_sol));
end

function J = ik_cost(q, robot, target_W)
    % q: 1x6
    q = q(:).';

    % Forward kinematics with your model
    [~, joints_mm] = forward_kinematics(q, robot);
    tcp_W = joints_mm(end,:);   % [Y X Z] in mm, WORLD frame (base already added)

    pos_err = tcp_W - target_W;
    J = pos_err*pos_err.';      % squared norm
end


