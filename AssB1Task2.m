% % Task 1
 %% ROS Initialization
    % Initialize ROS node (if not already initialized)
    try
        rosinit; % Connect to ROS master
    catch
        disp('ROS is already initialized.');
    end
    
    load('trajectory.mat'); % Load the predefined trajectory

    % Create ROS publisher
    pub = rospublisher('/mobile_base/commands/velocity', 'geometry_msgs/Twist');
    msg = rosmessage(pub);

    % Create subscriber for odometry to get robot's position
    odomSub = rossubscriber('/odom', 'nav_msgs/Odometry');

   % Proportional control gains and thresholds
distance_threshold = 0.5;
angle_threshold = 0.5;
max_linear_speed = 1;
max_angular_speed = 1;
k_linear = 0.5;
k_angular = 1.0;

load('trajectory.mat'); % Load the predefined trajectory
%waypoints = [4, 8; 10, 8; 10, 10; 8, 2];

for i = 1:size(waypoints, 1)
    target_x = waypoints(i, 1);
    target_y = waypoints(i, 2);
    plot(target_x, target_y, 'o', 'LineWidth', 2, 'Color', [1, 0, 0])
    reached = false;

    true_start = [2, 2, 2*pi]; % Adjust as needed

    while ~reached
        odom = receive(odomSub);
        pos = odom.Pose.Pose.Position;
        quat = odom.Pose.Pose.Orientation;

        % Position correction
        corrected_x = pos.X + true_start(1);
        corrected_y = pos.Y + true_start(2);

        % Orientation correction
        angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
        corrected_theta = mod(angles(1) + true_start(3), 2*pi);

        % Display pose
        disp(['Current Position: ', num2str(corrected_x), ', ', num2str(corrected_y), ...
              '; Orientation: ', num2str(corrected_theta * 180 / pi)]);

        % Target calculation
        dx = target_x - corrected_x;
        dy = target_y - corrected_y;
        distance_to_target = sqrt(dx^2 + dy^2);
        target_theta = atan2(dy, dx);
        angle_error = mod(target_theta - corrected_theta + pi, 2*pi) - pi;

        if distance_to_target < distance_threshold
            msg.Linear.X = 0;
            msg.Angular.Z = 0;
            send(pub, msg);
            disp('Target reached!');
            break;
        end

        % Proportional control logic
        if abs(angle_error) > angle_threshold
            msg.Linear.X = 0;
            msg.Angular.Z = sign(angle_error) * min(abs(k_angular * angle_error), max_angular_speed);
        else
            msg.Linear.X = min(k_linear * distance_to_target, max_linear_speed);
            msg.Angular.Z = sign(angle_error) * min(abs(k_angular * angle_error), max_angular_speed);
        end

        send(pub, msg);
        pause(0.1);
    end
end

disp('Navigation complete.');