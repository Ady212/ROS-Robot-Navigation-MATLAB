% Task 5

 %% ROS Initializationu
    % Initialize ROS node (if not already initialized)
    try
        rosinit; % Connect to ROS master
    catch
        disp('ROS is already initialized.');
    end

    % Create ROS publisher
    pub = rospublisher('/mobile_base/commands/velocity', 'geometry_msgs/Twist');
    msg = rosmessage(pub);

    % Create subscriber for odometry to get robot's position
    odomSub = rossubscriber('/odom', 'nav_msgs/Odometry');

    % Subscribe to Livox LIDAR topic
lidarSub = rossubscriber('/scan', 'sensor_msgs/LaserScan');


   % Proportional control gains and thresholds
safe_distance = 0.5; % Minimum distance to an obstacle (meters)
distance_threshold = 0.03;
angle_threshold = 0.05;
max_linear_speed = 1;
max_angular_speed = 1;
k_linear = 0.5;
k_angular = 1.0;

load("waypoints_obstacle.mat");
% load('waypoints_obstacle2.mat');

for i = 1:size(waypoints, 1)
    target_x = waypoints(i, 1);
    target_y = waypoints(i, 2);
    plot(target_x, target_y, 'o', 'LineWidth', 2, 'Color', [1, 0, 0])
    reached = false;

    true_start = [2, 2, 2*pi]; % Adjust as needed
    % true_start = [8, 2, pi]; % Adjust as needed (b)

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

         % Get LiDAR scan data
        scan = receive(lidarSub);

        % Extract the distance measurements (ranges) from the scan
        % Each element in 'ranges' represents the distance from the LiDAR to an obstacle in meters
        ranges = scan.Ranges; 

        % Generate an array of angle values corresponding to each range measurement
        % 'angles' contains the angle (in radians) at which each distance measurement was taken
        angles = linspace(scan.AngleMin, scan.AngleMax, numel(ranges));

        ranges(find(ranges==0)) = inf;

        % Get readings from 0 to 90 degress
        right_indices = (angles >= 0) & (angles <= pi/2);
        right_ranges = ranges(right_indices);
        right_angles = angles(right_indices);
        
        % Get the minimun
        [min_right_ranges, min_right_index] = min(right_ranges);

        left_indices = (angles <= 0) & (angles >= -pi/2);
        left_ranges = ranges(left_indices);
        left_angles = angles(left_indices);
        
        % Get the minimun
        [min_left_ranges, min_left_index] = min(left_ranges);

        %Get the min left angle 
        min_left_Angles = left_angles(min_left_index);

        %Get the min right angle
        min_right_Angles = right_angles(min_right_index);

        if min_left_ranges < safe_distance
             msg.Linear.X = 0.2;
            msg.Angular.Z = cos(min_left_Angles);
            % send(pub, msg);
            disp('Turning left');
        end

         if min_right_ranges < safe_distance
             msg.Linear.X = 0.2;
            msg.Angular.Z = cos(min_right_Angles);
            % send(pub, msg);
            disp('Turning right');
        end

        send(pub, msg);
        pause(0.1);
    end
end

disp('Navigation complete.');