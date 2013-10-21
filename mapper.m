function mapper(serPort)

    %=============================================================%
    % Description                                                 %
    %=============================================================%
    % This is a simple solution for Homework 1.                   %
    % The robot moves forward till it bumps on an object (wall).  %
    % The robot follows the object around till it reaches the     %
    % first bump position and then returns back to its initial    %
    % starting point with the same orientation.                   %
    %=============================================================%
    
    %%
    %=============================================================%
    % Clear cache & avoid NaN values                              %
    %=============================================================%
    clc;                                                          % Clear the cache
    
    % Poll for bump Sensors to avoid getting NaN values when the 
    % robot first hits a wall
    [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
              BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    %=============================================================%

    %%
    %=============================================================%
    % Variable Declaration                                        %
    %=============================================================%
    RESOLUTION    = 5; % # segments in meter (1D)
    METERS_TO_MAP = 7;
    DIST_PER_SEGMENT = 1/RESOLUTION;
    START_SPIRAL_DIST = 5;
    SPIRAL_DIST = 5;
    
    WAIT_TIME = 0.033;
    v = 0.4;
    w = 0.4;
    
    % must be odd so that you start within one segment
    width = RESOLUTION * METERS_TO_MAP;
    map = zeros(width,width);
    
    % start in the middle
    initial_map_pos = [ceil(width/2), ceil(width/2)];
    
    moving = 1;
    
    pos = [0 0 0];
    stuck = 0;
    dist = 0;
    %%
    %=======================%
    % Position Declaration  %
    %=======================%
    
    % Current Position
    current_pos_x = 0;
    current_pos_y = 0;
    current_angle = 0;
    
    % First Hit Position
    first_hit_pos_x = 0;
    first_hit_pos_y = 0;
    first_hit_angle = 0;
    
    %%
    %=======================%
    % Velocity Declaration  %
    %=======================%
    velocity_val = 0.2;
    angular_velocity_val = 0.1;
    %=======================%

    %=======================%
    % Distance Thresholds   %
    %=======================%
    dist_from_start_point = 0.3;
    dist_from_first_hit_point = 0.2;
    %=======================%
   
    %=======================%
    % Status Variable       %
    %=======================%
    status = 1;             % 1 -> Move Forward, 
                            % 2 -> Wall Follow | Haven't left the threshold of the hit point
                            % 3 -> Wall Follow | Left the threshold of the hit point
                            % 4 -> Go Back to Start Position  
                            % 5 -> Stop and Orient at Start Position
    %=============================================================%
    
    %% Main Loop
    while 1 
        
        %%
        %=============================================================%
        % Step 1 - Read Sensors                                       %
        %=============================================================%
        % For each loop, first read data from the robot sensor
        [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
              BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);     % Poll for Bump Sensors
        Wall = WallSensorReadRoomba(serPort);                         % Poll for Wall Sensor
        distance_temp = DistanceSensorRoomba(serPort);                % Poll for Distance delta
        angle_temp = AngleSensorRoomba(serPort);                      % Poll for Angle delta
        %=============================================================%
        
        %%
        %=============================================================%
        % Step 2 - Update Odometry                                    %
        %=============================================================%
        % Keep tracking the position and angle before the first hit
        current_angle = current_angle + angle_temp;               
        current_pos_x = current_pos_x + sin(current_angle) * distance_temp;
        current_pos_y = current_pos_y + cos(current_angle) * distance_temp;


        x = round_to_nearest(DIST_PER_SEGMENT, current_pos_x);
        y = round_to_nearest(DIST_PER_SEGMENT, current_pos_y);

        x_offset = 0;
        y_offset = 0;
        if BumpFront
            x_offset = cos(pos(3));
            y_offset = sin(pos(3));
        elseif BumpLeft
            x_offset = cos(pos(3) +  current_angle);
            y_offset = sin(pos(3) +  current_angle);
        elseif BumpRight
            x_offset = cos(pos(3) -  current_angle);
            y_offset = sin(pos(3) -  current_angle);
        end
        fprintf('L: %d, R: %d, F: %d, x_offset: %d, y_offset: %d', BumpLeft, BumpRight, BumpFront, round(x_offset), round(y_offset));
        map(initial_map_pos(2) - round(y*RESOLUTION) - round(y_offset), initial_map_pos(1)+round(x*RESOLUTION) + round(x_offset)) = 1;
        figure(3), imagesc(map);
        
        
        % Keep tracking the position and angle after the first hit
        first_hit_angle = first_hit_angle + angle_temp;
        first_hit_pos_x = first_hit_pos_x + sin(first_hit_angle) * distance_temp;
        first_hit_pos_y = first_hit_pos_y + cos(first_hit_angle) * distance_temp;
        
        drawnow;
        %=============================================================%
        
        %%
        %=============================================================%
        % Step 3 - Calculate Euclidean Distances                      %
        %=============================================================%
        start_distance = sqrt(current_pos_x ^ 2 + current_pos_y ^ 2);
        hit_distance = sqrt(first_hit_pos_x ^ 2 + first_hit_pos_y ^ 2);
        %=============================================================%
        
        %%
        %=============================================================%
        % Step 4 - Check Status                                       %
        %=============================================================%
        switch status
            case 1 % Move Forward
                display('Moving Forward');
                SetFwdVelAngVelCreate(serPort, velocity_val, 0 );
                if (BumpRight || BumpLeft || BumpFront)
                    status = 2; % Change Status to Wall Follow
                    first_hit_angle = 0;
                    first_hit_pos_x = 0;
                    first_hit_pos_y = 0;                    
                end
            case 2 % Wall Follow | Haven't left the threshold of the hit point
                WallFollow(velocity_val, angular_velocity_val, BumpRight, BumpLeft, BumpFront, Wall, serPort);
                if (hit_distance > dist_from_first_hit_point)
                    status = 3;
                end
            case 3 % Wall Follow | Left the threshold of the hit point
                WallFollow(velocity_val, angular_velocity_val, BumpRight, BumpLeft, BumpFront, Wall, serPort);
                if(hit_distance < dist_from_first_hit_point)
                   status = 4; 
                end
            case 4 % Go Back to Start Position                
                turnAngle(serPort, angular_velocity_val, current_angle);
                current_angle = mod(current_angle, pi) + pi;
                if (pi * 0.9 < current_angle) && (current_angle < pi * 1.1)
                    SetFwdVelAngVelCreate(serPort, velocity_val, 0 );
                    status = 5;
                end
            case 5 % Stop and Orient at Start Position
                if start_distance < dist_from_start_point
                    fprintf('Robot stopped to start point\n');
                    SetFwdVelAngVelCreate(serPort, 0, 0 );
                    fprintf('Turning to initial orientation\n');
                    turnAngle(serPort, angular_velocity_val, 180);
                    fprintf('Robot returned to start position\n');
                    return;
                end
        end
    end
end

%%
% Wall Follow Function
function WallFollow(velocity_val, angular_velocity_val, BumpRight, BumpLeft, BumpFront, Wall, serPort)

    % Angle Velocity for different bumps
    av_bumpright =  4 * angular_velocity_val;
    av_bumpleft  =  2 * angular_velocity_val;
    av_bumpfront =  3 * angular_velocity_val;
    av_nowall    = -4 * angular_velocity_val;
    
    if BumpRight || BumpLeft || BumpFront
        v = 0;                              % Set Velocity to 0
    elseif ~Wall
        v = 0.25 * velocity_val;            % Set Velocity to 1/4 of the default
    else
        v = velocity_val;                   % Set Velocity to the default
    end

    if BumpRight
    av = av_bumpright;                      % Set Angular Velocity to av_bumpright
    elseif BumpLeft
        av = av_bumpleft;                   % Set Angular Velocity to av_bumpleft
    elseif BumpFront
        av = av_bumpfront;                  % Set Angular Velocity to av_bumpfront
    elseif ~Wall
        av = av_nowall;                     % Set Angular Velocity to av_nowall
    else
        av = 0;                             % Set Angular Velocity to 0
    end
    SetFwdVelAngVelCreate(serPort, v, av );
end

function q_now = update_dist_orient(serPort, q_prev, turned_in_deg)    
   
    
    % Update current position & orientation
    tmp_dist = DistanceSensorRoomba(serPort);
    %pause(WAIT_TIME);
    
    tmp_t= 0;
    while abs(tmp_t) < (pi/180*turned_in_deg)
        turnAngle(serPort, 0.4, turned_in_deg/4);
        tmp_t= tmp_t+AngleSensorRoomba(serPort)
        pause(0.1)
    end
    %tmp_t = AngleSensorRoomba(serPort);
    
   % pause(WAIT_TIME);
    
    q_now(3) = q_prev(3) + round_to_nearest(pi/4, tmp_t);
    q_now(1) = q_prev(1) + tmp_dist*cos(q_now(3));
    q_now(2) = q_prev(2) + tmp_dist*sin(q_now(3));
    q_now
  
end

function rounded = round_to_nearest(res, val)
    rounded = round(val / res) * res;
end

function [bumped, R, L, F] = bump_check(serPort)
    bumped = false;
    [R, L, ~,~,~, F] = BumpsWheelDropsSensorsRoomba(serPort);
    
    if isnan(R) 
        R = 0;
    end
    if isnan(L) 
        L = 0;
    end
    if isnan(F) 
        F = 0;
    end

    if R || L || F
        bumped = true;
    end
end

function w = v2w(v)
    % Robot constants
    maxWheelVel = 0.5;   % Max linear velocity of each drive wheel (m/s)
    robotRadius = 0.2;   % Radius of the robot (m)
    
    % Max velocity combinations obey rule v+wr <= v_max
    w = (maxWheelVel-v)/robotRadius;
end
