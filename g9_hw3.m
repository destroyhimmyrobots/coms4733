function g9_hw3(serPort)

 
    clc;                                                          % Clear the cache
    
    % Stop after not updating for 180 seconds
    NUM_SECONDS_UNTIL_STOPPING = 130;
    
    % Poll for bump Sensors to avoid getting NaN values when the 
    % robot first hits a wall
    [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
              BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    %=============================================================%

  
    RESOLUTION    = 3; % # segments in meter (1D)
    METERS_TO_MAP = 20;
    DIST_PER_SEGMENT = 1/RESOLUTION;
    START_SPIRAL_DIST = 5;
    SPIRAL_DIST = 5;
    
    secs_since_updated = 0;
    rightnow = clock;
    before = clock;
    
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
    
    spiraling = 0;
    turn_speed = 0;
    while 1 
        rightnow = clock;
        if etime(rightnow, before) > NUM_SECONDS_UNTIL_STOPPING
            return
        end
        
        %%
        %=============================================================%
        % Step 1 - Read Sensors                                       %
        %=============================================================%
        % For each loop, first read data from the robot sensor
        [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
              BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);     % Poll for Bump Sensors
        Wall = WallSensorReadRoomba(serPort);                         % Poll for Wall Sensor
        distance_temp = DistanceSensorRoomba(serPort);                % Poll for Distance delta
        angle_temp = AngleSensorRoomba(serPort);                     % Poll for Angle delta
        %=============================================================%
        
        %%
        %=============================================================%
        % Step 2 - Update Odometry                                    %
        %=============================================================%
        % Keep tracking the position and angle before the first hit
        current_angle = current_angle + angle_temp;               
        current_pos_x = current_pos_x + sin(current_angle) * distance_temp;
        current_pos_y = current_pos_y + cos(current_angle) * distance_temp;

        already_mapped = 0;
        x = round_to_nearest(DIST_PER_SEGMENT, current_pos_x);
        y = round_to_nearest(DIST_PER_SEGMENT, current_pos_y);

        x_offset = 0;
        y_offset = 0;
        if BumpFront
            x_offset = cos(current_angle);
            y_offset = sin(current_angle);
        elseif BumpLeft
            x_offset = cos(current_angle + pi/3);
            y_offset = sin(current_angle + pi/3);
        elseif BumpRight || (status==2) || (status==3)
            x_offset = cos(current_angle - pi/3);
            y_offset = sin(current_angle - pi/3);
        end
        if (BumpRight || BumpLeft || BumpFront || (status==2) || (status==3))
            fprintf('L: %d, R: %d, F: %d, x_offset: %d, y_offset: %d', BumpLeft, BumpRight, BumpFront, round(x_offset), round(y_offset));
            if (map(initial_map_pos(2) - round(x*RESOLUTION) - round(x_offset), initial_map_pos(1)+round(y*RESOLUTION) + round(y_offset)))
                already_mapped = 1;
                disp('old bump');
            else
                before = clock;
            end
            
            map(initial_map_pos(2) - round(x*RESOLUTION) - round(x_offset), initial_map_pos(1)+round(y*RESOLUTION) + round(y_offset)) = 1;
            figure(3), imagesc(map);
        end
        
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
                if (BumpRight || BumpLeft || BumpFront)
                    if ~already_mapped
                        status = 2; % Change Status to Wall Follow
                        spiraling = 0;
                    else
                        if BumpRight
                            spiraling = 1;
                        else
                            spiraling = -1; 
                        end
                        
                    end
                    first_hit_angle = 0;
                    first_hit_pos_x = 0;
                    first_hit_pos_y = 0;                    
                end
                turn_speed = turn_speed - 0.0001;
                if turn_speed <= 0
                    status = 4;
                end
                if etime(rightnow, before) > 20
                    spiraling = -1*spiraling;
                    status = 4;
                end
                SetFwdVelAngVelCreate(serPort, velocity_val, turn_speed*spiraling);
            case 2 % Wall Follow | Haven't left the threshold of the hit point
                display('#2');
                WallFollow(velocity_val, angular_velocity_val, BumpRight, BumpLeft, BumpFront, Wall, serPort);
                if (hit_distance > dist_from_first_hit_point)
                    status = 3;
                end
            case 3 % Wall Follow | Left the threshold of the hit point
                display('#3');
                WallFollow(velocity_val, angular_velocity_val, BumpRight, BumpLeft, BumpFront, Wall, serPort);
                rightnow = clock;
                if etime(rightnow, before) > 15
                    spiraling = -1;
                    status = 4;
                end
                if(hit_distance < dist_from_first_hit_point)
                   status = 4; 
                end
            case 4 % Go Back to Start Position 
                display('#4');
                %r = rand;
                %if r <= 0.5
                %    SetFwdVelAngVelCreate(serPort, velocity_val, r*10);
                %elseif r > 0.5
                %    SetFwdVelAngVelCreate(serPort, velocity_val, -r*10);
                %end
                turn_speed = 0.1 + rand/4;
                status = 1;
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


function rounded = round_to_nearest(res, val)
    rounded = round(val / res) * res;
end
