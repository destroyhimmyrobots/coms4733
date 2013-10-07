function [end_x, end_y, end_t] = bug2(serPort)
    clc;
    
    global WAIT_TIME;
    WAIT_TIME = 0.1;
    global DEBUG;
    DEBUG = true;
    
    % Movement variables
    max_v = 0.5;
    v = 0.2;
    w = v2w(v);
    travel_dist = 0.2; % meters
    
    % Loop control & vector indices
    finished = false;
    unreachable = false;
    qhp_index = 0;
    q_index = 1;
    
    % Define a point 10 meters away on the x-axist
    global q_goal;
    q_goal = [10, 0, 0.0];
    
    % Define points along the m-line so as to search throught them.
    m_line_x = (-10:1/300:10)';
    m_line_y = m_line_x;
    global m_line;
    m_line   = [m_line_x, m_line_y zeros(length(m_line_x), 1)];
    
    % Define a vector of positions updated each time the robot moves.
    % This can, at the end, be used for the EXTRA CREDIT :3
    q_past = zeros(100,3);
    q_now  = zeros(1,3);
    
    % Initialize obstacle recollection vector:
    % Hitpoint 1: X1 Y1 Th1
    % Hitpoint 2: X2 Y2 Th2
    q_hit_points = zeros(100,3);
    
    while (true)       
        % Update array of prior positions
        q_past(q_index,:) = q_now;
        
        % Begin moving along the m-line
        SetFwdVelAngVelCreate(serPort, v, 0);
        
        % Update present distance
        q_now = update_dist_orient(serPort, q_past(q_index,:));
        
        % Read sensors and check for bumps
        % Read wall Sensor
        [bumped, ~, ~, ~] = bump_check(serPort);
        % wall = WallSensorReadRoomba(SerPort);
        
        % After moving, assess the current state
        if points_match(q_now, q_goal)
            if DEBUG
                fprintf('BUG2:\t\t%s\n', 'Point matched goal.');
            end
            finished = true;
            unreachable = false;
    
        elseif bumped
            if DEBUG
                fprintf('BUG2:\t\t%s\n', 'Hit the wall.');
            end
            qhp_index = qhp_index + 1;
            q_hit_points(qhp_index,:) = q_now;
            
            % Follow wall subject to Bug2 consraints.
            % NOTE: until_points must compensate for overshooting the m_line.
            [new_pos, finished, unreachable] = ...
                wall_follow_handler(serPort, q_now, q_hit_points(qhp_index,:));

            q_now = new_pos;
        end
        
        if finished
            break;
        elseif unreachable
            break;
        end

        % !!!!!!!!!!!!!!
        % Robot needs to be reoriented after doing wall-following
        % !!!!!!!!!!!!!!
        
        q_index = q_index + 1;
    end
    
    % Stop the robot.
    SetFwdVelAngVelCreate(serPort, 0, 0);
    
    % Turn to original orientation?
    
    if finished
        fprintf('%s\n', 'Reached goal!');
    elseif unreachable
        fprintf('Warning: %s\n', 'Goal is unreachable.');
    else
        fprintf('Error: %s\n', 'Loop exited prematurely.');
    end
    
    end_x = q_now(1);
    end_y = q_now(2);
    end_t = q_now(3);
    
    % Map robot's path for extra credit
    
end

function q_now = update_dist_orient(serPort, q_prev)    
    global DEBUG;
    
    % Update current position & orientation
    tmp_x = DistanceSensorRoomba(serPort);
    tmp_y = DistanceSensorRoomba(serPort);
    tmp_t = AngleSensorRoomba(serPort);
    q_now(1) = q_prev(1) + tmp_x*cos(tmp_t);
    q_now(2) = q_prev(2) + tmp_y*sin(tmp_t);
    q_now(3) = q_prev(3) + tmp_t;
    
    if DEBUG
        fprintf('UPDATE_DIST_ORIENT:\t[x: %0.3g, y:%0.3g, t:%0.3g]\n', q_now(1), q_now(2), q_now(3));
    end
end

function [new_pos, finished, unreachable] = wall_follow_handler(serPort, ...
        q_now, q_last_hit)

    global q_goal;
    global m_line;
    
    finished = false;    
    unreachable = false;
    
    status = 2;
    while (true)
        % Check if we have reached the goal before continuing to follow.
        if points_match(q_now, q_goal)
            finished = true;
            break;

        elseif points_match(q_now, q_last_hit)
            finished = true;
            unreachable = true;
            break;
        else
            satisfactory_m_line_encounter = false;
            
            for i=1:length(m_line)
                if points_match(q_now, m_line(i,:)) ...
                        && (distance(q_goal, m_line(i,:)) ...
                        < distance(q_last_hit, m_line(i,:))) ...
                        && ~points_match(q_now, q_last_hit)
                    
                    satisfactory_m_line_encounter = true;
                    break;
                end
            end
            if satisfactory_m_line_encounter
                break;
            end
        end

        [~, BumpRight, BumpLeft, BumpFront] = bump_check(serPort);
        wall = WallSensorReadRoomba(SerPort);        
        q_prev = q_now;
        
        switch status
            case 2 % Wall Follow | Haven't left the threshold of the hit point
                fprintf('WALL_FOLLOW_HANDER:\t\tCase 2\n');
                follow_wall(velocity_val, angular_velocity_val, ...
                    BumpRight, BumpLeft, BumpFront, wall, serPort);
                if (hit_distance > dist_from_first_hit_point)
                    status = 3;
                end
            case 3 % Wall Follow | Left the threshold of the hit point
                fprintf('WALL_FOLLOW_HANDER:\t\tCase 3\n');
                follow_wall(velocity_val, angular_velocity_val, ...
                    BumpRight, BumpLeft, BumpFront, wall, serPort);
                if(hit_distance < dist_from_first_hit_point)
                    status = 4;
                end
            case 4 % Go Back to Start Position
                fprintf('WALL_FOLLOW_HANDER:\t\tCase 4\n');
                turnAngle(serPort, angular_velocity_val, current_angle);
                current_angle = mod(current_angle, pi) + pi;
                if (pi * 0.9 < current_angle) && (current_angle < pi * 1.1)
                    SetFwdVelAngVelCreate(serPort, velocity_val, 0 );
                    %                    status = 5;
                    status = 2;
                end
        end
        
        % Update global distance values
        q_now = update_dist_orient(serPort, q_prev);
        
        %case 1 % Move Forward
        %fprintf('WALL_FOLLOW_HANDLER:\t\tMoving Forward\n');
        %SetFwdVelAngVelCreate(serPort, velocity_val, 0 );
        %[bumped, BumpRight, BumpLeft, bumpFront] = bump_check(serPort);
        %if bumped
        %status = 2; % Change Status to Wall Follow
        %first_hit_angle = 0;
        %first_hit_pos_x = 0;
        %first_hit_pos_y = 0;
        %end
        %case 5 % Stop and Orient at Start Position
        %if start_distance < dist_from_start_point
        %fprintf('Robot stopped to start point\n');
        %SetFwdVelAngVelCreate(serPort, 0, 0 );
        %fprintf('Turning to initial orientation\n');
        %turnAngle(serPort, angular_velocity_val, 180);
        %fprintf('Robot returned to start position\n');
        %return;
    end
    
    new_pos = q_now;
end

function follow_wall(serPort, fwd_vel, ang_vel, R, L, F, wall)
    % Angle Velocity for different bumps
    av_bumpright =  4 * ang_vel;
    av_bumpleft  =  2 * ang_vel;
    av_bumpfront =  3 * ang_vel;
    av_nowall    = -4 * ang_vel;

    if R || L || F
        v = 0;                              % Set Velocity to 0
    elseif ~wall
        v = 0.25 * fwd_vel;            % Set Velocity to 1/4 of the default
    else
        v = fwd_vel;                   % Set Velocity to the default
    end

    if R
    av = av_bumpright;                      % Set Angular Velocity to av_bumpright
    elseif L
        av = av_bumpleft;                   % Set Angular Velocity to av_bumpleft
    elseif F
        av = av_bumpfront;                  % Set Angular Velocity to av_bumpfront
    elseif ~wall
        av = av_nowall;                     % Set Angular Velocity to av_nowall
    else
        av = 0;                             % Set Angular Velocity to 0
    end
    SetFwdVelAngVelCreate(serPort, v, av );
end

function [yn, R, L, F] = bump_check(serPort)
    yn = false;
    [R, L, ~,~,~, F] = BumpsWheelDropsSensorsRoomba(serPort);
    
    if isnan(R) 
        R = 0; end
    if isnan(L) 
        L = 0; end
    if isnan(F) 
        F = 0; end

    if R || L || F
        yn = true; end
end

function match = points_match(p1, p2)
    global DEBUG;
    
    tol = 0.1;
    d = distance(p1, p2);
    
    if DEBUG
        fprintf('POINTS_MATCH:\tTesting [%0.3g, %0.3g]\t[%0.3g, %0.3g]', p1(1), p1(2), p2(1), p2(2));
    end
    
    if (d <= tol)
        match = true;
    else
        match = false;
    end
end

function d = distance(p1, p2)
    global DEBUG;
    x = 1;
    y = 2;
    d = sqrt ( (p2(x) - p1(x))^2 + (p2(y) - p1(y))^2 );
    
    if DEBUG
        fprintf('DISTANCE:\t[%0.3g, %0.3g]\n', d);
    end    
end

function w = v2w(v)
    % Robot constants
    maxWheelVel = 0.5;   % Max linear velocity of each drive wheel (m/s)
    robotRadius = 0.2;   % Radius of the robot (m)
    
    % Max velocity combinations obey rule v+wr <= v_max
    w = (maxWheelVel-v)/robotRadius;
end
