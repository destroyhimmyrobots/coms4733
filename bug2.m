function [end_x, end_y, end_t] = bug2(serPort)
    clc;
    
    global WAIT_TIME; WAIT_TIME = 0.050;
    global DEBUG;     DEBUG = true;
    
    % Movement variables
    global max_v; max_v = 0.5;
    global v;     v     = 0.2;
    global w;     w     = 0.1; % v2w(v);
    
    % Loop control & vector indices
    finished = false;
    unreachable = false;
    qhp_index = 0;
    q_index = 1;
    
    % Define a point 10 meters away on the x-axis
    global q_goal; q_goal = [10, 0, 0.0];
    global d_tol;  d_tol  = 0.1;
    
    % Define points along the m-line so as to search throught them.
    m_line_x = (-10: d_tol / 2 :10)';
    m_line_y = zeros(length(m_line_x), 1);
    global m_line; m_line = [m_line_x, m_line_y zeros(length(m_line_x), 1)];
    
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
        pause(WAIT_TIME);
        
        % Update present distance
        q_now = update_dist_orient(serPort, q_past(q_index,:));
        
        % Read sensors and check for bumps
        % Read wall Sensor
        [bumped, ~, ~, ~] = bump_check(serPort);
        % wall = WallSensorReadRoomba(SerPort);
        
        % After moving, assess the current state
        if points_match(q_now, q_goal)
            if DEBUG
                fprintf('\nBUG2:\t\t%s\n', 'Point matched goal.');
            end
            finished = true;
            unreachable = false;
    
        elseif bumped
            if DEBUG
                fprintf('\nBUG2:\t\t%s\n', 'Hit the wall.');
            end
            qhp_index = qhp_index + 1;
            q_hit_points(qhp_index,:) = q_now;
            
            % Follow wall subject to Bug2 consraints.
            % NOTE: must compensate for overshooting the m_line.
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
        fprintf('\nBUG2:\t%s\n', 'Finished.');
    end
    
    if unreachable
        fprintf('\nBUG2:\t%s\n', 'Goal is unreachable.');
    else
        fprintf('\nError: %s\n', 'Main loop exited prematurely.');
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
        fprintf('UPDATE_DIST_ORIENT:\t[ x: %0.3g , y:%0.3g , t:%0.3g ]\n', q_now(1), q_now(2), q_now(3));
    end
end

function d = distance(p1, p2)
    global DEBUG;
    x = 1;
    y = 2;
    d = sqrt ( (p2(x) - p1(x))^2 + (p2(y) - p1(y))^2 );
    
    if DEBUG
        fprintf('DISTANCE:\t%0.3g\n', d);
    end    
end

function match = points_match(p1, p2)
    global DEBUG;
    global d_tol;
    
    % This tolerance value can be seen at work testing against the m-line.
    d = distance(p1, p2);
    
    if DEBUG
        fprintf('POINTS_MATCH:\t\t\t\t[ %0.3g , %0.3g ] ? [ %0.3g , %0.3g ]\n', p1(1), p1(2), p2(1), p2(2));
    end
    
    if (d <= d_tol)
        match = true;
    else
        match = false;
    end
end

function [new_pos, finished, unreachable] = wall_follow_handler(serPort, ...
        q_now, q_last_hit)

    global DEBUG;
    global WAIT_TIME;
    global q_goal;
    global d_tol;
    global m_line;
    global v;
    global w;
    
    hit_now  = zeros(1,3);
    hit_prev = zeros(1,3);
    q_prev   = q_now;
    
    dist_from_init_hit = 0.2;

    finished    = false;
    unreachable = false;
    status = 2;

    while (true)
        [~, R, L, F] = bump_check(serPort);
        wall         = WallSensorReadRoomba(serPort);

        % Update local distance values
        if DEBUG
            fprintf('\nWALL_FOLLOW_HANDLER:\tHIT_NOW:\t');
        end
        hit_now      = update_dist_orient(serPort, hit_prev);
        hit_dist     = distance(hit_now, hit_prev);
        hit_prev     = hit_now;
        pause(WAIT_TIME);

        % Update global distance values
        if DEBUG
            fprintf('\nWALL_FOLLOW_HANDLER:\tQ_NOW:\t ');
        end
        q_now  = update_dist_orient(serPort, q_prev);
        q_prev = q_now;
        pause(WAIT_TIME);

        switch status
            case 2 % Wall Follow | Haven't left the threshold of the hit point
                fprintf('WALL_FOLLOW_HANDER:\t\tCase 2\n');
                follow_wall(serPort, v, w, R, L, F, wall);
                if (hit_dist > dist_from_init_hit)
                    status = 3;
                end
            case 3 % Wall Follow | Left the threshold of the hit point
                fprintf('WALL_FOLLOW_HANDER:\t\tCase 3\n');
                follow_wall(serPort, v, w, R, L, F, wall);
                if(hit_dist < dist_from_init_hit)
                    status = 4;
                end
            case 4 % Go Back to Start Position
                fprintf('WALL_FOLLOW_HANDER:\t\tCase 4\n');
                turnAngle(serPort, w, hit_now(3));
                hit_now(3) = mod(hit_now(3), pi) + pi;
                if (pi * 0.9 < hit_now(3)) && (hit_now(3) < pi * 1.1)
                    SetFwdVelAngVelCreate(serPort, v, 0 );
                    status = 2;
                end
        end

        % --------------------------------------------------------------
        % Check if we have reached the goal before continuing to follow.
        % --------------------------------------------------------------
        if points_match(q_now, q_goal)
            if DEBUG
                fprintf('\nWALL_FOLLOW_HANDLER:\t%s\n', 'Point matched goal.');
            end
            finished = true;
            break;
        elseif (hit_dist > 1) ... % Hack to ensure we move before checking q_hit
                && points_match(q_now, q_last_hit)
            if DEBUG
                fprintf('\nWALL_FOLLOW_HANDLER:\t%s\n', 'Point matched last hit.');
            end
            finished = true;
            unreachable = true;
            break;
        else
            satisfactory_m_line_encounter = false;
            if DEBUG
                fprintf('\nWALL_FOLLOW_HANDLER:\t%s\n', 'Testing M-LINE');
            end
            idx = 1;
            % Reduce number of values tested by ignoring mline_x > q_now_x
            % !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            % Need to test that q_now is within (-10, 10) on x-axis!
            % !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            while m_line(idx,1) <= q_now(1) + d_tol
            % for i=1:length(m_line)
                if points_match(q_now, m_line(idx,:)) ...
                        && (distance(q_goal, m_line(idx,:)) < distance(q_last_hit, m_line(idx,:))) ...
                        && ~points_match(q_now, q_last_hit)
                    satisfactory_m_line_encounter = true;
                    break;
                end
                idx = idx + 1;
            end
            if satisfactory_m_line_encounter
                if DEBUG
                    fprintf('\nWALL_FOLLOW_HANDLER:\t%s\n', 'M-line re-encounter OK.');
                end
                break;
            else
                if DEBUG
                    fprintf('\nWALL_FOLLOW_HANDLER:\t%s\n', 'M-line not encountered.');
                end
            end
        end
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

function w = v2w(v)
    % Robot constants
    maxWheelVel = 0.5;   % Max linear velocity of each drive wheel (m/s)
    robotRadius = 0.2;   % Radius of the robot (m)
    
    % Max velocity combinations obey rule v+wr <= v_max
    w = (maxWheelVel-v)/robotRadius;
end
