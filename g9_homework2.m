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
    goal_dist = 10;
    global q_goal; q_goal = [goal_dist, 0, 0.0];
    global d_tol;  d_tol  = 0.09;
    global a_tol;  a_tol  = 0.0001;
    global angle; angle = 0;
    
    % Define points along the m-line so as to search throught them.
    m_line_x = (-10: d_tol / 2 :10)';
    m_line_y = zeros(length(m_line_x), 1);
    global m_line; m_line = [m_line_x, m_line_y zeros(length(m_line_x), 1)];
    
    % Define a vector of positions updated each time the robot moves.
    % This can, at the end, be used for the EXTRA CREDIT 
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
        
        % After moving, assess the current state
        if q_now(1) >= q_goal(1)
            if DEBUG
                fprintf('\nBUG2:\n\t%s\n', 'Point matched goal.');
            end
            finished = true;
            unreachable = false;
    
        elseif bumped
            if DEBUG
                fprintf('\nBUG2:\n\t%s\n', 'Hit the wall.');
            end
            qhp_index = qhp_index + 1;
            q_hit_points(qhp_index,:) = q_now;
            
            % Follow wall subject to Bug2 consraints.
            % NOTE: must compensate for overshooting the m_line.
            [new_pos, q_past, finished, unreachable] = ...
                wall_follow_handler(serPort, q_now, q_past, q_hit_points(qhp_index,:));
            
            q_index = length(q_past);
            
            % Turn the robot back along the m-line (pi/2)
            if DEBUG
                fprintf('\n');
                fprintf('BUG2:\n%s:\t[%0.3g , %0.3g, %0.3g]\n', ...
                    'Position after follow:', new_pos(1), new_pos(2), new_pos(3));
            end
            
            %Convert rad->deg for turnAngle
            %Why must we add 45 deg? This could change if wall is not
            %straight.
            
            % ---> According to Piazza, this is a bug in the turnangle function.
            turnAngle(serPort, 0.2, -convert2deg(new_pos(3)) + 45 );
            new_pos = update_dist_orient(serPort, new_pos);
            
            % Set theta = 0 because turnAngle does not update the angle
            % sensor.
            q_now   = new_pos; q_now(3) = 0; q_now(2) = 0;

        end
        
        if finished
            break;
        elseif unreachable
            break;
        end

        q_index = q_index + 1;
        fprintf('\n');
    end
    
    % Stop the robot.
    SetFwdVelAngVelCreate(serPort, 0, 0);
    
    % Turn to original orientation?
    if finished
        fprintf('\nBUG2:\t%s\n', 'Finished.');
    end

    if unreachable
        fprintf('\nBUG2:\t%s\n', 'Goal is unreachable.');
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % GO GO EXTRA CREDIT GOGOGOGOGOGOGOGOGOGO
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    draw_path(q_past, q_now, q_hit_points);
    
    end_x = q_now(1);
    end_y = q_now(2);
    end_t = q_now(3);
    
end

function draw_path(prev, now, hits)
    figure(1);
    hold on;
    xlabel('Robot X coordinate');
    ylabel('Robot Y coordinate');

    plot(prev(:,1), prev(:,2), 'b.');
    plot(now(:,1), now(:,2), 'gs');
    plot(hits(:,1), hits(:,2), 'ro');
    
    composite = [prev; now; hits];
    plot(composite(:,1), composite(:,2), 'k-');
    legend('Followed points', 'Finishing point', 'Collision points');
    hold off;
end

function q_now = update_dist_orient(serPort, q_prev)    
    global DEBUG;
    global WAIT_TIME;
    global angle;
    
    % Update current position & orientation
    tmp_dist = DistanceSensorRoomba(serPort);
    pause(WAIT_TIME);
    tmp_t = AngleSensorRoomba(serPort);
    angle = angle + tmp_t;
    
    pause(WAIT_TIME);
    
    q_now(3) = q_prev(3) + tmp_t;
    q_now(1) = q_prev(1) + tmp_dist*cos(q_now(3));
    q_now(2) = q_prev(2) + tmp_dist*sin(q_now(3));

    if DEBUG
        fprintf('UPDATE_DIST_ORIENT:\t[x:%0.3g , y:%0.3g , t:%0.3g]\t angle:%0.3g\n', q_now(1), q_now(2), q_now(3), angle);
        
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

function [new_pos, q_prev, finished, unreachable] = wall_follow_handler(serPort, ...
        q_now, q_prev, q_last_hit)

    global DEBUG;
    global q_goal;
    global d_tol;
    global v;
    global w;
    
    hit_now  = zeros(1,3);
    hit_prev = zeros(1,3);
    q_prev_idx = length(q_prev);
    
    dist_from_init_hit = 0.2;

    finished    = false;
    unreachable = false;
    status = 2;

    while (true)      
        [~, R, L, F] = bump_check(serPort);
        wall         = WallSensorReadRoomba(serPort);

        switch status
            case 2 % Wall Follow | Haven't left the threshold of the hit point
                fprintf('WALL_FOLLOW_HANDER:\nCase 2\n');
                going_straight = follow_wall(serPort, v, w, R, L, F, wall);
                if (norm(hit_now(1:2), 2) > dist_from_init_hit)
                    status = 3;
                end
            case 3 % Wall Follow | Left the threshold of the hit point
                fprintf('WALL_FOLLOW_HANDER:\nCase 3\n');
                going_straight = follow_wall(serPort, v, w, R, L, F, wall);
                if(norm(hit_now(1:2), 2) < dist_from_init_hit)
                    status = 4;
                end
            case 4 % Go Back to Start Position
                fprintf('WALL_FOLLOW_HANDER:\nCase 4\n');
                turnAngle(serPort, w, hit_now(3));
                hit_now(3) = mod(hit_now(3), pi) + pi;
                if (pi * 0.9 < hit_now(3)) && (hit_now(3) < pi * 1.1)
                    SetFwdVelAngVelCreate(serPort, v, 0 );
                    status = 2;
                end
        end
        
        % Update local distance values
        if DEBUG
            fprintf('\nWALL_FOLLOW_HANDLER:\nHIT_NOW:\t');
        end
        hit_now      = update_dist_orient(serPort, hit_prev);
        hit_dist     = distance(hit_now, hit_prev);
        hit_prev     = hit_now;

        % Update global distance values
        if DEBUG
            fprintf('\nWALL_FOLLOW_HANDLER:\nQ_NOW:\t\t');
        end
        if going_straight
            if q_prev(q_prev_idx,3) >= 0
                q_prev(q_prev_idx,3) = (floor(q_prev(q_prev_idx,3) * 10) / 10.0);% Set Angular Velocity to 0
            else
                q_prev(q_prev_idx,3) = (ceil(q_prev(q_prev_idx,3) * 10) / 10.0);% Set Angular Velocity to 0
            end
        end
        q_now  = update_dist_orient(serPort, q_prev(q_prev_idx,:));
        q_prev_idx = q_prev_idx + 1;
        q_prev(q_prev_idx,:) = q_now;

        % --------------------------------------------------------------
        % Check if we have reached the goal before continuing to follow.
        % --------------------------------------------------------------
        if q_now(1) >= q_goal(1)
            if DEBUG
                fprintf('\nWALL_FOLLOW_HANDLER:\n%s\n', 'Point matched goal.');
            end
            finished = true;
            unreachable = false;
            break;
        elseif (norm(hit_now(1:2), 2) > 1) % Hack to ensure we move before checking q_hit
            
            if points_match(q_now, q_last_hit)
                if DEBUG
                    fprintf('\nWALL_FOLLOW_HANDLER:\n%s\n', 'Point matched last hit.');
                end
                finished = true;
                unreachable = true;
                break;
                
            else
                m_line_reencounter = false;
                % To test the m-line, we need only see how far the y-coordinate
                % is from the y-origin.
                if DEBUG
                    fprintf('\nWALL_FOLLOW_HANDLER:\n%s: %0.3g\n', 'q_now(y) = ', q_now(2));
                end
                if abs(q_now(2)) <= d_tol
                    d_goal     = distance(q_now, q_goal);
                    d_last_hit = distance(q_last_hit, q_goal);
                    if DEBUG
                        fprintf('\nWALL_FOLLOW_HANDLER:\n%s\n', 'Intersected m-line.');
                        fprintf('WALL_FOLLOW_HANDLER:\n%s:\t\t%0.3g\n', 'Distance to goal', d_goal);
                        fprintf('WALL_FOLLOW_HANDLER:\n%s:\t%0.3g\n', 'Distance to last hit', d_last_hit);
                    end
                    
                    if (d_goal < d_last_hit) && ~points_match(q_now, q_last_hit)
                        m_line_reencounter = true;
                        finished = false;
                        unreachable = false;
                        if DEBUG
                            fprintf('WALL_FOLLOW_HANDLER:\n%s\n', 'M-line re-encounter OK.');
                        end
                        break;
                    else
                        if DEBUG
                            fprintf('WALL_FOLLOW_HANDLER:\t%s\n', 'M-line not encountered.');
                        end
                    end
                end
                
            end
        end
        fprintf('\n');
    end
    
    new_pos = q_now;
end

function going_straight = follow_wall(serPort, fwd_vel, ang_vel, R, L, F, wall)
    global WAIT_TIME;
    global angle;
    
    going_straight = 0;
    % Angle Velocity for different bumps
    av_bumpright =  4 * ang_vel;
    av_bumpleft  =  2 * ang_vel;
    av_bumpfront =  3 * ang_vel;
    av_nowall    = -4 * ang_vel;

    if R || L || F
        v = 0;                         % Set Velocity to 0
    elseif ~wall
        v = 0.25 * fwd_vel;            % Set Velocity to 1/4 of the default
    else
        v = fwd_vel;                   % Set Velocity to the default
    end

    if R
        av = av_bumpright;                  % Set Angular Velocity to av_bumpright
    elseif L
        av = av_bumpleft;                   % Set Angular Velocity to av_bumpleft
    elseif F
        av = av_bumpfront;                  % Set Angular Velocity to av_bumpfront
    elseif ~wall
        av = av_nowall;                     % Set Angular Velocity to av_nowall
    else
        av = 0; 
        going_straight = 1;
        %q_now(3) = (floor(q_now(3) * 20) / 20.0)
        if angle >= 0
            angle = (floor(angle * 10) / 10.0);% Set Angular Velocity to 0
        else
            angle = (ceil(angle * 10) / 10.0);% Set Angular Velocity to 0
        end
    end
    

    SetFwdVelAngVelCreate(serPort, v, av );
end

function [yn, R, L, F] = bump_check(serPort)
    global WAIT_TIME;
    yn = false;
    [R, L, ~,~,~, F] = BumpsWheelDropsSensorsRoomba(serPort);
    pause(WAIT_TIME);
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
