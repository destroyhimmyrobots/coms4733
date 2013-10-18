function mapper(serPort)
    RESOLUTION    = 3; % # segments in meter (1D)
    METERS_TO_MAP = 10;
    DIST_PER_SEGMENT = 1/RESOLUTION;
    
    WAIT_TIME = 0.1;
    v = 0.2;
    w = 0.2;
    
    % must be odd so that you start within one segment
    width = RESOLUTION * METERS_TO_MAP;
    map = zeros(width,width);
    
    % start in the middle
    initial_map_pos = [ceil(width/2), ceil(width/2)];
    
    moving = 1;
    
    pos = [0 0 0];
    
    while (true)
        pause(WAIT_TIME);
        
        [any_bump, R, L, F] = bump_check(serPort);
        
        if any_bump
            moving = 0;
            % Mark this location as blocked
            pos = update_dist_orient(serPort, pos);
           
            x = round_to_nearest(DIST_PER_SEGMENT, pos(1))
            y = round_to_nearest(DIST_PER_SEGMENT, pos(2))
            
            map(initial_map_pos(2) + round(y*RESOLUTION), initial_map_pos(1)+round(x*RESOLUTION)) = 1
        else
            moving = 1;
        end
        
        if moving
            % Just go
            move_forward
        else
            % Turn a direction repending on R/L/F
            to_turn = 0;
            if F
                to_turn = 180-60;
            elseif L
                to_turn = -60;
            else
                to_turn = 60;
            end
            if rand < 0.3
                to_turn = to_turn + 30;
            end
            turnAngle(serPort, w, to_turn);
        end      
    end
    
    function move_forward
        SetFwdVelAngVelCreate(serPort, v, 0)
    end
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

function rounded = round_to_nearest(res, val)
    rounded = round(val / res) * res
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
