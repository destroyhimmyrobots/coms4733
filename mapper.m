function mapper(serPort)
    RESOLUTION    = 5; % # segments in meter (1D)
    METERS_TO_MAP = 7;
    DIST_PER_SEGMENT = 1/RESOLUTION;
    
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
    
    while (true)
        stuck
        pause(WAIT_TIME);
        
        [any_bump, R, L, F] = bump_check(serPort);

        if any_bump
            stuck = stuck + 1;
            moving = 0;
            % Mark this location as blocked
            pos = update_dist_orient(serPort, pos, 0);
           
            x = round_to_nearest(DIST_PER_SEGMENT, pos(1));
            y = round_to_nearest(DIST_PER_SEGMENT, pos(2));
            
            %dir_angle = round_to_nearest(45, pos(3) * 180 / pi);
            %dir_angle = mod(dir_angle, 360) * pi/180;
            
            x_offset = 0;
            y_offset = 0;
            bumper_angle = pi/4;
            if F
                x_offset = cos(pos(3));
                y_offset = sin(pos(3));
            elseif L
                x_offset = cos(pos(3) + bumper_angle);
                y_offset = sin(pos(3) + bumper_angle);
            elseif R
                x_offset = cos(pos(3) - bumper_angle);
                y_offset = sin(pos(3) - bumper_angle);
            end
            fprintf('L: %d, R: %d, F: %d, x_offset: %d, y_offset: %d', L, R, F, round(x_offset), round(y_offset));
            
            map(initial_map_pos(2) - round(y*RESOLUTION) - round(y_offset), initial_map_pos(1)+round(x*RESOLUTION) + round(x_offset)) = 1;
            figure(3), imagesc(map);
        else
            moving = 1;
        end
        
        if moving
            % Just go
            stuck = 0;
            move_forward
        else
            % Turn a direction repending on R/L/F
            to_turn = 0;
            ang_diff = 45
            if F
                to_turn = 180-ang_diff;
            elseif L
                to_turn = -ang_diff;
            else
                to_turn = ang_diff;
            end
            r = rand;
            if r < 0.3
                to_turn = to_turn + ang_diff;
            elseif r > 0.7
                to_turn = to_turn - ang_diff;
            end
            if stuck > 5
                to_turn = 180;
            end
            turnAngle(serPort, w, to_turn);
            pos = update_dist_orient(serPort, pos, to_turn);
            moving = 1;
        end      
    end
    
    function move_forward
        SetFwdVelAngVelCreate(serPort, v, 0)
    end
end

function q_now = update_dist_orient(serPort, q_prev, turned_in_deg)    
   
    
    % Update current position & orientation
    tmp_dist = DistanceSensorRoomba(serPort);
    %pause(WAIT_TIME);
    tmp_t = AngleSensorRoomba(serPort);
    
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
