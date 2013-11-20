function follow_points(serPort)
    clc; format long;
    ismac     = false;
    file      = 'ruby_xy.txt';
    simulator = true;

    if(~simulator)
        if(ismac)
            RoombaInit_mac(serPort);
        else
            RoombaInit(serPort);
        end
    end

    xy = parse_points(file);
    pos = [xy(1,:), 0];

    for i=2:length(xy)
        fprintf('\n\nMoving to point %d\n', i);
        pos = go_to_point(serPort, pos, xy(i,:));
        fprintf('\n\nDone.\n\n');
    end
    
    fprintf('Goal Reached.\n\n');
    clear;
end

function pos = go_to_point(serPort, pos, next)
    d        = dist(pos, next);
    % Create endpoint of vector of distance d straight ahead.
    % Create a vector from the current position to the new position

    % !!!!!!!!!!! FIX ME.
    x_ahead = next(1) / cos(pos(3)); 
    y_ahead = x_ahead * sin(pos(3));
    
    v1 = [x_ahead - pos(1), y_ahead - pos(2)];
    v2 = [next(1) - pos(1), next(2) - pos(2)]; % Technically d should be ||v_2||
    
    % Get angle between those vectors
    t = vec_angle(v1, v2);
    % Turn and advance
    pos = advance(serPort, t, d, pos);
end

function pos = advance(serPort, t, d, pos)
    spd      = 0.4;
    tick     = 0.1;
    ticks    = 0;
    d_sensor = 0;
    
    % Turn
    AngleSensorRoomba(serPort);
    turnAngle(serPort, 0.1, t);
    update_pos(0, AngleSensorRoomba(serPort), pos);
    
    % Advance by distance d
    SetFwdVelAngVelCreate(serPort, spd, 0);
    while(d_sensor <= d)
        ticks    = ticks + 1;
        meters   = DistanceSensorRoomba(serPort);
        d_sensor = d_sensor + meters;
        pos      = update_pos(meters, AngleSensorRoomba(serPort), pos);
        pause(tick);
    end
    SetFwdVelAngVelCreate(serPort, 0, 0);
    
    % Perform an update after stopping to account for deceleratory d.
    meters   = DistanceSensorRoomba(serPort);
    d_sensor = d_sensor + meters;
    pos      = update_pos(meters, AngleSensorRoomba(serPort), pos);
    pause(tick);
    
    % Correct for any angular errors during motion
    a = AngleSensorRoomba(serPort);
    if(abs(a) > 1e-3)
        fprintf('Correcting angle after displacement by %0.5g', -a);
        turnAngle(serPort, 0.1, -a);
        pos = update_pos(0, AngleSensorRoomba(serPort), pos);
        pause(tick);
    end
    pos = correct(d, d_sensor, pos);
end

function t = vec_angle(v1, v2)
    n1 = norm(v1, 2);
    n2 = norm(v2, 2);
    
    ct = dot(v1, v2) / (n1 .* n2);
    t  = acos(ct) * (180/pi);
    
    fprintf('Angle between current & next bearing:\t%0.5g', t);
end

function pos = update_pos(d, a, pos)
    pos(1) = pos(1) + cos(pos(3)) * d;
    pos(2) = pos(2) + sin(pos(3)) * d;
    if(~isnan(a))
        pos(3) = pos(3) + a;
    end

    fprintf('Measured position:\t x %0.5g |\t y %0.5g |\t t %0.5g\n', ...
        pos(1), pos(2), pos(3));
end

function d = dist(now, later)
    d = sqrt((later(1)-now(1)).^2 + (later(2)-now(2)).^2);
end

function pos = correct(d, d_sensor, pos)
    d_overshot = d_sensor - d;
    epsilon    = [d_overshot, d_overshot, 1]; % x y t
    
    pos(1) = pos(1) + cos(pos(3))*epsilon(1);
    pos(2) = pos(2) + sin(pos(3))*epsilon(2);
    % !!!!!!!!!!! FIX ME: ANGULAR COMPENSATION
    pos(3) = pos(3);
    
    fprintf('Adjusted position:\t x %0.5g |\t y %0.5g |\t t %0.5g\n', ...
        pos(1), pos(2), pos(3));
end

function xy = parse_points(filename)
    fid = fopen(filename, 'rt');
    xy_text  = textscan(fid, '[%f, %f]');
    xy = [xy_text{1} xy_text{2}];
end
