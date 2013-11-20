function follow_points(serPort)
    ismac    = false;
    file = 'ruby_xy.txt';
    
    if(~simulator)
        if(ismac)
            RoombaInit_mac(serPort);
        else
            RoombaInit(serPort);
        end
    end

    xy = parse_points(file);
    pos = xy(1,:);

    for i=2:length(xy)
        printf('\n\nMoving to point %d\n', i);
        pos = go_to_point(serPort, pos, xy(i));
        printf('\n\nDone.\n\n');
    end
    
    printf('Goal Reached.');
end

function [d, spd, tick, pos] = go_to_point(serPort, pos, next)
    spd      = 0.36;
    tick     = 0.1;
    ticks    = 0;
    d_sensor = 0;
    d        = dist(pos, next);
    
    % Create endpoint of vector of distance d straight ahead.
    % Create a vector from the current position to the new position

    % !!!!!!!!!!! FIX ME.
    v1 = [next(1) - pos(1),  cos(pos(3))*pos(1)];
    v2 = [next(1) - pos(1), next(2) - pos(2)];
    
    % Turn by the angle between those vectors
    t = vec_angle(v1, v2);

    AngleSensorReadRoomba();
    turnAngle(serPort, 0.1, t);
    update_pos(0, pos);
    
    % Advance by distance d
    SetFwdVelAngVelCreate(serPort, spd, 0);
    while(d_sensor <= d)
        ticks    = ticks + 1;
        meters   = DistanceSensorRoomba(serPort);
        d_sensor = d_sensor + meters;
        pos      = update_pos(meters, pos);
        pause(tick);
    end
    SetFwdVelAngVelCreate(serPort, 0, 0);
    pos = correct(d, d_sensor, pos);
end

function t = vec_angle(v1, v2)
    n1 = norm(v1, 2);
    n2 = norm(v2, 2);
    
    ct = dot(v1, v2) / (n1 .* n2);
    t  = acos(ct) * (180/pi);
end

function pos = update_pos(d, ang, pos)
    pos(1) = pos(1) + cos(pos(3)) * d;
    pos(2) = pos(2) + sin(pos(3)) * d;
    % !!!!!!!!!! FIX ME.
    pos(3) = pos(3) + AngleSensorReadRoomba();

    printf('Measured position:\t x %0.5g |\t y %0.5g |\t t %0.5g\n', ...
        pos(1), pos(2), pos(3));
end

function d = dist(now, later)
    d = sqrt((later(1)-now(1)).^2 + (later(2)-now(2)).^2);
end

function c = correct(d, d_sensor, pos)
    d_overshot = d_sensor - d;
    epsilon    = [d_overshot, d_overshot, 1];
    
    c(1) = pos(1) + cos(pos(3))*epsilon(1);
    c(2) = pos(2) + sin(pos(3))*epsilon(2);
    % !!!!!!!!!!! FIX ME: ANGULAR COMPENSATION
    % c(3) = pos(3)+epsilon(3);
    
    printf('Adjusted position:\t x %0.5g |\t y %0.5g |\t t %0.5g\n', ...
        c(1), c(2), c(3));
end

function xy = parse_points(filename)
    fid = fopen(filename, 'rt');
    xy_text  = textscan(fid, '[%f, %f]');
    xy = [xy_text{1} xy_text{2}];
end
