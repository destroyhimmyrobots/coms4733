function follow_points(serPort, ismac, xy)
    if(ismac)
        RoombaInit_mac(serPort);
    else
        RoombaInit(serPort);
    end
    
    abs = zeros(1,3);
    adj = zeros(1,3);

    for i=1:length(xy)
        printf('Moving to point %d\n', i);
        pos = go_to_point(serPort, adj, xy(i));
        printf('Done.\n\n');

        printf('Measured position:\t x %0.5g |\t y %0.5g |\t t %0.5g\n', ...
            pos(1), pos(2), pos(3));
        adj = correct(pos);

        printf('Adjusted position:\t x %0.5g |\t y %0.5g |\t t %0.5g\n', ...
            adj(1), adj(2), adj(3));
        
        fprintf('\n');
        pause(0.15);
    end
    
    printf('Goal Reached.');
end

function pos = go_to_point(serPort, pos, next)
    d = dist(pos, next);
    
    % Create endpoint of vector of distance d straight ahead.
    % Create a vector from the current position to the new position
    % Vector representation: subtract the starting coordinate from
    % the ending coordinate.
    v1 = [next(1) - pos(1),  0];
    v2 = [next(1) - pos(1), next(1) - pos(2)];
    
    % Turn by the angle between those vectors
    t = vec_angle(v1, v2);
    turnAngle(serPort, 0.1, t);
    
    % Advance by distance d
    traveled = 0;
    SetFwdVelAngVelCreate(serPort, 0.35, 0);
    while(traveled <= d)
        meters = DistanceSensorRoomba(serPort);
        pos = update_pos(meters, pos);
        pause(0.1);
    end
    SetFwdVelAngVelCreate(serPort, 0, 0);
end

function t = vec_angle(v1, v2)
    n1 = norm(v1, 2);
    n2 = norm(v2, 2);
    
    ct = dot(v1, v2) / (n1 .* n2);
    t  = acos(ct) * (180/pi);
end

function pos = update_pos(d, pos)
    pos(1) = pos(1) + cos(pos(3)) * d;
    pos(2) = pos(2) + sin(pos(3)) * d;
end

function d = dist(now, later)
    d = sqrt((later(1)-now(1)).^2 + (later(2)-now(2)).^2);
end

function c = correct(pos)
    epsilon = [1, 1, 1];
        
    c(1) = pos(1)*epsilon(1);
    c(2) = pos(2)*epsilon(2);
    c(3) = pos(3)*epsilon(3);
end
