%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Group    9 Point  Follower
% Usage:  g9_follow_points(serPort)
%
% Enrique Cruz (ec...)
% Ryder   Moody (rlm...)
% Marc    Szalkiewicz (mjs2251)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function g9_follow_points(serPort)
    clc; format long;
    file = 'ruby_xy.txt';

    xy = parse_points(file);
    pos = [xy(1,:), 0];
    
    fprintf('Starting point is: \t %0.5g |\t %0.5g\n', pos(1), pos(2));

    for i=2:length(xy)
        fprintf('\nMoving to point %d.\t px %0.3g |\t py %0.3g\n',...
            i, xy(i,1), xy(i,2));
        pos = go_to_point(serPort, pos, xy, i);
        fprintf('Point %d done. It was\t px %0.3g |\t py %0.3g\n',...
            i, xy(i,1), xy(i,2));        
    end
    
    fprintf('Goal Reached.\n\n');
    clear;
end

function pos = go_to_point(serPort, pos, xy, i)

    
    if i < 3
        last = [xy(1, 1)-5, xy(1, 2)];
    else
        last = xy(i - 2, :)
    end
   
    
    current = xy(i - 1, :)
    next = xy(i, :)
    d        = dist(pos, next);
    % Create endpoint of vector of distance d straight ahead.
    % Create a vector from the current position to the new position

    % dtmp = arbitrary distance in front current bearing
    v1 = [current(1) - last(1), current(2) - last(2)];
    v2 = [next(1) - current(1), next(2) - current(2)];

    % Get angle between those vectors
    t = vec_angle(v1, v2);
    
    % Turn and advance
    pos = advance(serPort, t, d, pos);
end

function pos = advance(serPort, t, d, pos)
    avel = 0.1;
    
    vel      = 0.4;
    tick     = 0.15;
    ticks    = 0;
    d_sensor = 0;
    
    % Reset sensors
    %AngleSensorRoomba(serPort);
    DistanceSensorRoomba(serPort);
    pause(tick);
    
    % Turn
    angle_to_robot = convert2deg(t)
    if(abs(t) > 1e-4)
        turnAngle(serPort, avel, fix_degrees(convert2deg(t)));
        pos     = update_pos(0, t, pos);
        fprintf('Turned %0.5g theoretical rads.\n', t);
        pause(tick);
    else
        fprintf('Chose not to turn. %0.5g is less than threshold\n', t);
    end
    
    % Advance by distance d
    SetFwdVelAngVelCreate(serPort, vel, 0.08);
    while(d_sensor <= d)
        ticks    = ticks + 1;
        meters   = DistanceSensorRoomba(serPort);
        d_sensor = d_sensor + meters;
        pos      = update_pos(meters, 0, pos);
        pause(tick);
    end
    SetFwdVelAngVelCreate(serPort, 0, 0);
    pause(tick);
    
    % Perform an update after stopping to account for deceleratory d
    meters   = DistanceSensorRoomba(serPort);
    d_sensor = d_sensor + meters;
    %pos      = update_pos(meters, 0, pos);
    pause(tick);
    % Correct for any angular errors during turn & motion.
    %pos = correct(d, d_sensor, pos);
end

function pos = correct(d, d_sensor, pos)
    d_overshot = d_sensor - d;
    epsilon    = [d_overshot, d_overshot, 0];
    
    pos = [pos(1) + cos(pos(3))*epsilon(1)*0.1 ...
        ,  pos(2) + sin(pos(3))*epsilon(2)*0.1 ...
        ,  pos(3)];
    
    fprintf('Final adjustment:\t x %0.5g |\t y %0.5g |\t t %0.5g\n', ...
        pos(1), pos(2), pos(3));
end

function t = vec_angle(v1, v2)
    v1
    v2
    n1 = norm(v1, 2);
    n2 = norm(v2, 2);
    
    ct = dot(v1, v2) / (n1 .* n2)
    % FIX ME! I #$@#@ UP POINT 5
    check_sign = asin(ct)
    
    t  = acos(ct)
    if check_sign < 0
        t = -t
    end
    
    fprintf('Angle between current & next bearing:\t%0.5g\n', t);
end

function pos = update_pos(d, rad, pos)
    pos(3) = pos(3) + rad;
    pos(1) = pos(1) + cos(pos(3) - rad) * d;
    pos(2) = pos(2) + sin(pos(3) - rad) * d;
    
    fprintf('Measured position:\t x %0.5g |\t y %0.5g |\t t %0.5g\n', ...
        pos(1), pos(2), pos(3));
end

function d = fix_dist(deg)
    d = deg * 1;
end

function f = fix_degrees(deg)
    p = polyfit([180 90 45], [176 86 41], 3);
    
    abs_deg = abs(deg)
    %coeff = [-5.486968449931409e-06,...
    %    0.001728395061728,...
    %    0.844444444444445,...
    %    0];
    f = polyval(p, abs_deg);%coeff(1)*abs_deg + coeff(2)*abs_deg + coeff(3)*abs_deg;
    if deg > 0
        f = -f;
    end
end

function d = dist(now, later)
    d = sqrt((later(1)-now(1)).^2 + (later(2)-now(2)).^2);
end

function rad = wrap_to_pi(rad)
    if(rad < -2*pi)
        while (rad < -2*pi)
            rad = rad + 2*pi;
        end
    elseif(rad > 2*pi)
        while (rad > 2*pi)
            rad = rad - 2*pi;
        end
    else
        rad = rad;
    end
end

function xy = parse_points(filename)
    fid = fopen(filename, 'rt');
    xy_text  = textscan(fid, '[%f, %f]');
    xy = [xy_text{1} xy_text{2}];
end
