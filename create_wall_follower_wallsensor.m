function finalRad = create_wall_follower_wallsensor(serPort)

    % Wall follower for iRobot Create.
    %
    % Authors:
    %    Ryder Moody
    %    Enrique Cruz
    %    Marc Szalkiewicz
    %
    % Begins by spiraling so as to strike a wall.
    %
    % After striking the wall, corrects until the wall sensor activates.
    %
    % Travels straight along the wall until the wall sensor deactives
    % or the robot strikes the wall again.
    %
    % Input:
    % serPort - Serial port object, used for communicating over bluetooth
    %
    % Output:
    % finalRad - Double, final turning radius of the Create (m)

    % Code from ExampleControlProgram.m, (C) 2011 Cornell University
    % released under the open-source BSD license
    % was used as an initial reference.

    clc;

    global WAIT_TIME;
    WAIT_TIME = 0.08;

    % Set constants for this program
    maxDuration     = 1200;   % Max time to allow the program to rcdasdun (s)
    maxDistSansBump = 10;     % Max distance to travel without obstacles (m)
    distSansBump = 0;         % Distance traveled without hitting obstacles (m)
    angTurned    = 0;         % Angle turned since turning radius increase (rad)    
    maxFwdVel       = 0.3;    % Max allowable forward velocity with no angular 
                              % velocity at the time (m/s)
    inc_vel         = 0.005;  % Max incrementation of forward velocity (m/s)
    max_odom_ang    = pi/4;   % Max angle to move around a circle before 
                              % increasing the turning radius (rad)
    

    tStart       = tic;       % Time limit marker
    hit_wall     = 0;
    seen_wall    = 0;
    norm_v       = 0.2;
    norm_dist    = 0.1;
    norm_angv    = 0.3;
    backup_dist  = -0.02;
    v            = norm_v;    % Forward velocity (m/s)
    w            = v2w(v);    % Angular velocity (rad/s)
    turn_v       = norm_v;
    corner_v     = 0.1;
    corner_angv  = -0.5;
    
    % Look for a wall
    while ~hit_wall && toc(tStart) <= maxDuration
        [bump_right, bump_left , ~, ~, ~, bump_front] = ...
            BumpsWheelDropsSensorsRoomba(serPort);
        
        hit_wall     = bump_right || bump_left || bump_front;
        fprintf('[Left:  %d]\t', bump_left);
        fprintf('[Right: %d]\t', bump_right);
        fprintf('[Front: %d]\n', bump_front);
        
        % Find wall
        if hit_wall
            fprintf('Found a wall!\n\n');
            DistanceSensorRoomba(serPort); % Reset odometry too
            AngleSensorRoomba(serPort);
            distSansBump = 0;
            angTurned    = 0;
        else
            fprintf('No wall yet.\n\n');
            SetFwdVelAngVelCreate(serPort, corner_v, corner_angv);
            corner_v = min(corner_v + inc_vel, maxFwdVel);
        end
        
        pause(WAIT_TIME);
    end
    
    v           = norm_v;
    corner_v    = norm_v;
    tStart      = tic;
    
    % Enter main loop
    while (toc(tStart) < maxDuration)
                
        [bump_right, bump_left , ~, ~, ~, bump_front] = ...
            BumpsWheelDropsSensorsRoomba(serPort);
        
        hit_wall     = bump_right || bump_left || bump_front;
        seen_wall    = WallSensorReadRoomba(serPort);
        
        if hit_wall
            corner_v = norm_v;
            
            fprintf('I hit the wall!\n\n');
            fprintf('[Left:  %d]\t', bump_left);
            fprintf('[Right: %d]\t', bump_right);
            fprintf('[Front: %d]\n', bump_front);
            
            travelDist(serPort, v, backup_dist);
            pause(WAIT_TIME);
            if bump_right
                ang = 10;
            elseif bump_left
                ang = 10;
            elseif bump_front
                ang = 10;
            end
            
            seen_wall = WallSensorReadRoomba(serPort);
            AngleSensorRoomba(serPort);
            total_turned = 0;
            
            while ~seen_wall && total_turned < (2*pi)
                fprintf('Turning...\n');
                total_turned = total_turned + AngleSensorRoomba(serPort);
                fprintf('Turned %0.3g so far', total_turned);
                turnAngle(serPort, turn_v, ang);
                pause(WAIT_TIME);
                seen_wall = WallSensorReadRoomba(serPort);
            end
            
            if total_turned >= (2*pi)
                fprintf('Did a barrel roll! Problem?\n');
            elseif seen_wall 
                % Don't do this if we 360. Corner instead.
                % Correct to travel straight
                fprintf('I see the wall!\n\n');
                turnAngle(serPort, v, 30);
                pause(WAIT_TIME);
            end

        elseif seen_wall
            corner_v = norm_v;
            fprintf('\nI see the wall!\n');
            travelDist(serPort, v, norm_dist);
        
        else % Cornering
            fprintf('Lost the wall. Cornering...\n\n');
            corner_v = min(corner_v + inc_vel, maxFwdVel);
            SetFwdVelAngVelCreate(serPort, corner_v , corner_angv);
        end
        
        % Briefly pause to avoid continuous loop iteration
        pause(WAIT_TIME);
    end
    
    % Specify output parameter
    finalRad = v/w;
    
    % Stop robot motion
    v = 0;
    SetFwdVelAngVelCreate(serPort,v,w2w(v));
    
    % If you call RoombaInit inside the control program, this would be a
    % good place to clean up the serial port with...
    % fclose(serPort)
    % delete(serPort)
    % clear(serPort)
    % Don't use these if you call RoombaInit prior to the control program
end

function w = v2w(v)
% Calculate the maximum allowable angular velocity from the linear velocity
%
% Input:
% v - Forward velocity of Create (m/s)
%
% Output:
% w - Angular velocity of Create (rad/s)
    
    % Robot constants
    maxWheelVel= 0.5;   % Max linear velocity of each drive wheel (m/s)
    robotRadius= 0.2;   % Radius of the robot (m)
    
    % Max velocity combinations obey rule v+wr <= v_max
    w= (maxWheelVel-v)/robotRadius;
end
