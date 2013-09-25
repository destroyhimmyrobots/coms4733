function finalRad = wall_shagger(serPort)
% Simple program for autonomously control the iRobot Create on either the
% physical Create or the simulated version. This will simply spiral outward
% and turn away from obstacles that detects with the bump sensors.
%
% For the physical Create only, it is assumed that the function call
% serPort= RoombaInit(comPort) was done prior to running this program.
% Calling RoombaInit is unnecessary if using the simulator.
%
% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Output:
% finalRad - Double, final turning radius of the Create (m)

% ExampleControlProgram.m
% Copyright (C) 2011 Cornell University
% This code is released under the open-source BSD license.  A copy of this
% license should be provided with the software.  If not, email:
% CreateMatlabSim@gmail.com

clc;

    % Set constants for this program
    global WAIT_TIME;
    WAIT_TIME               = 0.1;
    maxDuration             = 1500;   % Max time to allow the program to run (s)
    maxDistSansBump         = 100;    % Max distance to travel without obstacles (m)
    maxFwdVel               = 0.5;    % Max allowable forward velocity with no angular
                                      % velocity at the time (m/s)
    maxVelIncr              = 0.005;  % Max incrementation of forward velocity (m/s)

    right_turn_count            = 0;      % AKA turning left
    right_turn_count      = 0;      % CHECK IF > 3 for turning right

    % Initialize loop variables
    tStart                  = tic;    % Time limit marker
    x_dist              = 0;
    y_dist              = 0;
    angs_since_bump         = zeros(1,10);
    finished                = 0;
    v                       = maxFwdVel / 4;      % Forward velocity (m/s)
    w                       = v2w(v);             % Angular velocity (rad/s)
    found_wall              = 0;
    
    DistanceSensorRoomba(serPort);
    AngleSensorRoomba(serPort);
    pause(WAIT_TIME);
    
    % Enter main loop
    while (toc(tStart)  <= maxDuration) % &&  ~finished

        [bump_right, bump_left, ~, ~, ~, bump_front] ...
            = BumpsWheelDropsSensorsRoomba(serPort);
        bumped = bump_right || bump_left || bump_front;
        
        if bumped
            found_wall  = 1;
            v = maxFwdVel / 4;

            fprintf('Hit the wall...\t');
            ang_pre_rot  = AngleSensorRoomba(serPort);
            right_turn_count = ...
                rotate_handler(serPort, right_turn_count, bump_left, bump_right, bump_front);
            fprintf('Finished rotating.\n');
            ang_post_rot = AngleSensorRoomba(serPort);

            for i=length(angs_since_bump):-1:2
                angs_since_bump(i) = angs_since_bump(i-1);
            end
            angs_since_bump(1) = ang_pre_rot + ang_post_rot;

            % Update distance and angle sensors.
            current_dist = DistanceSensorRoomba(serPort);

            % Hack for odometry correction after 3 right bumps
            if right_turn_count > 0
                current_ang = ...
                    mean(angs_since_bump(1:min(length(angs_since_bump) ...
                    , right_turn_count)))...
                    + (pi / 8);
            else
                current_ang = AngleSensorRoomba(serPort);
            end

            x_dist          = x_dist + (current_dist)*cos(current_ang);
            y_dist          = y_dist + (current_dist)*sin(current_ang);
            
            fprintf('dist_bump:\t%0.5g\n', current_dist);
            fprintf('x_total:\t\t%0.5g\n', x_dist);
            fprintf('y_total:\t\t%0.5g\n', y_dist);
            fprintf('theta:\t\t\t%0.5g\n', current_ang);
            fprintf('bump_vec:\t\t[%0.5g\t\t%0.5g\t\t%0.5g]\n\n', ...
                angs_since_bump(1),angs_since_bump(2),angs_since_bump(3));
            
        elseif found_wall
            fprintf('Following wall...\n');            
            SetFwdVelAngVelCreate(serPort, v, -0.4);
        else
            % Cornering
            fprintf('Looking for the wall...\n');
            SetFwdVelAngVelCreate(serPort, v, -0.5);
            v            = min(maxFwdVel / 1.5,  v + maxVelIncr);
        end
        
        pause(WAIT_TIME);
%        finished = is_origin(x_traveled, y_traveled);
%        if found_wall && finished 
%            fprintf('Returned to origin. Exiting.\n\n');
%        end

    end
    
    fprintf('Exited loop. Goodbye.\n');
    % Specify output parameter
    finalRad = v / w;
    
    % Stop robot motion
    v = 0;
    w = 0;
    SetFwdVelAngVelCreate(serPort,v,w);
  end
    
    
function rtc = rotate_handler(serPort, rtc, bump_left, bump_right, bump_front)
% Check bump sensors and steer the robot away from obstacles if necessary
%
% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Output:
% bumped - Boolean, true if bump sensor is activated

global WAIT_TIME;

    ang       = 0;
    already_turned = 0;
    right_ang = pi / 4;
    left_ang  = pi / 2;
    front_ang = pi / 3.2;
    
    if isnan(bump_right) bump_right = 0; end
    if isnan(bump_left)  bump_left  = 0; end
    if isnan(bump_front) bump_front = 0; end
    
    % Turn away from obstacle
    if bump_right         % Turn in place anticlockwise
        if rtc < 3
            rtc = rtc + 1;
        end
        ang = right_ang;  % Angle to turn
    elseif bump_left      % Turn in place clockwise
        ang = left_ang;
    elseif bump_front
        turning_left = 1;
        ang = front_ang;
    end
    
    SetFwdVelAngVelCreate(serPort, 0, ang); % Wait for turn to complete
    while already_turned < ang
        already_turned  = already_turned + abs(AngleSensorRoomba(serPort));
        pause(WAIT_TIME);
    end
end

function finished = is_origin(x, y)
    d_epsilon = 0.4;
    a_epsilon = 1.9*pi;
    dist = sqrt((x^2) + (y^2));

    if (dist < d_epsilon) % && (abs(current_angle) >= a_epsilon)
        finished = 1;
    else
        finished = 0;
    end
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
    maxWheelVel = 0.5;   % Max linear velocity of each drive wheel (m/s)
    robotRadius = 0.2;   % Radius of the robot (m)
    
    % Max velocity combinations obey rule v+wr <= v_max
    w = (maxWheelVel-v)/robotRadius;
end
