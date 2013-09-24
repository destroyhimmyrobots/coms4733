function finalRad= WallFollower(serPort)
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

    % Set constants for this program
    maxDuration= 150;  % Max time to allow the program to run (s)
    maxDistSansBump= 100;% Max distance to travel without obstacles (m)
    maxFwdVel= 0.5;     % Max allowable forward velocity with no angular 
                        % velocity at the time (m/s)
    maxVelIncr= 0.005;  % Max incrementation of forward velocity (m/s)
    maxOdomAng= pi*10;   % Max angle to move around a circle before 
                        % increasing the turning radius (rad)
    turning_left = 0; % AKA turning left
    turning_right = 0; % CHECK IF > 3 for turning right
    
    % Initialize loop variables
    tStart= tic;        % Time limit marker
    distSansBump= 0;    % Distance traveled without hitting obstacles (m)
    angTurned= 0;       % Angle turned since turning radius increase (rad)
    x_traveled = 0;
    y_traveled = 0;
    current_angle = 0;
    angle_since_bump = 0;
    prev_angle_since_bump = 0;
    prev_angle_since_bump_2 = 0;
    distance_since_bump = 0;
    finished = 0;
    v= 0;               % Forward velocity (m/s)
    w= v2w(v);          % Angular velocity (rad/s)
    found_wall = 0;
    % Start robot moving
    SetFwdVelAngVelCreate(serPort,0,0)
    
    %
    BUMP_RIGHT = 1;
    BUMP_LEFT = 2;
    BUMP_FRONT = 3; 
    WALL = 4; 
    VIRT_WALL = 5;
    
    
    % Enter main loop
    while toc(tStart) < maxDuration && distSansBump <= maxDistSansBump && ~finished
        %sensor_data = AllSensorsReadRoomba(serPort)
        %bumped_front = sensor_data(BUMP_FRONT);
        %bumped_left = sensor_data(BUMP_LEFT);
        %bumped_right = sensor_data(BUMP_RIGHT);
        right_adjust = turning_right;
        bumped = bumpCheckReact(serPort);
        % If obstacle was hit reset distance and angle recorders
        bump_distance = 0;
        bump_angle = 0;
        if bumped
            found_wall = 1;
            bump_distance = DistanceSensorRoomba(serPort); % Reset odometry too
            bump_angle = AngleSensorRoomba(serPort);
            if (right_adjust > 3)
                angle_since_bump = angle_since_bump + pi/8;
            else
                angle_since_bump = angle_since_bump - bump_angle;
            end        
            if abs((angle_since_bump + prev_angle_since_bump + prev_angle_since_bump_2)/3) > pi/14 || turning_left
                if turning_left
                    disp 'because of turning left'
                else
                    disp((angle_since_bump + prev_angle_since_bump + prev_angle_since_bump_2)/3)
                end
                current_angle = current_angle + angle_since_bump;
            end
            prev_angle_since_bump_2 = prev_angle_since_bump;
            prev_angle_since_bump = angle_since_bump;
            angle_since_bump = 0;
            distSansBump = 0;
            angTurned = 0;
            
            % Start moving again at previous velocities
            %SetFwdVelAngVelCreate(serPort,0,0.5)
        elseif found_wall        
            turning_right = turning_right + 1;
            turning_left = 0;
            SetFwdVelAngVelCreate(serPort,maxFwdVel/4,-0.5)
        else
            turning_left  = 0;
            SetFwdVelAngVelCreate(serPort,v,-0.5)
            v = v + maxVelIncr
        end
       
        
        % Update distance and angle recorders from odometry
        current_dist = DistanceSensorRoomba(serPort);
        distSansBump= distSansBump+current_dist;
        current_turn = AngleSensorRoomba(serPort);
        angTurned= angTurned+current_turn;
        
        if found_wall
            angle_since_bump = angle_since_bump + current_turn;
            distance_since_bump = distance_since_bump + current_dist;
            x_traveled = x_traveled + (current_dist + bump_distance)*cos(current_angle);
            y_traveled = y_traveled + (current_dist + bump_distance)*sin(current_angle);
        end

            
        % Increase turning radius if it is time
 
        if angTurned >= maxOdomAng
            % Either increase forward velocity by the increment or by half
            % the difference to the max velocity, whichever is lower
            v= min(v+maxVelIncr,v+(maxFwdVel-v)/2);
            % Choose angular velocity based on max allowable wheel speed
            w= v2w(v);
            SetFwdVelAngVelCreate(serPort,v,w)
            % This could be accomplished more simply by using
            % SetFwdVelRadiusRoomba, this way is just done more fun
        end
        
        % Briefly pause to avoid continuous loop iteration
        pause(0.1)
        %disp x
        %disp (x_traveled);
        %disp y
        %disp (y_traveled);
        current_angle
        finished = check_return_to_origin(x_traveled, y_traveled);

    end
    
    % Specify output parameter
    finalRad= v/w;
    
    % Stop robot motion
    v= 0;
    w= 0;
    SetFwdVelAngVelCreate(serPort,v,w)
    
    % If you call RoombaInit inside the control program, this would be a
    % good place to clean up the serial port with...
    % fclose(serPort)
    % delete(serPort)
    % clear(serPort)
    % Don't use these if you call RoombaInit prior to the control program
    
function bumped= bumpCheckReact(serPort)
% Check bump sensors and steer the robot away from obstacles if necessary
%
% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Output:
% bumped - Boolean, true if bump sensor is activated

    % Check bump sensors (ignore wheel drop sensors)
    [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
    if isnan(BumpRight)
        BumpRight = 0;
    end
    if isnan(BumpLeft)
        BumpLeft = 0;
    end
    if isnan(BumpFront)
        BumpFront = 0;
    end
    
    bumped= BumpRight || BumpLeft || BumpFront;
    
    % Halt forward motion and turn only if bumped
    if bumped
        AngleSensorRoomba(serPort);     % Reset angular odometry
        v= 0;       % Forward velocity
        w= v2w(v);  % Angular velocity
        
        % Turn away from obstacle
        if BumpRight
            turning_right = 0;
            SetFwdVelAngVelCreate(serPort,0,0.5)  % Turn counter-clockwise
            ang= 0;  % Angle to turn
        elseif BumpLeft
            turning_left = 0;
            SetFwdVelAngVelCreate(serPort,v,v2w(v)) % Turn clockwise
            ang= pi/16;
        elseif BumpFront
            turning_left = 1;
            SetFwdVelAngVelCreate(serPort,0,0.3)  % Turn counter-clockwise
            ang= pi/8;                          % Turn further
        end
        
        % Wait for turn to complete
        angTurned= 0;
        while angTurned < ang
            angTurned= angTurned+abs(AngleSensorRoomba(serPort));
            pause(0.1)
        end
        % This could be accomplished more simply by using turnAngle, 
        % this way is just more fun
    else
         %SetFwdVelAngVelCreate(serPort,maxFwdVel,0.5)
    end
end

function finished= check_return_to_origin(x_pos, y_pos)
    dist = ((x_pos^2)+ (y_pos^2))^(1/2)
    if dist < 0.65 && toc(tStart) > 20
        finished = 1;
    else
        finished = 0;
    end
end


function w= v2w(v)
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
end

