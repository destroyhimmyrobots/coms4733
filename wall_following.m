function done = wall_following()
  % Djentlmen, start your robots.
  % RoombaInit(tty);

  % Define the starting point of the robot to be x = 0, y = 0 <- R^2.
  origin              = [0;0];

  % Tolerance of sensor distance from the wall on any side, in mm,
  tol                 = 1;

  % Speed of iRC in m/s
  fwd_vel             = 0.025;
  ang_vel             = fwd_to_ang_vel(fwd_vel);

  % Initial distance to travel in meters
  dist                = 1;
  backup_dist         = 0.5;
  dist_inc            = 0.5;

  % Initial touch of wall obstacle
  find_wall_turn_id   = 0;
  find_wall_max_turns = 4;
  wall_detected       = 0;

  % Constant sensor array to update
  sensors_bump        = [BumpRight, BumpLeft, ~, ~, ~, BumpFront] ...
                      = BumpsWheelDropsSensorsRoomba(tty);
  sensors_all         = AllSensorsReadRoomba(tty);
  sensor_wall         = sensors_all(3);

  % First, hit a wall.
  % Searches in a square pattern.
  % Perhaps spiral is superior?
  while ~wall_detected
    travelDist(tty, vel, dist);
    sensors_bump = BumpsWheelDropsSensorsRoomba(tty);
    first_bump = BumpRight || BumpLeft || BumpFront;
    if first_bump
      travelDist(tty, 0, 0);          % Stop robot
      AngleSensorRoomba(serPort);     % Reset angular odometry
      break
    else
      turnAngle(pi/2);
      if find_wall_turn_id == find_all_max_turns
        dist = dist + dist_inc;
        find_wall_turn_id = 0;
      else
        find_wall_turn_id = find_wall_turn_id + 1;
      end
    end
  end

  % Back up ever so slightly
  travelDist(tty, vel, backup_dist);
  
  % Turn parallel to obstacle
  if BumpRight
        % Push the robot against the wall lightly
        SetFwdVelAngVelCreate(serPort,v,w)  % Turn anticlockwise 30 deg
        ang= pi/4;  % Angle to turn
    elseif BumpLeft
        SetFwdVelAngVelCreate(serPort,v,-w) % Turn clockwise
        ang= pi/4;
    elseif BumpFront
        SetFwdVelAngVelCreate(serPort,v,w)  % Turn counter-clockwise
        ang= pi/2;                          % Turn further
    end

    % Wait for turn to complete
    angTurned= 0;
    while angTurned < ang
        angTurned= angTurned+abs(AngleSensorRoomba(serPort));
        pause(0.1)
    end
end
