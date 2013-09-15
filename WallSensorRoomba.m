function w = WallSensorRoomba(serPort)
  w = nan;
  try
    N = serPort.BytesAvailable();
    while(N ~= 0)
      fread(serPort, N);
      N = serPort.BytesAvailable();
    end

    warning off
    global td

    % Flush buffer
    confirmation = (fread(serPort,1));
    while ~isempty(confirmation)
      confirmation = (fread(serPort,26));
    end

    % Get (142) non-virtual WALL(8) data field
    fwrite(serPort, [142 8]);

    w =(fread(serPort,1));
    pause(td)
  catch
    disp('WARNING: function terminated improperly. Output is potentially unreliable.');
end

