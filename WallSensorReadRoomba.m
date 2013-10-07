function wall_sensor = WallSensorReadRoomba(serPort)
    try
        set(serPort,'timeout',.01);
        %Flush buffer
        N = serPort.BytesAvailable();
        while(N~=0)
            fread(serPort,N);
            N = serPort.BytesAvailable();
        end
    catch
    end

%% Get (142) Wall Reading(8) data fields

fwrite(serPort, [142 8]);
wall_sensor = fread(serPort, 1);
end
