function diagnose_turns(serPort)
    RoombaInit_mac(serPort);
    for i=1:8
        turnAngle(serPort, 0.1, 90);
        pause(3);
    end
end
