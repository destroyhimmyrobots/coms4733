im = imread('highway.jpg');
avel = 0.1;
vel      = 0.4;
tick     = 0.15;

active = 1;
[x, y] = imclick(im);
x = round(x);
y = round(y);
binary = thresh(im, x, y);
[cen_x, cen_y, size] = imstats(binary);

while(active)
    binary = thresh(im, cen_x, cen_y);
    [new_cen_x, new_cen_y, new_size] = imstats(binary);
    %binary(cen_y-6:cen_y+6, cen_x-6:cen_x+6) = 0;
    if (new_cen_x < cen_x - BUFFER)
        turnAngle(serPort, avel, -5);
    end
    
    if (new_cen_x > cen_x + BUFFER)
        turnAngle(serPort, avel, 5);
    end
    SetFwdVelAngVelCreate(serPort, vel, 0);
    pause(tick)
end
    
imshow(binary);