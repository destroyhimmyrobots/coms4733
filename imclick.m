function [x y] = imclick(im)
    imshow(im);
    [x y] = ginput;
end