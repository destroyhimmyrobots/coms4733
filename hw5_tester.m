im = imread('highway.jpg');

[x, y] = imclick(im)
x = round(x);
y = round(y);
binary = thresh(im, x, y);

[cen_x, cen_y, size] = imstats(binary);

binary(cen_y-6:cen_y+6, cen_x-6:cen_x+6) = 0;

imshow(binary);