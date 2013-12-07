% cv01.m
%
% Reading images, converting to gray-scale, smoothing, correlation, and
% edge detection.
%
clear all; close all; clc;

% Load in the image from file
color_img = imread('highway.jpg');

% Display the color image
figure;
imshow(color_img);
title('Color Image');

% Convert the color image to gray-scale
gray_img = rgb2gray(color_img);

% What are the dimensions of the image?
[height, width] = size(gray_img);

% Display the gray-scale image
figure;
imshow(gray_img);
title('Gray Image');

% To visualize the edges, it might useful to view the gray-scale image as a
% "3D Surface".  This is not a vision technique, but rather just to more
% easily visualize the changes in the luminance values:
xs = repmat([1:width], height, 1);
ys = repmat([1:height]', 1, width);
figure;
surf(xs, ys, double(gray_img));
colormap jet;
title('Gray Image as a Surface');

% Smooth the image and display it
%%filter_size = 11;
filter_size = 21;
gray_smoothed_gauss = imfilter(gray_img, fspecial('gaussian', [filter_size filter_size], 2.5));
gray_smoothed_avg = imfilter(gray_img, fspecial('average', [filter_size filter_size]));
figure;
subplot(1,2,1);
imshow(gray_smoothed_gauss);
title('Gaussian Smoothing');
subplot(1,2,2);
imshow(gray_smoothed_avg);
title('Average Smoothing');



%
% Testing Correlation
%

% Load in the template (and make sure it's gray-scale)
gray_template = rgb2gray(imread('numbers.jpg'));

% Display the template
figure;
imshow(gray_template);
title('Gray Template Image');

% What are the dimensions of this template?
[templ_height, templ_width] = size(gray_template);

% Compute normalized cross-correlation surface
corr_surface = normxcorr2(gray_template, gray_img);

% Display the surface
figure;
imagesc(corr_surface);
axis image;
colormap jet;
title('Normalized Cross-Correlation Surface');

% Find the peak (and if multiple instances, just take the first one)
[peak_y, peak_x] = find(corr_surface == max(corr_surface(:)));
peak_y = peak_y(1);
peak_x = peak_x(1);

% Account for the offset from the correlation operation.  We want this peak
% in terms of the original image (e.g., gray_img) coordinate frame.
peak_y = peak_y - templ_height;
peak_x = peak_x - templ_width;

% Show the position of the peak (e.g., the best matching location of the
% template in the image)
figure;
imagesc(gray_img);
axis image;
colormap gray;
hold on;
plot(peak_x, peak_y, 'or');
line([peak_x peak_x+templ_width peak_x+templ_width peak_x peak_x], ...
    [peak_y peak_y peak_y+templ_height peak_y+templ_height, peak_y], ...
    'Color', [0 0 1]);
title('Correlation Matching');


%
% Testing Edge Extraction
%

% Compute edges of gray-scale image using various methods
sobel_edges = edge(gray_img, 'sobel');
log_edges = edge(gray_img, 'log');
canny_edges = edge(gray_img, 'canny');
zerocross_edges = edge(gray_img, 'zerocross');

% Display the edges
figure;
subplot(2,2,1);
imshow(sobel_edges);
title('Sobel');
subplot(2,2,2);
imshow(log_edges);
title('Laplacian of Gaussian');
subplot(2,2,3);
imshow(canny_edges);
title('Canny');
subplot(2,2,4);
imshow(zerocross_edges);
title('Zero-Crossings');








