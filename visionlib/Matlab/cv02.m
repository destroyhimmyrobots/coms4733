% cv02.m
%
% Corner extraction and matching
%
clear all; close all; clc;

%
% Harris
%

% Load in two images for feature extraction and matching and make sure they
% are gray-scale format.
im1 = imread('img00000.png');
im2 = imread('img00034.png');
if size(im1,3) > 1
    im1 = rgb2gray(im1);
end
if size(im2,3) > 1
    im2 = rgb2gray(im2);
end

% Store the dimensions of the two images
[h1,w1] = size(im1);
[h2,w2] = size(im2);

% Extract Harris corners
sigma = 1;
thresh = 250;
radius = 2;
disp = 0;
[cim1, r1, c1] = harris(im1, sigma, thresh, radius, disp);
[cim2, r2, c2] = harris(im2, sigma, thresh, radius, disp);

% Match the corners using Normalized Cross-Correlation of small windows
% around each feature
p1 = [r1 c1]';
p2 = [r2 c2]';
w = 11;  % <--- 11x11 windows
[m1, m2, p1ind, p2ind] = matchbycorrelation(im1, p1, im2, p2, w);


%
% Let's plot the matches
%
numMatches = size(m1,2);

% Display the images side-by-side so we can draw lines b/w the matches
display = uint8(zeros(max([h1,h2]), w1+w2));
display(1:h1,1:w1) = im1;
display(1:h2,w1+1:end) = im2;

figure;
imagesc(display);
hold on;
axis image;
colormap gray;
title('Harris Feature Matching');

for i=1:numMatches
    plot(m1(2,i), m1(1,i), 'or');
    plot(m2(2,i)+w1, m2(1,i), 'or');
    line([m1(2,i) m2(2,i)+w1], [m1(1,i) m2(1,i)], 'Color', [0 0 1]);
end


%
% SIFT
%
%%match('scene.pgm', 'book.pgm');
match('scene.pgm', 'basmati.pgm');
%%match('img00000.pgm', 'img00034.pgm');







