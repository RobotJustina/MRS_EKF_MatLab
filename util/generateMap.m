function [map]  = generateMap(path, treshold, scale)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


image = imread(path);

%Convert image into grayscale.
grayimage = rgb2gray(image);

%Threshold  image in order to get a binary image.
bwimage = grayimage < treshold;

%Convert image into  binaryOccupancyMap  with scale  (pixes/meter).
map = binaryOccupancyMap(bwimage,scale);


end

