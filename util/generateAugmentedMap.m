function [map, mapDilatation]  = generateAugmentedMap(path, treshold, scale,robotRadio)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


image = imread(path);

%Convert image into grayscale.
grayimage = rgb2gray(image);

%Threshold  image in order to get a binary image.
bwimage = grayimage < treshold;
map = binaryOccupancyMap(bwimage,scale);

%morphological dilation
se = strel('sphere',  ceil(robotRadio*scale)); %sphere where radio equals to robot radio (in pixels)
mapDilatation = imdilate(bwimage,se);
mapDilatation = binaryOccupancyMap(mapDilatation,10);



end

