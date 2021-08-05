function [objects,colors,markers] = objectsByCoordinates(object_coordinates)
%objectsByCoordinates This function generates colors,objects and markers
%based on object_coordinates. THIS FUNCTION IS JUST FOR CLEAN CODE IN
%MRS_EKF_Matlab lessons
%   Detailed explanation goes here

[n, ~] = size(object_coordinates);
colors = zeros(n,3);  %Matrix to store each color marker
objects = zeros(n,3); %Matrix to store cordinates and  a numeric id
markers=''; %String where you defins the shape by a Char 


for i=1:n
    %Set x,y and id for each object
    objects(i,1) = object_coordinates(i,1);
    objects(i,2) = object_coordinates(i,2);
    objects(i,3) = i;
    %Set color for each object
    colors(i,1) = .3;
    colors(i,2) = .8;
    colors(i,3) = .5;
    %Set marker for each object
    markers = strcat(markers,'s');
end


end

