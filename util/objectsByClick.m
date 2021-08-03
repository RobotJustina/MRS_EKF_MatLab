

function [objects,colors,markers] = objectsByClick(map)
%OBJECTSBYCLICK This function generates colors,objects and markers
%based on map and clicks by user. THIS FUNCTION IS JUST FOR CLEAN CODE IN
%MRS_EKF_Matlab lessons
%   Detailed explanation goes here

show(map);
[x,y] = getpts();
object_coordinates = [x,y];

[objects,colors,markers] = objectsByCoordinates(object_coordinates);


end