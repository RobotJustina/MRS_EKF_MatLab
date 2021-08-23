function [controller] = controllerPP(pthObj,LookaheadDistance,DesiredLinearVelocity,MaxAngularVelocity)
%CONTROLLERPUREPURSUIT Summary of this function goes here
%   Detailed explanation goes here

controller = controllerPurePursuit;
controller.Waypoints = [pthObj.States(:,1), pthObj.States(:,2) ]; %Waypoints are  the poses calculate by RRT algorithm  in this case We just use x and y 
controller.LookaheadDistance = LookaheadDistance;  %How far along the path the robot should look from the current location to compute the angular velocity commands
controller.DesiredLinearVelocity = DesiredLinearVelocity;
controller.MaxAngularVelocity = MaxAngularVelocity;

end

