function [pthObj, solnInfo] = getPathRRT(map,initPose,goalPose,MinTurningRadius,ValidationDistance,MaxConnectionDistance,MaxIterations)
%GETPATHRRT Summary of this function goes here
%   Detailed explanation goes here

bounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = MinTurningRadius;
stateValidator = validatorOccupancyMap(ss); 
stateValidator.Map = map;
stateValidator.ValidationDistance = ValidationDistance; %Planner cheks if there is an obstacle between this distance in the path.
planner = plannerRRTStar(ss, stateValidator);
planner.MaxConnectionDistance = MaxConnectionDistance;
planner.MaxIterations = MaxIterations;
planner.GoalReachedFcn = @exampleHelperCheckIfGoal;
rng(1,'twister')
[pthObj, solnInfo] = plan(planner, initPose.', goalPose.');

    function isReached = exampleHelperCheckIfGoal(planner, goalState, newState)
        isReached = false;
        threshold = 0.1;
        if planner.StateSpace.distance(newState, goalState) < threshold
            isReached = true;
        end
    end

end


