function [sampleTime,tVec,r] = simulTime(sampleTime,executeTime)
%SIMULTIME Compute a vector time based on sampleTime and executeTime
%parameters. It returns sampleTime, tVec, and rate control. 
%   Detailed explanation goes here


tVec = 0:sampleTime:executeTime;     % Time array  in this example te execution will run for 3 seconds
r = rateControl(1/sampleTime);

end

