function [initPose,goalPose] = initGoalPosebyClick(map)
%INITGOALPOSEBYCLICK Summary of this function goes here
%   Detailed explanation goes here
show(map);
[xx,yy] = getpts();
[n,~] = size(xx);

initPose = [ xx(n-1); yy(n-1); 0]; % [x ; y ; theta]   
goalPose = [ xx(n); yy(n);0];


end

