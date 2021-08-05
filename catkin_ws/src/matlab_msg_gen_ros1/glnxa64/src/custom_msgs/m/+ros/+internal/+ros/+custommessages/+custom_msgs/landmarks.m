function [data, info] = landmarks
%Landmarks gives an empty data for custom_msgs/Landmarks
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'custom_msgs/Landmarks';
[data.Landmarks_, info.Landmarks_] = ros.internal.ros.custommessages.custom_msgs.landmark;
info.Landmarks_.MLdataType = 'struct';
info.Landmarks_.MaxLen = NaN;
info.Landmarks_.MinLen = 0;
data.Landmarks_ = data.Landmarks_([],1);
info.MessageType = 'custom_msgs/Landmarks';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'landmarks';
info.MatPath{2} = 'landmarks.distance';
info.MatPath{3} = 'landmarks.angle';
info.MatPath{4} = 'landmarks.id';
