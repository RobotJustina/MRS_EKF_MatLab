function [data, info] = landmark
%Landmark gives an empty data for custom_msgs/Landmark
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'custom_msgs/Landmark';
[data.Distance, info.Distance] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Angle, info.Angle] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'custom_msgs/Landmark';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'distance';
info.MatPath{2} = 'angle';
info.MatPath{3} = 'id';
