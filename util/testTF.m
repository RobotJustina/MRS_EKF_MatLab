rosshutdown
rosinit

tftree = rostf;
pause(2)


 
    tform = rosmessage('geometry_msgs/TransformStamped');
    tform.ChildFrameId = 'odom';
    tform.Header.FrameId = 'map';
    tform.Header.Stamp = rostime("now") ;
    tform.Transform.Translation.X = 1.0;
    tform.Transform.Rotation.W = 1.0;
    while true
        tform.Header.Stamp = rostime("now") ;
        sendTransform(tftree,tform);
        pause(0.1);

    end


