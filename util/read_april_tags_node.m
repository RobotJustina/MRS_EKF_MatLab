clear
sub_img = rossubscriber("/kinect/rgb/image_raw", "sensor_msgs/Image");
pub_april = rospublisher("/recognized_april_tags", "geometry_msgs/PoseArray","DataFormat","struct");
pub_lands = rospublisher("/landmarks", "custom_msgs/Landmarks", "DataFormat", "struct");
cam_info = cameraIntrinsics(554.254691191187, [320.5, 240.5], [480,640]);
msg_april = rosmessage(pub_april);
msg_landmarks = rosmessage(pub_lands);
pause(1)
while true
    msg_april.Header.FrameId = 'kinect_link';
    msg_april.Poses = [];
    msg_landmarks.Landmarks = [];
    %msg_april.Header.Stamp = rostime("now");
    msg_img = receive(sub_img,1);
    img = readImage(msg_img);
    [id,loc,pose] = readAprilTag(img, cam_info, 0.4);
    for i=1:length(id)
        msg_april.Poses(i).Position.X = pose(i).Translation(1);
        msg_april.Poses(i).Position.Y = pose(i).Translation(2);
        msg_april.Poses(i).Position.Z = pose(i).Translation(3);
        msg_april.Poses(i).Orientation.X = pose(i).Rotation(1);
        msg_april.Poses(i).Orientation.Y = pose(i).Rotation(2);
        msg_april.Poses(i).Orientation.Z = pose(i).Rotation(3);
        msg_april.Poses(i).Orientation.W = pose(i).Rotation(4);        
        msg_landmarks.Landmarks(i).Distance = 1.0;
        msg_landmarks.Landmarks(i).Angle = 1.0;
        msg_landmarks.Landmarks(i).Id = 123;
    end
    send(pub_april, msg_april);
    send(pub_lands, msg_landmarks);
    drawnow
    pause(0.1);
end