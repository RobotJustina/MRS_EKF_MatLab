clear
sub_rgb_image = rossubscriber("/kinect/rgb/image_raw", "sensor_msgs/Image");
pub_markers   = rospublisher("/landmark_markers", "geometry_msgs/PoseArray","DataFormat","struct");
pub_landmarks = rospublisher("/landmarks", "custom_msgs/Landmarks", "DataFormat", "struct");
camera_info   = cameraIntrinsics(554.254691191187, [320.5, 240.5], [480,640]);
pause(1)
while true
    msg_markers   = rosmessage(pub_markers);
    msg_landmarks = rosmessage(pub_landmarks);
    msg_markers.Header.FrameId = 'kinect_link';
    [id,loc,pose] = readAprilTag(readImage(receive(sub_rgb_image,1)), camera_info, 0.4);    
    for i=1:length(id)
        msg_markers.Poses(i).Position.X = pose(i).Translation(1);
        msg_markers.Poses(i).Position.Y = pose(i).Translation(2);
        msg_markers.Poses(i).Position.Z = pose(i).Translation(3);
        msg_markers.Poses(i).Orientation.X = pose(i).Rotation(1);
        msg_markers.Poses(i).Orientation.Y = pose(i).Rotation(2);
        msg_markers.Poses(i).Orientation.Z = pose(i).Rotation(3);
        msg_markers.Poses(i).Orientation.W = pose(i).Rotation(4);        
        msg_landmarks.Landmarks_(i).Distance = norm(pose(i).Translation);
        msg_landmarks.Landmarks_(i).Angle = atan2(-pose(i).Translation(1), pose(i).Translation(3));
        msg_landmarks.Landmarks_(i).Id = uint32(id(i));
    end
    send(pub_markers  , msg_markers);
    send(pub_landmarks, msg_landmarks);
    drawnow
    pause(0.1);
end