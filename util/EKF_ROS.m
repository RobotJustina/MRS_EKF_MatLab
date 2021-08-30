clear
sub_odom = rossubscriber("/odom", "nav_msgs/Odometry");
sub_rgb_image = rossubscriber("/kinect/rgb/image_raw", "sensor_msgs/Image");
pub_markers   = rospublisher("/landmark_markers", "geometry_msgs/PoseArray","DataFormat","struct");
pub_landmarks = rospublisher("/landmarks", "custom_msgs/Landmarks", "DataFormat", "struct");
pub_pose_ekf  = rospublisher("/localization_ekf", "geometry_msgs/PoseWithCovariance");
camera_info   = cameraIntrinsics(554.254691191187, [320.5, 240.5], [480,640]);
pause(1)


alpha= [ 0.2 0.02 0.02 0.02 ];



Q=  [0.000025, 0,    0;    
           0,    0.000025, 0;    
           0,    0,    0.02 ];
R =[0.009, 0,      0;    
   0,      0.0009, 0;    
   0,      0,      0.0000001 ];


sigma = [0 0 0;  %Covariance matrix start with all elements zero
         0 0 0; 
         0 0 0];
     
landmak_coordinates= [ 10 10;
                       40 4;
                       30 20;
                       50 3;
                       58 13;
                       65  13;
                       65  30;
                       60 40;
                       86 40;
                       80 22; 
                       59 60;
                       35 50;
                       35 80;
                       50 65;
                       45 63;
                       2 80;
                       23 83];


once = true;

while true
    msg_markers   = rosmessage(pub_markers);
    msg_landmarks = rosmessage(pub_landmarks);
    msg_markers.Header.FrameId = 'kinect_link';
    [id,loc,pose] = readAprilTag(readImage(receive(sub_rgb_image,1)), camera_info, 0.4);  
    detections = zeros(length(id),3);
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
        
        detections(i,:) = [ msg_landmarks.Landmarks_(i).Distance msg_landmarks.Landmarks_(i).Angle msg_landmarks.Landmarks_(i).Id   ];
        
        
    end
    
    odom = receive(sub_odom,1);
    quat = [odom.Pose.Pose.Orientation.X() odom.Pose.Pose.Orientation.Y()  odom.Pose.Pose.Orientation.Z()  odom.Pose.Pose.Orientation.W()];
    eu = quat2eul(quat);
   
            
            
    if(once)
        once = false;
        
         o_1 = [ odom.Pose.Pose.Position.X;
                odom.Pose.Pose.Position.Y;
                eu(1)  ]; %Yaw :o is correct 1  not 3
         quat_1 = quat;
         
         sv = o_1;
    end
    
    
    distance = sqrt( (o_1(1)-odom.Pose.Pose.Position.X)^2 +  (o_1(2)-odom.Pose.Pose.Position.Y)^2  )
    aux = quat2eul(quatmultiply(quatconj(quat_1),quat));
    spin =   abs(aux(1));
    
    if distance >.1 || spin > pi/6 && ~isempty(id)
        
        o = [ odom.Pose.Pose.Position.X;
                odom.Pose.Pose.Position.Y;
               eu(1) ];
            
        [sv,sigma] = ekf_differential(o_1,o,sv,alpha,sigma,detections,landmak_coordinates,Q,R );
    
        
         msg_pose_ekf   = rosmessage(pub_pose_ekf);
    
        msg_pose_ekf.Pose.Position.X = sv(1);
        msg_pose_ekf.Pose.Position.Y = sv(2);
        
        q = angle2quat(sv(3), 0, 0);
        msg_pose_ekf.Pose.Orientation.X = q(1);
        msg_pose_ekf.Pose.Orientation.Y = q(2);
        msg_pose_ekf.Pose.Orientation.Z = q(3);
        msg_pose_ekf.Pose.Orientation.W = q(4);
        send(pub_pose_ekf  , msg_pose_ekf);
    
        
        
        
        o_1 = o; 
        quat_1 = quat;
    end
    
   
    
    
    send(pub_markers  , msg_markers);
    send(pub_landmarks, msg_landmarks);
    
    drawnow
    pause(0.1);
end
    
    
