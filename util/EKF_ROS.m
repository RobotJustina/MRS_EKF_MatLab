clear
rosshutdown
rosinit
sub_odom = rossubscriber("/odom", "nav_msgs/Odometry");
sub_rgb_image = rossubscriber("/kinect/rgb/image_raw", "sensor_msgs/Image");
pub_markers   = rospublisher("/landmark_markers", "geometry_msgs/PoseArray","DataFormat","struct");
pub_landmarks = rospublisher("/landmarks", "custom_msgs/Landmarks", "DataFormat", "struct");
pub_pose_ekf  = rospublisher("/localization_ekf", "geometry_msgs/PoseWithCovariance");
camera_info   = cameraIntrinsics(554.254691191187, [320.5, 240.5], [480,640]);
pause(1)

tftree = rostf;
pause(.2)

initPose = [3;6.5;-1.57];
 
tform = rosmessage('geometry_msgs/TransformStamped');
tform.ChildFrameId = 'odom';
tform.Header.FrameId = 'map';



alpha= [ 0.02 0.01 0.02 0.01 ];

Q=  [0.000025, 0,    0;    
           0,    0.000025, 0;    
           0,    0,    0.0002 ];
R =[0.009, 0,      0;    
           0,      0.0009, 0;    
           0,      0,      0.001 ];


sigma = [0 0 0;  %Covariance matrix start with all elements zero
         0 0 0; 
         0 0 0];
     
landmak_coordinates= [    
    6.8900   10.0000 1.0;
    6.8900    8.0000 2.0;
    6.8900    6.0000 3.0;
    6.8900    3.3000 4.0;
    4.9500    2.8200 5.0;
    4.4800    2.3800 6.0;
    3.4900    1.8200 7.0;
    1.4500    1.8200 8.0;
    0.9700    9.8000 9.0;
    0.9700    7.6000 10.0;
    0.9700    5.4000 11.0;
    0.9700    3.2000 12.0;
    5.8000   11.0000 13.0;
    3.8000   11.0000 14.0;
    1.8000   11.0000 15.0;];
                    

once = true;

while true
    msg_markers   = rosmessage(pub_markers);
    msg_landmarks = rosmessage(pub_landmarks);
    msg_markers.Header.FrameId = 'kinect_link';
    [id,loc,pose] = readAprilTag( imnoise((readImage(receive(sub_rgb_image,1))),'gaussian',.6) ,"tag16h5", camera_info, 0.4);  
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

        if pose(i).Translation(3) > 2 && pose(i).Translation(3)< 1.4 
           continue; 
        end    
        detections(i,:) = [ msg_landmarks.Landmarks_(i).Distance msg_landmarks.Landmarks_(i).Angle double(msg_landmarks.Landmarks_(i).Id)   ];
        
        
    end
    
    odom = receive(sub_odom,1);
    quat = [odom.Pose.Pose.Orientation.W() odom.Pose.Pose.Orientation.X() odom.Pose.Pose.Orientation.Y()  odom.Pose.Pose.Orientation.Z()  ];
    eu = quat2eul(quat);
   
            
            
    if(once)
        once = false;

        o_1 = [ odom.Pose.Pose.Position.X;
                odom.Pose.Pose.Position.Y;
                eu(1)  ];
        quat_1 = quat;
        sv = [ 3.0
               6.5
               -1.57  ];
         
        msg_pose_ekf   = rosmessage(pub_pose_ekf);
    
        
        
        msg_pose_ekf.Pose.Position.X = 3;
        msg_pose_ekf.Pose.Position.Y = 6.5;
        
        q = angle2quat( initPose(3),0,0);
        
        msg_pose_ekf.Pose.Orientation.X = q(2);
        msg_pose_ekf.Pose.Orientation.Y = q(3);
        msg_pose_ekf.Pose.Orientation.Z = q(4);
        msg_pose_ekf.Pose.Orientation.W = q(1);
        

        
        
        
       inittThetaQuat =  quatmultiply(quatconj(q),quat);
       initTheta =   quat2eul( inittThetaQuat );
       initTheta = initTheta(1);
       %initTheta= normalizeAngle(normalizeAngle( initPose(3) )-normalizeAngle(eu(1)));
       
       
       
       tform.Transform.Translation.X =  initPose(1)- odom.Pose.Pose.Position.X *cos(initTheta) + odom.Pose.Pose.Position.Y * sin(initTheta);
       tform.Transform.Translation.Y =  initPose(2) - odom.Pose.Pose.Position.X*sin(initTheta) - odom.Pose.Pose.Position.Y * cos(initTheta);
       tform.Transform.Translation.Z = 0;
       
       %inittThetaQuat =  angle2quat(initTheta,0,0);
       
       tform.Transform.Rotation.X = inittThetaQuat(2);
       tform.Transform.Rotation.Y = inittThetaQuat(3);
       tform.Transform.Rotation.Z = inittThetaQuat(4);
       tform.Transform.Rotation.W = inittThetaQuat(1);
        
       send(pub_pose_ekf  , msg_pose_ekf);
         
    end
    
    
    distance = sqrt( (o_1(1)-odom.Pose.Pose.Position.X)^2 +  (o_1(2)-odom.Pose.Pose.Position.Y)^2  );
    aux = quat2eul(quatmultiply(quatconj(quat_1),quat));
    spin =   abs(aux(1));
    
    if distance >.1 || spin > pi/90 && ~isempty(id) && ~isempty(detections) 
        
        detections
        
        o = [ odom.Pose.Pose.Position.X;
                odom.Pose.Pose.Position.Y;
               eu(1) ];
            
        [sv,sigma] = ekf_differential(o_1,o,sv,alpha,sigma,detections,landmak_coordinates,Q,R );
    
        
         msg_pose_ekf   = rosmessage(pub_pose_ekf);
    
        msg_pose_ekf.Pose.Position.X = sv(1);
        msg_pose_ekf.Pose.Position.Y = sv(2);
        
        q = angle2quat(sv(3),0, 0);
        msg_pose_ekf.Pose.Orientation.X = q(2);
        msg_pose_ekf.Pose.Orientation.Y = q(3);
        msg_pose_ekf.Pose.Orientation.Z = q(4);
        msg_pose_ekf.Pose.Orientation.W = q(1);
        send(pub_pose_ekf  , msg_pose_ekf);
    
        
        
        
        
       inittThetaQuat =  quatmultiply(quatconj(q),quat);
       initTheta =   quat2eul( inittThetaQuat );
       initTheta = initTheta(1);

       
       tform.Transform.Translation.X =  sv(1)- odom.Pose.Pose.Position.X *cos(initTheta) + odom.Pose.Pose.Position.Y * sin(initTheta);
       tform.Transform.Translation.Y =  sv(2) - odom.Pose.Pose.Position.X*sin(initTheta) - odom.Pose.Pose.Position.Y * cos(initTheta);
       tform.Transform.Translation.Z = 0;
       
       
       tform.Transform.Rotation.X = inittThetaQuat(2);
       tform.Transform.Rotation.Y = inittThetaQuat(3);
       tform.Transform.Rotation.Z = inittThetaQuat(4);
       tform.Transform.Rotation.W = inittThetaQuat(1);
        
        
        
        
        
       
        error_ellipse(sigma(1:2,1:2), sv(1:2) );
         
        o_1 = o; 
        quat_1 = quat;
    end

    
    
    tform.Header.Stamp = rostime("now") ;
    sendTransform(tftree,tform);
        
    send(pub_markers  , msg_markers);
    send(pub_landmarks, msg_landmarks);
    
    drawnow
    pause(0.1);
    
end
    
    


