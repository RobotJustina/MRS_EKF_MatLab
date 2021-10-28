%This code is the same as R_2_EKF_ROSBAG_PRACTICE live script but without instructions
%for student practice.
clear
rosshutdown
rosinit
sub_odom = rossubscriber("/odom", "nav_msgs/Odometry");
sub_odom_ground = rossubscriber("/odom_ground_truth", "nav_msgs/Odometry");
sub_landmarks = rossubscriber("/landmarks", "custom_msgs/Landmarks");

pause(1)

pause(.2)
initPose = [ 0 0 0];


alpha= [ 0.02 0.01 0.02 0.01 ];

Q=  [0.000025, 0,    0;    
           0,    0.000025, 0;    
           0,    0,    0.0002 ];
R =[0.09, 0,      0;    
           0,      0.009, 0;    
           0,      0,      0.001 ];


sigma = [0 0 0;  %Covariance matrix start with all elements zero
         0 0 0; 
         0 0 0];

     
landmak_coordinates=[
	2.6501  2.2012 1.0  ;
   -0.8888 -0.9584 8.0  ;
   -0.6514  1.2021 14.0 ;
    4.5925  2.2038 17.0 ;
    5.0717  0.1753 18.0 ;
    5.0740 -2.5365 19.0 ;
    5.0730 -5.2918 20.0 ;
    5.0750 -6.2680 701.0;
   -0.8947  0.8557 702.0;
   -0.8962 -2.2954 703.0;
    4.2305 -6.9905 704.0;
    1.5000  1.7934 705.0;
    1.9894  2.2036 706.0;
   -0.8966 -5.1471 717.0;
   -0.1681 -7.0005 718.0;
    0.7395 -6.9997 719.0;
    3.1140 -6.9984 720.0;
];
     
odom_pose = zeros(3,1000);
odom_pose(:,1) = initPose;

odom_pose_ground = zeros(3,1000);
odom_pose_ground(:,1) = initPose;

ekf_pose = zeros(3,1000);
ekf_pose(:,1) = initPose;


once = true;

count = 1;
count2=1;
while true
   
    
    msg_landmarks = receive(sub_landmarks,1);
    detections = zeros(length(msg_landmarks),3);
    it = 1;
    if ~isempty(msg_landmarks.Landmarks_) 
        for i=1:length(msg_landmarks.Landmarks_)

            detections(i,:) = [ msg_landmarks.Landmarks_(i).Distance msg_landmarks.Landmarks_(i).Angle msg_landmarks.Landmarks_(i).Id ];

        end
    end
    
    odom_ground = receive(sub_odom_ground,1); 
    odom = receive(sub_odom,1); 
    
    quat = [odom.Pose.Pose.Orientation.W() odom.Pose.Pose.Orientation.X() odom.Pose.Pose.Orientation.Y()  odom.Pose.Pose.Orientation.Z()  ];
    eu = quat2eul(quat);
    
    odom_pose(1,count) =  odom.Pose.Pose.Position.X ;
    odom_pose(2,count) =  odom.Pose.Pose.Position.Y ;
    odom_pose(3,count) =  eu(1) ;
    
    odom_pose_ground(1,count) =  odom_ground.Pose.Pose.Position.X ;
    odom_pose_ground(2,count) =  odom_ground.Pose.Pose.Position.Y ;
    odom_pose_ground(3,count) =  eu(1) ;
    
    
    count = count +1;
    if(once)
        once = false;
        o_1 = [ odom.Pose.Pose.Position.X;
                odom.Pose.Pose.Position.Y;
                eu(1)  ];
        initPose=o_1;
        quat_1 = quat;
        sv = initPose;  
    end
    
    %Variables to check how long the robot has moved or turned.
    distance = sqrt( (o_1(1)-odom.Pose.Pose.Position.X)^2 +  (o_1(2)-odom.Pose.Pose.Position.Y)^2  );
    aux = quat2eul(quatmultiply(quatconj(quat_1),quat));
    spin =   abs(aux(1));
    
    if distance >.1 || spin > pi/90  && ~isempty(detections) 
        

        
        o = [ odom.Pose.Pose.Position.X;
              odom.Pose.Pose.Position.Y;
              eu(1) ];
            
        [sv,sigma] = ekf_differential(o_1,o,sv,alpha,sigma,detections,landmak_coordinates,Q,R );
        ekf_pose(1,count2)=sv(1);
        ekf_pose(2,count2)=sv(2);
        count2 = count2+1;
        
        o_1 = o; 
        quat_1 = quat;
    end
    

    
    hold on
    plot(odom_pose(1,1:count-1),odom_pose(2,1:count-1),'m');
    plot(odom_pose_ground(1,1:count-1),odom_pose_ground(2,1:count-1),'b');
    plot(ekf_pose(1,1:count2-1),ekf_pose(2,1:count2-1),'g');
    

    pause(0.1);
    
end
