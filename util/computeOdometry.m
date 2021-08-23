function [odom_pose] = computeOdometry(R,L,wR,wL,sampleTime ,lastOdom)
%COMPUTEODOMETRY Summary of this function goes here
%   Detailed explanation goes here


    d_sr = R * wR * sampleTime;
    d_sl = R * wL * sampleTime;
    d_s = (d_sr + d_sl)/2;
    d_theta = (d_sr - d_sl)/L;
    
    % Store each pose computed by odometry
    odom_pose = lastOdom + [ d_s *cos(lastOdom(3) +  d_theta/2 ) ; 
                             d_s *sin(lastOdom(3) +  d_theta/2 );
                             d_theta];  

end

