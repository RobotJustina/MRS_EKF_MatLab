function [sv,sigma] = ekf_differential(o_1,o,x,alpha,sigma,detections,objects,Q,RR )
             
   [sv, u] = sampleOdometry(o_1,o,x,alpha);
             
    G = [ 1 0  -u(2)*sin(u(1));
          0 1   u(2)*cos(u(1));
          0 0         1       ];
          
    sigma =  G*sigma*G.' + Q;

    for i = 1:size(detections,1)
        
        %Real measurement
         z = [ detections(i,1);
               detections(i,2) ;
               0                 ];
          
        id_landmark = -1;  
        %Get landmark id
        for j = 1:size(objects,1)
            if objects(j,3) == detections(i,3)
                id_landmark = j;
            end
        end
        
        if id_landmark == -1
           continue; 
        end
        if detections(i,1)>5 ||detections(i,2) > 1.57  ||detections(i,2) < -1.57 
           continue; 
        end
        
       
        dist = power(objects(id_landmark,1) - sv(1), 2) + power(objects(id_landmark,2) - sv(2), 2); 
        
        %Computing expected sensor readings
        z_hat = [ sqrt(dist);
                  normalizeAngle(  atan2(objects(id_landmark,2) - sv(2), objects(id_landmark,1) - sv(1) ) - sv(3) ) ;
                  0];
              
        % Jacobian of function h(sv,S)
        H = [ -( objects(id_landmark,1) - sv(1) ) / sqrt(dist),  -( objects(id_landmark,2) - sv(2) ) / sqrt(dist), 0;
            ( objects(id_landmark,2) - sv(2)) / dist, -( objects(id_landmark,1) - sv(1) ) / dist,-1;
			 	  0,0,0];
        
        
       %Computing Kalman gain
       K =  (sigma* (H.')) /((H*sigma*(H.')) + RR);
      
       
       %Update state vector (sv) and covariance matrix (sigma)
       sv = sv +  (K*(z-z_hat));
       sigma = (eye(3) - (K*H)  ) * sigma;
       
       sv(3) = normalizeAngle(sv(3));
        
    end
end

