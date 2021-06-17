close all
clear 
%%%%% ROBOT SETUP %%%%%%


    % Robot configuration
    R = 0.1;   % Wheels [m]
    L = 0.5;   % Base [m]
    dd = DifferentialDrive(R,L);

    
%%%%% SENSORS SETUP %%%%%


    % Create object Detector sensor
    detector = ObjectDetector;
    detector.fieldOfView = pi;
    detector.maxRange = 15; %[m]        
    detector.maxDetections = 25;
    
    % Create lidar sensor
    lidar = LidarSensor;
    lidar.sensorOffset = [0,0];
    lidar.scanAngles = linspace(-pi/2,pi/2,51);
    lidar.maxRange = 5;

    

%%%%% FIELD SETUP %%%%%

    % Initial position
    initPose = [2;2;1.5707];            % xy theta
    
    % Lead image as map
    image = imread('C:\Users\Diego\Pictures\wordx.png');
    grayimage = rgb2gray(imresize(image, 1.0));
    bwimage = grayimage < 0.5;
    map = binaryOccupancyMap(bwimage);

    %objects (as landmarks) in the field
    % Create objects and colors 
    colors = zeros(9,3);
    objects = zeros(9,3);
    cta = 1;
    for row=1:3
        for col=1:3
           objects(cta,1) = row*3;
           objects(cta,2) = col*3;
           objects(cta,3) = cta;
           colors(cta,1) = .3;
           colors(cta,2) = .8;
           colors(cta,3) = .5;
           cta = cta +1;
        end
    end

    % Waypoints
    waypoints = [2 10;
                 2 17
                 8 19;
                 11 5;
                 27 15];

    % Create visualizer
    viz = Visualizer2D;
    viz.hasWaypoints = true;
    viz.mapName = 'map';
    
    %Atach sensors
    attachLidarSensor(viz,lidar);
    attachObjectDetector(viz,detector);
    
    % Colores de los objetos
    viz.objectColors = colors;
    viz.objectMarkers = 'so<sd^<>phpo<sd^<>phpo<sd';
    
    
%%%%% ALGORITHMS SETUP

    % Pure Pursuit Controller
    controller = controllerPurePursuit;
    controller.Waypoints = waypoints;
    controller.LookaheadDistance = 0.5;
    controller.DesiredLinearVelocity = 0.75;
    controller.MaxAngularVelocity = 1.5;

    % Vector Field Histogram (VFH) for obstacle avoidance
    vfh = controllerVFH;
    vfh.DistanceLimits = [0.05 3];
    vfh.NumAngularSectors = 36;
    vfh.HistogramThresholds = [5 10];
    vfh.RobotRadius = L;
    vfh.SafetyDistance = L;
    vfh.MinTurningRadius = 0.25;


    %%% EKF
    
    varianceV= .3; % .03
    meanValueV = 0;
    
    varianceW= .1; % .02
    meanValueW = 0;
    
    sv = initPose; %State vector initialization
    
    sigma = [0 0 0; 0 0 0; 0 0 0];
    
    Q = [0.000001   0.00  0.000; 
         0.000  0.000001   0.000; 
         0.000 0.000 0.00001];
     
     
    R = [0.00000001 0 0;
         0 0.000000001 0;
         0 0 0.00000000001];
    
    

 %%% SIMULATION SETUP
 
    % Sample time and time array
    sampleTime = 0.1;              % Sample time [s]
    tVec = 0:sampleTime:70;        % Time array
    r = rateControl(1/sampleTime);

    %Ideal Pose 
    pose = zeros(3,numel(tVec));   % Pose matrix
    pose(:,1) = initPose;   % set first pose as the initial position

    %Pose with added noise
    poseWithNoise = zeros(3,numel(tVec)); 
    poseWithNoise(:,1) = initPose;  %Same initial pose
    
    
    %Kalman Pose 
    poseKalman = zeros(3,numel(tVec));   % Pose matrix
    poseKalman(:,1) = sv;   % set first pose as the initial position
    
    
    
    
    
  %%% SIMULATION
 eFig1 = figure;
 eAxes1 = gca;
 ePlot1 = plot(NaN,NaN,'r');
 title('Error in axis Y');
 grid on
 
 eFig2 = figure;
 eAxes2 = gca;
 ePlot2 = plot(NaN,NaN,'g');
 title('Error in axis X');
 grid on
 
 eFig3 = figure;
 eAxes3 = gca;
 ePlot3 = plot(NaN,NaN,'b');
 title('Numbers of LandMarks');
 grid on
 
 numberDetections = zeros(1,numel(tVec)); 
 

idx = 0;


n=1;
for idx = 2:numel(tVec) 
    
    %Update pose
    curPoseFake = pose(:,idx-1);
    curPoseReal = poseWithNoise(:,idx-1);
    curPoseKalman = poseKalman(:,idx-1);
    
    
    % Get the sensor readings from the real pose
    ranges = lidar(curPoseReal);
        
    % Run the path following and obstacle avoidance algorithms
    
    wp = curPoseKalman;
    %wp = curPoseFake;
    
    [vRef,wRef,lookAheadPt] = controller(wp);
    targetDir = atan2(lookAheadPt(2)-wp(2),lookAheadPt(1)-wp(1)) - wp(3);
    steerDir = vfh(ranges,lidar.scanAngles,targetDir);    
    if ~isnan(steerDir) && abs(steerDir-targetDir) > 0.1
        wRef = 0.5*steerDir;
    end
    
    
    velBFake = [vRef;0;wRef];                   % Body velocities [vx;vy;w]
    
    %%Add Noise
    vRef = vRef + normrnd(0,varianceV); % abs((sqrt(varianceV)*randn(size(vRef)) + meanValueV));
    wRef = wRef + normrnd(0,varianceW); %(sqrt(varianceW)*randn(size(vRef)) + meanValueW/2);
    
    % Control the robot
    velBReal = [vRef;0;wRef];                   % Body velocities [vx;vy;w]
    
    velFake = bodyToWorld(velBFake,curPoseFake);  % Convert from body to world
    velReal = bodyToWorld(velBReal,curPoseReal);  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = curPoseFake + velFake*sampleTime; 
    poseWithNoise(:,idx) = curPoseReal + velReal*sampleTime; 
    
    
    
    
    % Update visualization
    
    % Update object detector and visualization
    detections = detector(poseWithNoise(:,idx),objects);
    viz(poseWithNoise(:,idx),waypoints,ranges,objects);
    
    detections
    
    % EKF
    
    %Prediction
    
    tetha_1 = sv(3);
    [sv, u] = sampleOdometry(pose(:,idx-1),pose(:,idx),sv);
    
%     g1 = (( ( pose(1,idx-1) - pose(1,idx) ) * cos(tetha_1+ u(2)) )/ sqrt( power( pose(1,idx-1) - pose(1,idx),2)+power( pose(2,idx-1) - pose(2,idx),2 )))+1;
%     g2 = ( ( pose(2,idx-1) - pose(2,idx) ) * cos(tetha_1+ u(2)) )/ sqrt( power( pose(1,idx-1) - pose(1,idx),2)+power( pose(2,idx-1) - pose(2,idx),2 ) );
%     g3 = -sqrt( power( pose(1,idx-1) - pose(1,idx),2)+power( pose(2,idx-1) - pose(2,idx),2 )) *sin(tetha_1+ u(2));
%     
%     g4 = (( ( pose(1,idx-1) - pose(1,idx) ) * sin(tetha_1+ u(2)) )/ sqrt( power( pose(1,idx-1) - pose(1,idx),2)+power( pose(2,idx-1) - pose(2,idx),2 )))+1;
%     g5 = ( ( pose(2,idx-1) - pose(2,idx) ) * sin(tetha_1+ u(2)) )/ sqrt( power( pose(1,idx-1) - pose(1,idx),2)+power( pose(2,idx-1) - pose(2,idx),2 ));
%     g6 = sqrt( power( pose(1,idx-1) - pose(1,idx),2)+power( pose(2,idx-1) - pose(2,idx),2 )) *cos(tetha_1+ u(2));
%     
%     
%     G = [ g1 g2 g3;
%           g4 g5 g6
%           0 0   1    ];
      
    G = [ 1 0 -u(1)*sin(u(2));
          0 1  u(1)*cos(u(2));
          0 0         1       ];
  
    sigma =  G*sigma*G.' + Q;
    %End prediction
    if size(detections,1) == 0
        ro=0;
    end
    %Actualization
    for i = 1:size(detections,1)
        
        %%Real measurement
        
         z = [ detections(i,1);
              detections(i,2) ;
              0];
          
%         z = [ sqrt(power(detections(i,1), 2) + power(detections(i,2), 2));
%               normalizeAngle(atan2(detections(i,2), detections(i,1))) ;
%               0];
        
      
       
       
        for j = 1:size(objects,1)
            if objects(j,3) == detections(i,3)
                 
                id_landmark = j;
            end
        end
        
        dist = power(objects(id_landmark,1) - sv(1), 2) + power(objects(id_landmark,2) - sv(2), 2); % Radicando de la de la distancia entre un landmark y el robot 
        
        z_hat = [ sqrt(dist);
                  normalizeAngle(  atan2(objects(id_landmark,2) - sv(2), objects(id_landmark,1) - sv(1) ) - sv(3) ) ;
                  0];
              
        
        H = [ -( objects(id_landmark,1) - sv(1) ) / sqrt(dist),  -( objects(id_landmark,2) - sv(2) ) / sqrt(dist), 0;
            ( objects(id_landmark,2) - sv(2)) / dist, -( objects(id_landmark,1) - sv(1) ) / dist,-1;
			 	  0,0,0];
        
       L = (H*sigma*(H.')) + R;
       K =  (sigma* (H.')) /(L);
       V = z-z_hat;
       
%        if i ==2
%            break;
%        end
%        kv=(K*V);
%        if abs(kv(1)) > .7  ||  abs(kv(2)) > .7   || abs(kv(3)) > .05
%         continue;
%        end
%             KK=K;
%             KK(3,:)= [0 0 0];
%              sv = sv +  (KK*V);
%        else 
%            sv = sv +  (K*V);
%        end
       sv = sv +  (K*V);
       sigma = (eye(3) - (K*H)  ) * sigma;
       sv(3) = normalizeAngle(sv(3));
        
    end
    
    poseKalman(:,idx) = sv;
    hold on;
    plot( poseKalman(1,1:idx), poseKalman(2,1:idx),'g');
    
    numberDetections(idx) = size(detections,1);
    set(ePlot1,'XData',tVec,'YData',abs(poseKalman(1,:) - poseWithNoise(1,:)) );
    set(ePlot2,'XData',tVec,'YData',abs(poseKalman(3,:) - pose(3,:)) );
    set(ePlot3,'XData',tVec,'YData',numberDetections );
    
    
    waitfor(r);
    if n==1
        pause(0)
        n=2;
    end
end

% Update object detector and visualization
viz(poseWithNoise(:,idx),waypoints,ranges,objects);
hold on

%% show path with noise and ideal path 
plot(pose(1,:),pose(2,:),'m');
plot(poseWithNoise(1,:),poseWithNoise(2,:),'k');
plot( poseKalman(1,:), poseKalman(2,:),'g');
