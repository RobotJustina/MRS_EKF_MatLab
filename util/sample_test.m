
start = [ 2; 2; 0 ];
endd = [ 5; 5; .7 ];
x = [ 3; 3 ; 0];


alpha= [ 30.2
          0.0001
          0.001
          0.00001 ];

alpha= [ 0.002 0.0001 0.601 0.0001 ];

n = 500;
puntos = zeros(2,n); 

mu=0;
sigma=2;




for i=1:n
  [xk, u ] = sampleOdometry(odom_pose(:,idx-2),odom_pose(:,idx-1),svArray(:,idx-2),alpha) ;
  puntos(1,i) =  xk(1); %normrnd(mu,sigma);
  puntos(2,i) =  xk(2); %normrnd(mu,sigma-1.5);
end
r = 0.3;

plot( puntos(1,:) , puntos(2,:),'.');

hold on
plot(svArray(1,idx-2), svArray(2,idx-2), 'ro')
plot([svArray(1,idx-2), svArray(1,idx-2) + r*cos(svArray(3,idx-2))], [svArray(2,idx-2), svArray(2,idx-2) + r*sin(svArray(3,idx-3))], 'r-' )


