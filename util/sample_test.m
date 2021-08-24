close all
clear 

start = [ 2; 2; 0 ];
endd = [ 5; 5; .7 ];
x = [ 3; 3 ; 0];


alpha= [ 0.02
          0.0001
          0.001
          0.00001 ];


n = 500;
puntos = zeros(2,n); 

mu=0;
sigma=2;




for i=1:n
  [xk, u ] = sampleOdometry(start,endd,x,alpha);
  puntos(1,i) =  xk(1); %normrnd(mu,sigma);
  puntos(2,i) =  xk(2); %normrnd(mu,sigma-1.5);
end
r = 0.3;

plot( puntos(1,:) , puntos(2,:),'.');

hold on
plot(x(1), x(2), 'ro')
plot([x(1), x(1) + r*cos(x(3))], [x(2), x(2) + r*sin(x(3))], 'r-' )


