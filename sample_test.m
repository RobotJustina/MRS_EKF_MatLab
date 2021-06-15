close all
clear 

o_1 = [ 2; 2; 0 ];
o = [ 5; 5; .7 ];

x = [ 3; 3 ; 0];

n = 500;
puntos = zeros(2,n); 

mu=0;
sigma=2;

for i=1:n
  [xk, u ] = sampleOdometry(o_1,o,x);
  puntos(1,i) =  xk(1); %normrnd(mu,sigma);
  puntos(2,i) =  xk(2); %normrnd(mu,sigma-1.5);
end


plot( puntos(1,:) , puntos(2,:),'.');

xlim([0 10])
ylim([0 10])
pbaspect([1 1 1])
