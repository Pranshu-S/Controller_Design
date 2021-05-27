%% Learning the Kalman-Bucy Filter in Simulink
% Examples to run the Simulink model kalmanbucy in the command window.
% By Yi Cao at Cranfield University on 28 January 2008
%% Example 2: A 2-input 1-output 4-state system
A = [-1.4576   -0.3369    1.0503    3.7815
      0.0979   -0.5998    0.2727    0.4077
      1.7212    0.1711   -4.5537    1.1045
     -3.5418   -0.3277   -1.7419   -0.9578];
B = [-0.1072         0
           0    0.1000
     -0.9640         0
           0    0.1500];
C = [0.1685   -0.9595   -0.0755   -0.3771];
D = [0 0];
% process noise variance
Q=diag([0.5^2 0.2^2 0.3^2 0.5^2]);
% measurment noise variance
R=1;
% initial state
x0 = [-1.7073
       0.2279
       0.6856
      -0.6368];
% Kalman-Bucy filter setting
% The same model
A1 =A;
B1 = B;
C1 = C;
D1 = D;
Q1 = Q;
R1 = R;
% zeros initial state estimate
x1 = zeros(4,1);
% initial covariance estimate
P1 = 10*eye(4);
% time span
tspan=0:0.1:100;
% input
u = randn(11,2);
% run the simulation        
[t,x,y1,y2,y3,y4]=sim('kalmanbucy',tspan,[],[(0:10:100)' u]);
% Display results
figure
set(gcf,'Position',[100 100 500 700])
for k=1:4
    subplot(4,1,k)
    plot(t,y1(:,k),'b',t,y3(:,k),'r','linewidth',2);
    legend('Actual state','Estimated state','Location','best')
    title(sprintf('state %i',k))
end
xlabel('time, s')

