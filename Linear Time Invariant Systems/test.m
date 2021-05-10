A = [-20 -40 -60
     1    0    0
     0    1    0];
B = [1
     0
     0];
C = [0   0   1];
D = 0;
T = 0:0.01:10;                 % simulation time = 10 seconds
U = ones(size(T));             % u = 1, a step input
sys = ss(A,B,C,D);             % construct a system model
[Y, Tsim, X] = lsim(sys,U,T);  % simulate
plot(Tsim,Y)                   % plot the output vs. time
title('Step Response with Zero Initial Conditions')