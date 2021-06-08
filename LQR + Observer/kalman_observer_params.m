clc
close all
clear all


A = [0, 1; 4.1*10^4, 0];
B = [0; -9.12];
C = [1 0;0 1];
D = [0;0];


% Controller design
Q = eye(2)*100;
R = 0.00001;
M_a = rank(ctrb(A,B));
P = care(A,B,Q,R);
K_u  = inv(R)*B'*P;

% Sampling Time
dt = 1/1000;

% Observer design
Q_k = eye(2);
R_k = diag([0.00846698/4000,0.00846698/4000]);
P_k = care(A',C',Q_k,R_k);
K_k = P_k*C'*inv(R_k);
L = K_k;

% White noise characteristics
R_w = 0.00846698/4000*dt;

% Desired State
des_x = [0.0005; 0];
des_u = K_u*des_x;
