%% Model - 

% [x_dot; x_dot_dot] = [0 1; 4.1*10^4 0][x; x_dot] + [0; -9.12]i
% y = [1 0;0 1][x; x_dot] + [0; 0]i
% To get LQR controller -
%% Initial things

clear all
close all
clc

%% System Declaration

A = [0, 1; 4.1*10^4, 0];
B = [0; -9.12];
C = [1 0;0 1];
D = [0; 0];


%% Check for Stability 

Eigenvalues_of_A = eig(A)
Controllability_matrix = ctrb(A,B)
Controllability_rank = rank(Controllability_matrix)
Observability_matrix = obsv(A,C)
Observability_rank = rank(Observability_matrix)

%% System Model without stability

sys1 = ss(A,B,C,D);
t = 0:0.01:4;
step(sys1, t);

%% LQR
N = 0;
Q = eye(2)*3;
R = 10;

[Kc, sc, ec] = lqr(A,B,Q,R,N);

%% Kalman Filter

Q_k = eye(2);
R_k = eye(2);

Kf = (lqr(A',C',Q_k,R_k))';

sysKF = ss(A-Kf*C, [B Kf], eye(2), 0*[B Kf]);


%% Luenberger Params
eig_Kc = eig(A-B*Kc);
eig_Kc = eig_Kc * 10;
eig_Kc = eig_Kc';

L = place(A',C', eig_Kc);


