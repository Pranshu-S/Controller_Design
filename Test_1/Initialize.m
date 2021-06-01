%% Proposal - 

% x_dot = [2,3;-1,4]x + [0;1]u
% y = [1,0]x
% To get LQR controller -
%% Initial things

clear all
close all
clc

%% System Declaration

A = [2,3;-1,4];
B = [0;1];
C = [1,0];
D = 0;


%% Check for Stability 
Eigenvalues_of_A = eig(A)
Controllability_matrix = ctrb(A,B)
Controllability_rank = rank(Controllability_matrix)
Observability_matrix = obsv(A,C)
Observability_rank = rank(Observability_matrix)

%% LQR
% 
Q = [0.5, 0; 0, 0.2];
R = 0.5;
K = lqr(A,B,Q,R);
G = C*inv(A-B*K)*B
% G = -inv(C*inv(A-B*K)*B);

