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

%% LQR

Q = eye(2);
R = 1;
K = lqr(A,B,Q,R);
G = -inv(C*inv(A-B*K)*B);

