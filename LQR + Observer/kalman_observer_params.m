clc
close all
clear all


A = [0, 1; 4.1*10^4, 0];
B = [0; -9.12];
C = [1 0;0 1];
D = [0;0];


% Controller design
Q = eye(2)*3;
R = 3;
M_a = rank(ctrb(A,B));
P = care(A,B,Q,R);
K_u  = inv(R)*B'*P;

t = 0:0.001:5; 
dt = t(2) - t(1);
X(:,1) = [1;0];
y(:,1) = C*X;

% Observer design
Q_k = eye(2)*5;
R_k = eye(2)*0.00846698*100;
P_k = care(A',C',Q_k,R_k);
K_k = P_k*C'*inv(R_k);
L = K_k;

% X_hat(:,1) = [0;0];
% y_hat(:,1) = C*X_hat;
% for i = 2:length(t)
%     u = -K_u*X_hat(:,i-1);
%     
%     X(:,i) = X(:,i-1)  +dt * (A*X(:,i-1) + B*u);
%     y(:,i) = C*X(:,i) + sqrt(R_k)*randn(size(C,1),1);
% 
%     X_hat(:,i) = X_hat(:,i-1)  +dt * (A*X_hat(:,i-1) + B*u +L *(y(:,i-1)-y_hat(:,i-1)));
%     y_hat(:,i) = C*X_hat(:,i) ;
% end
% 
% 
% figure;
% subplot(2,1,1)
% plot(t,X(1,:),t,X_hat(1,:))
% legend('Actual','Estimate')
% xlabel('time')
% ylabel('Position')
% subplot(2,1,2)
% plot(t,X(2,:),t,X_hat(2,:))
% xlabel('time')
% ylabel('Velocity')