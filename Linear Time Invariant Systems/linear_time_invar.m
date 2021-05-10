clear
clc
a = [0 1
    4.1 * 10e4 0];
b = [0
    -9.12i];
c = [1 0
    0 1];
d = [0
    0];
T = 0:0.01:10;        
U = zeros(size(T));   
X0 = [0.1 0.1];    
sys = ss(a,b,c,d);    
lsim(sys, U, T, X0)   