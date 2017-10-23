function [A,B,C,D] = getParams()
%Utility function to get the system Matrices
M = 1000;
m1 = 100;
m2 = 100;
l1 = 20;
l2 = 10;
g = 9.81;

A = [0 1 0 0 0 0;0 0 -m1*g/M 0 -m2*g/M 0;0 0 0 1 0 0;0 0 -(M + m1)*g/(M*l1) 0 -m2*g/(M*l1) 0;0 0 0 0 0 1;0 0 -m1*g/(M*l2) 0 -(M + m2)*g/(M*l2) 0];
B = [0;1/M;0;1/(M*l1);0;1/(M*l2)];
C = [1 0 0 0 0 0;0 0 1 0 0 0;0 0 0 0 1 0];
D = 0;
end