function simulateLQG()
%Simulate the response to initial conditions
%This considers the Kalman Filter but does not explicitly add noise to the
%process and the measurement.
%It is meant for quick testing
M = 1000;
m1 = 100;
m2 = 100;
l1 = 20;
l2 = 10;
g = 9.81;

A = [0 1 0 0 0 0;0 0 -m1*g/M 0 -m2*g/M 0;0 0 0 1 0 0;0 0 -(M + m1)*g/(M*l1) 0 -m2*g/(M*l1) 0;0 0 0 0 0 1;0 0 -m1*g/(M*l2) 0 -(M + m2)*g/(M*l2) 0];
B = [0;1/M;0;1/(M*l1);0;1/(M*l2)];
C = [1 0 0 0 0 0];
%C = [1 0 0 0 0 0;0 0 1 0 0 0;0 0 0 0 1 0];
%C = [1 0 0 0 0 0;0 0 1 0 0 0];
D = 0;

%ctrb = [B A*B (A^2)*B (A^3)*B (A^4)*B (A^5)*B];

Q = C' * C;
% Q(1,1) = 900000000000;
% Q(3,3) = 800000000000000;
% Q(5,5) = 700000000000000;

Q(1,1) = 90000000;
Q(3,3) = 80000000000;
Q(5,5) = 70000000000;

R = 1;
[K,~,~] = lqr(A,B,Q,R);

%Get the Kalman State Estimator
sys_1 = ss(A,[B B],C,[zeros(1,1) zeros(1,1)]);
%sys_1 = ss(A,[B B],C,[zeros(3,1) zeros(3,1)]);
%sys_1 = ss(A,[B B],C,[zeros(2,1) zeros(2,1)]);

Rn = 10^-2 * eye(1);
Qn = 0.2;

% Rn = 10^-6 * eye(1);
% Qn = 20000;
sensors = [1];
%sensors = [1,2,3];
%sensors = [1,2];
known = [1];
[~,L,~] = kalman(sys_1,Qn,Rn,[],sensors,known)

states = {'x','x_dot','theta1','theta1_dot','theta2','theta2_dot','e_1','e_2','e_3','e_4','e_5','e_6'};
inputs = {'F'};
outputs = {'x'};
%outputs = {'x','theta_1','theta_2'};

Ac = [A-B*K B*K;zeros(size(A)) A-L*C];
Bc = zeros(12,1);
Cc = [C zeros(size(C))];
sys_cl = ss(Ac,Bc,Cc,D, 'statename',states,'inputname',inputs,'outputname',outputs);    

init_pos = [0.2,15*pi/180,20*pi/180];
%x0 = [init_pos(1);0;init_pos(2);0;init_pos(3);0;init_pos(1);0;0;0;0;0];
x0 = [init_pos(1);0;init_pos(2);0;init_pos(3);0;init_pos(1);0;0.001*init_pos(2);0;0.001*init_pos(3);0];
%x0 = [init_pos(1);0;init_pos(2);0;init_pos(3);0;init_pos(1);0;init_pos(2);0;init_pos(3);0];
t = 0:0.01:50;
F = zeros(size(t));
[Y,~,X] = lsim(sys_cl,F,t,x0);
%plot(t,Y(:,2),'g');
% hold on;
% plot(t,Y(:,3),'b');
figure
plot(t,Y(:,1),'r');
title('Response with LQG based control to inital conditions')
% 
u = zeros(size(t));
for i = 1:size(X,1)
   u(i) = K * (X(i,1:6))';
end
%Error between the estimate and the original
% figure
% subplot(3,1,1), plot(t,X(:,7))
% subplot(3,1,2), plot(t,X(:,9))
% subplot(3,1,3), plot(t,X(:,11))

Xhat = X(:,1) - X(:,7);
theta1hat = X(:,3) - X(:,9);
theta2hat = X(:,5) - X(:,11);

%Actual values of X and Xhat
figure
subplot(3,1,1), plot(t,Xhat), hold on, plot(t,X(:,1),'g')
subplot(3,1,2), plot(t,theta1hat), hold on, plot(t,X(:,3),'g')
subplot(3,1,3), plot(t,theta2hat), hold on, plot(t,X(:,5),'g')

figure
plot(t,u);
end
