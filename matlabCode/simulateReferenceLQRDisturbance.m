function simulateReferenceLQRDisturbance()
M = 1000;
m1 = 100;
m2 = 100;
l1 = 20;
l2 = 10;
g = 9.81;

A = [0 1 0 0 0 0;0 0 -m1*g/M 0 -m2*g/M 0;0 0 0 1 0 0;0 0 -(M + m1)*g/(M*l1) 0 -m2*g/(M*l1) 0;0 0 0 0 0 1;0 0 -m1*g/(M*l2) 0 -(M + m2)*g/(M*l2) 0];
B = [0;1/M;0;1/(M*l1);0;1/(M*l2)];
C = [1 0 0 0 0 0];
D = 0;

%rank(ctrb(A_I,B_I))

R = 1;

Q = C' * C;
Q(1,1) = 90000000;
Q(3,3) = 80000000000;
Q(5,5) = 70000000000;

[K,~,~] = lqr(A,B,Q,R);

%Obtained through Pole Placement
%K = 1E8*[0.02 0.048 0.0067 -1.5575 -0.1825 0.3006];

states = {'x','x_dot','theta1','theta1_dot','theta2','theta2_dot'};
inputs = {'F','disturb'};
%outputs = {'x','theta1','theta2'};
outputs = {'x'};
sys_ss = ss(A,B,C,0);
Nbar = rscale(sys_ss,K);
sys_cl = ss(A - B * K, [B*Nbar B], C, D, 'statename',states,'inputname',inputs,'outputname',outputs);
%eig(A - B * K)
%Step Response
%step(sys_cl)

%Impulse Response
%impulse(sys_cl);

%x0 = [0,0,0,0,0,0,0];
t = 0:0.01:50;
F = 10 * ones(size(t,2),1);
disturb = 5000*ones(size(t,2),1);
[Y,~,~] = lsim(sys_cl,[F disturb],t);

% plot(t,Y(:,2),'g');
% hold on;
% plot(t,Y(:,3),'b');
% figure
plot(t,Y(:,1),'r');

title('Response of a lqr system to inital conditions')
end
