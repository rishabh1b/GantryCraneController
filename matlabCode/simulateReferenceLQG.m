function simulateReferenceLQG()
%This illustrates steady state error with a LQG controller

[A,B,~,D] = getParams();
C = [1 0 0 0 0 0];

Q = C' * C;

Q(1,1) = 90000000;
Q(3,3) = 80000000000;
Q(5,5) = 70000000000;

R = 1;
[K,~,~] = lqr(A,B,Q,R);

%Get the Kalman State Estimator
sys_1 = ss(A,[B B],C,[zeros(1,1) zeros(1,1)]);

Rn = 10^-2 * eye(1);
Qn = 0.2;

sensors = [1];
known = [1];
[~,L,~] = kalman(sys_1,Qn,Rn,[],sensors,known);

states = {'x','x_dot','theta1','theta1_dot','theta2','theta2_dot','e_1','e_2','e_3','e_4','e_5','e_6'};
inputs = {'F'};
outputs = {'x'};

Ac = [A-B*K B*K;zeros(size(A)) A-L*C];
Bc = [B;zeros(size(B))];
Cc = [C zeros(size(C))];
sys_cl = ss(Ac,Bc,Cc,D, 'statename',states,'inputname',inputs,'outputname',outputs);    

init_pos = [0,0,0];
x0 = [init_pos(1);0;init_pos(2);0;init_pos(3);0;init_pos(1);0;0.001*init_pos(2);0;0.001*init_pos(3);0];
t = 0:0.01:50;
F = 10*ones(size(t));
[Y,~,~] = lsim(sys_cl,F,t,x0);
figure
plot(t,Y(:,1),'r','linewidth',2);
ylabel('Cart Position(m)')
xlabel('Time(sec)')
title('Response of an LQG system to Step-Reference of 10m')
end
