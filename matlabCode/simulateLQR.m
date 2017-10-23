function simulateLQR()
[A,B,C,D] = getParams();

Q = C' * C;
Q(1,1) = 90000000;
Q(3,3) = 80000000000;
Q(5,5) = 70000000000;

R = 1;
[K,~,~] = lqr(A,B,Q,R);

states = {'x','x_dot','theta1','theta1_dot','theta2','theta2_dot'};
inputs = {'F'};
outputs = {'x','theta1','theta2'};

sys_cl = ss(A - B * K, zeros(size(B)), C, D, 'statename',states,'inputname',inputs,'outputname',outputs);

%Check the eigen values of the closed loop system
%eig(A - B * K)

x0 = [0,0,15*pi/180,0,20*pi/180,0];
t = 0:0.01:50;
F = zeros(size(t));
[Y,~,X] = lsim(sys_cl,F,t,x0);

%Calculate the control input as a function of time
u = zeros(size(t));
for i = 1:size(X,1)
   u(i) = K * (X(i,1:6))';
end

%Plots
[AX,~,~] = plotyy(t,Y(:,2),t,Y(:,3),'plot');
set(get(AX(1),'Ylabel'),'String','pendulum angle theta1 (radians)')
set(get(AX(2),'Ylabel'),'String','pendulum angle theta2 (radians)')
xlabel('Time(t)');
figure
plot(t,Y(:,1),'r', 'linewidth',2)
xlabel('Time(t)')
ylabel('Cart Position (m)')
figure
plot(t,u)
xlabel('Time(sec)')
ylabel('Control Input(N)');
end
