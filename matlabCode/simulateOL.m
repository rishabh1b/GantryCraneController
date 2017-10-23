function simulateOL()
%Simulate response of the cart and the pendulum system in Open Loop
[A,B,C,D] = getParams();
states = {'x','x_dot','theta1','theta1_dot','theta2','theta2_dot'};
inputs = {'F'};
outputs = {'x','theta1','theta2'};

sys_ol = ss(A, B, C, D, 'statename',states,'inputname',inputs,'outputname',outputs);

%Response to some initial conditions and zero-input
x0 = [0,0,20*pi/180,0,20*pi/180,0];
t = 0:0.01:50;
F = zeros(size(t));
%Simulate using lsim, Can use 'initial' command too
[Y,~,~] = lsim(sys_ol,F,t,x0);

% plot(t,Y(:,2),'g');
% hold on;
% plot(t,Y(:,3),'b');
% figure 
plot(t,Y(:,1),'r');
xlabel('Time(t)')
ylabel('Position(X)')
title('Response of a cart and pendulum system to inital conditions')
end
