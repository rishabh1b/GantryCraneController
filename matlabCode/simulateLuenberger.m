function simulateLuenberger()
[A,B,~,D] = getParams();

%When only x(t) is the chosen output vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
C1 = [1 0 0 0 0 0];
C = C1;
P1 = [-2 -3 -4 -5 -6 -7];
%P1 = [-0.1 -0.2 -2.2 -1.9 -2.1 -1.6];
P = P1;
outputs = {'x'};

%When x(t) and theta2(t) are the chosen output vectors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% C3 = [1 0 0 0 0 0;0 0 0 0 1 0];
% C = C3;
% P3 = [-0.5 -1 -1.5 -2 -2.5 -3];
% P = P3;
% outputs = {'x','theta2'};

%when x(t), theta1(t) and theta2(t) are the Output Vectors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% C4 = [1 0 0 0 0 0;0 0 1 0 0 0;0 0 0 0 1 0];
% C = C4;
% P4 = [-2 -3 -4 -5 -6 -7];
% P = P4;
% outputs = {'x','theta1','theta2'};

X0 = [0.2;0;5*pi/180;0;5*pi/180;0];
%Xhat = [0;0;0;0;0;0];
Xhat = [0;0;5*pi/180;0;5*pi/180;0];

L = place(A',C',P)'

states = {'x','x_dot','theta1','theta1_dot','theta2','theta2_dot'};
inputs = {'F'};

sys_ol = ss(A, B, C, D, 'statename',states,'inputname',inputs,'outputname',outputs);

%Step Response
t = 0:0.01:20;
u = ones(size(t));
[Y,~,X] = lsim(sys_ol,u,t,X0);

X_est = Xhat';
k = 2;
for n = 0.01:0.01:20
    dXhat = A * Xhat + B .* u(k) + L * (Y(k,:)' - C*Xhat);
    Xhat = Xhat + 0.01.*dXhat;
    X_est = [X_est;Xhat'];
    k = k + 1;
end
subplot(3,1,1), plot(t,X(:,1)),hold on,plot(t,X_est(:,1),'r')
xlabel('Time(sec)'),ylabel('Cart Position(m)'),legend('X','X_est')
subplot(3,1,2), plot(t,X(:,3)),hold on,plot(t,X_est(:,3),'r')
xlabel('Time(sec)'),ylabel('Pendulum Angle(theta1))'),legend('theta1','theta1Est')
subplot(3,1,3), plot(t,X(:,5)),hold on,plot(t,X_est(:,5),'r')
xlabel('Time(sec)'),ylabel('Pendulum Angle(theta2))'),legend('theta2','theta2Est')
end