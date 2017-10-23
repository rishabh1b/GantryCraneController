function simulateReferenceLQRIntegral()
%Use this to illustrate the Integral Action to resolve the steady State
%Error
[A,B,~,D] = getParams();
C = [1 0 0 0 0 0];

%Prepare the Augmented Matrices
A_I = [A zeros(size(A,1),1);eye(1,size(A,2)) 0];
B_I = [B;zeros(size(B,2))];
B_2 = B_I;
B_2(end) = -1;
C_I = [C 0];

%After augmenting the state vector the system may or may not be
%controllable. This check is to confirm that the system is controllable
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%rank(ctrb(A_I,B_I))

%Setup the Weights and get the optimal gain
Q = C_I' * C_I;
Q(1,1) = 90000000;
Q(3,3) = 80000000000;
Q(5,5) = 70000000000;
Q(7,7) = 10000000;

R = 1;
[K,~,~] = lqr(A_I,B_I,Q,R);

states = {'x','x_dot','theta1','theta1_dot','theta2','theta2_dot','x_int'};
inputs = {'F','disturb'};
outputs = {'x'};

%Precompensation based on the original plant model
%Evaluate K2 First based on just the original plant model
% sys_ss = ss(A,B,C,zeros(size(C,1),size(B,2)));
% Nbar = rscale(sys_ss,K2)
%Apply this as B*Nbar. This cannot reject constant disturbances

%Closed Loop Plant with the Integral action
sys_cl = ss(A_I - B_I * K, [B_2 B_I], C_I, D, 'statename',states,'inputname',inputs,'outputname',outputs);

%Initial State
%X0 = [0,0,0,0,0,0,0];
r = 2;
Fd = 1000;
t = 0:0.01:50;
F = r * ones(size(t,2),1);
disturb = Fd*ones(size(t,2),1);
[Y,~,~] = lsim(sys_cl,[F disturb],t);

plot(t,Y(:,1),'r','linewidth',2);
xlabel('Time(sec)')
ylabel('Cart Position(m)')
hold on
plot(t,r*ones(size(t)),'g')

title('Response of the system to Step Reference of 2m with external disturbances')
end
