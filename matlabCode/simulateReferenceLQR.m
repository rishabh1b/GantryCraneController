function simulateReferenceLQR()
%Use this to illustrate the Precompensator
[A,B,~,D] = getParams();

C = [1 0 0 0 0 0];

Q = C' * C;
Q(1,1) = 90000000;
Q(3,3) = 80000000000;
Q(5,5) = 70000000000;

R = 1;
[K,~,~] = lqr(A,B,Q,R);

states = {'x','x_dot','theta1','theta1_dot','theta2','theta2_dot'};
inputs = {'F'};
outputs = {'x'};

%Use the Precompensator to magnify the input. Illustrated using B*Nbar
sys_ss = ss(A,B,C,0);
Nbar = rscale(sys_ss,K);
sys_cl = ss(A - B * K, B*Nbar, C, D, 'statename',states,'inputname',inputs,'outputname',outputs);

%If stability is to be checked
%eig(A - B * K)

x0 = [0,0,0,0,0,0];
t = 0:0.01:50;
F = 10* ones(size(t));
[Y,~,~] = lsim(sys_cl,F,t,x0);

plot(t,Y(:,1),'r', 'linewidth', 2);
ylabel('Cart Position(m)')
xlabel('Time(sec)')

hold on
plot(t,F,'g')

title('Response of a lqr system to a Step Reference with Precompensator')
end
