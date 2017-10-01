%% =========================================Part2==========================
clear;
clc;
time_start = 0; % initial Time 
step = 0.01; % Step for each step
total_time = 20; % Final TIme
num =15; % no of iterations
x = zeros(num,1); % array for x tracking
y = zeros(num,1); % array for y tracking
z = zeros(num,1); % array for z tracking
u = [0;0;1]; % initial testing u
q = [0  ; 0 ;  0]; % initial q for ode45
q_dot = [0;0;0]; % initial qdot for ode45
tau = nonlinear(u,q,q_dot);

%=========================== Loop to get Position =========================
for i=1:num

%============== Torque by considering U value =============================

    [T,X]=ode45(@(t,x) dy_var(x,tau),[time_start step],[q,q_dot]);
    q = X(length(X),1:3)';
    q_dot = X(length(X),4:6)';
    [Y, Y_dot] = f_k(q,q_dot);
%============== Putting Torque value in Linear Dynamics ===================
    x(i) = Y(1);
    y(i) = Y(2);
    z(i) = Y(3);
end
t = 0:num-1;
%%============ Graph Of X,Y and Z locations ===============================
set(gcf, 'Color', [1 1 1]);
plot(t,x,'g',t,y,'b--o',t,z,'c--*');
legend('X','Y','Z');
title('Position of End-effector(u3)');
xlabel('Time /s');
ylabel('Position /m');