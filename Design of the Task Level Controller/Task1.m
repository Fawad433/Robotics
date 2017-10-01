clear;
clc;
q = [0  ; 0  ;  0]; % initial condition of an ode45
q_dot = [0;0;0];  % initial condition of an ode45
time_start = 0; % Initial Time 
step = 0.05; % time step
total_time = 20; % Final time
torque = [0;1;0];  % Torque to check
[t,y]=ode45(@(t,x) dy_var(x,torque),[time_start step],[q,q_dot]);

set(gcf, 'Color', [1 1 1]);
%================================== Graph of Theta=========================
figure(2)
plot(t,y(:,1),'g',t,y(:,2),'b--o',t,y(:,3),'c--*');
legend('Q1','Q2','Q3');
title('Joint Variables');
xlabel('Time /s');
ylabel('Angle / rad');
%================================= Graph of Angular Velocities============
figure(1)
plot(t,y(:,4),'g',t,y(:,5),'b--o',t,y(:,6),'c--*');
legend('Q1dot','Q2dot','Q3dot');
title('Joint Velocities');
xlabel('Time /s');
ylabel('Angular velocity / rad/s');
