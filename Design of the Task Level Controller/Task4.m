clear all
clc
%% Initialization
time_start = 0; % Initial Time 
step = 0.01; % Time Step
total_time = 20; % Final Time
w = pi/2; % for yd
count = 1; % for iteration count
Kd = [36;31;20]; % for the stability of system
Kp = [9;9;2];  % for the stability of system
err_x = zeros(1,400); % For position error track of x
err_y = zeros(1,400); % For position error track of y
err_z = zeros(1,400); % For position error track of z
err_x_dot = zeros(1,400); % For velocity error track of x
err_y_dot = zeros(1,400); % For velocity error track of y
err_z_dot = zeros(1,400); % For velocity error track of z
%% Getting percentage matrix
per = 2; %Percentage
R = percentagematrix(per);
%% Inverse Kinematics
%Robot parameters from the given table
a = [0 431.8/1000 -20.32/1000 0]; 
d = [0 149.09/1000 0 433.07/1000]; 

%Initial Y and Y' given in the project
Y_zero_1 = [-0.7765 ; 0 ; 0.045]; % Given in project
Y_zero_2 = [-0.5 ; -0.1 ; 0]; % Given in project
Y_dot_zero_1 = [0;0;0];
Y_dot_zero_2 = [0;0;0];

% for getting inverse kinematics 
q0_1 = [0,0,0];
q_init_1 = fsolve(@init1,q0_1) % for given Y_zero_1
q0_2 = [pi/4,-pi/4,pi/4];
q_init_2 = fsolve(@init2,q0_2) % for given Y_zero_2

%% Finding initial  torque 
q = (q_init_1)';
q_dot = [0;0;0]; 
Y_init = Y_zero_1;
Y_dot_init = [0;0;0];
[Yd, Yd_dot, Yd_dot_dot] = trajectory(w,0);
e = Yd - Y_init; % Initial position Error
e_dot = Yd_dot - Y_dot_init; % Initial velocity Error
u = Yd_dot_dot + Kp.*e + Kd.*e_dot; % u = Yd" + Kd(e') + Kp(e) and initially Yd' and Yd" = 0 and e = Yd
torque = nonlinear(u,q,q_dot);
%% Main loop for minimizing error 

while count*step < total_time
    
    % Dynamics 
    [T,X]=ode45(@(t,x) dy_var1(x,torque,R),[time_start step],[q,q_dot]);
    q = X(length(X),1:3)';
    q_dot = X(length(X),4:6)';
    % forward Kinematics
    [Y, Y_dot] = f_k(q,q_dot);
    [Yd, Yd_dot, Yd_dot_dot] = trajectory(w,(count*step));
    % Error
    e = Yd - Y;
    e_dot = Yd_dot - Y_dot;
    u = Yd_dot_dot + Kd.*e_dot + Kp.*e;
    % Torque
    torque = nonlinear(u,q,q_dot);
    % Error Track
    count = count + 1;
    err_x(count) = e(1);
    err_y(count) = e(2);
    err_z(count) = e(3);
    err_x_dot(count) = e_dot(1);
    err_y_dot(count) = e_dot(2);
    err_z_dot(count) = e_dot(3);
end

t=0:0.01:19.99;
figure(1)
plot(t,err_x,'g',t,err_y,'b',t,err_z,'c');
axis([0 20 -1 1])
legend('ErrorX','ErrorY','ErrorZ');
title('Position Error Graph');
figure(2)
plot(t,err_x_dot,'g',t,err_y_dot,'b',t,err_z_dot,'c');
axis([0 20 -0.5 0.5])
legend('ErrorX','ErrorY','ErrorZ');
title('Velocity Error Graph');
