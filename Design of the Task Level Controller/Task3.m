clear;
clc;
%% Initialization
time_start = 0; % initial time 
step = 0.01; % Time stepsize
total_time = 40; % final time
w = pi/2; % for Yd
count = 1; % count of iteration
Kd = [36;31;20]; % for making system stable 
Kp = [9;9;2]; % for making system stable 
err_x = zeros(1,400); % for keeping error track of x 
err_y = zeros(1,400); % for keeping error track of y
err_z = zeros(1,400); % for keeping error track of z
err_x_dot = zeros(1,400); % for keeping error track of xdot
err_y_dot = zeros(1,400); % for keeping error track of ydot
err_z_dot = zeros(1,400); % for keeping error track of zdot
%% INVERSE KINEMATICS 
%Robot parameters from the given table
a = [0 431.8/1000 -20.32/1000 0]; 
d = [0 149.09/1000 0 433.07/1000]; 

%Initial Y and Y' given in the project
Y_zero_1 = [-0.7765 ; 0 ; 0.045];
Y_zero_2 = [-0.5 ; -0.1 ; 0];
Y_dot_zero_1 = [0;0;0];
Y_dot_zero_2 = [0;0;0];

% for getting inverse kinematics 
q0_1 = [0,0,0];
q_init_1 = fsolve(@init1,q0_1)
q0_2 = [pi/4,-pi/4,pi/4];
q_init_2 = fsolve(@init2,q0_2)


%% For finding initial torque
q = (q_init_1)'; % taking first y(0) angle
q_dot = [0;0;0]; 
Y_init = Y_zero_1; % taking first y(0)
Y_dot_init = [0;0;0];
[Yd, Yd_dot, Yd_dot_dot] = trajectory(w,0); 
e = Yd - Y_init; % Initial Error in distance 
e_dot = Yd_dot - Y_dot_init; % Initial Error in velocity 
u = Yd_dot_dot + Kp.*e + Kd.*e_dot; % u = Yd" + Kd(e') + Kp(e) and initially Yd' and Yd" = 0 and e = Yd
torque = nonlinear(u,q,q_dot);
%% Main Error Loop
while count*step < total_time
    
    %%Dynamics Block%%
    [T,X]=ode45(@(t,x) dy_var(x,torque),[time_start step],[q,q_dot]);
    q = X(length(X),1:3)';
    q_dot = X(length(X),4:6)'; 
    
    % Forward Kinematics
    [Y, Y_dot] = f_k(q,q_dot);
    [Yd, Yd_dot, Yd_dot_dot] = trajectory(w,(count*step));
    
    % Error 
    e = Yd - Y;
    e_dot = Yd_dot - Y_dot;
    u = Yd_dot_dot + Kd.*e_dot + Kp.*e;
    % Torque
    torque = nonlinear(u,q,q_dot);
    
    % Error Tracking
    count = count + 1;
    err_x(count) = e(1); % Error in distance along x
    err_y(count) = e(2); % Error in distance along y
    err_z(count) = e(3); % Error in distance along z
    err_x_dot(count) = e_dot(1); % Error in velocity along x
    err_y_dot(count) = e_dot(2); % Error in velocity along y
    err_z_dot(count) = e_dot(3); % Error in velocity along z
end
set(gcf, 'Color', [1 1 1]);
t=0:0.01:39.99;
figure(2)
plot(t,err_x,'g',t,err_y,'b',t,err_z,'c');
axis([0 20 -0.02 0.02])
legend('ErrorX','ErrorY','ErrorZ');
title('Position Error Graph');
figure(1)
plot(t,err_x_dot,'g',t,err_y_dot,'b',t,err_z_dot,'c');
axis([0 20 -0.05 0.05])
legend('ErrorX','ErrorY','ErrorZ');
title('Velocity Error Graph');
