function [Y Y_dot] = f_k(q,q_dot)

% Robot parameters taken from given table 
a = [0 431.8/1000 -20.32/1000 0];
d = [0 149.09/1000 0 433.07/1000];

%% for getting h matrix

q1 = q(1);
q2 = q(2);
q3 = q(3);
h1 = a(3)*cos(q1)*cos(q2+q3) + d(4)*cos(q1)*sin(q2+q3) + a(2)*cos(q1)*cos(q2) - d(2)*sin(q1);
h2 = a(3)*sin(q1)*cos(q2+q3) + d(4)*sin(q1)*sin(q2+q3) + a(2)*sin(q1)*cos(q2) + d(2)*cos(q1);
h3 = - a(3)*sin(q2+q3) + d(4)*cos(q2+q3) - a(2)*sin(q2);
h = [h1; h2; h3];
%% For getting Y and Y_dot
Y = h; % Y = h(q)
J = double([(127*cos(q2 + q3)*sin(q1))/6250-(2159*cos(q2)*sin(q1))/5000 - (14909*cos(q1))/100000 - ....
    (43307*sin(q2 + q3)*sin(q1))/100000, (43307*cos(q2 + q3)*cos(q1))/100000 - ....
    (2159*cos(q1)*sin(q2))/5000 + (127*sin(q2 + q3)*cos(q1))/6250, (43307*cos(q2 + q3)*cos(q1))/100000....
    + (127*sin(q2 + q3)*cos(q1))/6250 ;(2159*cos(q1)*cos(q2))/5000 - (14909*sin(q1))/100000....
    - (127*cos(q2 + q3)*cos(q1))/6250 + (43307*sin(q2 + q3)*cos(q1))/100000, (43307*cos(q2 + q3)*sin(q1))/100000....
    - (2159*sin(q1)*sin(q2))/5000 + (127*sin(q2 + q3)*sin(q1))/6250, (43307*cos(q2 + q3)*sin(q1))/100000....
    + (127*sin(q2 + q3)*sin(q1))/6250; 0,(127*cos(q2 + q3))/6250 - (43307*sin(q2 + q3))/100000....
    - (2159*cos(q2))/5000,(127*cos(q2 + q3))/6250 - (43307*sin(q2 + q3))/100000]);
 
Y_dot = J*q_dot;
