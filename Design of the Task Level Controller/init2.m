function F = init2(q)
% equation from y(q) = h(q) and in this case y(0) is given
a = [0 431.8/1000 -20.32/1000 0]; 
d = [0 149.09/1000 0 433.07/1000]; 
Y = [-0.5;-0.1; 0.0];

F(1) = a(3)*cos(q(1))*cos(q(2)+q(3)) + d(4)*cos(q(1))*sin(q(2)+q(3)) + a(2)*cos(q(1))*cos(q(2)) - d(2)*sin(q(1)) - Y(1);
F(2) = a(3)*sin(q(1))*cos(q(2)+q(3)) + d(4)*sin(q(1))*sin(q(2)+q(3)) + a(2)*sin(q(1))*cos(q(2)) + d(2)*cos(q(1)) - Y(2);
F(3) = - a(3)*sin(q(2)+q(3)) + d(4)*cos(q(2)+q(3)) - a(2)*sin(q(2)) - Y(3);