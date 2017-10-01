function out = dy_var(x,tau)

q = [x(1); x(2); x(3)];
x1 = q;
qdot = [x(4); x(5); x(6)];
x2 = qdot;

%% For the calculation of D 
d11 = 2.4574 + 1.7181*cos(q(2))*cos(q(2)) + 0.4430*sin(q(2)+q(3))*sin(q(2)+q(3)) - 0.0324*cos(q(2))*cos(q(2)+q(3)) - 0.0415*cos(q(2)+q(3))*sin(q(2)+q(3)) + 0.9378*cos(q(2))*sin(q(2)+q(3));
d12 = 2.2312*sin(q(2)) - 0.0068*sin(q(2)+q(3)) - 0.1634*cos(q(2)+q(3));
d13 = -0.0068*sin(q(2)+q(3)) - 0.1634*cos(q(2)+q(3));
d21 = d12;
d22 = 5.1285 + 0.9378*sin(q(3)) - 0.0324*cos(q(3));
d23 = 0.4424 + 0.4689*sin(q(3)) - 0.0162*cos(q(3));
d31 = d13;
d32 = d23;
d33 = 1.0236;
D = [d11 d12 d13; d21 d22  d23; d31 d32 d33];

%% for the calculation of C(q,q')
c111 = 0;
c121 = 0.0207 - 1.2752*cos(q(2))*sin(q(2)) + 0.4429*cos(q(3))*sin(q(3)) - 0.8859*sin(q(2))*sin(q(3))*sin(q(2)+q(3)) + 0.0325*cos(q(2))*sin(q(2)+q(3)) + 0.4689*cos(q(2))*cos(q(2)+q(3)) - 0.4689*sin(q(2))*sin(q(2)+q(3)) - 0.0461*cos(q(2)+q(2)) - 0.0415*cos(q(2)+q(3))*cos(q(2)+q(3)) - 0.0163*sin(q(3));
c131 = 0.0207 + 0.4429*cos(q(2))*sin(q(2)) + 0.4429*cos(q(3))*sin(q(3)) - 0.8859*sin(q(2))*sin(q(3))*sin(q(2)+q(3)) + 0.0163*cos(q(2))*sin(q(2)+q(3)) + 0.4689*cos(q(2))*cos(q(2)+q(3)) - 0.0415*cos(q(2)+q(3))*cos(q(2)+q(3));
c211 = c121;
c221 = 1.8181*cos(q(2)) + 0.1634*sin(q(2)+q(3)) - 0.0068*cos(q(2)+q(3));
c231 = 0.1634*sin(q(2)+q(3)) - 0.0068*cos(q(2)+q(3));
c311 = c131;
c321 = c231;
c331 = 0.1634*sin(q(2)+q(3)) - 0.0068*cos(q(2)+q(3));
c112 = - c121;
c122 = 0;
c132 = 0;
c212 = c122;
c222 = 0;
c232 = 0.4689*cos(q(3)) + 0.0162*sin(q(3));
c312 = 0;
c322 = c232;
c332 = 0.4689*cos(q(3)) + 0.0162*sin(q(3));
c113 = - c131;
c123 = - c132;
c133 = 0;
c213 = c123;
c223 = - c232;
c233 = 0;
c313 = c133;
c323 = c233;
c333 = 0;

c11 = c111*qdot(1) + c211*qdot(2) + c311*qdot(3);
c12 = c121*qdot(1) + c221*qdot(2) + c321*qdot(3);
c13 = c131*qdot(1) + c231*qdot(2) + c331*qdot(3);
c21 = c112*qdot(1) + c212*qdot(2) + c312*qdot(3);
c22 = c122*qdot(1) + c222*qdot(2) + c322*qdot(3);
c23 = c132*qdot(1) + c232*qdot(2) + c332*qdot(3);
c31 = c113*qdot(1) + c213*qdot(2) + c313*qdot(3);
c32 = c123*qdot(1) + c223*qdot(2) + c323*qdot(3);
c33 = c133*qdot(1) + c233*qdot(2) + c333*qdot(3);

C = [c11 c12 c13; c21 c22 c23;c31 c32 c33];

%% for the calculation of G(q)
g1 = 0;
g2 = - 48.5564*cos(q(2)) + 1.0462*sin(q(2)) + 0.3683*cos(q(2)+q(3)) - 10.6528*sin(q(2)+q(3));
g3 = 0.3683*cos(q(2)+q(3)) - 10.6528*sin(q(2)+q(3));

G = [g1 ; g2 ; g3];
%% for the input to the ode45 
x1dot = x2;
x2dot = (inv(D))*(tau - G - C*x2);
out = [x1dot ; x2dot];

end

