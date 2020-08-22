function [H, G, C, F] = TwoDOF_InvDyn(q, dq, l, m, g)
I = 10*(1/3).*m.*l.^2;
Jm = 0.47;  %(dos Santos, 2014)
I(1) = I(1) + Jm;

H11 = m(1)*l(1)/2^2 + m(2)*(l(1)^2 + l(2)/2^2 + 2*l(1)*l(2)/2*cos(q(2))) + I(1) + I(2);
H12 = m(2)*(l(2)/2^2 + l(1)*l(2)/2*cos(q(2))) + I(2);
H21 = m(2)*(l(2)/2^2 + l(1)*l(2)/2*cos(q(2))) + I(2);
H22 = m(2)*l(2)/2^2 + I(2);

H = [H11 H12; H21 H22];

h = sin(q(2))*m(2)*l(1)*l(2)/2;

C11 = -h*dq(2);
C12 = -h*(dq(1) + dq(2));
C21 = h*dq(1);
C22 = 0;

C = [C11 C12; C21 C22];

G11 = (m(1)*g*l(1)/2 + m(2)*g*l(1))*cos(q(1)) + m(2)*g*l(2)/2*cos(q(1) + q(2));
G21 = m(2)*g*l(2)/2*cos(q(1) + q(2));

G = [G11; 
    G21];

F11 = 0.1;
F21 = 0.1;

F = [F11, F21];

end