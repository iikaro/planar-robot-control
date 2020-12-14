function [M, C, G] = invDyn(q, dq)
%% Variables
q1 = q(1);
q2 = q(2);
q3 = q(3);

dq1 = dq(1);
dq2 = dq(2);
dq3 = dq(3);

%% Model parameters
% ExoTao parameters
m = [9.9 3.95 1.66];
l = [.4 .4 .1];
lc = [.2 .2 .05];
I = [.2 .053 .0044];

% Icaro's parameters
m = [1, 1, 1];
l = [1, 1, 1];
I = (1/3)*m.*l.^2;
lc = l./2;

m1 = m(1);
m2 = m(2);
m3 = m(3);

I1 = I(1);
I2 = I(2);
I3 = I(3);

l1 = l(1);
l2 = l(2);
l3 = l(3);

lc1 = lc(1);
lc2 = lc(2);
lc3 = lc(3);

g = -9.81;

%% Inertia
M11 = I1 + I2 + I3 + m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2)) + ...
    m3*(l1^2 + l2^2 + lc3^2 + 2*l1*l2*cos(q2) + 2*l1*lc3*cos(q2 + q3) + 2*l2*lc3*cos(q3));

M12 = I2 + I3 + m2*(lc2^2 + 2*l1*lc2*cos(q2)) + ...
    m3*(l2^2 + lc3^2 + l1*l2*cos(q2) + 2*l1*lc3*cos(q2 + q3) + 2*l2*lc3*cos(q3));

M13 = I3 + m3*(lc3^2 + l1*lc3*cos(q2 + q3) + l2*lc3*cos(q3));

M21 = M12;

M22 = I2 + I3 + m2*lc2^2 + m3*(l2^2 + lc3^2 + 2*l2*lc3*cos(q3));

M23 = I3 + m3*(lc3^2 + l2*lc3*cos(q3));

M31 = M13;

M32 = M23;

M33 = I3 + m3*lc3^2;

M = [M11, M12, M13;
    M21, M22, M23;
    M31, M32, M33];

%% Coriolis
C11 = -(m2*l1*lc2*sin(q2) + m3*l1*l2*sin(q2) + m3*l1*lc3*sin(q2 + q3))*dq2 + ...
    -(m3*l1*lc3*sin(q2 + q3) + m3*l2*lc3*sin(q3))*dq3;

C12 = -(m2*l1*lc2*sin(q2) + m3*l1*l2*sin(q2) + m3*l1*lc3*sin(q2 + q3))*(dq1 + dq2) + ...
    -(m3*l1*lc3*sin(q2 + q3) + m3*l2*lc3*sin(q3))*dq3;

C13 = -(m3*l1*lc3*sin(q2 + q3) + m3*l2*lc3*sin(q3))*(dq1 + dq2 + dq3);

C21 = (m2*l1*lc2*sin(q2) + m3*l1*l2*sin(q2) + m3*l1*lc3*sin(q2 + q3))*dq1 + ...
    -m3*l2*lc3*sin(q3)*dq3;

C22 = -(m3*l2*lc3*sin(q3))*dq3;

C23 = -(m3*l2*lc3*sin(q3))*(dq1 + dq2 + dq3);

C31 = (m3*l1*lc3*sin(q2 + q3) + m3*l2*lc3*sin(q3))*dq1 + m3*l2*lc3*sin(q3)*dq3;

C32 = m3*l2*lc3*sin(q3)*(dq1 + dq2);

C33 = 0;

C = [C11, C12, C13;
    C21, C22, C23;
    C31, C32, C33];

%% Gravity
G11 = g*(m1*lc1 + m2*l1 + m3*l1)*cos(q1) + g*(m2*lc2 + m3*l2)*cos(q1 + q2) + g*m3*lc3*cos(q1 + q2 + q3);

G21 = m2*g*lc2*cos(q1 + q2) +m3*g*(l2*cos(q1 + q2) + lc3*cos(q1 + q2 + q3));

G31 = g*m3*lc3*cos(q1 + q2 + q3);

G = [G11;
    G21;
    G31];
