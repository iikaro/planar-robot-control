% 1-DoF Model Parameters
dof = 1;
g = 9.81;               %m/s^2 gravity
rho = 2710;             %kg/m^3 Aluminium density
re = 1.5*10^-2;         %m radius
ri = 0.5*10^-2;         %m radius
L = 0.3;                %m length
V = pi*(re^2-ri^2)*L;   %m^3
m = rho*V;              %kg
J = 10*(1/3)*m*L^2;     %kg.m^2
B = 10;                 %N.s/rad
K = 0;                  %N/rad
offset = 0;             %rad

gear_ratio = 150;       %dimensionless
nominal_current = 3.17; %A
nominal_torque = 187e-3;%N.m
torque_cte = 60.3e-3;   %N.m/A
eff = 0.92;             %dimensionless

torque_max = nominal_current*torque_cte*eff*gear_ratio; %N.m
torque_min = -torque_max;	%N.m

% 1-DoF Variables

q_d = zeros(length(t),1);
dq_d = zeros(length(t),1);
ddq_d = zeros(length(t),1);

q = zeros(length(t),1);
dq = zeros(length(t),1);
ddq = zeros(length(t),1);

q_r = zeros(length(t),1);
dq_r = zeros(length(t),1);

q_error = zeros(length(t),1);
dq_error = zeros(length(t),1);
int_error = zeros(length(t),1);

tau_error = zeros(length(t),1);
tau_int_error = zeros(length(t),1);
dtau_error =  zeros(length(t),1);

tau_a = zeros(length(t),1);
tau_ext = zeros(length(t),1);
tau_d = zeros(length(t),1);
tau_r = zeros(length(t),1);

F_error = zeros(2,length(t));
F_int_error = zeros(2,length(t));
dF_error = zeros(2,length(t));
F_ext = zeros(2,length(t));
F_d = zeros(2,length(t));

x = zeros(length(t),1);
y = zeros(length(t),1);
dx = zeros(length(t),1);
dy = zeros(length(t),1);
ddX = zeros(2,length(t));