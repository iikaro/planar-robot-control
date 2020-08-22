% Degrees-of-freedom
dof = 2;
% Desirable q
q_d = zeros(2,length(t));
dq_d = zeros(2,length(t));
ddq_d = zeros(2,length(t));
% Measured q
q_m = zeros(2,length(t));
dq_m = zeros(2,length(t));
ddq_m = zeros(2,length(t));
% Torques
T_d = zeros(2,length(t));       %direct dynamics torque
T_a = zeros(2,length(t));       %actuator torque
T_g = zeros(2,length(t));       %gravity compensator
% Forces
F_error = zeros(2,length(t));   %force error (adm. controller)
F_ext = zeros(2,length(t));     %end-effector force/torque
F_d = zeros(2,length(t));

tau_ext = zeros(2,length(t));     %end-effector force/torque
% Errors
int_error = zeros(2,length(t));
q_error = zeros(2,length(t));
dq_error = zeros(2,length(t));
% End-effector variables
x = zeros(2,length(t));
dx = zeros(2,length(t));
theta = zeros(1,length(t));
x_d = zeros(2,length(t));
y_d = zeros(2,length(t));
x_m = zeros(1,length(t));
y_m = zeros(1,length(t));
% Admittance controller variables
q_v = zeros(2, length(t));
dq_v = zeros(2, length(t));
x_v = zeros(2, length(t));
dx_v = zeros(2, length(t));
% Model parameters
m = [10, 5];
l = [0.5, 0.5];
torque_max = [26.378838; 26.378838];
torque_min = -torque_max;
g = -9.81;