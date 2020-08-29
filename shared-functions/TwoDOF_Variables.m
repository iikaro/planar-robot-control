% Degrees-of-freedom
dof = 2;
% Desired state
q_d = zeros(2,length(t));
dq_d = zeros(2,length(t));
ddq_d = zeros(2,length(t));
% State
q = zeros(2,length(t));
dq = zeros(2,length(t));
ddq = zeros(2,length(t));
% State error
int_error = zeros(2,length(t));
q_error = zeros(2,length(t));
dq_error = zeros(2,length(t));
% Torques
T_d = zeros(2,length(t));       %direct dynamics torque
T_a = zeros(2,length(t));       %actuator torque
T_g = zeros(2,length(t));       %gravity compensator
T_ext = zeros(2,length(t));     %end-effector force/torque
T_error = zeros(2,length(t));     %end-effector force/torque
% Forces
F_ext = zeros(2,length(t));     %end-effector force/torque
F_d = zeros(2,length(t));       %desired force
F_r = zeros(2,length(t));       %force reference
% Force error
F_error = zeros(2,length(t));
dF_error = zeros(2,length(t));
int_F_error = zeros(2,length(t));
% Filtered force measurements
F_ext_filtered = zeros(2,length(t));
F_ext_unfiltered = zeros(2,length(t));

% End-effector state
X = zeros(2,length(t));
dX = zeros(2,length(t));
% End-effector orientation
theta = zeros(1,length(t));
% Desired end-effector position
x_d = zeros(2,length(t));
y_d = zeros(2,length(t));
% End-effector position
x = zeros(1,length(t));
y = zeros(1,length(t));
% Admittance controller variables
% Joint
q_v = zeros(2, length(t));
dq_v = zeros(2, length(t));
% Cartesian
x_v = zeros(2, length(t));
dx_v = zeros(2, length(t));

% Model parameters
m = [10, 5];
l = [0.5, 0.5];
torque_max = [26.378838; 26.378838];
torque_min = -torque_max;
g = -9.81;