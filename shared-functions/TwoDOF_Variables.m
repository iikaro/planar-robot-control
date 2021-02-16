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
T_r = zeros(2,length(t));       %
T_env = zeros(2,length(t));     %end-effector force/torque
T_error = zeros(2,length(t));     %end-effector force/torque
dT_error = zeros(2,length(t));
int_T_error = zeros(2,length(t));
% Forces
F_env = zeros(2,length(t));     %end-effector force/torque
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
q_r = zeros(2, length(t));
dq_r = zeros(2, length(t));
% Cartesian
x_r = zeros(2, length(t));
dx_r = zeros(2, length(t));
% Additional paramters
T_p = zeros(2,length(t));
T_error_out = zeros(2,length(t));
T_error_in = zeros(2,length(t));
T_error_in_int = zeros(2,length(t));
dT_error_in = zeros(2,length(t));
kalman_states = zeros(4,length(t));
T_d_in = T_d;
T_d_out = T_d;

% Kalman filter variables
hipState = zeros(2,1);
hipStateEstimate = zeros(2,1);

kneeState = zeros(2,1);
kneeStateEstimate = zeros(2,1);

P = diag([1e-4, 1e-4]);
Pk = zeros(2,2);
input = 0;
Qk = diag([1e-3, 1e-2]);
Rk = 1e-4;
Hk = [1, 0];
Fk = [1, dt; 0, 1];
Bk = [0.5*dt*dt; dt];
Kk = zeros(2,1);

% Model parameters
m = [5, 5];
%l = [0.5, 0.5];
l = [0.41, 0.5];

gear_ratio = 150;       %dimensionless
nominal_current = 3.17; %A
nominal_torque = 187e-3;%N.m
torque_cte = 60.3e-3;   %N.m/A
eff = 0.92;             %dimensionless

torque_max = nominal_current*torque_cte*eff*gear_ratio*ones(2,1); %N.m
torque_min = -torque_max;	%N.m

g = -9.81;