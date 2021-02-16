%% Kalman Filter
% Knee Joint
kneeStateEstimate = (Fk*kneeState) + (Bk*input);
Pk = Fk*(P*Fk') + Qk;
Kk = (Pk*Hk')*inv((Hk*(Pk*Hk')) + Rk);
% Values update
P = Pk - Kk*(Hk*Pk);
kneeState = kneeStateEstimate + Kk*(q(2,i) - Hk*kneeStateEstimate);
% Variable asignment
kalman_states(3:4,i) = kneeState;

q(2, i) = kneeState(1,1);
dq(2,i) = kneeState(2,1);

% Hip Joint
hipStateEstimate = (Fk*hipState) + (Bk*input);
Pk = Fk*(P*Fk') + Qk;
Kk = (Pk*Hk')*inv((Hk*(Pk*Hk')) + Rk);
% Values update
P = Pk - Kk*(Hk*Pk);
hipState = hipStateEstimate + Kk*(q(1,i) - Hk*hipStateEstimate);
% Variable asignment
kalman_states(1:2,i) = hipState;

q(1, i) = hipState(1,1);
dq(1,i) = hipState(2,1);
% End