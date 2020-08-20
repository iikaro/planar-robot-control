function [t, dt] = SimulationTime(t_initial, t_final, samples)
% SimulationTime  Return the simulation time array (t) and the time step (dt).
%
%   [t, dt] = SimulationTime(t_initial, t_final, total_samples)
%
%   [t, dt] = SimulationTime(t_initial, t_final, dt)
%
%   If samples is greater than unit, it is considered as the total number
%   of samples. If it is less than unit, it is considered as the time step
%   itself.
if samples < 1
    dt = samples;
    t = linspace(t_initial, t_final, t_final*1/dt + 1);
else
    dt = 1/samples;
    t = linspace(t_initial, t_final, samples + 1);
end